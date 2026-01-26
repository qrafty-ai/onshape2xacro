import re
import os
import asyncio
from typing import Any, TYPE_CHECKING, Dict, List
from pathlib import Path
from lxml import etree as ET
from loguru import logger
import yaml

from onshape_robotics_toolkit.formats.base import RobotSerializer
from onshape2xacro.config import ConfigOverride

if TYPE_CHECKING:
    from onshape_robotics_toolkit.robot import Robot
    from onshape_robotics_toolkit.models.link import Link
    from onshape_robotics_toolkit.models.joint import BaseJoint


def is_joint(name: str) -> bool:
    """Check if a mate name indicates a robot joint."""
    return name.startswith("joint_")


def get_joint_name(name: str) -> str:
    """Extract joint name by removing 'joint_' prefix."""
    if is_joint(name):
        return name[6:]
    return name


def sanitize_name(name: str) -> str:
    """Sanitize Onshape names to valid ROS identifiers."""
    s = name.lower()
    s = s.replace(" ", "_").replace("-", "_")
    s = re.sub(r"[^a-z0-9_]", "", s)
    s = re.sub(r"_+", "_", s)
    if s and s[0].isdigit():
        s = "_" + s
    if not s:
        s = "_"
    return s


def is_module_boundary(graph) -> bool:
    """
    Check if a kinematic graph (representing a subassembly)
    should be treated as a xacro module boundary.

    Rule: Boundary if it contains any edges with 'joint_' prefix.
    """
    for edge in graph.edges:
        name = getattr(edge, "name", str(edge))
        if is_joint(name):
            return True
    return False


class XacroSerializer(RobotSerializer):
    """Serializer for Onshape assemblies to hierarchical Xacro."""

    def serialize(self, robot: "Robot", **options: Any) -> str:
        """Serialize robot to a flattened xacro string (for preview)."""
        config = options.get("config") or ConfigOverride()
        root = self._create_xacro_root(robot.name)
        self._add_robot_macro(root, robot, config=config, mesh_map={})
        return ET.tostring(root, pretty_print=True, encoding="unicode")

    def save(
        self, robot: "Robot", path: str, download_assets: bool = True, **options: Any
    ):
        """Save robot to hierarchical xacro structure."""
        out_dir = Path(path)
        urdf_dir = out_dir / "urdf"
        mesh_dir = out_dir / "meshes"
        config_dir = out_dir / "config"

        config = options.get("config") or ConfigOverride()

        for d in [urdf_dir, mesh_dir, config_dir]:
            d.mkdir(parents=True, exist_ok=True)

        # 1. Export meshes (Stage 6)
        mesh_map = {}
        if download_assets:
            mesh_map = self._export_meshes(robot, mesh_dir)

        # 2. Group by subassembly (Stage 4)
        module_groups = self._group_by_subassembly(robot)
        main_name = sanitize_name(robot.name)

        # 3. Generate Xacro files for each subassembly
        for parent_key, elements in module_groups.items():
            name = sanitize_name(parent_key.name if parent_key else robot.name)

            # Root assembly goes directly in urdf/
            # Subassemblies go in urdf/<name>/
            if parent_key is None:
                module_path = urdf_dir / f"{main_name}.xacro"
            else:
                sub_dir = urdf_dir / name
                sub_dir.mkdir(exist_ok=True)
                module_path = sub_dir / f"{name}.xacro"

            root = self._create_xacro_root(name)

            # Calculate mesh relative path from this xacro file
            mesh_rel_path = os.path.relpath(mesh_dir, module_path.parent)

            # Add includes for children modules
            children = [k for k in module_groups.keys() if k and k.parent == parent_key]
            for child in children:
                child_name = sanitize_name(child.name)
                # Child path is urdf/<child_name>/<child_name>.xacro
                child_module_path = urdf_dir / child_name / f"{child_name}.xacro"
                rel_inc_path = os.path.relpath(child_module_path, module_path.parent)

                inc = ET.SubElement(root, "{http://www.ros.org/wiki/xacro}include")
                inc.set("filename", rel_inc_path)

            # Add macro
            self._add_module_macro(
                root,
                name,
                elements,
                config,
                mesh_map,
                children,
                mesh_rel_path=mesh_rel_path,
            )

            with open(module_path, "w") as f:
                f.write(ET.tostring(root, pretty_print=True, encoding="unicode"))

        # 4. Generate main entry point (.urdf.xacro)
        # This file includes the root macro and instantiates it.
        entry_point_root = self._create_xacro_root(robot.name)

        # Include the macro file
        inc = ET.SubElement(entry_point_root, "{http://www.ros.org/wiki/xacro}include")
        inc.set("filename", f"{main_name}.xacro")

        # Instantiate the macro
        ET.SubElement(
            entry_point_root, f"{{http://www.ros.org/wiki/xacro}}{main_name}", prefix=""
        )

        entry_point_content = ET.tostring(
            entry_point_root, pretty_print=True, encoding="unicode"
        )
        entry_point_path = urdf_dir / f"{main_name}.urdf.xacro"
        with open(entry_point_path, "w") as f:
            f.write(entry_point_content)

        # 5. Generate default configs (Stage 7)
        self._generate_default_configs(robot, config_dir)

    def _group_by_subassembly(self, robot: "Robot") -> Dict[Any, Dict[str, List]]:
        groups = {}
        # PathKey objects in networkx graph
        for node, data in robot.nodes(data=True):
            parent = node.parent
            if parent not in groups:
                groups[parent] = {"links": [], "joints": []}
            groups[parent]["links"].append(data.get("data"))

        for parent_node, child_node in robot.edges:
            edge_data = robot.get_edge_data(parent_node, child_node)
            joint = edge_data.get("data")
            # Joint belongs to the module of its parent link
            parent_key = parent_node.parent
            if parent_key not in groups:
                groups[parent_key] = {"links": [], "joints": []}
            groups[parent_key]["joints"].append(joint)
        return groups

    def _create_xacro_root(self, name: str) -> ET._Element:
        root = ET.Element("robot", nsmap={"xacro": "http://www.ros.org/wiki/xacro"})
        root.set("name", sanitize_name(name))
        return root

    def _add_module_macro(
        self,
        root: ET._Element,
        name: str,
        elements: Dict,
        config: ConfigOverride,
        mesh_map: Dict,
        children: List,
        mesh_rel_path: str = "meshes",
    ):
        macro = ET.SubElement(root, "{http://www.ros.org/wiki/xacro}macro")
        macro.set("name", sanitize_name(name))
        macro.set("params", "prefix:=''")

        for link in elements["links"]:
            if link:
                self._link_to_xacro(
                    macro, link, config, mesh_map, mesh_rel_path=mesh_rel_path
                )

        for joint in elements["joints"]:
            if joint and is_joint(joint.name):
                self._joint_to_xacro(macro, joint, config)

        # Call children macros
        for child in children:
            child_name = sanitize_name(child.name)
            ET.SubElement(
                macro,
                "{http://www.ros.org/wiki/xacro}" + child_name,
                prefix="${prefix}",
            )

    def _add_robot_macro(
        self,
        root: ET._Element,
        robot: "Robot",
        config: ConfigOverride,
        mesh_map: Dict,
        mesh_rel_path: str = "meshes",
    ):
        macro = ET.SubElement(root, "{http://www.ros.org/wiki/xacro}macro")
        macro.set("name", sanitize_name(robot.name))
        macro.set("params", "prefix:=''")

        for node, data in robot.nodes(data=True):
            link = data.get("data")
            if link:
                self._link_to_xacro(
                    macro, link, config, mesh_map, mesh_rel_path=mesh_rel_path
                )

        for parent, child in robot.edges:
            edge_data = robot.get_edge_data(parent, child)
            joint = edge_data.get("data")
            if joint and is_joint(joint.name):
                self._joint_to_xacro(macro, joint, config)

    def _link_to_xacro(
        self,
        root: ET._Element,
        link: "Link",
        config: ConfigOverride,
        mesh_map: Dict,
        mesh_rel_path: str = "meshes",
    ):
        name = sanitize_name(link.name)
        link_el = ET.SubElement(root, "link")
        link_el.set("name", f"${{prefix}}{name}")

        default_inertial = {
            "mass": 1.0,
            "origin": {"xyz": "0 0 0", "rpy": "0 0 0"},
            "inertia": {
                "ixx": 0.01,
                "iyy": 0.01,
                "izz": 0.01,
                "ixy": 0,
                "ixz": 0,
                "iyz": 0,
            },
        }
        val = config.get_inertial(name, default_inertial)
        inertial = ET.SubElement(link_el, "inertial")
        ET.SubElement(inertial, "mass", value=str(val["mass"]))
        origin = val.get("origin", default_inertial["origin"])
        ET.SubElement(
            inertial,
            "origin",
            xyz=origin.get("xyz", "0 0 0"),
            rpy=origin.get("rpy", "0 0 0"),
        )
        inertia = val.get("inertia", default_inertial["inertia"])
        ET.SubElement(
            inertial,
            "inertia",
            ixx=str(inertia.get("ixx", 0.01)),
            iyy=str(inertia.get("iyy", 0.01)),
            izz=str(inertia.get("izz", 0.01)),
            ixy=str(inertia.get("ixy", 0)),
            ixz=str(inertia.get("ixz", 0)),
            iyz=str(inertia.get("iyz", 0)),
        )

        if name in mesh_map:
            for tag in ["visual", "collision"]:
                el = ET.SubElement(link_el, tag)
                geom = ET.SubElement(el, "geometry")
                mesh = ET.SubElement(geom, "mesh")
                mesh.set(
                    "filename",
                    f"{mesh_rel_path}/{mesh_map[name]}",
                )

    def _joint_to_xacro(
        self, root: ET._Element, joint: "BaseJoint", config: ConfigOverride
    ):
        name = sanitize_name(get_joint_name(joint.name))
        joint_el = ET.SubElement(root, "joint")
        joint_el.set("name", f"${{prefix}}{name}")

        jtype = getattr(joint, "joint_type", "fixed")
        joint_el.set(
            "type",
            "revolute"
            if jtype == "revolute"
            else "prismatic"
            if jtype == "prismatic"
            else "fixed",
        )

        ET.SubElement(
            joint_el, "parent", link=f"${{prefix}}{sanitize_name(joint.parent)}"
        )
        ET.SubElement(
            joint_el, "child", link=f"${{prefix}}{sanitize_name(joint.child)}"
        )

        if jtype in ["revolute", "prismatic"]:
            default_limit = {
                "lower": -3.14,
                "upper": 3.14,
                "effort": 100,
                "velocity": 1.0,
            }
            val = config.get_joint_limit(name, default_limit)
            ET.SubElement(
                joint_el,
                "limit",
                lower=str(val["lower"]),
                upper=str(val["upper"]),
                effort=str(val["effort"]),
                velocity=str(val["velocity"]),
            )

    def _export_meshes(self, robot: "Robot", mesh_dir: Path) -> Dict[str, str]:
        mesh_map = {"__robot_name__": robot.name}

        async def _download_all():
            tasks = []
            for node, data in robot.nodes(data=True):
                asset = data.get("asset")
                if asset:
                    asset.mesh_dir = str(mesh_dir)
                    tasks.append(asset.download())
                    mesh_map[sanitize_name(data.get("data").name)] = asset.file_name
            if tasks:
                await asyncio.gather(*tasks)

        print(f"Downloading meshes to {mesh_dir}...")
        try:
            asyncio.run(_download_all())
        except Exception as e:
            logger.error(f"Failed to download meshes: {e}")
        return mesh_map

    def _generate_default_configs(self, robot: "Robot", config_dir: Path):
        joint_limits = {}
        inertials = {}
        for parent, child in robot.edges:
            edge_data = robot.get_edge_data(parent, child)
            joint = edge_data.get("data")
            if joint and is_joint(joint.name):
                name = sanitize_name(get_joint_name(joint.name))
                joint_limits[name] = {
                    "lower": -3.14,
                    "upper": 3.14,
                    "effort": 100.0,
                    "velocity": 1.0,
                }
        for node, data in robot.nodes(data=True):
            link = data.get("data")
            if link:
                name = sanitize_name(link.name)
                inertials[name] = {
                    "mass": 1.0,
                    "origin": {"xyz": "0 0 0", "rpy": "0 0 0"},
                    "inertia": {
                        "ixx": 0.01,
                        "iyy": 0.01,
                        "izz": 0.01,
                        "ixy": 0,
                        "ixz": 0,
                        "iyz": 0,
                    },
                }

        with open(config_dir / "joint_limits.yaml", "w") as f:
            yaml.dump({"joint_limits": joint_limits}, f)
        with open(config_dir / "inertials.yaml", "w") as f:
            yaml.dump({"inertials": inertials}, f)
