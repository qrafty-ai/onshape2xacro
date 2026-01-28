import os
from typing import Any, TYPE_CHECKING, Dict, List, Optional
from pathlib import Path
from lxml import etree as ET
from loguru import logger
import yaml

from onshape_robotics_toolkit.formats.base import RobotSerializer
from onshape2xacro.config import ConfigOverride
from onshape2xacro.naming import sanitize_name
from onshape2xacro.mesh_exporters.step import StepMeshExporter

if TYPE_CHECKING:
    from onshape_robotics_toolkit.robot import Robot


def is_joint(name: str) -> bool:
    """Check if a mate name indicates a robot joint."""
    return name.startswith("joint_")


def get_joint_name(name: str) -> str:
    """Extract joint name by removing 'joint_' prefix."""
    if is_joint(name):
        return name[6:]
    return name


# sanitize_name was here, moved to naming.py


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
        self,
        robot: "Robot",
        file_path: str,
        download_assets: bool = True,
        mesh_dir: Optional[str] = None,
        **options: Any,
    ):
        """Save robot to hierarchical xacro structure."""
        out_dir = Path(file_path)
        urdf_dir = out_dir / "urdf"
        if mesh_dir:
            mesh_dir_path = Path(mesh_dir)
        else:
            mesh_dir_path = out_dir / "meshes"
        config_dir = out_dir / "config"

        config = options.get("config") or ConfigOverride()

        for d in [urdf_dir, mesh_dir_path, config_dir]:
            d.mkdir(parents=True, exist_ok=True)

        # 1. Export meshes (Stage 6)
        mesh_map = {}
        missing_meshes = {}
        if download_assets:
            mesh_map, missing_meshes = self._export_meshes(robot, mesh_dir_path)

        # 2. Group by subassembly (Stage 4)
        module_groups = self._group_by_subassembly(robot)
        main_name = sanitize_name(robot.name)

        # 3. Generate Xacro files for each subassembly
        for parent_key, elements in module_groups.items():
            if parent_key is None:
                name = sanitize_name(robot.name)
            else:
                name = sanitize_name(getattr(parent_key, "name", str(parent_key)))

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
            mesh_rel_path = os.path.relpath(str(mesh_dir_path), str(module_path.parent))

            # Add includes for children modules
            children = [
                k
                for k in module_groups.keys()
                if k and getattr(k, "parent", None) == parent_key
            ]
            for child in children:
                child_name = sanitize_name(getattr(child, "name", str(child)))
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

        # 6. Write missing meshes prompt file if any parts failed
        if missing_meshes:
            self._write_missing_meshes_prompt(
                missing_meshes, mesh_dir_path, robot, out_dir
            )

    def _group_by_subassembly(self, robot: "Robot") -> Dict[Any, Dict[str, List]]:
        groups = {}
        # PathKey objects in networkx graph
        for node, data in robot.nodes(data=True):
            link = data.get("link") or data.get("data")
            parent = getattr(node, "parent", None)
            if parent is None and link is not None:
                parent = getattr(link, "parent", None)
            if parent not in groups:
                groups[parent] = {"links": [], "joints": []}
            if link is not None:
                groups[parent]["links"].append(link)

        for parent_node, child_node in robot.edges:
            edge_data = robot.get_edge_data(parent_node, child_node)
            joint = edge_data.get("joint") or edge_data.get("data")
            # Joint belongs to the module of its parent link
            parent_key = getattr(parent_node, "parent", None)
            if parent_key is None:
                parent_data = robot.nodes[parent_node]
                parent_link = parent_data.get("link") or parent_data.get("data")
                if parent_link is not None:
                    parent_key = getattr(parent_link, "parent", None)
            if parent_key not in groups:
                groups[parent_key] = {"links": [], "joints": []}
            if joint is not None:
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
            if joint:
                if is_joint(joint.name):
                    # Export as movable joint (revolute/prismatic)
                    self._joint_to_xacro(macro, joint, config, force_fixed=False)
                else:
                    # Export as fixed joint for fasten mates and other non-joint connections
                    self._joint_to_xacro(macro, joint, config, force_fixed=True)

        # Call children macros
        for child in children:
            child_name = sanitize_name(getattr(child, "name", str(child)))
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
            link = data.get("link") or data.get("data")
            if link:
                self._link_to_xacro(
                    macro, link, config, mesh_map, mesh_rel_path=mesh_rel_path
                )

        for parent, child in robot.edges:
            edge_data = robot.get_edge_data(parent, child)
            joint = edge_data.get("joint") or edge_data.get("data")
            if joint:
                if is_joint(joint.name):
                    # Export as movable joint (revolute/prismatic)
                    self._joint_to_xacro(macro, joint, config, force_fixed=False)
                else:
                    # Export as fixed joint for fasten mates and other non-joint connections
                    self._joint_to_xacro(macro, joint, config, force_fixed=True)

    def _link_to_xacro(
        self,
        root: ET._Element,
        link: Any,
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
            if hasattr(link, "to_xml"):
                # Use link.to_xml() to get complete link definition with origins
                try:
                    link_xml = link.to_xml()
                    if isinstance(link_xml, str):
                        link_xml_el = ET.fromstring(link_xml)
                    else:
                        link_xml_el = link_xml

                    # Extract visual and collision elements with origins
                    for tag in ["visual", "collision"]:
                        existing_el = link_xml_el.find(tag)
                        if existing_el is not None:
                            # Copy the element (includes origin if present)
                            new_el = ET.fromstring(
                                ET.tostring(existing_el, encoding="unicode")
                            )
                            # Update mesh filename
                            mesh_elem = new_el.find(".//mesh")
                            if mesh_elem is not None:
                                mesh_elem.set(
                                    "filename", f"{mesh_rel_path}/{mesh_map[name]}"
                                )
                            link_el.append(new_el)
                        else:
                            # Fallback: create without origin
                            el = ET.Element(tag)
                            geom = ET.SubElement(el, "geometry")
                            mesh = ET.SubElement(geom, "mesh")
                            mesh.set("filename", f"{mesh_rel_path}/{mesh_map[name]}")
                            link_el.append(el)
                except Exception:
                    # Fallback: create visual and collision without origin
                    for tag in ["visual", "collision"]:
                        el = ET.Element(tag)
                        geom = ET.SubElement(el, "geometry")
                        mesh = ET.SubElement(geom, "mesh")
                        mesh.set(
                            "filename",
                            f"{mesh_rel_path}/{mesh_map[name]}",
                        )
                        link_el.append(el)
            else:
                # Condensed link without to_xml
                for tag in ["visual", "collision"]:
                    el = ET.Element(tag)
                    geom = ET.SubElement(el, "geometry")
                    mesh = ET.SubElement(geom, "mesh")
                    mesh.set("filename", f"{mesh_rel_path}/{mesh_map[name]}")
                    link_el.append(el)

    def _joint_to_xacro(
        self,
        root: ET._Element,
        joint: Any,
        config: ConfigOverride,
        force_fixed: bool = False,
    ):
        # For non-joint_* mates, use the original name (not removing joint_ prefix)
        if force_fixed:
            name = sanitize_name(joint.name)
        else:
            name = sanitize_name(get_joint_name(joint.name))

        # Use to_xml() to get complete joint definition with origin, axis, etc.
        try:
            if hasattr(joint, "to_xml"):
                joint_xml = joint.to_xml()
                # Parse the XML to get the element
                if isinstance(joint_xml, str):
                    joint_el = ET.fromstring(joint_xml)
                else:
                    joint_el = joint_xml
            else:
                # JointRecord or similar
                joint_el = ET.Element("joint")
                joint_el.set(
                    "type",
                    "revolute"
                    if "REVOLUTE" in str(getattr(joint, "joint_type", "")).upper()
                    else "fixed",
                )
        except Exception:
            # Fallback to manual creation if to_xml() fails
            joint_el = ET.Element("joint")

        # Override name with prefix-aware version
        joint_el.set("name", f"${{prefix}}{name}")

        # Force fixed type if requested
        if force_fixed:
            # But check if it has axis and limit - might be a revolute joint misnamed
            has_axis = joint_el.find("axis") is not None
            has_limit = joint_el.find("limit") is not None
            if has_axis and has_limit:
                # This looks like a revolute/prismatic joint, not fixed
                # Check axis to determine type
                axis_elem = joint_el.find("axis")
                if axis_elem is not None:
                    axis_elem.get("xyz", "0 0 1")
                    # If it has axis and limit, it's likely revolute
                    joint_el.set("type", "revolute")
            else:
                joint_el.set("type", "fixed")
        else:
            # Ensure type is set (to_xml() should have it, but verify)
            jtype = str(getattr(joint, "joint_type", "fixed")).upper()
            joint_el.set(
                "type",
                "revolute"
                if "REVOLUTE" in jtype
                else "prismatic"
                if "PRISMATIC" in jtype
                else "fixed",
            )

        # Update parent and child links with prefix
        parent_elem = joint_el.find("parent")
        if parent_elem is not None:
            parent_elem.set("link", f"${{prefix}}{sanitize_name(joint.parent)}")
        else:
            ET.SubElement(
                joint_el, "parent", link=f"${{prefix}}{sanitize_name(joint.parent)}"
            )

        child_elem = joint_el.find("child")
        if child_elem is not None:
            child_elem.set("link", f"${{prefix}}{sanitize_name(joint.child)}")
        else:
            ET.SubElement(
                joint_el, "child", link=f"${{prefix}}{sanitize_name(joint.child)}"
            )

        # Update joint limits if it's a movable joint
        jtype = joint_el.get("type", "fixed")
        if jtype in ["revolute", "prismatic"]:
            # First, try to get limits from joint.to_xml() (from Onshape)
            limit_elem = joint_el.find("limit")
            onshape_limit = None
            if limit_elem is not None:
                # Extract existing limits from Onshape
                onshape_limit = {
                    "lower": float(limit_elem.get("lower", "-3.14")),
                    "upper": float(limit_elem.get("upper", "3.14")),
                    "effort": float(limit_elem.get("effort", "100")),
                    "velocity": float(limit_elem.get("velocity", "1.0")),
                }

            # Default limits (used only if Onshape didn't provide limits)
            default_limit = {
                "lower": -3.14,
                "upper": 3.14,
                "effort": 100,
                "velocity": 1.0,
            }

            # Use Onshape limits as base, or default if not available
            base_limit = onshape_limit if onshape_limit else default_limit

            # Apply config overrides (if any)
            val = config.get_joint_limit(name, base_limit)

            # Update or create limit element
            if limit_elem is not None:
                limit_elem.set("lower", str(val["lower"]))
                limit_elem.set("upper", str(val["upper"]))
                limit_elem.set("effort", str(val["effort"]))
                limit_elem.set("velocity", str(val["velocity"]))
            else:
                ET.SubElement(
                    joint_el,
                    "limit",
                    lower=str(val["lower"]),
                    upper=str(val["upper"]),
                    effort=str(val["effort"]),
                    velocity=str(val["velocity"]),
                )

        # Add axis element if missing (for revolute/prismatic joints)
        if jtype in ["revolute", "prismatic"]:
            axis_elem = joint_el.find("axis")
            if axis_elem is None:
                # Default to Z-axis rotation
                ET.SubElement(joint_el, "axis", xyz="0 0 1")

        # Append to root
        root.append(joint_el)

    def _export_meshes(
        self, robot: "Robot", mesh_dir: Path
    ) -> tuple[dict[str, str], dict[str, list[dict[str, str]]]]:
        """Export meshes for robot links.

        Returns:
            Tuple of (mesh_map, missing_meshes) where:
            - mesh_map: Dict mapping link_name -> stl filename
            - missing_meshes: Dict mapping link_name -> list of missing part info
        """
        link_groups: Dict[str, list[Any]] = {}
        for _, data in robot.nodes(data=True):
            link = data.get("link") or data.get("data")
            if not link or not hasattr(link, "keys"):
                continue
            link_groups[sanitize_name(link.name)] = link.keys
        logger.info(f"[DEBUG] link_groups count: {len(link_groups)}")
        # StepMeshExporter needs robot.client and robot.cad which should be set in pipeline
        client = getattr(robot, "client", None)
        cad = getattr(robot, "cad", None)
        asset_path = getattr(robot, "asset_path", None)
        logger.info(
            f"[DEBUG] client={client is not None}, cad={cad is not None}, asset_path={asset_path}"
        )
        if (client and cad) or (cad and asset_path):
            exporter = StepMeshExporter(client, cad, asset_path=asset_path)
            return exporter.export_link_meshes(link_groups, mesh_dir)

        logger.warning(
            "Robot missing client/CAD or CAD/asset_path, skipping STEP mesh export"
        )
        return {}, {}

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

    def _write_missing_meshes_prompt(
        self,
        missing_meshes: dict[str, list[dict[str, str]]],
        mesh_dir: Path,
        robot: "Robot",
        out_dir: Path,
    ):
        """Write a detailed prompt file for user to manually export missing meshes."""
        prompt_path = out_dir / "MISSING_MESHES.md"

        # Count total missing parts
        total_missing = sum(len(parts) for parts in missing_meshes.values())

        lines = [
            "# Missing Mesh Files",
            "",
            f"**{total_missing} parts** could not be automatically exported to STL meshes.",
            "",
            "The robot model has been generated, but some links are missing their mesh files.",
            "You need to manually export these parts from Onshape and place the STL files",
            f"in the `{mesh_dir.relative_to(out_dir)}` directory.",
            "",
            "---",
            "",
            "## Instructions",
            "",
            "For each link listed below:",
            "",
            "1. Open the Onshape assembly in your browser",
            "2. Select all the parts listed under that link",
            "3. Right-click and choose **Export...**",
            "4. Select **STL** format",
            "5. Save the file with the exact filename shown (e.g., `link_name.stl`)",
            f"6. Place the exported STL file in: `{mesh_dir}`",
            "",
            "---",
            "",
            "## Missing Parts by Link",
            "",
        ]

        for link_name, parts in sorted(missing_meshes.items()):
            lines.append(f"### Link: `{link_name}`")
            lines.append("")
            lines.append(f"**Expected STL filename:** `{link_name}.stl`")
            lines.append("")
            lines.append("**Parts to include:**")
            lines.append("")
            lines.append("| Part Name | Export Name | Reason |")
            lines.append("|-----------|-------------|--------|")

            for part_info in parts:
                part_name = part_info.get("part_name", "unknown")
                export_name = part_info.get("export_name", "unknown")
                reason = part_info.get("reason", "unknown")
                lines.append(f"| {part_name} | {export_name} | {reason} |")

            lines.append("")

        lines.extend(
            [
                "---",
                "",
                "## After Manual Export",
                "",
                "Once you've exported all missing STL files:",
                "",
                "1. Verify all files exist in the meshes directory",
                "2. Re-run your URDF/Xacro validator to check the model loads correctly",
                "3. The robot model should now render properly in RViz or Gazebo",
                "",
                "---",
                "",
                "*This file was auto-generated by onshape2xacro.*",
            ]
        )

        with open(prompt_path, "w") as f:
            f.write("\n".join(lines))

        logger.warning(
            f"Missing meshes for {len(missing_meshes)} links ({total_missing} parts). "
            f"See {prompt_path} for manual export instructions."
        )
