import os
from typing import Any, TYPE_CHECKING, Dict, List, Optional
from pathlib import Path
import lxml.etree as ET
from loguru import logger
import yaml

from onshape_robotics_toolkit.formats.base import RobotSerializer
from onshape2xacro.config import ConfigOverride
from onshape2xacro.naming import sanitize_name
from onshape2xacro.mesh_exporters.step import StepMeshExporter
from onshape2xacro.condensed_robot import JointRecord

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
        bom_path: Optional[Path] = None,
        visual_mesh_format: str = "obj",
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

        # 1. Export meshes (Stage 6) & Compute Inertials
        mesh_map = {}
        missing_meshes = {}
        computed_inertials = {}

        # Prepare link records
        link_records = {}
        for _, data in robot.nodes(data=True):
            link = data.get("link") or data.get("data")
            if link:
                link_records[sanitize_name(link.name)] = link

        if download_assets:
            # We need to manually invoke exporter logic here to support BOM/inertials
            client = getattr(robot, "client", None)
            cad = getattr(robot, "cad", None)
            asset_path = getattr(robot, "asset_path", None)

            if (client and cad) or (cad and asset_path):
                exporter = StepMeshExporter(client, cad, asset_path=asset_path)
                mesh_map, missing_meshes, report = exporter.export_link_meshes(
                    link_records,
                    mesh_dir_path,
                    bom_path=bom_path,
                    visual_mesh_format=visual_mesh_format,
                    collision_mesh_method=options.get("collision_mesh_method", "fast"),
                )

                if report and report.link_properties:
                    for name, props in report.link_properties.items():
                        computed_inertials[name] = props.to_yaml_dict()

                if report:
                    report.print_summary()

                    if report.link_parts:
                        debug_table_path = out_dir / "inertia_debug.md"
                        report.save_debug_table(debug_table_path)

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

        # Load configuration from YAML files (Global properties)
        ET.SubElement(
            entry_point_root,
            "{http://www.ros.org/wiki/xacro}property",
            name="joint_limits_file",
            value="${load_yaml('../config/joint_limits.yaml')}",
        )
        ET.SubElement(
            entry_point_root,
            "{http://www.ros.org/wiki/xacro}property",
            name="joint_limits",
            value="${joint_limits_file['joint_limits']}",
        )
        ET.SubElement(
            entry_point_root,
            "{http://www.ros.org/wiki/xacro}property",
            name="inertials_file",
            value="${load_yaml('../config/inertials.yaml')}",
        )
        ET.SubElement(
            entry_point_root,
            "{http://www.ros.org/wiki/xacro}property",
            name="inertials",
            value="${inertials_file['inertials']}",
        )

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
        self._generate_default_configs(robot, config_dir, config, computed_inertials)

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
        mesh_map: Dict[str, str | Dict[str, str | List[str]]],
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
        mesh_map: Dict[str, str | Dict[str, str | List[str]]],
        mesh_rel_path: str = "meshes",
    ):
        macro = ET.SubElement(root, "{http://www.ros.org/wiki/xacro}macro")
        macro.set("name", sanitize_name(robot.name))
        macro.set("params", "prefix:=''")

        # Construct configuration dictionaries for inline properties (support for preview/memory-only export)
        joint_limits = {}
        inertials = {}

        for parent, child in robot.edges:
            edge_data = robot.get_edge_data(parent, child)
            joint = edge_data.get("data")
            if joint and is_joint(joint.name):
                name = sanitize_name(get_joint_name(joint.name))

                default_limit = {
                    "lower": -3.14,
                    "upper": 3.14,
                    "effort": 100.0,
                    "velocity": 1.0,
                    "damping": 0.0,
                    "friction": 0.0,
                }

                stored_limits = getattr(joint, "limits", None)
                if stored_limits:
                    if isinstance(stored_limits, dict):
                        if "min" in stored_limits and "max" in stored_limits:
                            default_limit["lower"] = stored_limits["min"]
                            default_limit["upper"] = stored_limits["max"]
                        if "effort" in stored_limits:
                            default_limit["effort"] = stored_limits["effort"]
                        if "velocity" in stored_limits:
                            default_limit["velocity"] = stored_limits["velocity"]
                    else:
                        if hasattr(stored_limits, "min") and hasattr(
                            stored_limits, "max"
                        ):
                            default_limit["lower"] = stored_limits.min
                            default_limit["upper"] = stored_limits.max
                        if hasattr(stored_limits, "effort"):
                            default_limit["effort"] = stored_limits.effort
                        if hasattr(stored_limits, "velocity"):
                            default_limit["velocity"] = stored_limits.velocity

                joint_limits[name] = config.get_joint_limit(name, default_limit)

        for node, data in robot.nodes(data=True):
            link = data.get("data")
            if link:
                name = sanitize_name(link.name)

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
                inertials[name] = config.get_inertial(name, default_inertial)

        # Inject properties inline using Python dictionary syntax
        ET.SubElement(
            macro,
            "{http://www.ros.org/wiki/xacro}property",
            name="joint_limits",
            value=f"${{{str(joint_limits)}}}",
        )
        ET.SubElement(
            macro,
            "{http://www.ros.org/wiki/xacro}property",
            name="inertials",
            value=f"${{{str(inertials)}}}",
        )

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
        mesh_map: Dict[str, str | Dict[str, str | List[str]]],
        mesh_rel_path: str = "meshes",
    ):
        name = sanitize_name(link.name)
        link_el = ET.SubElement(root, "link")
        link_el.set("name", f"${{prefix}}{name}")

        # Use runtime YAML configuration for inertials
        inertial = ET.SubElement(link_el, "inertial")
        ET.SubElement(inertial, "mass", value=f"${{inertials['{name}']['mass']}}")

        ET.SubElement(
            inertial,
            "origin",
            xyz=f"${{inertials['{name}']['origin']['xyz']}}",
            rpy=f"${{inertials['{name}']['origin']['rpy']}}",
        )

        ET.SubElement(
            inertial,
            "inertia",
            ixx=f"${{inertials['{name}']['inertia']['ixx']}}",
            iyy=f"${{inertials['{name}']['inertia']['iyy']}}",
            izz=f"${{inertials['{name}']['inertia']['izz']}}",
            ixy=f"${{inertials['{name}']['inertia']['ixy']}}",
            ixz=f"${{inertials['{name}']['inertia']['ixz']}}",
            iyz=f"${{inertials['{name}']['inertia']['iyz']}}",
        )

        if name in mesh_map:
            # Check for origin in link (baked mesh means identity origin)
            # LinkRecord now contains frame_transform used for baking

            entry = mesh_map[name]
            # Handle both old (str) and new (dict) formats
            if isinstance(entry, dict):
                visual_file = entry.get("visual", f"{name}.stl")
                collision_file = entry.get("collision", f"{name}.stl")
            else:
                visual_file = entry
                collision_file = entry

            for tag in ["visual", "collision"]:
                files_to_add = []
                if tag == "visual":
                    files_to_add = [visual_file]
                else:
                    if isinstance(collision_file, list):
                        files_to_add = collision_file
                    else:
                        files_to_add = [collision_file]

                for filename in files_to_add:
                    el = ET.SubElement(link_el, tag)
                    # Identity origin for baked meshes
                    ET.SubElement(el, "origin", xyz="0 0 0", rpy="0 0 0")
                    geom = ET.SubElement(el, "geometry")
                    mesh = ET.SubElement(geom, "mesh")

                    mesh.set("filename", f"{mesh_rel_path}/{filename}")
                    mesh.set("scale", "0.001 0.001 0.001")

    def _joint_to_xacro(
        self,
        root: ET._Element,
        joint: JointRecord,
        config: ConfigOverride,
        force_fixed: bool = False,
    ):
        # For non-joint_* mates, use the original name (not removing joint_ prefix)
        if force_fixed:
            name = sanitize_name(joint.name)
        else:
            name = sanitize_name(get_joint_name(joint.name))

        # JointRecord with stored origin from transformation pipeline
        joint_el = ET.Element("joint")

        # Set name and type
        joint_el.set("name", f"${{prefix}}{name}")

        jtype = "fixed"
        if not force_fixed:
            jtype_str = str(getattr(joint, "joint_type", "fixed")).upper()
            if "REVOLUTE" in jtype_str:
                jtype = "revolute"
            elif "PRISMATIC" in jtype_str:
                jtype = "prismatic"
            elif "CONTINUOUS" in jtype_str:
                jtype = "continuous"
        joint_el.set("type", jtype)

        # Set Origin
        if hasattr(joint, "origin") and joint.origin is not None:
            origin = joint.origin
            ET.SubElement(
                joint_el,
                "origin",
                xyz=f"{origin.xyz[0]} {origin.xyz[1]} {origin.xyz[2]}",
                rpy=f"{origin.rpy[0]} {origin.rpy[1]} {origin.rpy[2]}",
            )
        else:
            ET.SubElement(joint_el, "origin", xyz="0 0 0", rpy="0 0 0")

        # Update parent and child links with prefix
        ET.SubElement(
            joint_el, "parent", link=f"${{prefix}}{sanitize_name(joint.parent)}"
        )
        ET.SubElement(
            joint_el, "child", link=f"${{prefix}}{sanitize_name(joint.child)}"
        )

        # Update joint limits and axis if it's a movable joint
        if jtype in ["revolute", "prismatic", "continuous"]:
            axis_xyz = f"{joint.axis[0]} {joint.axis[1]} {joint.axis[2]}"
            ET.SubElement(joint_el, "axis", xyz=axis_xyz)

        if jtype in ["revolute", "prismatic"]:
            # Limits from runtime YAML configuration
            ET.SubElement(
                joint_el,
                "limit",
                lower=f"${{joint_limits['{name}']['lower']}}",
                upper=f"${{joint_limits['{name}']['upper']}}",
                effort=f"${{joint_limits['{name}']['effort']}}",
                velocity=f"${{joint_limits['{name}']['velocity']}}",
            )
            # Add dynamics from runtime YAML configuration
            ET.SubElement(
                joint_el,
                "dynamics",
                damping=f"${{joint_limits['{name}']['damping']}}",
                friction=f"${{joint_limits['{name}']['friction']}}",
            )

        # Append to root
        root.append(joint_el)

    def _export_meshes(
        self, robot: "Robot", mesh_dir: Path
    ) -> tuple[
        dict[str, str | dict[str, str | list[str]]], dict[str, list[dict[str, str]]]
    ]:
        """Export meshes for robot links."""
        link_records: Dict[str, Any] = {}
        for _, data in robot.nodes(data=True):
            link = data.get("link") or data.get("data")
            if not link:
                continue
            link_records[sanitize_name(link.name)] = link

        client = getattr(robot, "client", None)
        cad = getattr(robot, "cad", None)
        asset_path = getattr(robot, "asset_path", None)

        if (client and cad) or (cad and asset_path):
            exporter = StepMeshExporter(client, cad, asset_path=asset_path)
            # Pass link records instead of just keys
            mesh_map, missing_meshes, _ = exporter.export_link_meshes(
                link_records, mesh_dir
            )
            return mesh_map, missing_meshes

        return {}, {}

    def _generate_default_configs(
        self,
        robot: "Robot",
        config_dir: Path,
        config: Optional[ConfigOverride] = None,
        computed_inertials: Optional[Dict[str, Any]] = None,
    ):
        if config is None:
            config = ConfigOverride()
        if computed_inertials is None:
            computed_inertials = {}

        joint_limits = {}
        inertials = {}
        for parent, child in robot.edges:
            edge_data = robot.get_edge_data(parent, child)
            joint = edge_data.get("data")
            if joint and is_joint(joint.name):
                name = sanitize_name(get_joint_name(joint.name))

                default_limit = {
                    "lower": -3.14,
                    "upper": 3.14,
                    "effort": 100.0,
                    "velocity": 1.0,
                    "damping": 0.0,
                    "friction": 0.0,
                }

                # Try to get from stored limits if available
                stored_limits = getattr(joint, "limits", None)
                if stored_limits:
                    if isinstance(stored_limits, dict):
                        if "min" in stored_limits and "max" in stored_limits:
                            default_limit["lower"] = stored_limits["min"]
                            default_limit["upper"] = stored_limits["max"]
                        if "effort" in stored_limits:
                            default_limit["effort"] = stored_limits["effort"]
                        if "velocity" in stored_limits:
                            default_limit["velocity"] = stored_limits["velocity"]
                    else:
                        if hasattr(stored_limits, "min") and hasattr(
                            stored_limits, "max"
                        ):
                            default_limit["lower"] = stored_limits.min
                            default_limit["upper"] = stored_limits.max
                        if hasattr(stored_limits, "effort"):
                            default_limit["effort"] = stored_limits.effort
                        if hasattr(stored_limits, "velocity"):
                            default_limit["velocity"] = stored_limits.velocity

                joint_limits[name] = config.get_joint_limit(name, default_limit)

        for node, data in robot.nodes(data=True):
            link = data.get("data")
            if link:
                name = sanitize_name(link.name)

                if name in computed_inertials:
                    inertials[name] = computed_inertials[name]
                else:
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
                    inertials[name] = config.get_inertial(name, default_inertial)

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
            f"6. Place the exported STL file in: `{mesh_dir}/visual`",
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
