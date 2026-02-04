import os
import os
import shutil
from pathlib import Path
from typing import Any, Dict, List, Optional, TYPE_CHECKING
import lxml.etree as ET
import yaml
from onshape2xacro.serializers import XacroSerializer, is_joint, get_joint_name
from onshape2xacro.naming import sanitize_name
from onshape2xacro.config import ConfigOverride
from onshape2xacro.condensed_robot import JointRecord
from onshape2xacro.module_boundary import (
    detect_module_boundaries,
    calculate_interface_transforms,
)

if TYPE_CHECKING:
    from onshape_robotics_toolkit.robot import Robot
    from onshape2xacro.config.export_config import CollisionOptions


class ModularXacroSerializer(XacroSerializer):
    def save(
        self,
        robot: Any,
        file_path: str,
        download_assets: bool = True,
        mesh_dir: Optional[str] = None,
        bom_path: Optional[Path] = None,
        visual_mesh_format: str = "obj",
        collision_option: Optional[Any] = None,
        **options: Any,
    ):
        out_dir = Path(file_path)
        urdf_dir = out_dir / "urdf"
        urdf_dir.mkdir(parents=True, exist_ok=True)
        config = options.get("config") or ConfigOverride()

        boundaries = getattr(robot, "module_boundaries", None)
        if boundaries is None and getattr(robot, "cad", None):
            boundaries = detect_module_boundaries(robot.cad, robot.kinematic_graph)

        interface_transforms = {}
        if boundaries and getattr(robot, "cad", None):
            interface_transforms = calculate_interface_transforms(
                robot.cad, robot, boundaries
            )

        module_groups = self._group_by_subassembly(robot)
        main_name = sanitize_name(robot.name)

        instance_to_def = {}
        instance_to_def_name = {}
        def_to_representative = {}

        for inst_key in module_groups.keys():
            if inst_key is None:
                instance_to_def[inst_key] = None
                instance_to_def_name[inst_key] = main_name
                def_to_representative[None] = None
                continue

            subasm = robot.cad.subassemblies.get(inst_key)
            if subasm:
                def_id = (
                    subasm.documentId,
                    subasm.elementId,
                    getattr(subasm, "documentMicroversion", "")
                    or getattr(subasm, "documentVersion", "")
                    or getattr(subasm, "workspaceId", ""),
                )
                def_name_source = (
                    getattr(subasm, "name", None)
                    or getattr(inst_key, "name", None)
                    or str(inst_key)
                )
                def_name = sanitize_name(def_name_source)
            else:
                def_id = str(inst_key)
                def_name = sanitize_name(str(inst_key))

            instance_to_def[inst_key] = def_id
            instance_to_def_name[inst_key] = def_name
            if def_id not in def_to_representative:
                def_to_representative[def_id] = inst_key

        temp_mesh_dir = out_dir / "all_meshes"
        temp_mesh_dir.mkdir(parents=True, exist_ok=True)

        module_mesh_dirs: Dict[Any, Path] = {}
        for inst_key in module_groups.keys():
            if inst_key is None:
                module_mesh_dirs[None] = urdf_dir / "meshes"
                continue
            def_name = instance_to_def_name[inst_key]
            module_mesh_dirs[inst_key] = urdf_dir / def_name / "meshes"

        mesh_map: Dict[Any, Any] = {}
        missing_meshes: Dict[str, Any] = {}
        computed_inertials: Dict[str, Any] = {}

        if download_assets:
            mesh_map, missing_meshes, report = self._export_meshes(
                robot,
                temp_mesh_dir,
                visual_mesh_format=visual_mesh_format,
                bom_path=bom_path,
                collision_option=collision_option,
                module_mesh_dirs=module_mesh_dirs,
            )
            if report and report.link_properties:
                for name, props in report.link_properties.items():
                    computed_inertials[name] = props.to_yaml_dict()

        for def_id, rep_inst_key in def_to_representative.items():
            elements = module_groups[rep_inst_key]
            name = instance_to_def_name[rep_inst_key]

            if rep_inst_key is None:
                module_dir = urdf_dir
                module_path = urdf_dir / f"{main_name}.xacro"
            else:
                module_dir = urdf_dir / name
                module_dir.mkdir(parents=True, exist_ok=True)
                module_path = module_dir / f"{name}.xacro"

            module_mesh_dir = module_dir / "meshes"
            module_mesh_dir.mkdir(parents=True, exist_ok=True)
            mesh_rel_path = os.path.relpath(module_mesh_dir, urdf_dir)

            module_config_dir = module_dir / "config"
            module_config_dir.mkdir(parents=True, exist_ok=True)

            self._generate_module_configs(
                elements, module_config_dir, config, computed_inertials, name
            )

            root = self._create_xacro_root(name)

            children_instances = [
                k
                for k in module_groups.keys()
                if k and getattr(k, "parent", None) == rep_inst_key
            ]
            included_defs = set()
            for child_key in children_instances:
                child_def_id = instance_to_def[child_key]
                if child_def_id in included_defs:
                    continue
                included_defs.add(child_def_id)

                child_def_name = instance_to_def_name[child_key]
                child_module_path = (
                    urdf_dir / child_def_name / f"{child_def_name}.xacro"
                )
                rel_inc_path = os.path.relpath(child_module_path, module_path.parent)

                inc = ET.SubElement(root, "{http://www.ros.org/wiki/xacro}include")
                inc.set("filename", rel_inc_path)

            module_elements = {"links": elements["links"], "joints": []}

            for joint in elements["joints"]:
                child_link_data = robot.nodes[joint.child]
                child_link_rec = child_link_data.get("link") or child_link_data.get(
                    "data"
                )
                child_module_key = getattr(child_link_rec, "parent", None)

                if (
                    child_module_key in interface_transforms
                    and interface_transforms[child_module_key].parent_link
                    == joint.parent
                ):
                    info = interface_transforms[child_module_key]
                    inst_name = sanitize_name(
                        getattr(child_module_key, "name", str(child_module_key))
                    )
                    interface_joint = JointRecord(
                        name=joint.name,
                        joint_type=joint.joint_type,
                        parent=info.parent_link,
                        child=f"{inst_name}_{info.child_root_link}",
                        origin=info.origin,
                        axis=info.axis,
                        limits=joint.limits,
                        mate=joint.mate,
                    )
                    module_elements["joints"].append(interface_joint)
                else:
                    module_elements["joints"].append(joint)

            rel_config_path = os.path.relpath(module_config_dir, urdf_dir)

            module_mesh_map: Dict[str, Any] = {}
            for link in elements["links"]:
                link_name = sanitize_name(link.name)
                mesh_key = (rep_inst_key, link_name)
                if mesh_key in mesh_map:
                    module_mesh_map[link_name] = mesh_map[mesh_key]
                elif link_name in mesh_map:
                    module_mesh_map[link_name] = mesh_map[link_name]

            self._add_module_macro(
                root,
                name,
                module_elements,
                config,
                module_mesh_map,
                children_instances,
                instance_to_def_name=instance_to_def_name,
                mesh_rel_path=mesh_rel_path,
                module_name=name,
                config_rel_path=rel_config_path,
            )

            with open(module_path, "w") as f:
                f.write(ET.tostring(root, pretty_print=True, encoding="unicode"))

        entry_point_root = self._create_xacro_root(robot.name)
        inc = ET.SubElement(entry_point_root, "{http://www.ros.org/wiki/xacro}include")
        inc.set("filename", f"{main_name}.xacro")
        ET.SubElement(
            entry_point_root,
            f"{{http://www.ros.org/wiki/xacro}}{main_name}",
            prefix="",
        )

        entry_point_path = urdf_dir / f"{main_name}.urdf.xacro"
        with open(entry_point_path, "w") as f:
            f.write(
                ET.tostring(entry_point_root, pretty_print=True, encoding="unicode")
            )

        if missing_meshes:
            mesh_dir_path = out_dir / "meshes"
            self._write_missing_meshes_prompt(
                missing_meshes, mesh_dir_path, robot, out_dir
            )

        if temp_mesh_dir.exists():
            shutil.rmtree(temp_mesh_dir)

    def _generate_module_configs(
        self,
        elements: dict[str, list[Any]],
        config_dir: Path,
        config: ConfigOverride,
        computed_inertials: dict[str, Any],
        module_name: Optional[str] = None,
    ):
        joint_limits = {}
        inertials = {}

        prop_prefix = f"{module_name}_" if module_name else ""

        # ...

        for joint in elements.get("joints", []):
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

        for link in elements.get("links", []):
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

                    if not getattr(link, "part_ids", []):
                        default_inertial["mass"] = 1e-9
                        default_inertial["inertia"] = {
                            "ixx": 1e-9,
                            "iyy": 1e-9,
                            "izz": 1e-9,
                            "ixy": 0,
                            "ixz": 0,
                            "iyz": 0,
                        }

                    inertials[name] = config.get_inertial(name, default_inertial)

        if joint_limits or inertials:
            config_dir.mkdir(parents=True, exist_ok=True)

        if joint_limits:
            with open(config_dir / f"{prop_prefix}joint_limits.yaml", "w") as f:
                yaml.dump({"joint_limits": joint_limits}, f)

        if inertials:
            with open(config_dir / f"{prop_prefix}inertials.yaml", "w") as f:
                yaml.dump({"inertials": inertials}, f)

    def _add_config_loading(
        self, root: Any, name: str, config_rel_path: str = "config"
    ):
        prop_prefix = f"{name}_" if name else ""
        ET.SubElement(
            root,
            "{http://www.ros.org/wiki/xacro}property",
            name=f"{prop_prefix}joint_limits_file",
            value=f"${{xacro.load_yaml('{config_rel_path}/{prop_prefix}joint_limits.yaml')}}",
        )
        ET.SubElement(
            root,
            "{http://www.ros.org/wiki/xacro}property",
            name=f"{prop_prefix}joint_limits",
            value=f"${{{prop_prefix}joint_limits_file['joint_limits']}}",
        )
        ET.SubElement(
            root,
            "{http://www.ros.org/wiki/xacro}property",
            name=f"{prop_prefix}inertials_file",
            value=f"${{xacro.load_yaml('{config_rel_path}/{prop_prefix}inertials.yaml')}}",
        )
        ET.SubElement(
            root,
            "{http://www.ros.org/wiki/xacro}property",
            name=f"{prop_prefix}inertials",
            value=f"${{{prop_prefix}inertials_file['inertials']}}",
        )

    def _add_module_macro(
        self,
        root: Any,
        name: str,
        elements: dict[str, list[Any]],
        config: ConfigOverride,
        mesh_map: dict[str, str | dict[str, str | list[str]]],
        children: list[Any],
        mesh_rel_path: str = "meshes",
        module_name: Optional[str] = None,
        config_rel_path: str = "config",
        instance_to_def_name: Optional[dict[Any, str]] = None,
    ):
        macro = ET.SubElement(root, "{http://www.ros.org/wiki/xacro}macro")
        macro.set("name", sanitize_name(name))
        macro.set("params", "prefix:=''")

        self._add_config_loading(macro, name, config_rel_path=config_rel_path)

        for link in elements["links"]:
            if link:
                self._link_to_xacro(
                    macro,
                    link,
                    config,
                    mesh_map,
                    mesh_rel_path=mesh_rel_path,
                    module_name=name,
                )

        for joint in elements["joints"]:
            if joint:
                if is_joint(joint.name):
                    self._joint_to_xacro(
                        macro, joint, config, force_fixed=False, module_name=name
                    )
                else:
                    self._joint_to_xacro(
                        macro, joint, config, force_fixed=True, module_name=name
                    )

        if instance_to_def_name:
            for child_inst in children:
                def_name = instance_to_def_name[child_inst]
                inst_name = sanitize_name(getattr(child_inst, "name", str(child_inst)))
                ET.SubElement(
                    macro,
                    "{http://www.ros.org/wiki/xacro}" + def_name,
                    prefix=f"${{prefix}}{inst_name}_",
                )
        else:
            for child in children:
                child_name = sanitize_name(getattr(child, "name", str(child)))
                ET.SubElement(
                    macro,
                    "{http://www.ros.org/wiki/xacro}" + child_name,
                    prefix="${prefix}",
                )
