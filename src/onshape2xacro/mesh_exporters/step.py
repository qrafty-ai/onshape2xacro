import re
import time
import zipfile
from pathlib import Path
from typing import Any, Dict, Iterable, List, Optional, Tuple, cast, TYPE_CHECKING

if TYPE_CHECKING:
    from onshape2xacro.inertia import InertiaReport

import numpy as np
from onshape_robotics_toolkit.connect import Client, HTTP
from OCP.STEPCAFControl import STEPCAFControl_Reader
from OCP.TDocStd import TDocStd_Document
from OCP.TCollection import TCollection_ExtendedString
from OCP.XCAFDoc import XCAFDoc_DocumentTool
from OCP.TDataStd import TDataStd_Name, TDataStd_NamedData
from OCP.TDF import TDF_Label, TDF_LabelSequence
from OCP.IFSelect import IFSelect_RetDone
from OCP.TopLoc import TopLoc_Location
from OCP.BRep import BRep_Builder
from OCP.BRepBuilderAPI import BRepBuilderAPI_Transform
from OCP.TopoDS import TopoDS_Compound
from OCP.BRepMesh import BRepMesh_IncrementalMesh
from OCP.StlAPI import StlAPI_Writer
from OCP.gp import gp_Trsf
from OCP.STEPControl import STEPControl_Writer, STEPControl_AsIs
from loguru import logger
import trimesh
import pymeshlab


EXPORT_ID_REGEX = re.compile(
    r"(?:export:|exportId:|partId=|id=|OCCURRENCE_ID: ?\[?)([A-Za-z0-9+/_-]+)"
)
INSTANCE_SUFFIX_REGEX = re.compile(r"^(?P<base>.+?)\s*<(?P<index>\d+)>$")


def _get_shape_tool(doc: Any):
    shape_tool = getattr(XCAFDoc_DocumentTool, "ShapeTool_s", None)
    if callable(shape_tool):
        return shape_tool(doc.Main())
    return XCAFDoc_DocumentTool.ShapeTool(doc.Main())


def _label_name(label: Any) -> str:
    def _name_id():
        get_id = getattr(TDataStd_Name, "GetID_s", None)
        if callable(get_id):
            return get_id()
        get_id = getattr(TDataStd_Name, "GetID", None)
        if callable(get_id):
            return get_id()
        get_id = getattr(TDataStd_Name, "ID", None)
        if callable(get_id):
            return get_id()
        return None

    name_id = _name_id()
    if name_id is None:
        return ""

    name_attr = TDataStd_Name()
    if label.FindAttribute(name_id, name_attr):
        return name_attr.Get().ToExtString()
    return ""


def _get_named_data(label: Any) -> TDataStd_NamedData | None:
    get_id = getattr(TDataStd_NamedData, "GetID_s", None)
    if callable(get_id):
        attr_id = get_id()
    else:
        attr_id = TDataStd_NamedData.GetID()
    named = TDataStd_NamedData()
    if label.FindAttribute(attr_id, named):
        return named
    return None


def _named_data_value(named: TDataStd_NamedData, key: str) -> str | None:
    get_val = getattr(named, "GetString", None)
    if callable(get_val):
        value = get_val(TCollection_ExtendedString(key))
        if not value.IsEmpty():
            return value.ToExtString()
    return None


def _get_occurrence_id(label: Any) -> str | None:
    named = _get_named_data(label)
    if named is not None:
        for key in ("occurrenceId", "onshape:occurrenceId", "OCCURRENCE_ID"):
            value = _named_data_value(named, key)
            if value:
                return value

    name = _label_name(label)
    return _parse_export_id(name)


def _parse_export_id(label_name: str) -> str:
    match = EXPORT_ID_REGEX.search(label_name or "")
    if match:
        return match.group(1)

    # Fallback: check if the name itself looks like an ID (long base64-like string)
    if re.match(r"^[A-Za-z0-9+/]{10,}$", label_name):
        return label_name

    return ""


def _is_step_payload(content: bytes) -> bool:
    return content.lstrip().startswith(b"ISO-10303-21")


def _read_file_header(path: Path, max_bytes: int = 512) -> bytes:
    with path.open("rb") as handle:
        return handle.read(max_bytes)


def _matrix_to_trsf(matrix: np.ndarray) -> gp_Trsf:
    from scipy.spatial.transform import Rotation

    rot_3x3 = matrix[:3, :3]
    r = Rotation.from_matrix(rot_3x3)
    rot_normalized = r.as_matrix()

    trsf = gp_Trsf()
    trsf.SetValues(
        rot_normalized[0][0],
        rot_normalized[0][1],
        rot_normalized[0][2],
        matrix[0][3],
        rot_normalized[1][0],
        rot_normalized[1][1],
        rot_normalized[1][2],
        matrix[1][3],
        rot_normalized[2][0],
        rot_normalized[2][1],
        rot_normalized[2][2],
        matrix[2][3],
    )
    return trsf


def _get_label_location(shape_tool: Any, label: TDF_Label) -> TopLoc_Location:
    get_location = getattr(shape_tool, "GetLocation_s", None)
    if callable(get_location):
        try:
            return get_location(label)
        except Exception:
            pass
    get_location = getattr(shape_tool, "GetLocation", None)
    if callable(get_location):
        try:
            return get_location(label)
        except Exception:
            pass
    return TopLoc_Location()


def _combine_locations(
    parent_loc: TopLoc_Location, local_loc: TopLoc_Location
) -> TopLoc_Location:
    if hasattr(parent_loc, "Multiplied"):
        try:
            return parent_loc.Multiplied(local_loc)
        except Exception:
            pass
    if hasattr(parent_loc, "Multiply"):
        try:
            parent_loc.Multiply(local_loc)
            return parent_loc
        except Exception:
            pass
    return local_loc


def _iter_components(shape_tool: Any, label: TDF_Label) -> Iterable[TDF_Label]:
    get_components = getattr(shape_tool, "GetComponents_s", None)
    if not callable(get_components):
        get_components = getattr(shape_tool, "GetComponents", None)
    if callable(get_components):
        comps = TDF_LabelSequence()
        get_components(label, comps)
        for i in range(comps.Length()):
            yield comps.Value(i + 1)


def _collect_shapes(
    shape_tool: Any,
    label: TDF_Label,
    parent_loc: TopLoc_Location,
    part_shapes: Dict[Any, Any],
    part_locations: Dict[Any, TopLoc_Location],
    current_path: Tuple[str, ...] = (),
):
    is_assembly = getattr(shape_tool, "IsAssembly_s", None)
    if not callable(is_assembly):
        is_assembly = getattr(shape_tool, "IsAssembly", None)

    if callable(is_assembly) and is_assembly(label):
        for comp in _iter_components(shape_tool, label):
            comp_loc = _combine_locations(
                parent_loc, _get_label_location(shape_tool, comp)
            )

            # Extract occurrence ID from instance label or NamedData; fallback to label name
            occ_id = _get_occurrence_id(comp) or _label_name(comp)

            ref_label = TDF_Label()
            get_ref = getattr(shape_tool, "GetReferredShape_s", None)
            if not callable(get_ref):
                get_ref = getattr(shape_tool, "GetReferredShape", None)

            if callable(get_ref) and get_ref(comp, ref_label):
                new_path = current_path + (occ_id,) if occ_id else current_path
                _collect_shapes(
                    shape_tool,
                    ref_label,
                    comp_loc,
                    part_shapes,
                    part_locations,
                    new_path,
                )
            else:
                # If no reference, just recurse on the component itself
                _collect_shapes(
                    shape_tool,
                    comp,
                    comp_loc,
                    part_shapes,
                    part_locations,
                    current_path,
                )
        return

    # Leaf part or subassembly treated as a shape
    shape_label = label
    ref_label = TDF_Label()
    get_ref = getattr(shape_tool, "GetReferredShape_s", None)
    if not callable(get_ref):
        get_ref = getattr(shape_tool, "GetReferredShape", None)
    if callable(get_ref) and get_ref(label, ref_label):
        shape_label = ref_label

    get_shape = getattr(shape_tool, "GetShape_s", None)
    if not callable(get_shape):
        get_shape = getattr(shape_tool, "GetShape", None)
    if not callable(get_shape):
        raise RuntimeError("ShapeTool missing GetShape method")

    shape = cast(Any, get_shape(shape_label))
    if shape.IsNull():
        return

    # Store by full occurrence path (tuple of IDs)
    if current_path not in part_shapes:
        part_shapes[current_path] = []
        part_locations[current_path] = []
    part_shapes[current_path].append(shape)
    part_locations[current_path].append(parent_loc)

    # Also store by individual part ID and occurrence ID for fallbacks
    parsed_id = _get_occurrence_id(label) or _get_occurrence_id(shape_label)
    if parsed_id:
        if parsed_id not in part_shapes:
            part_shapes[parsed_id] = []
            part_locations[parsed_id] = []
        # Avoid adding the same shape twice if parsed_id == current_path (not likely but safe)
        if isinstance(part_shapes[parsed_id], list):
            part_shapes[parsed_id].append(shape)
            part_locations[parsed_id].append(parent_loc)

    label_name = _label_name(label) or _label_name(shape_label)
    if label_name:
        if label_name not in part_shapes:
            part_shapes[label_name] = []
            part_locations[label_name] = []
        if isinstance(part_shapes[label_name], list):
            part_shapes[label_name].append(shape)
            part_locations[label_name].append(parent_loc)


def _get_free_shape_labels(shape_tool: Any) -> TDF_LabelSequence:
    labels = TDF_LabelSequence()
    get_free_shapes = getattr(shape_tool, "GetFreeShapes", None)
    if not callable(get_free_shapes):
        get_free_shapes = getattr(shape_tool, "GetFreeShapes_s", None)
    if not callable(get_free_shapes):
        raise RuntimeError("ShapeTool missing GetFreeShapes method")
    get_free_shapes(labels)
    return labels


def _part_world_matrix(part: Any) -> np.ndarray:
    part_tf = getattr(part, "worldToPartTF", None)
    if part_tf is None:
        return np.eye(4)
    tf_value = getattr(part_tf, "to_tf", None)
    mat = np.eye(4)
    if callable(tf_value):
        mat = cast(np.ndarray, tf_value())
    elif tf_value is not None:
        mat = cast(np.ndarray, tf_value)
    else:
        return np.eye(4)
    if not np.allclose(mat[3, :], [0, 0, 0, 1]) and np.allclose(
        mat[:, 3], [0, 0, 0, 1]
    ):
        return mat.T
    return mat


class StepMeshExporter:
    def __init__(
        self,
        client: Client | None,
        cad: Any,
        asset_path: Path | None = None,
        deflection: float = 0.01,
    ):
        self.client = client
        self.cad = cad
        self.asset_path = asset_path
        self.deflection = deflection

    def export_step(self, output_path: Path) -> Path:
        if self.client is None:
            raise RuntimeError("Cannot export STEP without Onshape client")
        did = self.cad.document_id
        wtype = getattr(self.cad, "wtype", None) or getattr(self.cad, "wvm", None)
        wid = getattr(self.cad, "workspace_id", None) or getattr(
            self.cad, "wvm_id", None
        )
        eid = self.cad.element_id
        if not wtype or not wid:
            raise AttributeError(
                "CAD object missing workspace identifiers (wtype/workspace_id)"
            )

        response = self.client.request(
            HTTP.POST,
            f"/api/assemblies/d/{did}/{wtype}/{wid}/e/{eid}/translations",
            body={
                "formatName": "STEP",
                "stepUnit": "MILLIMETER",
                "stepVersionString": "AP242",
                "storeInDocument": False,
                "includeExportIds": True,
                "extractAssemblyHierarchy": False,
                "flattenAssemblies": False,
            },
        )
        response.raise_for_status()
        translation_id = response.json().get("id")
        if not translation_id:
            raise RuntimeError("Missing translation id from Onshape response")

        status = {}
        while True:
            status = self.client.request(
                HTTP.GET,
                f"/api/translations/{translation_id}",
                log_response=False,
            ).json()
            state = status.get("requestState")
            if state == "DONE":
                break
            if state in {"FAILED", "CANCELED"}:
                raise RuntimeError(f"STEP translation failed: {state}")
            time.sleep(0.5)

        external_ids = status.get("resultExternalDataIds") or []
        if not external_ids:
            raise RuntimeError("No external data ids returned for STEP translation")

        def _extract_step_from_content(raw: bytes) -> bytes | None:
            if raw.lstrip().startswith(b"ISO-10303-21"):
                return raw

            # Check for zip header (PK\x03\x04)
            if raw.startswith(b"PK\x03\x04"):
                import io

                try:
                    with zipfile.ZipFile(io.BytesIO(raw)) as zf:
                        for name in zf.namelist():
                            if name.lower().endswith(".step") or name.lower().endswith(
                                ".stp"
                            ):
                                with zf.open(name) as f:
                                    content = f.read()
                                    if content.lstrip().startswith(b"ISO-10303-21"):
                                        return content
                except zipfile.BadZipFile:
                    pass
            return None

        found_step = False

        for file_id in external_ids:
            download = self.client.request(
                HTTP.GET,
                f"/api/documents/d/{did}/externaldata/{file_id}",
                headers={"Accept": "application/octet-stream"},
                log_response=False,
            )
            download.raise_for_status()
            content = download.content

            extracted = _extract_step_from_content(content)
            if extracted is not None:
                output_path.write_bytes(extracted)
                found_step = True
                break

            if content.strip().startswith(b"<?xml") or content.strip().startswith(
                b"<manifest"
            ):
                continue

        if not found_step:
            try:
                download = self.client.request(
                    HTTP.GET,
                    f"/api/translations/{translation_id}/download",
                    headers={"Accept": "application/octet-stream"},
                    log_response=False,
                )
                download.raise_for_status()
                extracted = _extract_step_from_content(download.content)
                if extracted is not None:
                    output_path.write_bytes(extracted)
                    return output_path
            except Exception:
                pass

        if found_step:
            return output_path

        raise RuntimeError(
            f"No STEP content found in translation results for {output_path.name}. "
            "Received XML payload (Parasolid tree?) instead."
        )

    def export_link_meshes(
        self,
        link_records: Dict[str, Any],
        mesh_dir: Path,
        bom_path: Optional[Path] = None,
        visual_mesh_format: str = "obj",
    ) -> Tuple[
        Dict[str, str | Dict[str, str]],
        Dict[str, List[Dict[str, str]]],
        Optional["InertiaReport"],
    ]:
        """Export link meshes from STEP files.

        Returns:
            Tuple of (mesh_map, missing_meshes, inertia_report) where:
            - mesh_map: Dict mapping link_name -> stl filename
            - missing_meshes: Dict mapping link_name -> list of missing part info dicts
            - inertia_report: Optional InertiaReport containing computed mass properties
        """
        from onshape2xacro.inertia import (
            InertiaCalculator,
            InertiaReport,
            BOMParser,
        )

        mesh_dir.mkdir(parents=True, exist_ok=True)
        (mesh_dir / "visual").mkdir(parents=True, exist_ok=True)
        (mesh_dir / "collision").mkdir(parents=True, exist_ok=True)

        report = None
        calc = None
        bom_entries = {}

        if bom_path:
            bom_path = Path(bom_path) if isinstance(bom_path, str) else bom_path
            report = InertiaReport()
            calc = InertiaCalculator()
            if bom_path.exists():
                parser = BOMParser()
                bom_entries = parser.parse(bom_path)
                logger.info(f"Loaded {len(bom_entries)} entries from BOM")
            else:
                logger.warning(f"BOM file not found: {bom_path}")

        if self.asset_path and self.asset_path.exists():
            asset_path = self.asset_path
        else:
            asset_path = mesh_dir / "assembly.step"
            if not asset_path.exists():
                self.export_step(asset_path)

        if asset_path.suffix.lower() != ".zip":
            header = _read_file_header(asset_path)
            if not _is_step_payload(header) and not zipfile.is_zipfile(asset_path):
                raise RuntimeError(
                    f"Invalid STEP file: {asset_path}. Please delete it and re-export."
                )

        doc = TDocStd_Document(TCollection_ExtendedString("step"))
        reader = STEPCAFControl_Reader()
        reader.SetNameMode(True)
        reader.SetPropsMode(True)
        reader.SetColorMode(True)
        reader.SetLayerMode(True)
        for mode in ["SetMatMode", "SetViewMode", "SetGDTMode", "SetSHUOMode"]:
            if hasattr(reader, mode):
                getattr(reader, mode)(True)

        status = reader.ReadFile(str(asset_path))
        if status != IFSelect_RetDone:
            raise RuntimeError(f"STEP read failed with status {status}: {asset_path}")

        if not reader.Transfer(doc):
            raise RuntimeError("STEP transfer failed")

        shape_tool = _get_shape_tool(doc)
        labels = _get_free_shape_labels(shape_tool)

        part_shapes: Dict[Any, Any] = {}
        part_locations: Dict[Any, TopLoc_Location] = {}
        for i in range(labels.Length()):
            _collect_shapes(
                shape_tool,
                labels.Value(i + 1),
                TopLoc_Location(),
                part_shapes,
                part_locations,
            )

        has_occurrence_ids = any(
            isinstance(key, tuple) and len(key) > 0 for key in part_shapes.keys()
        )

        if not has_occurrence_ids:
            part_counts: Dict[str, int] = {}
            for part in self.cad.parts.values():
                pid = getattr(part, "partId", None)
                if pid:
                    part_counts[pid] = part_counts.get(pid, 0) + 1

            if any(c > 1 for c in part_counts.values()):
                raise RuntimeError(
                    f"STEP file '{asset_path.name}' is missing 'Occurrence Export IDs' required to disambiguate parts.\n"
                    "Resolution: Delete the file to force a fresh export, or manually export with 'includeExportIds'."
                )

        stl_writer = StlAPI_Writer()
        mesh_map: Dict[str, str | Dict[str, str]] = {}
        missing_meshes: Dict[str, List[Dict[str, str]]] = {}

        occ_path_to_name: Dict[Tuple[str, ...], str] = {}
        for occ_key, occ in self.cad.occurrences.items():
            occ_path = getattr(occ, "path", None)
            if not occ_path:
                continue
            occ_path_to_name[tuple(occ_path)] = str(occ_key)

        def _instance_leaf_name(key: Any) -> str | None:
            inst = self.cad.instances.get(key)
            if inst is None:
                return None
            name = getattr(inst, "name", None)
            if not name:
                return None
            return re.sub(r"\s*<\d+>$", "", name)

        def _name_path_for_key(key: Any) -> Tuple[str, ...] | None:
            path = getattr(key, "path", None)
            if not path:
                return None
            names: List[str] = []
            for i in range(1, len(path) + 1):
                prefix = tuple(path[:i])
                name = occ_path_to_name.get(prefix)
                if not name:
                    return None
                names.append(name)
            return tuple(names) if names else None

        def _normalized_leaf_name(name_path: Tuple[str, ...] | None) -> str | None:
            if not name_path:
                return None
            leaf = name_path[-1]
            leaf = re.sub(r"_\d+$", "", leaf)
            if len(name_path) >= 2:
                parent = re.sub(r"_\d+$", "", name_path[-2])
                prefix = f"{parent}_"
                if leaf.startswith(prefix):
                    leaf = leaf[len(prefix) :]
            return leaf

        for link_name, link in link_records.items():
            keys = link.keys
            if not keys:
                continue

            # Use CAD-derived transforms (same as ZIP path) for consistency with joint origins.
            # The STEP shapes are in local part coordinates, so we use CAD API transforms
            # to place them relative to the link frame.
            valid_keys = [
                k
                for k in keys
                if self.cad.parts.get(k)
                and not getattr(self.cad.parts[k], "isRigidAssembly", False)
            ]
            if not valid_keys:
                continue

            link_world = getattr(link, "frame_transform", None)

            # Scale CAD transform (meters) to STEP coordinate system (millimeters)
            link_world_mm = link_world.copy()
            link_world_mm[:3, 3] *= 1000.0
            link_world_inv = np.linalg.inv(link_world_mm)

            compound = TopoDS_Compound()
            builder = BRep_Builder()
            builder.MakeCompound(compound)
            link_missing_parts: List[Dict[str, str]] = []
            has_valid_shapes = False
            part_metadata_list: List[Dict[str, str]] = []

            part_names_list = getattr(link, "part_names", [])
            used_indices: Dict[Any, int] = {}

            for idx, key in enumerate(keys):
                part = self.cad.parts.get(key)
                if part is None or getattr(part, "isRigidAssembly", False):
                    continue

                part_world = _part_world_matrix(part)
                part_world_mm = part_world.copy()
                part_world_mm[:3, 3] *= 1000.0

                def _pick_shape(match_key: Any):
                    shapes = part_shapes.get(match_key)
                    if not shapes:
                        return None

                    locs = part_locations.get(match_key)
                    if locs and len(shapes) > 1:
                        # Try to match by location (nearest neighbor)
                        best_idx = -1
                        min_dist = 1e9  # 1000 km is a safe upper bound

                        target_pos = part_world_mm[:3, 3]

                        for i, loc in enumerate(locs):
                            trsf = loc.Transformation()
                            # Check translation distance
                            trans = trsf.TranslationPart()
                            dx = trans.X() - target_pos[0]
                            dy = trans.Y() - target_pos[1]
                            dz = trans.Z() - target_pos[2]
                            dist_sq = dx * dx + dy * dy + dz * dz

                            if dist_sq < min_dist:
                                min_dist = dist_sq
                                best_idx = i

                        # Tolerance check: 1mm error allowed (3mm distance)
                        if best_idx >= 0 and min_dist < 100.0:
                            return shapes[best_idx]

                    # Fallback to sequential index
                    shape_idx = used_indices.get(match_key, 0)
                    shape_idx = min(shape_idx, len(shapes) - 1)
                    used_indices[match_key] = shape_idx + 1
                    return shapes[shape_idx]

                part_path = getattr(key, "path", None)
                if part_path:
                    part_path = tuple(part_path)
                shape = _pick_shape(part_path)

                if shape is None:
                    inst_name = _instance_leaf_name(key)
                    shape = _pick_shape(inst_name)

                if shape is None:
                    name_path = _name_path_for_key(key)
                    shape = _pick_shape(name_path)

                if shape is None:
                    leaf_name = _normalized_leaf_name(name_path)
                    shape = _pick_shape(leaf_name)

                if shape is None:
                    part_id = getattr(part, "partId", str(key))
                    shape = _pick_shape(part_id)

                if shape is None:
                    link_missing_parts.append(
                        {
                            "part_id": getattr(part, "partId", str(key)),
                            "export_name": "N/A",
                            "part_name": getattr(part, "name", "unknown"),
                            "reason": (
                                "Part not found in STEP (path="
                                f"{getattr(key, 'path', 'N/A')}, "
                                f"name_path={_name_path_for_key(key)})"
                            ),
                        }
                    )
                    continue

                # part_world was computed above
                link_from_part = link_world_inv @ part_world_mm
                trsf = _matrix_to_trsf(link_from_part)
                transformed = BRepBuilderAPI_Transform(shape, trsf, True).Shape()
                builder.Add(compound, transformed)
                has_valid_shapes = True

                part_name_from_list = (
                    part_names_list[idx] if idx < len(part_names_list) else None
                )
                part_metadata_list.append(
                    {
                        "part_id": getattr(part, "partId", str(key)),
                        "part_name": part_name_from_list or str(key),
                        "mesh_match": "FOUND",
                    }
                )

            if link_missing_parts:
                missing_meshes[link_name] = link_missing_parts

            if has_valid_shapes:
                BRepMesh_IncrementalMesh(compound, self.deflection)

                # Generate intermediate high-res STL
                temp_stl = mesh_dir / f"{link_name}_raw.stl"
                stl_writer.Write(compound, str(temp_stl))

                if report is not None and calc is not None:
                    try:
                        temp_step_inertia = mesh_dir / f"{link_name}_inertia.step"
                        step_writer = STEPControl_Writer()
                        step_writer.Transfer(compound, STEPControl_AsIs)
                        step_writer.Write(str(temp_step_inertia))

                        props = calc.compute_from_step_with_bom(
                            temp_step_inertia,
                            bom_entries,
                            link_name,
                            report,
                            part_metadata=part_metadata_list,
                        )

                        if props:
                            report.link_properties[link_name] = props
                            logger.info(
                                f"Computed inertia for {link_name}: mass={props.mass:.4f} kg"
                            )

                        if temp_step_inertia.exists():
                            temp_step_inertia.unlink()
                    except Exception as e:
                        logger.warning(
                            f"Failed to compute inertia for {link_name}: {e}"
                        )

                # Process with trimesh and pymeshlab
                try:
                    vis_filename = f"visual/{link_name}.{visual_mesh_format}"
                    vis_path = mesh_dir / vis_filename

                    try:
                        if visual_mesh_format == "stl":
                            import shutil

                            shutil.copy(temp_stl, vis_path)
                        else:
                            mesh = trimesh.load(str(temp_stl), force="mesh")

                            if visual_mesh_format == "dae":
                                mesh.export(vis_path, file_type="dae")
                            elif visual_mesh_format == "obj":
                                mesh.export(vis_path, file_type="obj")
                            else:
                                mesh.export(vis_path)

                    except Exception as e:
                        print(f"Error creating visual mesh for {link_name}: {e}")
                        raise e

                    # 2. Collision: Simplified (using pymeshlab)
                    col_filename = f"collision/{link_name}.stl"
                    col_path = mesh_dir / col_filename

                    try:
                        ms = pymeshlab.MeshSet()
                        ms.load_new_mesh(str(temp_stl))

                        # Generate Convex Hull
                        ms.generate_convex_hull()

                        # Simplify if needed (target 200 faces)
                        if ms.current_mesh().face_number() > 2000:
                            ms.meshing_decimation_quadric_edge_collapse(
                                targetfacenum=2000,
                            )

                        ms.save_current_mesh(str(col_path))

                        # Check file size constraint (<50KB)
                        if col_path.stat().st_size > 50 * 1024:
                            print(f"Warning: Collision mesh {col_filename} is > 50KB")

                    except Exception as e:
                        print(
                            f"Error creating collision mesh for {link_name} with pymeshlab: {e}"
                        )
                        # Fallback: copy raw STL
                        import shutil

                        shutil.copy(temp_stl, col_path)

                    # Store both
                    mesh_map[link_name] = {
                        "visual": vis_filename,
                        "collision": col_filename,
                    }

                    # Clean up temp
                    temp_stl.unlink()

                except Exception as e:
                    print(f"Error processing mesh for {link_name}: {e}")
                    # Fallback to original STL behavior if trimesh/pymeshlab fails
                    final_stl = mesh_dir / "visual" / f"{link_name}.stl"
                    if temp_stl.exists():
                        temp_stl.rename(final_stl)
                    mesh_map[link_name] = f"visual/{final_stl.name}"

        return mesh_map, missing_meshes, report
