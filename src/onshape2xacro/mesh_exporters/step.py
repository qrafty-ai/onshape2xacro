import io
import re
import tempfile
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
    return ""


def _is_step_payload(content: bytes) -> bool:
    return content.lstrip().startswith(b"ISO-10303-21")


def _read_file_header(path: Path, max_bytes: int = 512) -> bytes:
    with path.open("rb") as handle:
        return handle.read(max_bytes)


def _export_name_from_instance_name(instance_name: str) -> str:
    match = INSTANCE_SUFFIX_REGEX.match(instance_name or "")
    if not match:
        return instance_name
    base = match.group("base")
    index = int(match.group("index"))
    if index <= 1:
        return base
    return f"{base} ({index - 1})"


def _map_keys_to_export_names(cad: Any) -> Dict[Any, str]:
    """Map CAD part keys to their expected export names in Onshape STEP.

    Onshape STEP export names instances as:
    - BaseName (if index 1 or no index)
    - BaseName (1) (if index 2)
    - BaseName (2) (if index 3)
    ...
    Even if there are gaps in indices (e.g. <1>, <3>), they are assigned
    sequential names based on their sorted order.
    """
    groups: Dict[str, List[Tuple[int, Any]]] = {}
    for key in cad.parts:
        instance = cad.instances.get(key)
        if not instance:
            continue
        name = instance.name
        match = INSTANCE_SUFFIX_REGEX.match(name)
        if match:
            base = match.group("base")
            index = int(match.group("index"))
        else:
            base = name
            index = 0
        if base not in groups:
            groups[base] = []
        groups[base].append((index, key))

    mapping = {}
    for base, items in groups.items():
        # Sort by index, then by key as tie-breaker
        items.sort(key=lambda x: (x[0], str(x[1])))
        for i, (_, key) in enumerate(items):
            if i == 0:
                mapping[key] = base
            else:
                mapping[key] = f"{base} ({i})"
    return mapping


def _export_name_from_filename(filename: str) -> str:
    name = re.sub(r"^.* - ", "", filename)
    if name.lower().endswith(".step"):
        name = name[:-5]
    elif name.lower().endswith(".stp"):
        name = name[:-4]
    return name


def _matrix_to_trsf(matrix: np.ndarray) -> gp_Trsf:
    trsf = gp_Trsf()
    trsf.SetValues(
        matrix[0][0],
        matrix[0][1],
        matrix[0][2],
        matrix[0][3],
        matrix[1][0],
        matrix[1][1],
        matrix[1][2],
        matrix[1][3],
        matrix[2][0],
        matrix[2][1],
        matrix[2][2],
        matrix[2][3],
    )
    return trsf


def _matrix_to_loc(matrix: np.ndarray) -> TopLoc_Location:
    return TopLoc_Location(_matrix_to_trsf(matrix))


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


def _relative_location(
    part_loc: TopLoc_Location, link_loc: TopLoc_Location
) -> TopLoc_Location:
    if hasattr(link_loc, "Inverted"):
        try:
            inv = link_loc.Inverted()
            if hasattr(inv, "Multiplied"):
                return inv.Multiplied(part_loc)
            if hasattr(inv, "Multiply"):
                inv.Multiply(part_loc)
                return inv
        except Exception:
            pass
    return part_loc


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


def _get_shape(shape_tool: Any, label: TDF_Label):
    get_shape = getattr(shape_tool, "GetShape_s", None)
    if not callable(get_shape):
        get_shape = getattr(shape_tool, "GetShape", None)
    if not callable(get_shape):
        raise RuntimeError("ShapeTool missing GetShape method")
    return get_shape(label)


def _load_step_shapes(step_path: Path) -> list[Any]:
    doc = TDocStd_Document(TCollection_ExtendedString("step"))
    reader = STEPCAFControl_Reader()
    reader.SetNameMode(True)

    status = reader.ReadFile(str(step_path))
    if status != IFSelect_RetDone:
        raise RuntimeError(f"STEP read failed: {status}")

    reader.Transfer(doc)
    shape_tool = _get_shape_tool(doc)
    labels = _get_free_shape_labels(shape_tool)

    shapes = []
    for i in range(labels.Length()):
        label = labels.Value(i + 1)
        shape = cast(Any, _get_shape(shape_tool, label))
        if not shape.IsNull():
            shapes.append(shape)
    return shapes


def _load_step_shapes_from_zip(zip_path: Path) -> Dict[str, Any]:
    shapes_by_name: Dict[str, Any] = {}
    with zipfile.ZipFile(zip_path) as archive:
        step_names = [
            name
            for name in archive.namelist()
            if name.lower().endswith(".step") or name.lower().endswith(".stp")
        ]
        with tempfile.TemporaryDirectory() as tmp_dir:
            tmp_path = Path(tmp_dir)
            for name in step_names:
                export_name = _export_name_from_filename(name)
                if export_name in shapes_by_name:
                    raise RuntimeError(f"Duplicate STEP export name: {export_name}")

                step_path = tmp_path / Path(name).name
                step_path.write_bytes(archive.read(name))
                shapes = _load_step_shapes(step_path)

                compound = TopoDS_Compound()
                builder = BRep_Builder()
                builder.MakeCompound(compound)
                for shape in shapes:
                    builder.Add(compound, shape)
                shapes_by_name[export_name] = compound

    return shapes_by_name


def _part_world_matrix(part: Any) -> np.ndarray:
    part_tf = getattr(part, "worldToPartTF", None)
    if part_tf is None:
        return np.eye(4)
    tf_value = getattr(part_tf, "to_tf", None)
    if callable(tf_value):
        return cast(np.ndarray, tf_value())
    if tf_value is not None:
        return cast(np.ndarray, tf_value)
    return np.eye(4)


def split_step_to_meshes(
    step_path: Path, link_groups: Dict[str, list[str]], mesh_dir: Path
) -> Dict[str, str]:
    doc = TDocStd_Document(TCollection_ExtendedString("step"))
    reader = STEPCAFControl_Reader()
    reader.SetNameMode(True)

    status = reader.ReadFile(str(step_path))
    if status != IFSelect_RetDone:
        raise RuntimeError(f"STEP read failed: {status}")

    reader.Transfer(doc)
    shape_tool = _get_shape_tool(doc)

    labels = TDF_LabelSequence()
    get_free_shapes = getattr(shape_tool, "GetFreeShapes", None)
    if not callable(get_free_shapes):
        get_free_shapes = getattr(shape_tool, "GetFreeShapes_s", None)
    if not callable(get_free_shapes):
        raise RuntimeError("ShapeTool missing GetFreeShapes method")
    get_free_shapes(labels)

    part_shapes: Dict[str, Any] = {}
    part_locations: Dict[str, TopLoc_Location] = {}

    for i in range(labels.Length()):
        label = labels.Value(i + 1)
        _collect_shapes(
            shape_tool, label, TopLoc_Location(), part_shapes, part_locations
        )

    if not part_shapes:
        raise RuntimeError("No shapes found in STEP file")

    mesh_dir.mkdir(parents=True, exist_ok=True)
    stl_writer = StlAPI_Writer()
    mesh_map: Dict[str, str] = {}

    for link_name, part_ids in link_groups.items():
        if not part_ids:
            continue

        link_loc = part_locations.get(part_ids[0], TopLoc_Location())
        compound = TopoDS_Compound()
        builder = BRep_Builder()
        builder.MakeCompound(compound)

        for part_id in part_ids:
            shapes = part_shapes.get(part_id)
            if not shapes:
                raise RuntimeError(f"Missing part id in STEP: {part_id}")

            locs = part_locations.get(part_id, [TopLoc_Location()])
            # Add all occurrences of this part ID
            for shape, part_loc in zip(shapes, locs):
                rel_loc = _relative_location(part_loc, link_loc)
                builder.Add(compound, shape.Located(rel_loc))

        BRepMesh_IncrementalMesh(compound, 0.01)
        out_path = mesh_dir / f"{link_name}.stl"
        stl_writer.Write(compound, str(out_path))
        mesh_map[link_name] = out_path.name

    return mesh_map


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
            content = raw
            if zipfile.is_zipfile(io.BytesIO(content)):
                with zipfile.ZipFile(io.BytesIO(content)) as zf:
                    step_files = [
                        n
                        for n in zf.namelist()
                        if n.lower().endswith((".step", ".stp"))
                    ]
                    if step_files:
                        step_files.sort(
                            key=lambda x: (
                                x.lower() != "assembly.step",
                                x.lower() != "assembly.stp",
                                x,
                            )
                        )
                        return zf.read(step_files[0])
            if content.strip().startswith(b"ISO-10303-21"):
                return content
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

            # 3. If it's XML, it might be a manifest; continue to next external_id
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
        self, link_records: Dict[str, Any], mesh_dir: Path
    ) -> Tuple[Dict[str, str | Dict[str, str]], Dict[str, List[Dict[str, str]]]]:
        """Export link meshes from STEP files.

        Returns:
            Tuple of (mesh_map, missing_meshes) where:
            - mesh_map: Dict mapping link_name -> stl filename
            - missing_meshes: Dict mapping link_name -> list of missing part info dicts
              Each missing part dict has keys: part_id, export_name, part_name, reason
        """
        mesh_dir.mkdir(parents=True, exist_ok=True)
        if self.asset_path and self.asset_path.exists():
            asset_path = self.asset_path
        else:
            asset_path = mesh_dir / "assembly.step"
            self.export_step(asset_path)

        if asset_path.suffix.lower() != ".zip":
            header = _read_file_header(asset_path)
            if not _is_step_payload(header) and not zipfile.is_zipfile(asset_path):
                if self.client is None:
                    raise RuntimeError(
                        f"Invalid STEP file: {asset_path} (re-export requires client)"
                    )
                asset_path = mesh_dir / "assembly.step"
                self.export_step(asset_path)

        if asset_path.suffix.lower() == ".zip":
            # For backward compatibility if 'asset_path' ends in '.zip'
            shapes_by_name = _load_step_shapes_from_zip(asset_path)
            export_name_by_key = _map_keys_to_export_names(self.cad)

            stl_writer = StlAPI_Writer()
            mesh_map: Dict[str, str | Dict[str, str]] = {}
            missing_meshes: Dict[str, List[Dict[str, str]]] = {}

            for link_name, link in link_records.items():
                keys = link.keys
                if not keys:
                    continue

                valid_keys = [
                    k
                    for k in keys
                    if self.cad.parts.get(k)
                    and not getattr(self.cad.parts[k], "isRigidAssembly", False)
                ]
                if not valid_keys:
                    continue

                link_world = getattr(link, "frame_transform", None)
                if link_world is None:
                    ref_key = valid_keys[0]
                    link_world = _part_world_matrix(self.cad.parts[ref_key])

                link_world_inv = np.linalg.inv(link_world)

                compound = TopoDS_Compound()
                builder = BRep_Builder()
                builder.MakeCompound(compound)
                link_missing_parts: List[Dict[str, str]] = []
                has_valid_shapes = False

                for key in keys:
                    part = self.cad.parts.get(key)
                    if part is None or getattr(part, "isRigidAssembly", False):
                        continue

                    export_name = export_name_by_key.get(key)
                    shape = shapes_by_name.get(export_name) if export_name else None

                    if shape is None:
                        link_missing_parts.append(
                            {
                                "part_id": getattr(part, "partId", str(key)),
                                "export_name": export_name or "unknown",
                                "part_name": getattr(part, "name", "unknown"),
                                "reason": f"STEP file '{export_name}' not found in zip"
                                if export_name
                                else "no export name mapping",
                            }
                        )
                        continue

                    part_world = _part_world_matrix(part)
                    link_from_part = link_world_inv @ part_world
                    trsf = _matrix_to_trsf(link_from_part)
                    transformed = BRepBuilderAPI_Transform(shape, trsf, True).Shape()
                    builder.Add(compound, transformed)
                    has_valid_shapes = True

                if link_missing_parts:
                    missing_meshes[link_name] = link_missing_parts

                if has_valid_shapes:
                    BRepMesh_IncrementalMesh(compound, self.deflection)

                    # Generate intermediate high-res STL
                    temp_stl = mesh_dir / f"{link_name}_raw.stl"
                    stl_writer.Write(compound, str(temp_stl))

                # Process with trimesh and pymeshlab
                try:
                    # 1. Visual: GLB (using trimesh)
                    try:
                        # Load with force='mesh' to ensure we get a Trimesh object, not Scene
                        mesh = trimesh.load(str(temp_stl), force="mesh")
                        vis_filename = f"{link_name}.glb"
                        vis_path = mesh_dir / vis_filename
                        mesh.export(vis_path)
                    except Exception as e:
                        print(f"Error creating visual mesh for {link_name}: {e}")
                        # Fallback for visual if trimesh fails? Just use STL or fail?
                        # We'll rely on original STL if GLB fails, but maybe better to propagate error
                        raise e

                    # 2. Collision: Simplified (using pymeshlab)
                    col_filename = f"{link_name}_collision.stl"
                    col_path = mesh_dir / col_filename

                    try:
                        ms = pymeshlab.MeshSet()
                        ms.load_new_mesh(str(temp_stl))

                        # Generate Convex Hull
                        ms.generate_convex_hull()

                        # Simplify if needed (target 200 faces)
                        # We use Quadric Edge Collapse Decimation
                        # We check face count of current mesh (which is the hull)
                        if ms.current_mesh().face_number() > 200:
                            ms.meshing_decimation_quadric_edge_collapse(
                                targetfacenum=200
                            )

                        ms.save_current_mesh(str(col_path))

                        # Check file size constraint (<50KB)
                        if col_path.stat().st_size > 50 * 1024:
                            print(f"Warning: Collision mesh {col_filename} is > 50KB")

                    except Exception as e:
                        print(
                            f"Error creating collision mesh for {link_name} with pymeshlab: {e}"
                        )
                        # Fallback: copy raw STL as collision
                        # But raw STL might be huge.
                        # If pymeshlab fails, we might just use the raw one or try trimesh logic as backup
                        # For now, just copy raw
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
                    # Fallback to original STL behavior if everything fails
                    final_stl = mesh_dir / f"{link_name}.stl"
                    if temp_stl.exists():
                        temp_stl.rename(final_stl)
                    mesh_map[link_name] = final_stl.name

            return mesh_map, missing_meshes

        # Primary logic for single STEP file
        doc = TDocStd_Document(TCollection_ExtendedString("step"))
        reader = STEPCAFControl_Reader()
        reader.SetNameMode(True)
        reader.SetPropsMode(True)
        reader.SetColorMode(True)
        reader.SetLayerMode(True)
        if hasattr(reader, "SetMatMode"):
            reader.SetMatMode(True)
        if hasattr(reader, "SetViewMode"):
            reader.SetViewMode(True)
        if hasattr(reader, "SetGDTMode"):
            reader.SetGDTMode(True)
        if hasattr(reader, "SetSHUOMode"):
            reader.SetSHUOMode(True)
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
        has_any_ids = any(
            (isinstance(key, tuple) and len(key) > 0)
            or (isinstance(key, str) and bool(key))
            for key in part_shapes.keys()
        )

        part_id_counts: Dict[str, int] = {}
        for part in self.cad.parts.values():
            part_id = getattr(part, "partId", None)
            if part_id is None:
                continue
            part_id_counts[part_id] = part_id_counts.get(part_id, 0) + 1
        has_duplicate_part_ids = any(count > 1 for count in part_id_counts.values())

        needs_reexport = (not has_any_ids) or (
            not has_occurrence_ids and has_duplicate_part_ids
        )

        if needs_reexport:
            # Re-export to ensure export IDs are embedded
            asset_path = mesh_dir / "assembly.step"
            try:
                self.export_step(asset_path)
            except Exception as exc:
                if self.client is None:
                    raise RuntimeError(
                        "STEP file missing occurrence export IDs and no client available to re-export."
                    ) from exc
                raise

            doc = TDocStd_Document(TCollection_ExtendedString("step"))
            reader = STEPCAFControl_Reader()
            reader.SetNameMode(True)
            reader.SetPropsMode(True)
            reader.SetColorMode(True)
            reader.SetLayerMode(True)
            if hasattr(reader, "SetMatMode"):
                reader.SetMatMode(True)
            if hasattr(reader, "SetViewMode"):
                reader.SetViewMode(True)
            if hasattr(reader, "SetGDTMode"):
                reader.SetGDTMode(True)
            if hasattr(reader, "SetSHUOMode"):
                reader.SetSHUOMode(True)

            status = reader.ReadFile(str(asset_path))
            if status != IFSelect_RetDone:
                raise RuntimeError(
                    f"STEP read failed with status {status}: {asset_path}"
                )
            if not reader.Transfer(doc):
                raise RuntimeError("STEP transfer failed")
            shape_tool = _get_shape_tool(doc)
            labels = _get_free_shape_labels(shape_tool)
            part_shapes.clear()
            part_locations.clear()
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
            has_any_ids = any(
                (isinstance(key, tuple) and len(key) > 0)
                or (isinstance(key, str) and bool(key))
                for key in part_shapes.keys()
            )
            needs_reexport = (not has_any_ids) or (
                not has_occurrence_ids and has_duplicate_part_ids
            )

        if needs_reexport:
            raise RuntimeError(
                "STEP file missing occurrence export IDs. Re-export with includeExportIds enabled."
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

            used_indices: Dict[Any, int] = {}

            # Link frame transform (World to Link)
            link_matrix = getattr(link, "frame_transform", None)
            if link_matrix is not None:
                # Scale translation from meters to millimeters to match STEP export
                link_matrix = link_matrix.copy()
                link_matrix[:3, 3] *= 1000.0
                link_loc = TopLoc_Location(_matrix_to_trsf(link_matrix))
            else:
                # Fallback to first part's location from STEP
                link_loc = None
                for key in keys:
                    part = self.cad.parts.get(key)
                    if part is None or getattr(part, "isRigidAssembly", False):
                        continue

                    # Try occurrence path first
                    part_path = getattr(key, "path", None)
                    if part_path:
                        part_path = tuple(part_path)
                    if part_path in part_locations:
                        link_loc = part_locations[part_path][0]
                        break

                    inst_name = _instance_leaf_name(key)
                    if inst_name in part_locations:
                        link_loc = part_locations[inst_name][0]
                        break

                    # Try name path from instances
                    name_path = _name_path_for_key(key)
                    if name_path in part_locations:
                        link_loc = part_locations[name_path][0]
                        break

                    leaf_name = _normalized_leaf_name(name_path)
                    if leaf_name in part_locations:
                        link_loc = part_locations[leaf_name][0]
                        break

                    # Try part ID
                    part_id = getattr(part, "partId", str(key))
                    if part_id in part_locations:
                        link_loc = part_locations[part_id][0]
                        break
                if link_loc is None:
                    link_loc = TopLoc_Location()

            compound = TopoDS_Compound()
            builder = BRep_Builder()
            builder.MakeCompound(compound)
            link_missing_parts: List[Dict[str, str]] = []
            has_valid_shapes = False

            for key in keys:
                part = self.cad.parts.get(key)
                if part is None or getattr(part, "isRigidAssembly", False):
                    continue

                # Helper to pick shape and location from lists based on used indices
                def _pick_match(match_key: Any):
                    shapes = part_shapes.get(match_key)
                    locs = part_locations.get(match_key)
                    if shapes and locs:
                        idx = used_indices.get(match_key, 0)
                        # If we have multiple occurrences, distribute them 1-to-1
                        # If we run out, reuse the last one (better than missing)
                        shape_idx = min(idx, len(shapes) - 1)
                        used_indices[match_key] = idx + 1
                        return shapes[shape_idx], locs[shape_idx]
                    return None, None

                # 1. Primary match: Occurrence Path (Tuple of IDs)
                part_path = getattr(key, "path", None)
                if part_path:
                    part_path = tuple(part_path)
                shape, part_loc = _pick_match(part_path)
                name_path = None

                if shape is None or part_loc is None:
                    inst_name = _instance_leaf_name(key)
                    shape, part_loc = _pick_match(inst_name)

                if shape is None or part_loc is None:
                    name_path = _name_path_for_key(key)
                    shape, part_loc = _pick_match(name_path)

                if shape is None or part_loc is None:
                    leaf_name = _normalized_leaf_name(name_path)
                    shape, part_loc = _pick_match(leaf_name)

                if shape is None or part_loc is None:
                    # 2. Secondary match: Part Studio ID (String)
                    part_id = getattr(part, "partId", str(key))
                    shape, part_loc = _pick_match(part_id)

                if shape is None or part_loc is None:
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

                rel_loc = _relative_location(part_loc, link_loc)
                builder.Add(compound, shape.Located(rel_loc))
                has_valid_shapes = True

            if link_missing_parts:
                missing_meshes[link_name] = link_missing_parts

            if has_valid_shapes:
                BRepMesh_IncrementalMesh(compound, self.deflection)

                # Generate intermediate high-res STL
                temp_stl = mesh_dir / f"{link_name}_raw.stl"
                stl_writer.Write(compound, str(temp_stl))

                # Process with trimesh and pymeshlab
                try:
                    # 1. Visual: GLB (using trimesh)
                    try:
                        # Load with force='mesh' to ensure we get a Trimesh object, not Scene
                        mesh = trimesh.load(str(temp_stl), force="mesh")
                        vis_filename = f"{link_name}.glb"
                        vis_path = mesh_dir / vis_filename
                        mesh.export(vis_path)
                    except Exception as e:
                        print(f"Error creating visual mesh for {link_name}: {e}")
                        # Fallback to STL for visual if needed, or just warn
                        # For now, we proceed. If GLB fails, Xacro might point to a missing file
                        # unless we handle it. But let's assume it works or the collision handling covers it.
                        pass

                    # 2. Collision: Simplified (using pymeshlab)
                    col_filename = f"{link_name}_collision.stl"
                    col_path = mesh_dir / col_filename

                    try:
                        ms = pymeshlab.MeshSet()
                        ms.load_new_mesh(str(temp_stl))

                        # Generate Convex Hull
                        ms.generate_convex_hull()

                        # Simplify if needed (target 200 faces)
                        if ms.current_mesh().face_number() > 200:
                            ms.meshing_decimation_quadric_edge_collapse(
                                targetfacenum=200
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
                    final_stl = mesh_dir / f"{link_name}.stl"
                    if temp_stl.exists():
                        temp_stl.rename(final_stl)
                    mesh_map[link_name] = final_stl.name

        return mesh_map, missing_meshes

    def compute_link_inertials(
        self,
        link_records: Dict[str, Any],
        output_dir: Path,
        bom_path: Optional[Path] = None,
        density: float = 1000.0,
    ) -> "InertiaReport":
        """
        Compute inertial properties for each link.

        Args:
            link_records: Dict mapping link names to link data (must have 'compound' key)
            output_dir: Directory for intermediate files and config output
            bom_path: Optional path to Onshape BOM CSV export
            density: Default material density in kg/m³ (used when no BOM)

        Returns:
            InertiaReport with results and warnings
        """
        from onshape2xacro.inertia import (
            InertiaCalculator,
            InertiaConfigWriter,
            BOMParser,
            InertiaReport,
        )

        calc = InertiaCalculator(default_density=density)
        writer = InertiaConfigWriter()
        report = InertiaReport()

        # Parse BOM if provided
        bom_entries = {}
        if bom_path and bom_path.exists():
            parser = BOMParser()
            bom_entries = parser.parse(bom_path)
            logger.info(f"Loaded {len(bom_entries)} entries from BOM")

        # Create links directory for intermediate STEP files
        links_dir = output_dir / "links"
        links_dir.mkdir(parents=True, exist_ok=True)

        for link_name, link_data in link_records.items():
            compound = link_data.get("compound")
            if compound is None:
                report.add_warning(link_name, link_name, "No geometry found for link")
                continue

            # Export link compound to STEP file
            temp_step = links_dir / f"{link_name}.step"
            try:
                step_writer = STEPControl_Writer()
                step_writer.Transfer(compound, STEPControl_AsIs)
                status = step_writer.Write(str(temp_step))
                if status != 1:
                    report.add_warning(
                        link_name, link_name, "Failed to write STEP file"
                    )
                    continue
            except Exception as e:
                report.add_warning(link_name, link_name, f"Error exporting STEP: {e}")
                continue

            # Compute inertia
            try:
                if bom_entries:
                    props = calc.compute_from_step_with_bom(
                        temp_step, bom_entries, link_name, report
                    )
                else:
                    props = calc.compute_from_step(temp_step, material="default")

                report.link_properties[link_name] = props
                logger.info(
                    f"Computed inertia for {link_name}: mass={props.mass:.4f} kg"
                )
            except Exception as e:
                report.add_warning(
                    link_name, link_name, f"Failed to compute inertia: {e}"
                )

        # Write config YAML
        if report.link_properties:
            config_dir = output_dir / "config"
            writer.write(report.link_properties, config_dir)
            logger.info(f"Wrote inertials.yaml to {config_dir}")

        # Print warning summary
        report.print_summary()

        return report
