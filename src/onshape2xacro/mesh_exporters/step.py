import re
import tempfile
import time
import zipfile
from pathlib import Path
from typing import Any, Dict, Iterable, List, Tuple, cast

import numpy as np
from onshape_robotics_toolkit.connect import Client, HTTP
from OCP.STEPCAFControl import STEPCAFControl_Reader
from OCP.TDocStd import TDocStd_Document
from OCP.TCollection import TCollection_ExtendedString
from OCP.XCAFDoc import XCAFDoc_DocumentTool
from OCP.TDataStd import TDataStd_Name
from OCP.TDF import TDF_Label, TDF_LabelSequence
from OCP.IFSelect import IFSelect_RetDone
from OCP.TopLoc import TopLoc_Location
from OCP.BRep import BRep_Builder
from OCP.BRepBuilderAPI import BRepBuilderAPI_Transform
from OCP.TopoDS import TopoDS_Compound
from OCP.BRepMesh import BRepMesh_IncrementalMesh
from OCP.StlAPI import StlAPI_Writer
from OCP.gp import gp_Trsf


EXPORT_ID_REGEX = re.compile(
    r"(?:export:|exportId:|partId=|id=|OCCURRENCE_ID: ?\[?)([A-Za-z0-9+/_-]+)"
)
INSTANCE_SUFFIX_REGEX = re.compile(r"^(?P<base>.+?)\s*<(?P<index>\d+)>$")


def _get_shape_tool(doc: TDocStd_Document):
    shape_tool = getattr(XCAFDoc_DocumentTool, "ShapeTool_s", None)
    if callable(shape_tool):
        return shape_tool(doc.Main())
    return XCAFDoc_DocumentTool.ShapeTool(doc.Main())


def _label_name(label: TDF_Label) -> str:
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


def _parse_export_id(label_name: str) -> str:
    match = EXPORT_ID_REGEX.search(label_name or "")
    if match:
        return match.group(1)
    return ""


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
    part_shapes: Dict[str, Any],
    part_locations: Dict[str, TopLoc_Location],
):
    is_assembly = getattr(shape_tool, "IsAssembly_s", None)
    if not callable(is_assembly):
        is_assembly = getattr(shape_tool, "IsAssembly", None)
    if callable(is_assembly) and is_assembly(label):
        for comp in _iter_components(shape_tool, label):
            comp_loc = _combine_locations(
                parent_loc, _get_label_location(shape_tool, comp)
            )
            ref_label = TDF_Label()
            get_ref = getattr(shape_tool, "GetReferredShape_s", None)
            if not callable(get_ref):
                get_ref = getattr(shape_tool, "GetReferredShape", None)
            if callable(get_ref) and get_ref(comp, ref_label):
                _collect_shapes(
                    shape_tool, ref_label, comp_loc, part_shapes, part_locations
                )
            else:
                _collect_shapes(shape_tool, comp, comp_loc, part_shapes, part_locations)
        return

    shape_label = label
    ref_label = TDF_Label()
    get_ref = getattr(shape_tool, "GetReferredShape_s", None)
    if not callable(get_ref):
        get_ref = getattr(shape_tool, "GetReferredShape", None)
    if callable(get_ref) and get_ref(label, ref_label):
        shape_label = ref_label

    name = _label_name(label) or _label_name(shape_label)
    part_id = _parse_export_id(name)
    if not part_id:
        return

    get_shape = getattr(shape_tool, "GetShape_s", None)
    if not callable(get_shape):
        get_shape = getattr(shape_tool, "GetShape", None)
    if not callable(get_shape):
        raise RuntimeError("ShapeTool missing GetShape method")

    shape = cast(Any, get_shape(shape_label))
    if shape.IsNull():
        return

    part_shapes[part_id] = shape
    part_locations[part_id] = parent_loc


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
            shape = part_shapes.get(part_id)
            if shape is None:
                raise RuntimeError(f"Missing part id in STEP: {part_id}")

            part_loc = part_locations.get(part_id, TopLoc_Location())
            rel_loc = _relative_location(part_loc, link_loc)
            builder.Add(compound, shape.Located(rel_loc))

        BRepMesh_IncrementalMesh(compound, 0.001)
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
    ):
        self.client = client
        self.cad = cad
        self.asset_path = asset_path

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
            f"/api/assemblies/d/{did}/{wtype}/{wid}/e/{eid}/export/step",
            body={
                "stepUnit": "METER",
                "stepVersionString": "AP242",
                "storeInDocument": False,
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

        file_id = external_ids[0]
        download = self.client.request(
            HTTP.GET,
            f"/api/documents/d/{did}/externaldata/{file_id}",
            headers={"Accept": "application/octet-stream"},
            log_response=False,
        )
        download.raise_for_status()
        output_path.write_bytes(download.content)
        return output_path

    def export_link_meshes(
        self, link_records: Dict[str, Any], mesh_dir: Path
    ) -> Tuple[Dict[str, str], Dict[str, List[Dict[str, str]]]]:
        """Export link meshes from STEP files.

        Returns:
            Tuple of (mesh_map, missing_meshes) where:
            - mesh_map: Dict mapping link_name -> stl filename
            - missing_meshes: Dict mapping link_name -> list of missing part info dicts
              Each missing part dict has keys: part_id, export_name, part_name, reason
        """
        mesh_dir.mkdir(parents=True, exist_ok=True)
        if self.asset_path and self.asset_path.exists():
            archive_path = self.asset_path
        else:
            archive_path = mesh_dir / "assembly.zip"
            self.export_step(archive_path)

        shapes_by_name = _load_step_shapes_from_zip(archive_path)
        export_name_by_key = _map_keys_to_export_names(self.cad)

        stl_writer = StlAPI_Writer()
        mesh_map: Dict[str, str] = {}
        missing_meshes: Dict[str, List[Dict[str, str]]] = {}

        for link_name, link in link_records.items():
            keys = link.keys
            if not keys:
                continue

            # Filter out rigid assemblies and non-existent parts
            valid_keys = []
            for key in keys:
                part = self.cad.parts.get(key)
                if part is None:
                    continue
                if getattr(part, "isRigidAssembly", False):
                    continue
                valid_keys.append(key)

            if not valid_keys:
                continue

            # Link frame transform (World to Link)
            # If LinkRecord has frame_transform, use it. Otherwise fallback to first part.
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
                if part is None:
                    link_missing_parts.append(
                        {
                            "part_id": str(key),
                            "export_name": "unknown",
                            "part_name": "unknown",
                            "reason": "key not found in CAD parts",
                        }
                    )
                    continue

                if getattr(part, "isRigidAssembly", False):
                    continue

                export_name = export_name_by_key.get(key)
                if not export_name:
                    link_missing_parts.append(
                        {
                            "part_id": getattr(part, "partId", str(key)),
                            "export_name": "unknown",
                            "part_name": getattr(part, "name", "unknown"),
                            "reason": "no export name mapping found",
                        }
                    )
                    continue

                shape = shapes_by_name.get(export_name)
                if shape is None:
                    link_missing_parts.append(
                        {
                            "part_id": getattr(part, "partId", str(key)),
                            "export_name": export_name,
                            "part_name": getattr(part, "name", "unknown"),
                            "reason": f"STEP file '{export_name}' not found in zip",
                        }
                    )
                    continue

                # Compute transform: link_world_inv * part_world
                part_world = _part_world_matrix(part)
                link_from_part = link_world_inv @ part_world
                trsf = _matrix_to_trsf(link_from_part)
                transformed = BRepBuilderAPI_Transform(shape, trsf, True).Shape()
                builder.Add(compound, transformed)
                has_valid_shapes = True

            if link_missing_parts:
                missing_meshes[link_name] = link_missing_parts

            if has_valid_shapes:
                BRepMesh_IncrementalMesh(compound, 0.001)
                out_path = mesh_dir / f"{link_name}.stl"
                stl_writer.Write(compound, str(out_path))
                mesh_map[link_name] = out_path.name

        return mesh_map, missing_meshes
