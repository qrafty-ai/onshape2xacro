import re
import time
from pathlib import Path
from typing import Any, Dict, Iterable

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
from OCP.TopoDS import TopoDS_Compound
from OCP.BRepMesh import BRepMesh_IncrementalMesh
from OCP.StlAPI import StlAPI_Writer


EXPORT_ID_REGEX = re.compile(
    r"(?:export:|exportId:|partId=|id=|OCCURRENCE_ID: ?\[?)([A-Za-z0-9+/_-]+)"
)


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

    shape = get_shape(shape_label)
    if shape.IsNull():
        return

    part_shapes[part_id] = shape
    part_locations[part_id] = parent_loc


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
    def __init__(self, client: Client, cad: Any):
        self.client = client
        self.cad = cad

    def export_step(self, output_path: Path) -> Path:
        did = self.cad.document_id
        wtype = self.cad.wvm
        wid = self.cad.wvm_id
        eid = self.cad.element_id

        payload = {
            "formatName": "STEP",
            "storeInDocument": False,
            "stepVersionString": "AP242",
            "extractAssemblyHierarchy": True,
            "includeExportIds": True,
            "flattenAssemblies": False,
        }

        response = self.client.request(
            HTTP.POST,
            f"/api/assemblies/d/{did}/{wtype}/{wid}/e/{eid}/translations",
            body=payload,
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
        self, link_groups: Dict[str, list[str]], mesh_dir: Path
    ) -> Dict[str, str]:
        mesh_dir.mkdir(parents=True, exist_ok=True)
        step_path = mesh_dir / "assembly.step"
        self.export_step(step_path)
        return split_step_to_meshes(step_path, link_groups, mesh_dir)
