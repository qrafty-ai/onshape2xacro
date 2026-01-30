import re
import time
from pathlib import Path
from typing import Any, Dict, Iterable, List, Optional, Tuple, cast, TYPE_CHECKING

if TYPE_CHECKING:
    pass

import numpy as np
import trimesh
import pymeshlab
from loguru import logger
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
from OCP.TopoDS import TopoDS_Compound, TopoDS_Iterator
from OCP.BRepMesh import BRepMesh_IncrementalMesh
from OCP.StlAPI import StlAPI_Writer
from OCP.gp import gp_Trsf
from OCP.STEPControl import STEPControl_Writer, STEPControl_AsIs

EXPORT_ID_REGEX = re.compile(
    r"(?:export:|exportId:|partId=|id=|OCCURRENCE_ID: ?\[?)([A-Za-z0-9+/_-]+)"
)
INSTANCE_SUFFIX_REGEX = re.compile(r"^(?P<base>.+?)\s*<(?P<index>\d+)>$")


def _get_shape_tool(doc: Any):
    shape_tool = getattr(XCAFDoc_DocumentTool, "ShapeTool_s", None)
    if callable(shape_tool):
        return shape_tool(doc.Main())
    return XCAFDoc_DocumentTool.ShapeTool(doc.Main())


def _decode_step_name(s: str) -> str:
    """Decode STEP ISO-10303-21 Annex B Unicode sequences (\\X2\\HHHH\\X0\\)."""
    if not s or "\\X2\\" not in s:
        return s

    def replace_match(match):
        hex_str = match.group(1)
        res = ""
        try:
            for i in range(0, len(hex_str), 4):
                res += chr(int(hex_str[i : i + 4], 16))
        except (ValueError, IndexError):
            return match.group(0)
        return res

    return re.sub(r"\\X2\\([0-9A-F]+)\\X0\\", replace_match, s)


def _label_name(label: Any) -> str:
    """Robustly extract name from a TDF_Label, handling Unicode corruption."""

    def _name_id():
        for attr in ("GetID_s", "GetID", "ID"):
            get_id = getattr(TDataStd_Name, attr, None)
            if callable(get_id):
                return get_id()
        return None

    name_id = _name_id()
    if name_id is None:
        return ""

    name_attr = TDataStd_Name()
    if label.FindAttribute(name_id, name_attr):
        ext_str = name_attr.Get()
        try:
            name = ""
            for i in range(1, ext_str.Length() + 1):
                name += ext_str.Value(i)
        except Exception:
            name = ext_str.ToExtString()

        if "\\X2\\" in name:
            return _decode_step_name(name)

        if all(ord(c) < 256 for c in name):
            try:
                return bytes([ord(c) for c in name]).decode("utf-8")
            except (UnicodeDecodeError, ValueError):
                pass
        return name
    return ""


def _get_occurrence_id(label: Any) -> str | None:
    get_id = getattr(TDataStd_NamedData, "GetID_s", None)
    attr_id = get_id() if callable(get_id) else TDataStd_NamedData.GetID()
    named = TDataStd_NamedData()
    if label.FindAttribute(attr_id, named):
        get_val = getattr(named, "GetString", None)
        if callable(get_val):
            for key in ("occurrenceId", "onshape:occurrenceId", "OCCURRENCE_ID"):
                value = get_val(TCollection_ExtendedString(key))
                if not value.IsEmpty():
                    return value.ToExtString()

    return _parse_export_id(_label_name(label))


def _parse_export_id(label_name: str) -> str:
    match = EXPORT_ID_REGEX.search(label_name or "")
    return match.group(1) if match else ""


def _is_step_payload(content: bytes) -> bool:
    return content.lstrip().startswith(b"ISO-10303-21")


def _matrix_to_trsf(matrix: np.ndarray) -> gp_Trsf:
    from scipy.spatial.transform import Rotation

    rot_normalized = Rotation.from_matrix(matrix[:3, :3]).as_matrix()
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
    for attr in ("GetLocation_s", "GetLocation"):
        get_loc = getattr(shape_tool, attr, None)
        if callable(get_loc):
            try:
                return get_loc(label)
            except Exception:
                continue
    return TopLoc_Location()


def _combine_locations(
    parent_loc: TopLoc_Location, local_loc: TopLoc_Location
) -> TopLoc_Location:
    if hasattr(parent_loc, "Multiplied"):
        return parent_loc.Multiplied(local_loc)
    if hasattr(parent_loc, "Multiply"):
        parent_loc.Multiply(local_loc)
        return parent_loc
    return local_loc


def _relative_location(
    part_loc: TopLoc_Location, link_loc: TopLoc_Location
) -> TopLoc_Location:
    try:
        inv = link_loc.Inverted()
        return (
            inv.Multiplied(part_loc)
            if hasattr(inv, "Multiplied")
            else inv.Multiply(part_loc) or inv
        )
    except Exception:
        return part_loc


def _iter_components(shape_tool: Any, label: TDF_Label) -> Iterable[TDF_Label]:
    get_comps = getattr(shape_tool, "GetComponents_s", None) or getattr(
        shape_tool, "GetComponents", None
    )
    if callable(get_comps):
        comps = TDF_LabelSequence()
        get_comps(label, comps)
        for i in range(comps.Length()):
            yield comps.Value(i + 1)


def _collect_shapes(
    shape_tool: Any,
    label: TDF_Label,
    parent_loc: TopLoc_Location,
    part_shapes: Dict[Any, List[Any]],
    part_locations: Dict[Any, List[TopLoc_Location]],
    current_path: Tuple[str, ...] = (),
):
    is_asm = getattr(shape_tool, "IsAssembly_s", None) or getattr(
        shape_tool, "IsAssembly", None
    )
    if callable(is_asm) and is_asm(label):
        for comp in _iter_components(shape_tool, label):
            comp_loc = _combine_locations(
                parent_loc, _get_label_location(shape_tool, comp)
            )
            occ_id = _get_occurrence_id(comp) or _label_name(comp)
            ref_label = TDF_Label()
            get_ref = getattr(shape_tool, "GetReferredShape_s", None) or getattr(
                shape_tool, "GetReferredShape", None
            )

            if callable(get_ref) and get_ref(comp, ref_label):
                _collect_shapes(
                    shape_tool,
                    ref_label,
                    comp_loc,
                    part_shapes,
                    part_locations,
                    current_path + (occ_id,) if occ_id else current_path,
                )
            else:
                _collect_shapes(
                    shape_tool,
                    comp,
                    comp_loc,
                    part_shapes,
                    part_locations,
                    current_path,
                )
        return

    get_shape = getattr(shape_tool, "GetShape_s", None) or getattr(
        shape_tool, "GetShape", None
    )
    ref_label = TDF_Label()
    get_ref = getattr(shape_tool, "GetReferredShape_s", None) or getattr(
        shape_tool, "GetReferredShape", None
    )
    shape_label = (
        ref_label if callable(get_ref) and get_ref(label, ref_label) else label
    )
    shape = cast(Any, get_shape(shape_label))
    if shape.IsNull():
        return

    for key in (current_path, _get_occurrence_id(label), _label_name(label)):
        if key:
            part_shapes.setdefault(key, []).append(shape)
            part_locations.setdefault(key, []).append(parent_loc)


def _normalize_for_match(s: str) -> str:
    if not s:
        return ""
    s = s.replace("_", "").replace("-", "").replace(" ", "")
    s = s.replace("[", "").replace("]", "").replace("(", "").replace(")", "")
    return re.sub(r"[^\w\?]", "", s).lower()


def _wildcard_match(pattern: str, target: str) -> bool:
    if not pattern or not target:
        return False
    p = re.escape(pattern).replace(r"\?", ".")
    try:
        return re.search(p, target, re.IGNORECASE) is not None
    except re.error:
        return False


def _part_world_matrix(part: Any) -> np.ndarray:
    part_tf = getattr(part, "worldToPartTF", None)
    if part_tf is None:
        return np.eye(4)
    tf_value = getattr(part_tf, "to_tf", None)
    return cast(np.ndarray, tf_value() if callable(tf_value) else tf_value or np.eye(4))


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
        did, eid = self.cad.document_id, self.cad.element_id
        wtype = getattr(self.cad, "wtype", None) or getattr(self.cad, "wvm", None)
        wid = getattr(self.cad, "workspace_id", None) or getattr(
            self.cad, "wvm_id", None
        )

        resp = self.client.request(
            HTTP.POST,
            f"/api/assemblies/d/{did}/{wtype}/{wid}/e/{eid}/translations",
            body={
                "formatName": "STEP",
                "stepUnit": "MILLIMETER",
                "stepVersionString": "AP242",
                "includeExportIds": True,
                "extractAssemblyHierarchy": False,
                "flattenAssemblies": False,
            },
        )
        resp.raise_for_status()
        tid = resp.json().get("id")

        while True:
            status = self.client.request(HTTP.GET, f"/api/translations/{tid}").json()
            state = status.get("requestState")
            if state == "DONE":
                break
            if state in {"FAILED", "CANCELED"}:
                raise RuntimeError(f"STEP translation failed: {state}")
            time.sleep(0.5)

        for file_id in status.get("resultExternalDataIds") or []:
            dl = self.client.request(
                HTTP.GET, f"/api/documents/d/{did}/externaldata/{file_id}"
            )
            if dl.content.lstrip().startswith(b"ISO-10303-21"):
                output_path.write_bytes(dl.content)
                return output_path

        raise RuntimeError("No STEP content found in translation results")

    def export_link_meshes(
        self,
        link_records: Dict[str, Any],
        mesh_dir: Path,
        bom_path: Optional[Path] = None,
    ) -> Tuple[Dict, Dict, Any]:
        from onshape2xacro.inertia import InertiaCalculator, InertiaReport, BOMParser

        mesh_dir.mkdir(parents=True, exist_ok=True)
        report, calc = InertiaReport(), InertiaCalculator()
        bom_entries = (
            BOMParser().parse(bom_path) if bom_path and bom_path.exists() else {}
        )

        asset_path = self.asset_path or (mesh_dir / "assembly.step")
        if not asset_path.exists():
            self.export_step(asset_path)

        doc = TDocStd_Document(TCollection_ExtendedString("step"))
        reader = STEPCAFControl_Reader()
        for attr in ("SetNameMode", "SetPropsMode", "SetColorMode", "SetLayerMode"):
            getattr(reader, attr)(True)
        if reader.ReadFile(str(asset_path)) != IFSelect_RetDone or not reader.Transfer(
            doc
        ):
            raise RuntimeError("STEP loading failed")

        shape_tool = _get_shape_tool(doc)
        get_free = (
            getattr(shape_tool, "GetFreeShapes_s", None) or shape_tool.GetFreeShapes
        )
        labels = TDF_LabelSequence()
        get_free(labels)

        part_shapes, part_locations = {}, {}
        for i in range(labels.Length()):
            _collect_shapes(
                shape_tool,
                labels.Value(i + 1),
                TopLoc_Location(),
                part_shapes,
                part_locations,
            )

        occ_path_to_name = {
            tuple(occ.path): str(k)
            for k, occ in self.cad.occurrences.items()
            if getattr(occ, "path", None)
        }

        def _instance_leaf_name(key):
            inst = self.cad.instances.get(key)
            return re.sub(r"\s*<\d+>$", "", inst.name) if inst and inst.name else None

        def _name_path_for_key(key):
            path = getattr(key, "path", None)
            if not path:
                return None
            names = [
                occ_path_to_name.get(tuple(path[:i])) for i in range(1, len(path) + 1)
            ]
            return tuple(names) if all(names) else None

        def _normalized_leaf_name(name_path):
            if not name_path:
                return None
            leaf = re.sub(r"_\d+$", "", name_path[-1])
            if len(name_path) >= 2:
                parent = re.sub(r"_\d+$", "", name_path[-2])
                if leaf.startswith(f"{parent}_"):
                    leaf = leaf[len(parent) + 1 :]
            return leaf

        mesh_map, missing_meshes, used_indices = {}, {}, {}

        for link_name, link in link_records.items():
            if not link.keys:
                continue

            # Find Link Location
            link_matrix = getattr(link, "frame_transform", None)
            if link_matrix is not None:
                link_matrix = link_matrix.copy()
                link_matrix[:3, 3] *= 1000.0
                link_loc = TopLoc_Location(_matrix_to_trsf(link_matrix))
            else:
                # Fallback to first part's location
                link_loc = TopLoc_Location()
                for key in link.keys:
                    p_path = tuple(getattr(key, "path", ()))
                    i_name = _instance_leaf_name(key)
                    n_path = _name_path_for_key(key)
                    l_name = _normalized_leaf_name(n_path)
                    part = self.cad.parts.get(key)
                    p_id = getattr(part, "partId", str(key)) if part else str(key)

                    for mkey in (p_path, i_name, n_path, l_name, p_id):
                        if mkey in part_locations:
                            link_loc = part_locations[mkey][0]
                            break
                    else:
                        continue
                    break

            compound, link_missing, part_metadata = TopoDS_Compound(), [], []
            builder = BRep_Builder()
            builder.MakeCompound(compound)

            for idx, key in enumerate(link.keys):
                part = self.cad.parts.get(key)
                if not part or getattr(part, "isRigidAssembly", False):
                    continue

                name_full = (
                    link.part_names[idx]
                    if idx < len(link.part_names)
                    else getattr(part, "name", str(key))
                )

                def _pick_match(mkey):
                    if mkey not in part_shapes:
                        return None, None, None
                    used = used_indices.get(mkey, 0)
                    used_indices[mkey] = used + 1
                    if used >= len(part_shapes[mkey]):
                        logger.warning(
                            f"Cloning shape for {mkey} ({used + 1} > {len(part_shapes[mkey])})"
                        )
                    return (
                        part_shapes[mkey][min(used, len(part_shapes[mkey]) - 1)],
                        part_locations[mkey][min(used, len(part_shapes[mkey]) - 1)],
                        mkey,
                    )

                # Full Matching Hierarchy
                shape, p_loc, m_info = _pick_match(tuple(getattr(key, "path", ())))
                if not shape:
                    shape, p_loc, m_info = _pick_match(_instance_leaf_name(key))
                if not shape:
                    shape, p_loc, m_info = _pick_match(_name_path_for_key(key))
                if not shape:
                    shape, p_loc, m_info = _pick_match(
                        _normalized_leaf_name(_name_path_for_key(key))
                    )
                if not shape:
                    n_path = _name_path_for_key(key)
                    if n_path:
                        shape, p_loc, m_info = _pick_match(n_path[-1])
                if not shape:
                    shape, p_loc, m_info = _pick_match(name_full)
                if not shape:
                    shape, p_loc, m_info = _pick_match(
                        getattr(part, "partId", str(key))
                    )

                # Fuzzy/Wildcard
                if not shape:
                    p_norm, path_norm = (
                        _normalize_for_match(name_full),
                        _normalize_for_match("".join(_name_path_for_key(key) or [])),
                    )
                    for l_key in part_shapes:
                        if not isinstance(l_key, str):
                            continue
                        l_norm = _normalize_for_match(l_key)
                        if not l_norm:
                            continue

                        match = False
                        if "?" in l_norm:
                            match = _wildcard_match(l_norm, p_norm) or _wildcard_match(
                                l_norm, path_norm
                            )
                        elif len(l_norm) >= 4:
                            match = (
                                l_norm == p_norm
                                or l_norm == path_norm
                                or l_norm in p_norm
                                or p_norm in l_norm
                            )

                        if match:
                            shape, p_loc, m_info = _pick_match(l_key)
                            if shape:
                                break

                if not shape:
                    link_missing.append(
                        {
                            "part_id": getattr(part, "partId", str(key)),
                            "name": getattr(part, "name", "unknown"),
                        }
                    )
                    continue

                builder.Add(
                    compound, shape.Located(_relative_location(p_loc, link_loc))
                )
                part_metadata.append(
                    {
                        "part_id": getattr(part, "partId", str(key)),
                        "part_name": name_full,
                        "mesh_match": str(m_info),
                    }
                )

            if link_missing:
                missing_meshes[link_name] = link_missing

            # Check if compound is effectively empty
            it = TopoDS_Iterator(compound)
            if not it.More():
                continue

            # Generate Mesh Assets
            BRepMesh_IncrementalMesh(compound, self.deflection)
            raw_stl = mesh_dir / f"{link_name}_raw.stl"
            StlAPI_Writer().Write(compound, str(raw_stl))

            # Inertia
            if report and calc:
                try:
                    tmp_step = mesh_dir / f"{link_name}_inertia.step"
                    sw = STEPControl_Writer()
                    sw.Transfer(compound, STEPControl_AsIs)
                    sw.Write(str(tmp_step))
                    props = calc.compute_from_step_with_bom(
                        tmp_step,
                        bom_entries,
                        link_name,
                        report,
                        part_metadata=part_metadata,
                    )
                    if props:
                        report.link_properties[link_name] = props
                    if tmp_step.exists():
                        tmp_step.unlink()
                except Exception as e:
                    logger.warning(f"Inertia failed for {link_name}: {e}")

            # Visual/Collision
            try:
                vis_file, col_file = f"{link_name}.glb", f"{link_name}_collision.stl"
                trimesh.load(str(raw_stl), force="mesh").export(mesh_dir / vis_file)
                ms = pymeshlab.MeshSet()
                ms.load_new_mesh(str(raw_stl))
                ms.generate_convex_hull()
                if ms.current_mesh().face_number() > 200:
                    ms.meshing_decimation_quadric_edge_collapse(targetfacenum=200)
                ms.save_current_mesh(str(mesh_dir / col_file))
                mesh_map[link_name] = {"visual": vis_file, "collision": col_file}
                raw_stl.unlink()
            except Exception as e:
                logger.error(f"Mesh assets failed for {link_name}: {e}")
                final_stl = mesh_dir / f"{link_name}.stl"
                if raw_stl.exists():
                    raw_stl.rename(final_stl)
                mesh_map[link_name] = final_stl.name

        return mesh_map, missing_meshes, report


def split_step_to_meshes(
    step_path: Path, link_groups: Dict[str, List[str]], mesh_dir: Path
) -> Dict[str, str]:
    from onshape2xacro.mesh_exporters.step import StepMeshExporter

    class DummyCAD:
        def __init__(self):
            self.parts = {}
            self.instances = {}
            self.occurrences = {}
            self.document_id = "dummy"
            self.element_id = "dummy"

    exporter = StepMeshExporter(None, DummyCAD(), asset_path=step_path)

    class DummyLink:
        def __init__(self, keys):
            self.keys = keys
            self.frame_transform = None
            self.part_names = []

    link_records = {name: DummyLink(keys) for name, keys in link_groups.items()}
    mesh_map, _, _ = exporter.export_link_meshes(link_records, mesh_dir)

    return {k: (v if isinstance(v, str) else v["visual"]) for k, v in mesh_map.items()}
