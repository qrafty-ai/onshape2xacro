from pathlib import Path
from onshape2xacro.mesh_exporters.step import (
    StepMeshExporter,
    split_step_to_meshes,
    _is_step_payload,
)
from OCP.BRepPrimAPI import BRepPrimAPI_MakeBox
from OCP.STEPCAFControl import STEPCAFControl_Writer
from OCP.TDocStd import TDocStd_Document
from OCP.TCollection import TCollection_ExtendedString
from OCP.XCAFDoc import XCAFDoc_DocumentTool
from OCP.TDataStd import TDataStd_Name
from OCP.STEPControl import STEPControl_AsIs
import zipfile
from typing import Any, Dict
from dataclasses import dataclass


def make_step(tmp_path: Path) -> Path:
    doc = TDocStd_Document(TCollection_ExtendedString("step"))
    # In OCP, many static methods require _s suffix
    try:
        shape_tool = XCAFDoc_DocumentTool.ShapeTool_s(doc.Main())
    except AttributeError:
        shape_tool = XCAFDoc_DocumentTool.ShapeTool(doc.Main())

    box1 = BRepPrimAPI_MakeBox(1.0, 1.0, 1.0).Shape()
    box2 = BRepPrimAPI_MakeBox(2.0, 1.0, 1.0).Shape()

    label1 = shape_tool.AddShape(box1)
    label2 = shape_tool.AddShape(box2)

    try:
        TDataStd_Name.Set_s(label1, TCollection_ExtendedString("export:pA"))
        TDataStd_Name.Set_s(label2, TCollection_ExtendedString("export:pB"))
    except AttributeError:
        TDataStd_Name.Set(label1, TCollection_ExtendedString("export:pA"))
        TDataStd_Name.Set(label2, TCollection_ExtendedString("export:pB"))

    writer = STEPCAFControl_Writer()
    writer.Transfer(doc, STEPControl_AsIs)
    out = tmp_path / "assembly.step"
    assert writer.Write(str(out)) == 1
    return out


def make_step_without_ids(tmp_path: Path) -> Path:
    doc = TDocStd_Document(TCollection_ExtendedString("step"))
    try:
        shape_tool = XCAFDoc_DocumentTool.ShapeTool_s(doc.Main())
    except AttributeError:
        shape_tool = XCAFDoc_DocumentTool.ShapeTool(doc.Main())

    box1 = BRepPrimAPI_MakeBox(1.0, 1.0, 1.0).Shape()
    box2 = BRepPrimAPI_MakeBox(2.0, 1.0, 1.0).Shape()

    shape_tool.AddShape(box1)
    shape_tool.AddShape(box2)

    writer = STEPCAFControl_Writer()
    writer.Transfer(doc, STEPControl_AsIs)
    out = tmp_path / "assembly.step"
    assert writer.Write(str(out)) == 1
    return out


def test_split_step_to_meshes(tmp_path: Path):
    step_path = make_step(tmp_path)
    link_groups = {"link_ab": ["pA", "pB"]}

    mesh_map = split_step_to_meshes(step_path, link_groups, tmp_path)
    assert (tmp_path / "link_ab.stl").exists()
    assert mesh_map["link_ab"] == "link_ab.stl"


def test_export_step_uses_workspace_id(tmp_path: Path):
    class DummyCad:
        document_id = "doc123"
        wtype = "w"
        workspace_id = "ws456"
        element_id = "elem789"

    class DummyResponse:
        def __init__(self, payload=None, content=b""):
            self._payload = payload or {}
            self.content = content

        def raise_for_status(self):
            return None

        def json(self):
            return self._payload

    class DummyClient:
        def __init__(self):
            self.paths = []
            self.bodies = []

        def request(self, method, path, **kwargs):
            self.paths.append(path)
            if path.endswith("/translations"):
                self.bodies.append(kwargs.get("body"))
                return DummyResponse({"id": "t1"})
            if path.startswith("/api/translations/"):
                return DummyResponse(
                    {"requestState": "DONE", "resultExternalDataIds": ["f1"]}
                )
            return DummyResponse({}, content=b"STEP")

    exporter = StepMeshExporter(DummyClient(), DummyCad())
    output_path = tmp_path / "assembly.step"
    exporter.export_step(output_path)

    assert exporter.client.paths
    assert f"/{DummyCad.wtype}/{DummyCad.workspace_id}/" in exporter.client.paths[0]
    assert exporter.client.paths[0].endswith("/translations")
    assert exporter.client.bodies
    assert exporter.client.bodies[0]["formatName"] == "STEP"
    assert exporter.client.bodies[0]["stepVersionString"] == "AP242"
    assert exporter.client.bodies[0]["storeInDocument"] is False
    assert exporter.client.bodies[0]["includeExportIds"] is True
    assert exporter.client.bodies[0]["extractAssemblyHierarchy"] is True
    assert exporter.client.bodies[0]["flattenAssemblies"] is False


def test_export_step_extracts_step_from_zip(tmp_path: Path):
    class DummyCad:
        document_id = "doc123"
        wtype = "w"
        workspace_id = "ws456"
        element_id = "elem789"

    class DummyResponse:
        def __init__(self, payload=None, content=b""):
            self._payload = payload or {}
            self.content = content

        def raise_for_status(self):
            return None

        def json(self):
            return self._payload

    class DummyClient:
        def __init__(self):
            self.paths = []
            self.bodies = []

        def request(self, method, path, **kwargs):
            self.paths.append(path)
            if path.endswith("/translations"):
                self.bodies.append(kwargs.get("body"))
                return DummyResponse({"id": "t1"})
            if path.startswith("/api/translations/"):
                return DummyResponse(
                    {
                        "requestState": "DONE",
                        "resultExternalDataIds": ["f_manifest", "f_zip"],
                    }
                )

            if path.endswith("/externaldata/f_manifest"):
                return DummyResponse(content=b'<?xml version="1.0"?><manifest />')

            if path.endswith("/externaldata/f_zip"):
                import io
                import zipfile

                buf = io.BytesIO()
                with zipfile.ZipFile(buf, "w", zipfile.ZIP_DEFLATED) as zf:
                    zf.writestr("assembly.step", "ISO-10303-21;\nEND-ISO-10303-21;")
                return DummyResponse(content=buf.getvalue())

            return DummyResponse({}, content=b"")

    exporter = StepMeshExporter(DummyClient(), DummyCad())
    output_path = tmp_path / "assembly.step"
    exporter.export_step(output_path)

    contents = output_path.read_text()
    assert contents.startswith("ISO-10303-21")


def test_export_step_uses_translation_download_when_only_xml(tmp_path: Path):
    class DummyCad:
        document_id = "doc123"
        wtype = "w"
        workspace_id = "ws456"
        element_id = "elem789"

    class DummyResponse:
        def __init__(self, payload=None, content=b""):
            self._payload = payload or {}
            self.content = content

        def raise_for_status(self):
            return None

        def json(self):
            return self._payload

    class DummyClient:
        def __init__(self):
            self.paths = []

        def request(self, method, path, **kwargs):
            self.paths.append(path)
            if path.endswith("/translations"):
                return DummyResponse({"id": "t1"})
            if path.startswith("/api/translations/") and path.endswith("/download"):
                import io
                import zipfile

                buf = io.BytesIO()
                with zipfile.ZipFile(buf, "w", zipfile.ZIP_DEFLATED) as zf:
                    zf.writestr("assembly.step", "ISO-10303-21;\nEND-ISO-10303-21;")
                return DummyResponse(content=buf.getvalue())
            if path.startswith("/api/translations/"):
                return DummyResponse(
                    {"requestState": "DONE", "resultExternalDataIds": ["f_xml"]}
                )
            if path.endswith("/externaldata/f_xml"):
                return DummyResponse(content=b'<?xml version="1.0"?><TreeNode />')
            return DummyResponse({}, content=b"")

    exporter = StepMeshExporter(DummyClient(), DummyCad())
    output_path = tmp_path / "assembly.step"
    exporter.export_step(output_path)

    contents = output_path.read_text()
    assert contents.startswith("ISO-10303-21")


def test_is_step_payload_detects_invalid_header():
    assert _is_step_payload(b'<?xml version="1.0"?><TreeNode />') is False
    assert _is_step_payload(b"ISO-10303-21;\nEND-ISO-10303-21;") is True


@dataclass
class DummyPart:
    partId: str
    name: str = "Part"
    isRigidAssembly: bool = False


@dataclass
class DummyInstance:
    name: str


class DummyCadMapping:
    def __init__(
        self, parts: Dict[Any, DummyPart], instances: Dict[Any, DummyInstance]
    ):
        self.parts = parts
        self.instances = instances
        self.document_id = "doc"
        self.wtype = "w"
        self.workspace_id = "ws"
        self.element_id = "elem"


def test_step_mapping_with_gaps(tmp_path: Path):
    # Setup CAD with gaps in instance indices: <1>, <3>
    parts = {
        "key1": DummyPart(partId="p1", name="A"),
        "key3": DummyPart(partId="p1", name="A"),
    }
    instances = {
        "key1": DummyInstance(name="A <1>"),
        "key3": DummyInstance(name="A <3>"),
    }
    cad = DummyCadMapping(parts, instances)

    from onshape2xacro.mesh_exporters.step import _map_keys_to_export_names

    mapping = _map_keys_to_export_names(cad)

    # Expected:
    # key1 (index 1) -> "A"
    # key3 (index 3) -> "A (1)" because it's the second instance of "A"
    assert mapping["key1"] == "A"
    assert mapping["key3"] == "A (1)"


def test_export_link_meshes_integration(tmp_path: Path):
    # This tests the full flow with a mocked zip response
    parts = {
        "k1": DummyPart(partId="p1", name="PartA"),
        "k2": DummyPart(partId="p2", name="PartB"),
    }
    instances = {
        "k1": DummyInstance(name="PartA <1>"),
        "k2": DummyInstance(name="PartB <1>"),
    }
    cad = DummyCadMapping(parts, instances)

    # Create a dummy assembly.zip with STEP files
    zip_path = tmp_path / "meshes" / "assembly.zip"
    zip_path.parent.mkdir(parents=True, exist_ok=True)

    with zipfile.ZipFile(zip_path, "w"):
        pass

    class DummyResponse:
        def raise_for_status(self):
            pass

        def json(self):
            return {"id": "t1", "requestState": "DONE", "resultExternalDataIds": ["f1"]}

    class DummyClient:
        def request(self, *args, **kwargs):
            if "/externaldata/" in args[1]:
                import io

                buf = io.BytesIO()
                with zipfile.ZipFile(buf, "w"):
                    pass
                return type(
                    "Obj",
                    (),
                    {"content": buf.getvalue(), "raise_for_status": lambda: None},
                )
            return DummyResponse()

    exporter = StepMeshExporter(DummyClient(), cad)

    import onshape2xacro.mesh_exporters.step as step_mod

    def mock_load_zip(path):
        from OCP.BRepPrimAPI import BRepPrimAPI_MakeBox

        c1 = BRepPrimAPI_MakeBox(1.0, 1.0, 1.0).Shape()
        c2 = BRepPrimAPI_MakeBox(1.0, 1.0, 1.0).Shape()
        return {"PartA": c1, "PartB": c2}

    original_load = step_mod._load_step_shapes_from_zip
    step_mod._load_step_shapes_from_zip = mock_load_zip

    try:
        link_groups = {"link1": ["k1", "k2"]}
        mesh_map, missing = exporter.export_link_meshes(
            link_groups, tmp_path / "meshes"
        )

        assert "link1" in mesh_map
        assert mesh_map["link1"] == "link1.stl"
        assert (tmp_path / "meshes" / "link1.stl").exists()
    finally:
        step_mod._load_step_shapes_from_zip = original_load


def test_export_link_meshes_reexports_when_missing_ids(tmp_path: Path):
    # Initial STEP file without export IDs
    no_id_step = make_step_without_ids(tmp_path)

    class DummyCad:
        def __init__(self):
            self.parts = {
                "k1": DummyPart(partId="pA", name="PartA"),
                "k2": DummyPart(partId="pB", name="PartB"),
            }
            self.instances = {}
            self.document_id = "doc"
            self.wtype = "w"
            self.workspace_id = "ws"
            self.element_id = "elem"

    class DummyLink:
        def __init__(self, keys):
            self.keys = keys
            self.frame_transform = None

    class DummyExporter(StepMeshExporter):
        def __init__(self, cad: DummyCad, asset_path: Path):
            super().__init__(client=None, cad=cad, asset_path=asset_path)
            self.export_called = False

        def export_step(self, output_path: Path):
            self.export_called = True
            # Re-export with IDs so parsing can succeed
            make_step(output_path.parent)

    exporter = DummyExporter(DummyCad(), no_id_step)
    link_records = {"link": DummyLink(["k1", "k2"])}

    mesh_map, missing = exporter.export_link_meshes(link_records, tmp_path)

    assert exporter.export_called is True
    assert "link" in mesh_map
    assert missing == {}


def test_export_step_raises_when_only_xml_everywhere(tmp_path: Path):
    class DummyCad:
        document_id = "doc123"
        wtype = "w"
        workspace_id = "ws456"
        element_id = "elem789"

    class DummyResponse:
        def __init__(self, payload=None, content=b""):
            self._payload = payload or {}
            self.content = content

        def raise_for_status(self):
            return None

        def json(self):
            return self._payload

    class DummyClient:
        def __init__(self):
            self.paths = []

        def request(self, method, path, **kwargs):
            self.paths.append(path)
            if path.endswith("/translations"):
                return DummyResponse({"id": "t1"})
            if path.startswith("/api/translations/") and path.endswith("/download"):
                return DummyResponse(content=b'<?xml version="1.0"?><TreeNode />')
            if path.startswith("/api/translations/"):
                return DummyResponse(
                    {"requestState": "DONE", "resultExternalDataIds": ["f_xml"]}
                )
            if path.endswith("/externaldata/f_xml"):
                return DummyResponse(content=b'<?xml version="1.0"?><TreeNode />')
            return DummyResponse({}, content=b"")

    exporter = StepMeshExporter(DummyClient(), DummyCad())
    import pytest

    with pytest.raises(RuntimeError, match="No STEP content"):
        exporter.export_step(tmp_path / "assembly.step")
