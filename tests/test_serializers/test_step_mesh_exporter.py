from pathlib import Path

from onshape2xacro.mesh_exporters.step import (
    StepMeshExporter,
    _is_step_payload,
)

# OCP imports for mock data generation
from OCP.BRepPrimAPI import BRepPrimAPI_MakeBox
from OCP.STEPCAFControl import STEPCAFControl_Writer
from OCP.TDocStd import TDocStd_Document
from OCP.TCollection import TCollection_ExtendedString
from OCP.XCAFDoc import XCAFDoc_DocumentTool
from OCP.TDataStd import TDataStd_Name
from OCP.STEPControl import STEPControl_AsIs


def make_step(tmp_path: Path) -> Path:
    doc = TDocStd_Document(TCollection_ExtendedString("step"))
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


class DummyResponse:
    def __init__(self, payload=None, content=b""):
        self._payload = payload or {}
        self.content = content

    def raise_for_status(self):
        return None

    def json(self):
        return self._payload


def test_export_step_uses_workspace_id(tmp_path: Path, monkeypatch):
    import time

    monkeypatch.setattr(time, "sleep", lambda x: None)

    class DummyCad:
        document_id = "doc123"
        wtype = "w"
        workspace_id = "ws456"
        element_id = "elem789"

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
            return DummyResponse({}, content=b"ISO-10303-21")

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
    assert exporter.client.bodies[0]["extractAssemblyHierarchy"] is False
    assert exporter.client.bodies[0]["flattenAssemblies"] is False


def test_export_step_uses_translation_download_when_only_xml(
    tmp_path: Path, monkeypatch
):
    import time

    monkeypatch.setattr(time, "sleep", lambda x: None)

    class DummyCad:
        document_id = "doc123"
        wtype = "w"
        workspace_id = "ws456"
        element_id = "elem789"

    class DummyClient:
        def __init__(self):
            self.paths = []
            self.bodies = []

        def request(self, method, path, **kwargs):
            self.paths.append(path)
            if path.endswith("/translations"):
                self.bodies.append(kwargs.get("body"))
                return DummyResponse({"id": "t1"})
            if path.startswith("/api/translations/") and path.endswith("/download"):
                return DummyResponse(content=b"ISO-10303-21;\nEND-ISO-10303-21;")
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
