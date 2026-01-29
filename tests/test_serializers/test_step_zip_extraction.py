import sys
from unittest.mock import MagicMock

# Mock dependencies to allow importing StepMeshExporter
m = MagicMock()
sys.modules["onshape_robotics_toolkit"] = m
sys.modules["onshape_robotics_toolkit.connect"] = m
sys.modules["onshape_robotics_toolkit.robot"] = m
sys.modules["onshape_robotics_toolkit.models"] = m
sys.modules["onshape_robotics_toolkit.models.link"] = m
sys.modules["onshape_robotics_toolkit.models.joint"] = m
sys.modules["onshape_robotics_toolkit.models.assembly"] = m
sys.modules["OCP"] = m
sys.modules["OCP.STEPCAFControl"] = m
sys.modules["OCP.TDocStd"] = m
sys.modules["OCP.TCollection"] = m
sys.modules["OCP.XCAFDoc"] = m
sys.modules["OCP.TDataStd"] = m
sys.modules["OCP.TDF"] = m
sys.modules["OCP.IFSelect"] = m
sys.modules["OCP.TopLoc"] = m
sys.modules["OCP.BRep"] = m
sys.modules["OCP.BRepBuilderAPI"] = m
sys.modules["OCP.TopoDS"] = m
sys.modules["OCP.BRepMesh"] = m
sys.modules["OCP.StlAPI"] = m
sys.modules["OCP.gp"] = m

from pathlib import Path  # noqa: E402
import io  # noqa: E402
import zipfile  # noqa: E402
from onshape2xacro.mesh_exporters.step import StepMeshExporter  # noqa: E402


def test_export_step_extracts_from_zip(tmp_path: Path):
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
            pass

        def json(self):
            return self._payload

    # Create a zip containing a STEP file
    zip_buffer = io.BytesIO()
    with zipfile.ZipFile(zip_buffer, "w") as zf:
        zf.writestr("assembly.step", b"MOCK STEP CONTENT")
        zf.writestr("manifest.xml", b"<manifest/>")

    class DummyClient:
        def request(self, method, path, **kwargs):
            if path.endswith("/translations"):
                return DummyResponse({"id": "t1"})
            if path.startswith("/api/translations/"):
                return DummyResponse(
                    {"requestState": "DONE", "resultExternalDataIds": ["f1"]}
                )
            if "/externaldata/" in path:
                return DummyResponse(content=zip_buffer.getvalue())
            return DummyResponse()

    exporter = StepMeshExporter(DummyClient(), DummyCad())
    output_path = tmp_path / "result.step"
    exporter.export_step(output_path)

    with open(output_path, "rb") as f:
        content = f.read()

    assert content == b"MOCK STEP CONTENT"


def test_export_step_detects_step_directly(tmp_path: Path):
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
            pass

        def json(self):
            return self._payload

    class DummyClient:
        def request(self, method, path, **kwargs):
            if path.endswith("/translations"):
                return DummyResponse({"id": "t1"})
            if path.startswith("/api/translations/"):
                return DummyResponse(
                    {"requestState": "DONE", "resultExternalDataIds": ["f1"]}
                )
            if "/externaldata/" in path:
                return DummyResponse(content=b"ISO-10303-21;\nRAW STEP DATA")
            return DummyResponse()

    exporter = StepMeshExporter(DummyClient(), DummyCad())
    output_path = tmp_path / "result.step"
    exporter.export_step(output_path)

    with open(output_path, "rb") as f:
        content = f.read()

    assert content == b"ISO-10303-21;\nRAW STEP DATA"
