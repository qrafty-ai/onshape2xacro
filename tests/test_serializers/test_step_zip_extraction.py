import sys
from unittest.mock import MagicMock

# Save original modules to restore later
_original_modules = {
    "onshape_robotics_toolkit": sys.modules.get("onshape_robotics_toolkit"),
    "onshape_robotics_toolkit.parse": sys.modules.get("onshape_robotics_toolkit.parse"),
    "onshape_robotics_toolkit.connect": sys.modules.get(
        "onshape_robotics_toolkit.connect"
    ),
    "onshape_robotics_toolkit.robot": sys.modules.get("onshape_robotics_toolkit.robot"),
    "onshape_robotics_toolkit.models": sys.modules.get(
        "onshape_robotics_toolkit.models"
    ),
    "onshape_robotics_toolkit.models.link": sys.modules.get(
        "onshape_robotics_toolkit.models.link"
    ),
    "onshape_robotics_toolkit.models.joint": sys.modules.get(
        "onshape_robotics_toolkit.models.joint"
    ),
    "onshape_robotics_toolkit.models.assembly": sys.modules.get(
        "onshape_robotics_toolkit.models.assembly"
    ),
    "OCP": sys.modules.get("OCP"),
    "OCP.STEPControl": sys.modules.get("OCP.STEPControl"),
    "OCP.STEPCAFControl": sys.modules.get("OCP.STEPCAFControl"),
    "OCP.TDocStd": sys.modules.get("OCP.TDocStd"),
    "OCP.TCollection": sys.modules.get("OCP.TCollection"),
    "OCP.XCAFDoc": sys.modules.get("OCP.XCAFDoc"),
    "OCP.TDataStd": sys.modules.get("OCP.TDataStd"),
    "OCP.TDF": sys.modules.get("OCP.TDF"),
    "OCP.IFSelect": sys.modules.get("OCP.IFSelect"),
    "OCP.TopLoc": sys.modules.get("OCP.TopLoc"),
    "OCP.BRep": sys.modules.get("OCP.BRep"),
    "OCP.BRepBuilderAPI": sys.modules.get("OCP.BRepBuilderAPI"),
    "OCP.TopoDS": sys.modules.get("OCP.TopoDS"),
    "OCP.BRepMesh": sys.modules.get("OCP.BRepMesh"),
    "OCP.StlAPI": sys.modules.get("OCP.StlAPI"),
    "OCP.gp": sys.modules.get("OCP.gp"),
}

# Mock dependencies to allow importing StepMeshExporter
m = MagicMock()
for mod in _original_modules:
    sys.modules[mod] = m

from pathlib import Path  # noqa: E402
import io  # noqa: E402
import zipfile  # noqa: E402
from onshape2xacro.mesh_exporters.step import StepMeshExporter  # noqa: E402

for mod, orig in _original_modules.items():
    if orig is None:
        if mod in sys.modules:
            del sys.modules[mod]
    else:
        sys.modules[mod] = orig


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
        zf.writestr("assembly.step", b"ISO-10303-21; MOCK STEP CONTENT")
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

    assert content == b"ISO-10303-21; MOCK STEP CONTENT"


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
