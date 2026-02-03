from pathlib import Path
import numpy as np
import trimesh
from onshape2xacro.mesh_exporters.step import StepMeshExporter

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

    # Create a simple L-shape or box that is concave if possible to test decomposition?
    # A simple box is convex, so CoACD might just return 1 hull.
    # That's fine for integration testing.
    box1 = BRepPrimAPI_MakeBox(10.0, 10.0, 10.0).Shape()  # 10mm box

    label1 = shape_tool.AddShape(box1)

    try:
        TDataStd_Name.Set_s(label1, TCollection_ExtendedString("export:part1"))
    except AttributeError:
        TDataStd_Name.Set(label1, TCollection_ExtendedString("export:part1"))

    writer = STEPCAFControl_Writer()
    writer.Transfer(doc, STEPControl_AsIs)
    out = tmp_path / "assembly.step"
    assert writer.Write(str(out)) == 1
    return out


class DummyPart:
    def __init__(self, pid):
        self.partId = pid
        self.name = pid
        self.isRigidAssembly = False
        self.worldToPartTF = np.eye(4)  # Identity


class DummyLink:
    def __init__(self, keys):
        self.keys = keys
        self.frame_transform = np.eye(4)  # Identity
        # Fix: handle keys that might be strings or objects
        self.part_names = [getattr(k, "partId", str(k)) for k in keys]


class Key:
    def __init__(self, pid):
        self.partId = pid
        self.path = (pid,)


class DummyCad:
    def __init__(self):
        self.parts = {}
        # We will populate parts dynamically
        self.occurrences = {}
        self.instances = {}
        self.document_id = "d"
        self.wtype = "w"
        self.workspace_id = "w"
        self.element_id = "e"


def test_coacd_integration(tmp_path):
    # 1. Generate STEP file
    step_path = make_step(tmp_path)

    # 2. Setup Exporter
    cad = DummyCad()

    # Create key and part
    key1 = Key("part1")
    cad.parts[key1] = DummyPart("part1")

    exporter = StepMeshExporter(None, cad, asset_path=step_path)

    link_records = {"link1": DummyLink([key1])}

    mesh_dir = tmp_path / "meshes"

    # 4. Run Export
    mesh_map, missing, report = exporter.export_link_meshes(
        link_records, mesh_dir, visual_mesh_format="stl"
    )

    # 5. Verify
    print("Mesh Map:", mesh_map)
    print("Missing:", missing)

    assert "link1" in mesh_map
    entry = mesh_map["link1"]

    # Check visual
    assert entry["visual"] == "visual/link1.stl"
    assert (mesh_dir / "visual/link1.stl").exists()

    # Check collision
    collision = entry["collision"]
    # It should be a list of strings
    assert isinstance(collision, list)
    assert len(collision) >= 1
    for p in collision:
        assert p.startswith("collision/link1_")
        assert p.endswith(".stl")
        assert (mesh_dir / p).exists()

    # Verify content of collision mesh
    # Load one and check it's valid
    c_mesh = trimesh.load(str(mesh_dir / collision[0]))
    assert isinstance(c_mesh, trimesh.Trimesh)
    assert len(c_mesh.vertices) > 0


if __name__ == "__main__":
    # Allow running directly
    import shutil

    p = Path("tmp_test_coacd")
    if p.exists():
        shutil.rmtree(p)
    p.mkdir()
    try:
        test_coacd_integration(p)
        print("Test passed!")
    finally:
        pass  # shutil.rmtree(p)
