from pathlib import Path
from onshape2xacro.mesh_exporters.step import split_step_to_meshes
from OCP.BRepPrimAPI import BRepPrimAPI_MakeBox
from OCP.STEPCAFControl import STEPCAFControl_Writer
from OCP.TDocStd import TDocStd_Document
from OCP.TCollection import TCollection_ExtendedString
from OCP.XCAFDoc import XCAFDoc_DocumentTool
from OCP.TDataStd import TDataStd_Name
from OCP.STEPControl import STEPControl_AsIs


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


def test_split_step_to_meshes(tmp_path: Path):
    step_path = make_step(tmp_path)
    link_groups = {"link_ab": ["pA", "pB"]}

    mesh_map = split_step_to_meshes(step_path, link_groups, tmp_path)
    assert (tmp_path / "link_ab.stl").exists()
    assert mesh_map["link_ab"] == "link_ab.stl"
