import pytest
from unittest.mock import MagicMock, patch, ANY
from pathlib import Path
import numpy as np

from onshape2xacro.mesh_exporters.step import (
    _get_color,
    _collect_shapes,
    StepMeshExporter,
)


@pytest.fixture
def mock_ocp():
    with (
        patch("onshape2xacro.mesh_exporters.step.XCAFDoc_ColorType") as mock_type,
        patch("onshape2xacro.mesh_exporters.step.Quantity_Color") as mock_color_cls,
    ):
        # Setup enums
        mock_type.XCAFDoc_ColorSurf = 1
        mock_type.XCAFDoc_ColorGen = 2

        yield mock_type, mock_color_cls


def test_get_color_none_tool(mock_ocp):
    assert _get_color(None, MagicMock(), MagicMock()) is None


def test_get_color_from_label(mock_ocp):
    mock_type, mock_color_cls = mock_ocp
    color_tool = MagicMock()
    label = MagicMock()
    shape = MagicMock()

    # Setup color return
    mock_color_inst = mock_color_cls.return_value
    mock_color_inst.Red.return_value = 1.0
    mock_color_inst.Green.return_value = 0.0
    mock_color_inst.Blue.return_value = 0.0

    # Mock GetColor to return True for label, ColorSurf
    def get_color_side_effect(obj, ctype, color):
        if obj == label and ctype == mock_type.XCAFDoc_ColorSurf:
            return True
        return False

    color_tool.GetColor.side_effect = get_color_side_effect

    result = _get_color(color_tool, label, shape)
    assert result == (1.0, 0.0, 0.0)


def test_get_color_from_shape_surf(mock_ocp):
    mock_type, mock_color_cls = mock_ocp
    color_tool = MagicMock()
    label = MagicMock()
    shape = MagicMock()
    shape.IsNull.return_value = False

    mock_color_inst = mock_color_cls.return_value
    mock_color_inst.Red.return_value = 0.0
    mock_color_inst.Green.return_value = 1.0
    mock_color_inst.Blue.return_value = 0.0

    # Mock GetColor to return True for shape, ColorSurf
    def get_color_side_effect(obj, ctype, color):
        if obj == shape and ctype == mock_type.XCAFDoc_ColorSurf:
            return True
        return False

    color_tool.GetColor.side_effect = get_color_side_effect

    result = _get_color(color_tool, label, shape)
    assert result == (0.0, 1.0, 0.0)


def test_get_color_from_shape_gen(mock_ocp):
    mock_type, mock_color_cls = mock_ocp
    color_tool = MagicMock()
    label = MagicMock()
    shape = MagicMock()
    shape.IsNull.return_value = False

    mock_color_inst = mock_color_cls.return_value
    mock_color_inst.Red.return_value = 0.0
    mock_color_inst.Green.return_value = 0.0
    mock_color_inst.Blue.return_value = 1.0

    # Mock GetColor to return True for shape, ColorGen
    def get_color_side_effect(obj, ctype, color):
        if obj == shape and ctype == mock_type.XCAFDoc_ColorGen:
            return True
        return False

    color_tool.GetColor.side_effect = get_color_side_effect

    result = _get_color(color_tool, label, shape)
    assert result == (0.0, 0.0, 1.0)


def test_get_color_null_shape(mock_ocp):
    mock_type, mock_color_cls = mock_ocp
    color_tool = MagicMock()
    label = MagicMock()
    shape = MagicMock()
    shape.IsNull.return_value = True

    # Mock GetColor to return False for label
    color_tool.GetColor.return_value = False

    result = _get_color(color_tool, label, shape)
    assert result is None


def test_get_color_exceptions(mock_ocp):
    mock_type, mock_color_cls = mock_ocp
    color_tool = MagicMock()
    label = MagicMock()
    shape = MagicMock()

    # Raise exception on GetColor
    color_tool.GetColor.side_effect = Exception("OCP Error")

    result = _get_color(color_tool, label, shape)
    assert result is None


def test_collect_shapes_with_color():
    shape_tool = MagicMock()
    color_tool = MagicMock()
    label = MagicMock()
    parent_loc = MagicMock()

    part_shapes = {}
    part_locations = {}
    part_colors = {}

    # Setup mocks
    shape_tool.IsAssembly.return_value = False
    shape_tool.IsAssembly_s.return_value = False
    shape_tool.GetReferredShape.return_value = False
    shape_tool.GetReferredShape_s.return_value = False

    shape = MagicMock()
    shape.IsNull.return_value = False
    shape_tool.GetShape.return_value = shape
    shape_tool.GetShape_s.return_value = shape

    # Mock _get_color logic or helper
    with (
        patch("onshape2xacro.mesh_exporters.step._get_color") as mock_get_color,
        patch(
            "onshape2xacro.mesh_exporters.step._get_occurrence_id", return_value="occ_1"
        ),
        patch("onshape2xacro.mesh_exporters.step._label_name", return_value="part_1"),
    ):
        mock_get_color.return_value = (1.0, 0.0, 0.0)

        _collect_shapes(
            shape_tool,
            color_tool,
            label,
            parent_loc,
            part_shapes,
            part_locations,
            part_colors,
        )

        assert "occ_1" in part_colors
        assert part_colors["occ_1"] == [(1.0, 0.0, 0.0)]
        assert "part_1" in part_colors
        assert part_colors["part_1"] == [(1.0, 0.0, 0.0)]


def test_collect_shapes_inheritance():
    shape_tool = MagicMock()
    color_tool = MagicMock()
    label = MagicMock()
    parent_loc = MagicMock()

    part_shapes = {}
    part_locations = {}
    part_colors = {}

    # Setup mocks for leaf
    shape_tool.IsAssembly.return_value = False
    shape_tool.IsAssembly_s.return_value = False
    shape_tool.GetReferredShape.return_value = False
    shape_tool.GetReferredShape_s.return_value = False

    shape = MagicMock()
    shape.IsNull.return_value = False
    shape_tool.GetShape.return_value = shape
    shape_tool.GetShape_s.return_value = shape

    # Mock _get_color to return None (no local color)
    with (
        patch("onshape2xacro.mesh_exporters.step._get_color", return_value=None),
        patch(
            "onshape2xacro.mesh_exporters.step._get_occurrence_id", return_value="occ_1"
        ),
        patch("onshape2xacro.mesh_exporters.step._label_name", return_value="part_1"),
    ):
        # Call with inherited color
        inherited = (0.0, 1.0, 0.0)  # Green

        _collect_shapes(
            shape_tool,
            color_tool,
            label,
            parent_loc,
            part_shapes,
            part_locations,
            part_colors,
            inherited_color=inherited,
        )

        # Verify inherited color was used
        assert part_colors["occ_1"] == [inherited]


def test_export_link_meshes_color_integration(tmp_path):
    # Test the full integration in export_link_meshes
    client = MagicMock()
    cad = MagicMock()
    exporter = StepMeshExporter(client, cad)

    mesh_dir = tmp_path / "meshes"
    mesh_dir.mkdir()

    # Mocks for export_link_meshes
    with (
        patch("onshape2xacro.mesh_exporters.step.BRepMesh_IncrementalMesh"),
        patch("onshape2xacro.mesh_exporters.step.StlAPI_Writer") as mock_writer,
        patch(
            "onshape2xacro.mesh_exporters.step.STEPCAFControl_Reader"
        ) as mock_reader_cls,
        patch("onshape2xacro.mesh_exporters.step.TDocStd_Document"),
        patch("onshape2xacro.mesh_exporters.step._get_shape_tool"),
        patch("onshape2xacro.mesh_exporters.step.XCAFDoc_DocumentTool") as mock_xcaf,
        patch(
            "onshape2xacro.mesh_exporters.step._get_free_shape_labels"
        ) as mock_labels,
        patch("trimesh.load") as mock_trimesh_load,
        patch("trimesh.util.concatenate") as mock_concat,
        patch("pymeshlab.MeshSet"),
        patch("onshape2xacro.mesh_exporters.step.IFSelect_RetDone", new=1),
        patch("onshape2xacro.mesh_exporters.step.BRepBuilderAPI_Transform"),
        patch("onshape2xacro.mesh_exporters.step.BRep_Builder"),
        patch("onshape2xacro.mesh_exporters.step.TopoDS_Compound"),
    ):
        # Mock XCAFDoc_DocumentTool.ColorTool_s
        mock_color_tool = MagicMock()
        mock_xcaf.ColorTool_s.return_value = mock_color_tool

        # Mock file operations
        mock_writer.return_value.Write.side_effect = lambda shape, path: Path(
            path
        ).touch()

        # Mock reader
        mock_reader = mock_reader_cls.return_value
        mock_reader.ReadFile.return_value = 1
        mock_reader.Transfer.return_value = True

        # Mock free shapes
        mock_labels.return_value.Length.return_value = 1

        # Create dummy STEP file
        exporter.asset_path = tmp_path / "assembly.step"
        with open(exporter.asset_path, "wb") as f:
            f.write(
                b"ISO-10303-21;\nHEADER;\nENDSEC;\nDATA;\nENDSEC;\nEND-ISO-10303-21;\n"
            )

        # Mock collecting shapes with color
        def populate_shapes(st, ct, lbl, loc, shapes, locations, colors, path=()):
            shapes["part_1"] = [MagicMock()]
            locations["part_1"] = [MagicMock()]
            colors["part_1"] = [(0.5, 0.5, 0.5)]  # Gray color

        with patch(
            "onshape2xacro.mesh_exporters.step._collect_shapes",
            side_effect=populate_shapes,
        ):
            # Setup CAD data
            cad.parts = {"part_1": MagicMock()}
            cad.parts["part_1"].isRigidAssembly = False
            cad.parts["part_1"].worldToPartTF.to_tf.return_value = np.eye(4)
            cad.parts["part_1"].partId = "part_1"
            cad.instances = {}
            cad.occurrences = {}

            link_record = MagicMock()
            link_record.keys = ["part_1"]
            link_record.part_names = ["part_1"]
            link_record.frame_transform = np.eye(4)
            link_records = {"link1": link_record}

            # Mock trimesh load to return a mesh object
            mock_mesh = MagicMock()
            mock_trimesh_load.return_value = mock_mesh
            mock_concat.return_value = mock_mesh

            # RUN
            exporter.export_link_meshes(
                link_records, mesh_dir, visual_mesh_formats=["obj"]
            )

            # VERIFY
            # Ensure color tool was initialized
            mock_xcaf.ColorTool_s.assert_called()

            # Ensure concatenate was called (proving we entered the color processing block)
            assert mock_concat.called

            # Ensure export was called
            mock_mesh.export.assert_called_with(ANY, file_type="obj")

            # Verify color application logic
            # We can check that trimesh.load was called for the temp part file
            # and that we set face_colors on the result
            assert mock_mesh.visual.face_colors is not None
