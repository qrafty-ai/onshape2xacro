from unittest.mock import MagicMock, patch, ANY
from pathlib import Path
import numpy as np
from onshape2xacro.mesh_exporters.step import StepMeshExporter, _collect_shapes
from onshape2xacro.config.export_config import VisualMeshOptions


def test_dae_pymeshlab_fallback(tmp_path):
    client = MagicMock()
    cad = MagicMock()
    exporter = StepMeshExporter(client, cad)

    mesh_dir = tmp_path / "meshes"
    mesh_dir.mkdir()

    # Mock everything needed for export_link_meshes
    with (
        patch("onshape2xacro.mesh_exporters.step.BRepMesh_IncrementalMesh"),
        patch("onshape2xacro.mesh_exporters.step.StlAPI_Writer") as mock_writer,
        patch(
            "onshape2xacro.mesh_exporters.step.STEPCAFControl_Reader"
        ) as mock_reader_cls,
        patch("onshape2xacro.mesh_exporters.step.TDocStd_Document"),
        patch("onshape2xacro.mesh_exporters.step._get_shape_tool"),
        patch("onshape2xacro.mesh_exporters.step.XCAFDoc_DocumentTool"),
        patch(
            "onshape2xacro.mesh_exporters.step._get_free_shape_labels"
        ) as mock_labels,
        patch("trimesh.load") as mock_trimesh_load,
        patch("trimesh.util.concatenate") as mock_concat,
        patch("pymeshlab.MeshSet") as mock_mesh_set,
        patch("onshape2xacro.mesh_exporters.step.IFSelect_RetDone", new=1),
        patch("onshape2xacro.mesh_exporters.step.BRepBuilderAPI_Transform"),
        patch("onshape2xacro.mesh_exporters.step.BRep_Builder"),
        patch("onshape2xacro.mesh_exporters.step.TopoDS_Compound"),
    ):
        # Mock writer to create file
        mock_writer.return_value.Write.side_effect = lambda shape, path: Path(
            path
        ).touch()

        # Mock reader
        mock_reader = mock_reader_cls.return_value
        mock_reader.ReadFile.return_value = 1
        mock_reader.Transfer.return_value = True

        # Mock free shapes
        mock_labels.return_value.Length.return_value = 1

        # Setup CAD parts
        cad.parts = {"part_1": MagicMock()}
        cad.parts["part_1"].isRigidAssembly = False
        cad.parts["part_1"].worldToPartTF.to_tf.return_value = np.eye(4)
        cad.parts["part_1"].partId = "part_1"
        cad.instances = {}

        link_record = MagicMock()
        link_record.keys = ["part_1"]
        link_record.part_names = ["part_1"]
        link_record.frame_transform = np.eye(4)
        link_records = {"link1": link_record}

        # Mock trimesh
        mock_mesh = MagicMock()
        mock_trimesh_load.return_value = mock_mesh
        mock_concat.return_value = mock_mesh

        # Create dummy STEP file
        exporter.asset_path = tmp_path / "assembly.step"
        with open(exporter.asset_path, "wb") as f:
            f.write(
                b"ISO-10303-21;\nHEADER;\nENDSEC;\nDATA;\nENDSEC;\nEND-ISO-10303-21;\n"
            )

        # Mock shape collection
        def populate_shapes(
            st, ct, lbl, loc, shapes, locations, colors, path=(), inherited_color=None
        ):
            shapes["part_1"] = [MagicMock()]
            locations["part_1"] = [MagicMock()]
            colors["part_1"] = [(0.5, 0.5, 0.5)]

        with patch(
            "onshape2xacro.mesh_exporters.step._collect_shapes",
            side_effect=populate_shapes,
        ):
            # FORCE PYMESHLAB FAILURE
            ms_instance = mock_mesh_set.return_value
            ms_instance.load_new_mesh.side_effect = Exception("PyMeshLab failed")

            # RUN
            exporter.export_link_meshes(
                link_records, mesh_dir, visual_option=VisualMeshOptions(formats=["dae"])
            )

            # VERIFY fallback
            # Should have called combined.export with dae
            mock_mesh.export.assert_any_call(ANY, file_type="dae")
            # Should have printed error (we can't easily verify print, but code path execution is enough)


def test_collect_shapes_no_reference():
    # Test branch where GetReferredShape returns False
    shape_tool = MagicMock()
    color_tool = MagicMock()
    label = MagicMock()
    parent_loc = MagicMock()

    # Mock assembly structure
    shape_tool.IsAssembly.return_value = True
    shape_tool.IsAssembly_s.return_value = True

    # Mock one component
    comp_label = MagicMock()

    # Mock GetComponents
    def get_components(lbl, seq):
        seq.Length.return_value = 1
        seq.Value.return_value = comp_label

    shape_tool.GetComponents.side_effect = get_components
    shape_tool.GetComponents_s.side_effect = get_components

    # Mock GetReferredShape -> False
    shape_tool.GetReferredShape.return_value = False
    shape_tool.GetReferredShape_s.return_value = False

    part_shapes = {}
    part_locations = {}
    part_colors = {}

    # We need to break recursion to avoid infinite loop since we mock behavior
    # But _collect_shapes calls itself recursively.
    # To test the "else" branch, we need IsAssembly=True, GetComponents gives a comp, and GetReferredShape(comp)=False
    # Then it calls _collect_shapes(..., comp, ...)
    # In the recursive call, we want it to stop.
    # So we need dynamic mocking: First call -> Assembly, Second call -> Leaf

    call_count = 0

    def is_assembly_side_effect(lbl):
        nonlocal call_count
        if call_count == 0:
            call_count += 1
            return True
        return False

    shape_tool.IsAssembly_s.side_effect = is_assembly_side_effect

    # Mock leaf shape for the second call
    shape = MagicMock()
    shape.IsNull.return_value = False
    shape_tool.GetShape_s.return_value = shape

    with (
        patch("onshape2xacro.mesh_exporters.step._iter_components") as mock_iter,
        patch("onshape2xacro.mesh_exporters.step._get_label_location"),
        patch("onshape2xacro.mesh_exporters.step._combine_locations"),
        patch(
            "onshape2xacro.mesh_exporters.step._get_occurrence_id", return_value="occ1"
        ),
        patch("onshape2xacro.mesh_exporters.step._label_name", return_value="name1"),
        patch("onshape2xacro.mesh_exporters.step._get_color", return_value=None),
    ):
        mock_iter.return_value = [comp_label]

        _collect_shapes(
            shape_tool,
            color_tool,
            label,
            parent_loc,
            part_shapes,
            part_locations,
            part_colors,
        )

        # Should have collected the shape from the recursion
        assert "occ1" in part_shapes
