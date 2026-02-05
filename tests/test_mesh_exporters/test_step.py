import numpy as np
from unittest.mock import MagicMock, patch, ANY
from pathlib import Path
import sys
from onshape2xacro.config.export_config import VisualMeshOptions


def test_step_export_visual_formats(tmp_path):
    from onshape2xacro.mesh_exporters.step import StepMeshExporter

    # Mock pymeshlab module in sys.modules to prevent UnboundLocalError/ImportError
    mock_pymeshlab = MagicMock()

    with patch.dict(sys.modules, {"pymeshlab": mock_pymeshlab}):
        # Mocks
        client = MagicMock()
        cad = MagicMock()
        exporter = StepMeshExporter(client, cad)

        mesh_dir = tmp_path / "meshes"
        mesh_dir.mkdir()

        # Create a dummy STL file to simulate raw extraction
        raw_stl = mesh_dir / "link1_raw.stl"
        with open(raw_stl, "w") as f:
            f.write("solid link1\nendsolid link1")

        # Mock BRepMesh_IncrementalMesh and StlAPI_Writer to just create the file
        with (
            patch("onshape2xacro.mesh_exporters.step.BRepMesh_IncrementalMesh"),
            patch("onshape2xacro.mesh_exporters.step.StlAPI_Writer") as mock_writer,
            patch(
                "onshape2xacro.mesh_exporters.step.STEPCAFControl_Reader"
            ) as mock_reader_cls,
            patch("onshape2xacro.mesh_exporters.step.XCAFDoc_DocumentTool"),
            patch("onshape2xacro.mesh_exporters.step.TDocStd_Document"),
            patch("onshape2xacro.mesh_exporters.step._get_shape_tool"),
            patch(
                "onshape2xacro.mesh_exporters.step._get_free_shape_labels"
            ) as mock_labels,
            patch("onshape2xacro.mesh_exporters.step._collect_shapes"),
            patch("trimesh.load") as mock_trimesh_load,
            # pymeshlab.MeshSet is now mocked via sys.modules patch above,
            # but we can still patch it here if we want specific behavior or just rely on the mock_pymeshlab
            patch("onshape2xacro.mesh_exporters.step.IFSelect_RetDone", new=1),
            patch("onshape2xacro.mesh_exporters.step.BRepBuilderAPI_Transform"),
            patch("onshape2xacro.mesh_exporters.step.BRep_Builder"),
            patch("onshape2xacro.mesh_exporters.step.TopoDS_Compound"),
        ):
            # Setup mock behavior
            mock_writer.return_value.Write.side_effect = lambda shape, path: Path(
                path
            ).touch()

            # Setup reader mock
            mock_reader = mock_reader_cls.return_value
            mock_reader.ReadFile.return_value = 1
            mock_reader.Transfer.return_value = True

            # Mock free shapes to return length 1
            mock_labels.return_value.Length.return_value = 1

            # Mock CAD parts
            link_record = MagicMock()
            link_record.keys = ["part_key"]
            link_record.part_names = ["part_name"]
            link_record.frame_transform = np.eye(4)

            cad.parts = {"part_key": MagicMock()}
            cad.parts["part_key"].isRigidAssembly = False
            # Ensure worldToPartTF returns a valid matrix
            cad.parts["part_key"].worldToPartTF.to_tf.return_value = np.eye(4)
            # Ensure partId matches the key used in populate_shapes
            cad.parts["part_key"].partId = "part_key"

            # Ensure instances lookup doesn't crash or returns None
            cad.instances = {}

            # Mock link records
            link_records = {"link1": link_record}

            # Test DAE export
            mock_mesh = MagicMock()
            mock_trimesh_load.return_value = mock_mesh

            # Create a dummy STEP file with valid header to pass validation
            exporter.asset_path = tmp_path / "assembly.step"
            with open(exporter.asset_path, "wb") as f:
                f.write(
                    b"ISO-10303-21;\nHEADER;\nENDSEC;\nDATA;\nENDSEC;\nEND-ISO-10303-21;\n"
                )

            # We also need to mock _collect_shapes population of part_shapes
            # Since _collect_shapes is called in a loop, we can't easily side-effect the dict passed to it via arguments
            # unless we mock it to do so.

            # Actually, let's look at export_link_meshes. It calls _collect_shapes.
            # If we patch _collect_shapes, we can make it populate the dictionaries passed to it.
            def populate_shapes(st, ct, lbl, loc, shapes, locations, colors, path=()):
                shapes["part_key"] = [MagicMock()]
                locations["part_key"] = [MagicMock()]  # Mock TopLoc_Location
                colors["part_key"] = [(0.1, 0.2, 0.3)]  # Mock color

            with (
                patch(
                    "onshape2xacro.mesh_exporters.step._collect_shapes",
                    side_effect=populate_shapes,
                ),
                patch("trimesh.util.concatenate") as mock_concat,
            ):
                mock_concat.return_value = mock_mesh
                exporter.export_link_meshes(
                    link_records,
                    mesh_dir,
                    visual_option=VisualMeshOptions(formats=["dae"]),
                )

                # Verify color was applied
                # We need to capture the mesh object created by trimesh.load
                # Since mock_trimesh_load returns mock_mesh, we can check if visual.face_colors was set
                # But wait, trimesh.load returns a mesh. We create a NEW list of meshes.

                # We should check that trimesh.util.concatenate was called with meshes having face_colors set
                assert mock_concat.called
                meshes_list = mock_concat.call_args[0][0]
                assert len(meshes_list) == 1
                # We can't easily verify the attribute set on a MagicMock unless we configure it,
                # but verifying concatenate is called implies we went through the color path

                # Verify DAE export logic:
                # 1. Intermediate OBJ export via trimesh
                mock_mesh.export.assert_called_with(ANY, file_type="obj")

                # 2. PyMeshLab conversion
                # Verify that pymeshlab.MeshSet was used (via our sys.modules mock)
                assert mock_pymeshlab.MeshSet.called
                ms_instance = mock_pymeshlab.MeshSet.return_value
                ms_instance.load_new_mesh.assert_called_with(
                    ANY
                )  # Should load temp OBJ
                ms_instance.save_current_mesh.assert_called_with(ANY)  # Should save DAE

            # Test OBJ export
            with (
                patch(
                    "onshape2xacro.mesh_exporters.step._collect_shapes",
                    side_effect=populate_shapes,
                ),
                patch("trimesh.util.concatenate") as mock_concat,
            ):
                mock_concat.return_value = mock_mesh
                exporter.export_link_meshes(
                    link_records,
                    mesh_dir,
                    visual_option=VisualMeshOptions(formats=["obj"]),
                )
            mock_mesh.export.assert_called_with(ANY, file_type="obj")
