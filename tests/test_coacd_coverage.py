import sys
from pathlib import Path
from unittest.mock import MagicMock, patch
from onshape2xacro.config.export_config import CoACDOptions


def test_process_coacd_task(tmp_path):
    from onshape2xacro.mesh_exporters.step import _process_coacd_task

    # Setup
    link_name = "test_link"
    stl_path = tmp_path / "test.stl"
    stl_path.touch()
    options = CoACDOptions()
    mesh_dir = tmp_path / "meshes"
    mesh_dir.mkdir()
    (mesh_dir / "collision").mkdir()

    # Mock coacd and trimesh
    with (
        patch("onshape2xacro.mesh_exporters.step.trimesh") as mock_trimesh,
        patch("onshape2xacro.mesh_exporters.step.coacd") as mock_coacd,
    ):
        # Setup mocks
        mock_mesh = MagicMock()
        mock_mesh.vertices = [1, 2, 3]
        mock_mesh.faces = [4, 5, 6]
        mock_trimesh.load.return_value = mock_mesh

        # coacd.run_coacd returns list of (verts, faces)
        mock_coacd.run_coacd.return_value = [
            ([1, 1, 1], [2, 2, 2]),
            ([3, 3, 3], [4, 4, 4]),
        ]

        # Run
        name, result_files = _process_coacd_task(
            (link_name, stl_path, options, mesh_dir)
        )

        # Verify
        assert name == link_name
        assert len(result_files) == 2
        assert result_files[0] == f"collision/{link_name}_0.stl"
        assert result_files[1] == f"collision/{link_name}_1.stl"

        # Verify coacd was called with correct params
        mock_coacd.run_coacd.assert_called_once()
        _, kwargs = mock_coacd.run_coacd.call_args
        assert kwargs["threshold"] == options.threshold
        assert kwargs["max_convex_hull"] == options.max_convex_hull


def test_process_coacd_task_fallback(tmp_path):
    from onshape2xacro.mesh_exporters.step import _process_coacd_task

    # Setup
    link_name = "test_link"
    stl_path = tmp_path / "test.stl"
    stl_path.touch()
    options = CoACDOptions()
    mesh_dir = tmp_path / "meshes"
    mesh_dir.mkdir()
    (mesh_dir / "collision").mkdir()

    # Mock coacd to fail
    with (
        patch("onshape2xacro.mesh_exporters.step.trimesh") as mock_trimesh,
        patch("onshape2xacro.mesh_exporters.step.coacd"),
    ):
        mock_trimesh.load.side_effect = Exception("Load failed")

        # Run
        name, result_files = _process_coacd_task(
            (link_name, stl_path, options, mesh_dir)
        )

        # Verify fallback
        assert name == link_name
        assert len(result_files) == 1
        assert result_files[0] == f"collision/{link_name}_0.stl"
        # Check that file exists (was copied)
        assert (mesh_dir / result_files[0]).exists()


def test_step_export_with_concurrent_coacd(tmp_path):
    from onshape2xacro.mesh_exporters.step import StepMeshExporter
    from onshape2xacro.config.export_config import CollisionOptions

    # Mock pymeshlab module in sys.modules
    mock_pymeshlab = MagicMock()
    with patch.dict(sys.modules, {"pymeshlab": mock_pymeshlab}):
        # Mocks
        client = MagicMock()
        cad = MagicMock()
        exporter = StepMeshExporter(client, cad)

        # Prepare data
        import numpy as np
        from types import SimpleNamespace

        link_records = {
            "link1": SimpleNamespace(
                keys=["k1"], part_names=["p1"], frame_transform=np.eye(4)
            ),
            "link2": SimpleNamespace(
                keys=["k2"], part_names=["p2"], frame_transform=np.eye(4)
            ),
        }
        mesh_dir = tmp_path / "output"

        # Setup mocks for all the OCP/STL stuff
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
            patch("onshape2xacro.mesh_exporters.step._collect_shapes") as mock_collect,
            patch(
                "onshape2xacro.mesh_exporters.step.ProcessPoolExecutor"
            ) as mock_executor,
            patch("onshape2xacro.mesh_exporters.step.IFSelect_RetDone", new=1),
            patch("onshape2xacro.mesh_exporters.step.BRepBuilderAPI_Transform"),
            patch("onshape2xacro.mesh_exporters.step.gp_Trsf"),
            patch("onshape2xacro.mesh_exporters.step.BRep_Builder"),
            patch("onshape2xacro.mesh_exporters.step.TopoDS_Compound"),
        ):
            # Setup basic mocks to pass file reading
            mock_reader = mock_reader_cls.return_value
            mock_reader.ReadFile.return_value = 1
            mock_reader.Transfer.return_value = True
            mock_labels.return_value.Length.return_value = 1

            # Mock _collect_shapes to populate shapes so the loop continues
            def side_effect_collect(
                shape_tool, color_tool, label, loc, shapes, locations, colors, path=None
            ):
                # Use "k1" to match one of the parts
                shapes["k1"] = [MagicMock()]
                locations["k1"] = [MagicMock()]
                colors["k1"] = [None]
                shapes["k2"] = [MagicMock()]
                locations["k2"] = [MagicMock()]
                colors["k2"] = [None]

            mock_collect.side_effect = side_effect_collect

            # Make writer create the temp stl file
            def side_effect_write(shape, path):
                Path(path).touch()

            mock_writer.return_value.Write.side_effect = side_effect_write

            # Mock BRepMesh_IncrementalMesh to avoid segfaults/errors if called

            # and Ensure temp_stl creation logic passes

            # Run export

            mock_executor_instance = mock_executor.return_value
            mock_executor_instance.__enter__.return_value = mock_executor_instance

            # map should return an iterator of results
            # Result format: (link_name, [collision_files])
            mock_executor_instance.map.return_value = [
                ("link1", ["collision/link1_0.stl", "collision/link1_1.stl"]),
                ("link2", ["collision/link2_0.stl"]),
            ]

            # Setup dummy STEP file
            exporter.asset_path = tmp_path / "assembly.step"
            exporter.asset_path.write_bytes(
                b"ISO-10303-21;\nHEADER;\nENDSEC;\nDATA;\nENDSEC;\nEND-ISO-10303-21;\n"
            )

            # Mock CAD parts
            part_k1 = MagicMock(partId="k1")
            part_k1.isRigidAssembly = False
            part_k1.worldToPartTF.to_tf = np.eye(4)
            part_k2 = MagicMock(partId="k2")
            part_k2.isRigidAssembly = False
            part_k2.worldToPartTF.to_tf = np.eye(4)
            cad.parts = {"k1": part_k1, "k2": part_k2}
            cad.instances = {}
            cad.occurrences = {}

            # Run export

            collision_opts = CollisionOptions(method="coacd")
            mesh_map, _, _ = exporter.export_link_meshes(
                link_records,
                mesh_dir,
                collision_option=collision_opts,
                visual_mesh_format="stl",
            )

            # Verify executor usage
            mock_executor.assert_called_with(
                max_workers=collision_opts.coacd.max_workers
            )
            assert mock_executor_instance.map.called

            # Verify results updated
            assert mesh_map["link1"]["collision"] == [
                "collision/link1_0.stl",
                "collision/link1_1.stl",
            ]
            assert mesh_map["link2"]["collision"] == ["collision/link2_0.stl"]
