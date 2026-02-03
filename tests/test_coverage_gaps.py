import sys
import pytest
from unittest.mock import MagicMock, patch, ANY
from pathlib import Path
from onshape2xacro.cli import parse_args
from onshape2xacro.pipeline import _get_client_and_cad, run_fetch_cad
from onshape2xacro.config.export_config import FetchCadConfig

# --- CLI Tests ---


def test_cli_version(monkeypatch, capsys):
    monkeypatch.setattr(sys, "argv", ["onshape2xacro", "--version"])
    with pytest.raises(SystemExit) as exc:
        parse_args()
    assert exc.value.code == 0
    captured = capsys.readouterr()
    assert "onshape2xacro" in captured.out


# --- Pipeline Tests ---


def test_pipeline_credentials_error(monkeypatch):
    monkeypatch.setattr("onshape2xacro.pipeline.get_credentials", lambda: (None, None))
    monkeypatch.setattr("os.environ", {})

    with pytest.raises(ValueError, match="Onshape credentials not found"):
        _get_client_and_cad("http://url", 5)


def test_fetch_cad_existing_config_and_bom_warning(monkeypatch, tmp_path):
    # Setup
    output_dir = tmp_path / "output"
    output_dir.mkdir()
    (output_dir / "configuration.yaml").touch()

    config = FetchCadConfig(
        url="http://url", output=output_dir, max_depth=5, bom=Path("missing_bom.csv")
    )

    # Mocks
    mock_client = MagicMock()
    mock_cad = MagicMock()
    mock_cad.name = "robot"

    monkeypatch.setattr(
        "onshape2xacro.pipeline._get_client_and_cad",
        lambda u, d: (mock_client, mock_cad),
    )
    monkeypatch.setattr(
        "onshape2xacro.optimized_cad.OptimizedCAD.from_url",
        lambda *args, **kwargs: mock_cad,
    )

    # Mock StepMeshExporter
    mock_exporter = MagicMock()
    monkeypatch.setattr(
        "onshape2xacro.mesh_exporters.step.StepMeshExporter",
        lambda c, cad: mock_exporter,
    )

    # Mock internal helpers
    monkeypatch.setattr(
        "onshape2xacro.pipeline._generate_default_mate_values", lambda cad: {}
    )
    monkeypatch.setattr(
        "onshape2xacro.pipeline.KinematicGraph.from_cad", lambda cad: MagicMock()
    )

    mock_robot = MagicMock()
    mock_robot.nodes = []
    monkeypatch.setattr(
        "onshape2xacro.pipeline.CondensedRobot.from_graph",
        lambda *args, **kwargs: mock_robot,
    )

    # Mock pickle dump
    monkeypatch.setattr("pickle.dump", lambda obj, f: None)

    # Run
    with patch("builtins.print") as mock_print:
        run_fetch_cad(config)

        # Verify warnings
        calls = [str(c) for c in mock_print.mock_calls]
        assert any("Warning: missing_bom.csv" in c for c in calls) or any(
            "BOM file not found" in c for c in calls
        )
        assert any("already exists. Skipping generation" in c for c in calls)


# --- Step Exporter Tests ---
# Covering visual_mesh_format logic in export_link_meshes


def test_step_export_visual_formats(tmp_path):
    from onshape2xacro.mesh_exporters.step import StepMeshExporter

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
        patch("onshape2xacro.mesh_exporters.step.STEPCAFControl_Reader"),
        patch("onshape2xacro.mesh_exporters.step.TDocStd_Document"),
        patch("onshape2xacro.mesh_exporters.step._get_shape_tool"),
        patch(
            "onshape2xacro.mesh_exporters.step._get_free_shape_labels"
        ) as mock_labels,
        patch("onshape2xacro.mesh_exporters.step._collect_shapes"),
        patch("trimesh.load") as mock_trimesh_load,
        patch("pymeshlab.MeshSet"),
    ):
        # Setup mock behavior
        mock_writer.return_value.Write.side_effect = lambda shape, path: Path(
            path
        ).touch()

        # Mock free shapes to return length 1
        mock_labels.return_value.Length.return_value = 1

        # Mock CAD parts
        link_record = MagicMock()
        link_record.keys = ["part_key"]
        link_record.part_names = ["part_name"]

        cad.parts = {"part_key": MagicMock()}
        cad.parts["part_key"].isRigidAssembly = False

        # Mock link records
        link_records = {"link1": link_record}

        # Test DAE export
        mock_mesh = MagicMock()
        mock_trimesh_load.return_value = mock_mesh

        # We need to bypass the STEP reading logic or mock it sufficiently
        # The easiest way is to mock _is_step_payload and zipfile check or ensure asset_path exists
        exporter.asset_path = tmp_path / "assembly.step"
        exporter.asset_path.touch()

        # We also need to mock _collect_shapes population of part_shapes
        def populate_shapes(st, lbl, loc, shapes, locations, path=()):
            shapes["part_key"] = [MagicMock()]
            locations["part_key"] = [MagicMock()]  # Mock TopLoc_Location

        with patch(
            "onshape2xacro.mesh_exporters.step._collect_shapes",
            side_effect=populate_shapes,
        ):
            exporter.export_link_meshes(
                link_records, mesh_dir, visual_mesh_format="dae"
            )

        # Verify trimesh export called with dae
        mock_mesh.export.assert_called_with(ANY, file_type="dae")

        # Test OBJ export
        with patch(
            "onshape2xacro.mesh_exporters.step._collect_shapes",
            side_effect=populate_shapes,
        ):
            exporter.export_link_meshes(
                link_records, mesh_dir, visual_mesh_format="obj"
            )
        mock_mesh.export.assert_called_with(ANY, file_type="obj")
