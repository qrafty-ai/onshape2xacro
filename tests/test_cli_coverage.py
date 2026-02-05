from unittest.mock import MagicMock, patch
from onshape2xacro.config.export_config import (
    ExportConfiguration,
    ExportOptions,
    CoACDOptions,
    VisualMeshOptions,
)


def test_cli_overrides(tmp_path):
    from onshape2xacro.cli import main

    # Create dummy config file and cad.pickle
    input_dir = tmp_path / "input"
    input_dir.mkdir()
    (input_dir / "cad.pickle").touch()

    config_path = input_dir / "configuration.yaml"

    # Create a config file with specific settings
    config = ExportConfiguration(
        export=ExportOptions(
            name="file_robot",
            visual_option=VisualMeshOptions(formats=["stl"]),
            output=tmp_path / "file_output",
        )
    )
    config.save(config_path)

    # Mock pipeline.run_export
    with (
        patch("onshape2xacro.pipeline.run_export") as mock_run_export,
        patch(
            "sys.argv",
            [
                "onshape2xacro",
                "export",
                str(input_dir),
                "--visual-option.formats",
                "glb",
                "--skip-confirmation",
            ],
        ),
    ):
        main()

        mock_run_export.assert_called_once()
        args, kwargs = mock_run_export.call_args
        export_config = kwargs["export_configuration"]

        # Verify CLI override: should be glb (from CLI) not stl (from file)
        assert export_config.export.visual_option.formats == ["glb"]

        # Verify file preservation: name should still be file_robot
        assert export_config.export.name == "file_robot"


def test_cli_bom_autodetect(tmp_path):
    from onshape2xacro.cli import main

    # Create dummy config file and cad.pickle
    input_dir = tmp_path / "input"
    input_dir.mkdir()
    (input_dir / "cad.pickle").touch()
    (input_dir / "configuration.yaml").touch()

    # Create a BOM file
    bom_file = input_dir / "my_bom.csv"
    bom_file.touch()

    # Mock pipeline.run_export and ExportConfiguration.load
    with (
        patch("onshape2xacro.pipeline.run_export") as mock_run_export,
        patch(
            "onshape2xacro.config.export_config.ExportConfiguration.load"
        ) as mock_load,
        patch(
            "sys.argv",
            ["onshape2xacro", "export", str(input_dir), "--skip-confirmation"],
        ),
    ):
        mock_config_obj = MagicMock()
        mock_config_obj.export.bom = None
        mock_config_obj.export.output = tmp_path / "output"
        mock_load.return_value = mock_config_obj

        main()

        mock_run_export.assert_called_once()
        args, kwargs = mock_run_export.call_args
        export_config = kwargs["export_configuration"]

        # Verify BOM was auto-detected
        assert export_config.export.bom == bom_file


def test_cli_coacd_override(tmp_path):
    from onshape2xacro.cli import main
    from onshape2xacro.config.export_config import (
        ExportConfiguration,
        ExportOptions,
        CollisionOptions,
    )

    # Create dummy config file and cad.pickle
    input_dir = tmp_path / "input"
    input_dir.mkdir()
    (input_dir / "cad.pickle").touch()

    config_path = input_dir / "configuration.yaml"

    # Create a config file with max_workers = 5
    config = ExportConfiguration(
        export=ExportOptions(
            output=tmp_path / "output",
            collision_option=CollisionOptions(
                method="coacd", coacd=CoACDOptions(max_workers=5, threshold=0.1)
            ),
        )
    )
    config.save(config_path)

    # Run with CLI override for max-workers = 20
    with (
        patch("onshape2xacro.pipeline.run_export") as mock_run_export,
        patch(
            "sys.argv",
            [
                "onshape2xacro",
                "export",
                str(input_dir),
                "--collision-option.coacd.max-workers",
                "20",
                "--skip-confirmation",
            ],
        ),
    ):
        main()

        mock_run_export.assert_called_once()
        args, kwargs = mock_run_export.call_args
        export_config = kwargs["export_configuration"]

        # Verify override
        assert export_config.export.collision_option.coacd.max_workers == 20
        # Verify preservation
        assert export_config.export.collision_option.coacd.threshold == 0.1
