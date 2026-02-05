from unittest.mock import patch
from onshape2xacro.config.export_config import ExportConfiguration, ExportOptions


def test_pipeline_name_fallback(tmp_path):
    from onshape2xacro.pipeline import run_export
    from onshape2xacro.schema import ExportConfig

    # Setup environment
    input_dir = tmp_path / "my_robot_dir"
    input_dir.mkdir()
    (input_dir / "cad.pickle").touch()

    # Config with empty name
    config_path = input_dir / "configuration.yaml"
    config = ExportConfiguration(
        export=ExportOptions(name="", output=tmp_path / "output")
    )
    config.save(config_path)

    cli_config = ExportConfig(path=input_dir)

    # Mock everything around the robot creation
    with (
        patch("pickle.load"),
        patch("onshape2xacro.pipeline.KinematicGraph"),
        patch("onshape2xacro.pipeline.CondensedRobot") as mock_robot_cls,
        patch("onshape2xacro.pipeline.ConfigOverride"),
        patch("onshape2xacro.pipeline.XacroSerializer"),
    ):
        # Setup robot mock

        # Run export
        run_export(cli_config)

        # Verify CondensedRobot was initialized with directory name
        mock_robot_cls.from_graph.assert_called_once()
        _, kwargs = mock_robot_cls.from_graph.call_args
        assert kwargs["name"] == "my_robot_dir"


def test_pipeline_name_sanitize_fallback(tmp_path):
    from onshape2xacro.pipeline import run_export
    from onshape2xacro.schema import ExportConfig

    # Setup environment
    input_dir = tmp_path / "unicode_robot"
    input_dir.mkdir()
    (input_dir / "cad.pickle").touch()

    # Config with name that would sanitize to "_" if not for the fix,
    # but let's test a case that WOULD sanitize to empty string if we didn't have the directory fallback
    # e.g. purely special chars
    config_path = input_dir / "configuration.yaml"
    config = ExportConfiguration(
        export=ExportOptions(name="!!!", output=tmp_path / "output")
    )
    config.save(config_path)

    cli_config = ExportConfig(path=input_dir)

    with (
        patch("pickle.load"),
        patch("onshape2xacro.pipeline.KinematicGraph"),
        patch("onshape2xacro.pipeline.CondensedRobot") as mock_robot_cls,
        patch("onshape2xacro.pipeline.ConfigOverride"),
        patch("onshape2xacro.pipeline.XacroSerializer"),
    ):
        run_export(cli_config)

        # Verify fallback used directory name
        _, kwargs = mock_robot_cls.from_graph.call_args
        assert kwargs["name"] == "unicode_robot"
