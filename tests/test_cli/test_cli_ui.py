from pathlib import Path
from unittest.mock import MagicMock, patch
import pytest
from onshape2xacro.cli import _confirm_export_config, main
from onshape2xacro.schema import (
    ExportConfig,
    VisualizeConfig,
    AuthConfig,
    AuthLoginConfig,
)
from onshape2xacro.config.export_config import (
    ExportConfiguration,
    ExportOptions,
    CollisionOptions,
    CoACDOptions,
)


def test_confirm_export_config_skip():
    cli_config = ExportConfig(path=Path("."), skip_confirmation=True)
    export_config = MagicMock()

    with patch("rich.console.Console") as mock_console:
        _confirm_export_config(cli_config, export_config)
        mock_console.assert_not_called()


def test_confirm_export_config_yes(capsys):
    cli_config = ExportConfig(path=Path("."), skip_confirmation=False, max_depth=5)
    export_config = ExportConfiguration(
        export=ExportOptions(
            name="test_robot",
            output=Path("out"),
            visual_mesh_formats=["obj", "glb"],
            collision_option=CollisionOptions(
                method="coacd", coacd=CoACDOptions(threshold=0.1)
            ),
            bom=Path("bom.csv"),
        )
    )

    with patch("rich.prompt.Confirm.ask", return_value=True) as mock_ask:
        with patch("rich.console.Console"):
            _confirm_export_config(cli_config, export_config)

            mock_ask.assert_called_once()


def test_confirm_export_config_no(capsys):
    cli_config = ExportConfig(path=Path("."), skip_confirmation=False)
    export_config = ExportConfiguration(export=ExportOptions())

    with patch("rich.prompt.Confirm.ask", return_value=False):
        with patch("rich.console.Console"):
            with pytest.raises(SystemExit):
                _confirm_export_config(cli_config, export_config)


def test_cli_visualize(monkeypatch):
    with patch("onshape2xacro.cli.parse_args") as mock_parse:
        mock_parse.return_value = VisualizeConfig(
            url="http://url", output=Path("out.png")
        )

        with patch("onshape2xacro.pipeline.run_visualize") as mock_run:
            main()
            mock_run.assert_called_once()


def test_cli_auth_status(monkeypatch):
    with patch("onshape2xacro.cli.parse_args") as mock_parse:
        mock_parse.return_value = AuthConfig(command=AuthLoginConfig())

        with patch("onshape2xacro.pipeline.run_auth") as mock_run:
            main()
            mock_run.assert_called_once()


def test_cli_auth(monkeypatch):
    # Test main() with AuthConfig
    with patch("onshape2xacro.cli.parse_args") as mock_parse:
        mock_parse.return_value = AuthConfig(command=AuthLoginConfig())

        with patch("onshape2xacro.pipeline.run_auth") as mock_run:
            main()
            mock_run.assert_called_once()
