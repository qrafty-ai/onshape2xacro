import pytest
from pathlib import Path
import sys


def test_visualize_config_defaults():
    from onshape2xacro.cli import VisualizeConfig

    config = VisualizeConfig(
        url="https://cad.onshape.com/documents/123", output=Path("viz.png")
    )
    assert config.url == "https://cad.onshape.com/documents/123"
    assert config.output == Path("viz.png")
    assert config.max_depth == 5


def test_visualize_config_custom():
    from onshape2xacro.cli import VisualizeConfig

    config = VisualizeConfig(
        url="https://cad.onshape.com/documents/123",
        output=Path("/tmp/viz"),
        max_depth=10,
    )
    assert config.url == "https://cad.onshape.com/documents/123"
    assert config.output == Path("/tmp/viz")
    assert config.max_depth == 10


def test_cli_parsing_visualize(monkeypatch):
    from onshape2xacro.cli import VisualizeConfig, parse_args

    test_args = [
        "onshape2xacro",
        "visualize",
        "https://cad.onshape.com/documents/123",
        "--output",
        "viz.png",
    ]
    monkeypatch.setattr(sys, "argv", test_args)

    config = parse_args()
    assert isinstance(config, VisualizeConfig)
    assert config.url == "https://cad.onshape.com/documents/123"
    assert config.output == Path("viz.png")


def test_cli_parsing_visualize_output(monkeypatch):
    from onshape2xacro.cli import VisualizeConfig, parse_args

    test_args = [
        "onshape2xacro",
        "visualize",
        "https://cad.onshape.com/documents/123",
        "--output",
        "/tmp/viz",
    ]
    monkeypatch.setattr(sys, "argv", test_args)

    config = parse_args()
    assert isinstance(config, VisualizeConfig)
    assert config.output == Path("/tmp/viz")


def test_cli_visualize_help(monkeypatch, capsys):
    from onshape2xacro.cli import main

    test_args = ["onshape2xacro", "visualize", "--help"]
    monkeypatch.setattr(sys, "argv", test_args)

    with pytest.raises(SystemExit) as excinfo:
        main()
    assert excinfo.value.code == 0
    out, err = capsys.readouterr()
    assert "visualize" in out.lower()
    assert "--output" in out
    assert "--max-depth" in out
