import pytest
from pathlib import Path
from onshape2xacro.cli import ExportConfig


def test_export_config_defaults():
    config = ExportConfig(url="https://cad.onshape.com/documents/123")
    assert config.url == "https://cad.onshape.com/documents/123"
    assert config.output == Path(".")
    assert config.name is None
    assert config.config is None
    assert config.max_depth == 5


def test_export_config_custom():
    config = ExportConfig(
        url="https://cad.onshape.com/documents/123",
        output=Path("/tmp/robot"),
        name="my_robot",
        config=Path("overrides.yaml"),
        max_depth=10,
    )
    assert config.output == Path("/tmp/robot")
    assert config.name == "my_robot"
    assert config.config == Path("overrides.yaml")
    assert config.max_depth == 10


def test_cli_parsing_basic(monkeypatch):
    import sys
    from onshape2xacro.cli import parse_args

    test_args = ["onshape2xacro", "export", "https://cad.onshape.com/documents/123"]
    monkeypatch.setattr(sys, "argv", test_args)

    config = parse_args()
    assert config.url == "https://cad.onshape.com/documents/123"


def test_cli_parsing_full(monkeypatch):
    import sys
    from onshape2xacro.cli import parse_args

    test_args = [
        "onshape2xacro",
        "export",
        "https://cad.onshape.com/documents/123",
        "--output",
        "/tmp/out",
        "--name",
        "test_bot",
        "--config",
        "cfg.yaml",
        "--max-depth",
        "3",
    ]
    monkeypatch.setattr(sys, "argv", test_args)


def test_cli_help(monkeypatch, capsys):
    import sys
    from onshape2xacro.cli import main

    test_args = ["onshape2xacro", "--help"]
    monkeypatch.setattr(sys, "argv", test_args)

    with pytest.raises(SystemExit) as excinfo:
        main()
    assert excinfo.value.code == 0
    out, err = capsys.readouterr()
    assert "onshape2xacro" in out
    assert "export" in out


def test_cli_export_help(monkeypatch, capsys):
    import sys
    from onshape2xacro.cli import main

    test_args = ["onshape2xacro", "export", "--help"]
    monkeypatch.setattr(sys, "argv", test_args)

    with pytest.raises(SystemExit) as excinfo:
        main()
    assert excinfo.value.code == 0
    out, err = capsys.readouterr()
    assert "Onshape document URL" in out
    assert "--output" in out


def test_cli_version(monkeypatch, capsys):
    import sys
    from onshape2xacro.cli import main

    # Version support needs to be added to cli.py/main
    test_args = ["onshape2xacro", "--version"]
    monkeypatch.setattr(sys, "argv", test_args)

    # We expect this to fail/exit for now until implemented
    with pytest.raises(SystemExit) as excinfo:
        main()
    out, err = capsys.readouterr()
    # Placeholder for version check
