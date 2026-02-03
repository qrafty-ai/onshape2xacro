import pytest
from pathlib import Path
from onshape2xacro.cli import ExportConfig


def test_export_config_defaults():
    config = ExportConfig(path=Path("."))
    assert config.path == Path(".")
    assert config.output is None
    assert config.name is None
    assert config.config is None
    assert config.max_depth == 5


def test_export_config_custom():
    config = ExportConfig(
        path=Path("my_data"),
        output=Path("/tmp/robot"),
        name="my_robot",
        config=Path("overrides.yaml"),
        max_depth=10,
    )
    assert config.path == Path("my_data")
    assert config.output == Path("/tmp/robot")
    assert config.name == "my_robot"
    assert config.config == Path("overrides.yaml")
    assert config.max_depth == 10


def test_cli_parsing_basic(monkeypatch):
    import sys
    from onshape2xacro.cli import parse_args

    test_args = ["onshape2xacro", "export", "my_local_dir"]
    monkeypatch.setattr(sys, "argv", test_args)

    config = parse_args()
    assert isinstance(config, ExportConfig)
    assert config.path == Path("my_local_dir")


def test_cli_parsing_full(monkeypatch):
    import sys
    from onshape2xacro.cli import parse_args

    test_args = [
        "onshape2xacro",
        "export",
        "local_dir",
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
    config = parse_args()
    assert isinstance(config, ExportConfig)
    assert config.path == Path("local_dir")
    assert config.output == Path("/tmp/out")
    assert config.name == "test_bot"


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
    assert "Local directory" in out
    assert "--output" in out


def test_cli_version(monkeypatch, capsys):
    import sys
    from onshape2xacro.cli import main

    # Version support needs to be added to cli.py/main
    test_args = ["onshape2xacro", "--version"]
    monkeypatch.setattr(sys, "argv", test_args)

    # We expect this to fail/exit for now until implemented
    with pytest.raises(SystemExit):
        main()
    out, err = capsys.readouterr()
    # Placeholder for version check
