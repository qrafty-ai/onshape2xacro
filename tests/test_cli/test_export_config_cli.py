import pytest
from unittest.mock import MagicMock
from pathlib import Path
import yaml
import sys
from onshape2xacro.cli import main


def test_export_auto_loads_configuration_yaml(monkeypatch, tmp_path):
    import onshape2xacro.pipeline as pipeline
    import pickle

    local_dir = tmp_path / "robot_data"
    local_dir.mkdir()

    with open(local_dir / "cad.pickle", "wb") as f:
        pickle.dump({}, f)

    config_data = {
        "export": {
            "name": "yaml_robot",
            "output": "yaml_output",
            "visual_mesh_formats": ["stl"],
        }
    }
    with open(local_dir / "configuration.yaml", "w") as f:
        yaml.safe_dump(config_data, f)

    mock_run_export = MagicMock()
    monkeypatch.setattr(pipeline, "run_export", mock_run_export)

    test_args = ["onshape2xacro", "export", str(local_dir)]
    monkeypatch.setattr(sys, "argv", test_args)

    main()

    mock_run_export.assert_called_once()
    config = mock_run_export.call_args[0][0]

    assert config.name == "yaml_robot"
    assert Path(config.output).name == "yaml_output"
    assert config.visual_mesh_formats == ["stl"]


def test_export_fails_missing_configuration_yaml(monkeypatch, tmp_path):
    local_dir = tmp_path / "robot_data"
    local_dir.mkdir()

    (local_dir / "cad.pickle").touch()

    test_args = ["onshape2xacro", "export", str(local_dir)]

    monkeypatch.setattr(sys, "argv", test_args)

    with pytest.raises(Exception, match="configuration.yaml not found"):
        main()


def test_export_cli_overrides_yaml(monkeypatch, tmp_path):
    import onshape2xacro.pipeline as pipeline
    import pickle

    local_dir = tmp_path / "robot_data"
    local_dir.mkdir()

    with open(local_dir / "cad.pickle", "wb") as f:
        pickle.dump({}, f)

    config_data = {
        "export": {
            "name": "yaml_robot",
            "output": "yaml_output",
            "visual_mesh_formats": ["stl"],
        }
    }
    with open(local_dir / "configuration.yaml", "w") as f:
        yaml.safe_dump(config_data, f)

    mock_run_export = MagicMock()
    monkeypatch.setattr(pipeline, "run_export", mock_run_export)

    test_args = [
        "onshape2xacro",
        "export",
        str(local_dir),
        "--name",
        "override_name",
        "--output",
        "override_output",
        "--visual-mesh-formats",
        "glb",
    ]
    monkeypatch.setattr(sys, "argv", test_args)

    main()

    mock_run_export.assert_called_once()
    config = mock_run_export.call_args[0][0]

    assert config.name == "override_name"
    assert Path(config.output).name == "override_output"
    assert config.visual_mesh_formats == ["glb"]
