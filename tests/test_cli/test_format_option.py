from unittest.mock import MagicMock
import sys

# Mock dependencies that are hard to install or trigger heavy imports
for m in [
    "onshape_robotics_toolkit",
    "onshape_robotics_toolkit.robot",
    "onshape_robotics_toolkit.models",
    "onshape_robotics_toolkit.models.assembly",
    "onshape_robotics_toolkit.models.link",
    "onshape_robotics_toolkit.parse",
    "onshape_robotics_toolkit.formats",
    "onshape_robotics_toolkit.formats.base",
    "onshape2xacro.pipeline",
]:
    sys.modules[m] = MagicMock()

from onshape2xacro.cli import main  # noqa: E402
import onshape2xacro.pipeline as mock_pipeline  # noqa: E402


def test_export_format_option(monkeypatch, tmp_path):
    import pickle
    import yaml

    local_dir = tmp_path / "robot_data"
    local_dir.mkdir()

    with open(local_dir / "cad.pickle", "wb") as f:
        pickle.dump({"fake": "data"}, f)

    # Need a configuration.yaml for main to load
    config_data = {
        "export": {
            "name": "test_robot",
            "output": "test_output",
        }
    }
    with open(local_dir / "configuration.yaml", "w") as f:
        yaml.safe_dump(config_data, f)

    # Test default format
    test_args = ["onshape2xacro", "export", str(local_dir)]
    monkeypatch.setattr(sys, "argv", test_args)
    main()

    # Check that run_export was called with the expected config
    mock_pipeline.run_export.assert_called_once()
    config = mock_pipeline.run_export.call_args[0][0]
    assert config.format == "xacro"

    mock_pipeline.run_export.reset_mock()

    # Test format override
    test_args = ["onshape2xacro", "export", str(local_dir), "--format", "xacro_module"]
    monkeypatch.setattr(sys, "argv", test_args)
    main()

    mock_pipeline.run_export.assert_called_once()
    config = mock_pipeline.run_export.call_args[0][0]
    assert config.format == "xacro_module"
