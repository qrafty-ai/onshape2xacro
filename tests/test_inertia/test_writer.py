"""Tests for inertia config writer."""

import yaml
from pathlib import Path
from tempfile import TemporaryDirectory

from onshape2xacro.inertia.types import InertialProperties
from onshape2xacro.inertia.writer import InertiaConfigWriter


def test_write_inertials_yaml():
    props = {
        "base_link": InertialProperties(
            mass=1.0, com=(0, 0, 0), ixx=0.01, iyy=0.01, izz=0.01
        ),
        "arm_link": InertialProperties(
            mass=0.5, com=(0.1, 0, 0), ixx=0.005, iyy=0.005, izz=0.002
        ),
    }

    with TemporaryDirectory() as tmpdir:
        config_dir = Path(tmpdir)
        writer = InertiaConfigWriter()
        writer.write(props, config_dir)

        # Verify file exists
        yaml_path = config_dir / "inertials.yaml"
        assert yaml_path.exists()

        # Verify content
        with open(yaml_path) as f:
            data = yaml.safe_load(f)

        assert "inertials" in data
        assert "base_link" in data["inertials"]
        assert data["inertials"]["base_link"]["mass"] == 1.0
        assert "arm_link" in data["inertials"]


def test_write_creates_directory():
    """Test that writer creates config directory if it doesn't exist."""
    props = {
        "link1": InertialProperties(
            mass=1.0, com=(0, 0, 0), ixx=0.01, iyy=0.01, izz=0.01
        ),
    }

    with TemporaryDirectory() as tmpdir:
        config_dir = Path(tmpdir) / "nested" / "config"
        writer = InertiaConfigWriter()
        output_path = writer.write(props, config_dir)

        assert output_path.exists()
        assert output_path.name == "inertials.yaml"
