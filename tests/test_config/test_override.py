import yaml
from onshape2xacro.config import ConfigOverride


def test_config_load(tmp_path):
    d = tmp_path / "config"
    d.mkdir()
    cfg_file = d / "overrides.yaml"
    content = {
        "joint_limits": {"shoulder": {"lower": -1.0, "upper": 1.0}},
        "inertials": {"link1": {"mass": 5.0}},
    }
    with open(cfg_file, "w") as f:
        yaml.dump(content, f)

    override = ConfigOverride.load(cfg_file)
    assert override.joint_limits["shoulder"]["lower"] == -1.0
    assert override.inertials["link1"]["mass"] == 5.0


def test_config_empty():
    override = ConfigOverride()
    assert override.joint_limits == {}
    assert override.inertials == {}


def test_config_merge():
    # Test logic for merging with default values
    pass
