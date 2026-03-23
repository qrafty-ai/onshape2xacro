import yaml
from onshape2xacro.config import ConfigOverride


def test_config_load(tmp_path):
    d = tmp_path / "config"
    d.mkdir()
    cfg_file = d / "overrides.yaml"
    content = {
        "joint_limits": {"shoulder": {"lower": -1.0, "upper": 1.0}},
        "inertials": {"link1": {"mass": 5.0}},
        "joint_transforms": {"shoulder": {"xyz": "1 2 3", "rpy": "0 0 1.57"}},
    }
    with open(cfg_file, "w") as f:
        yaml.dump(content, f)

    override = ConfigOverride.load(cfg_file)
    assert override.joint_limits["shoulder"]["lower"] == -1.0
    assert override.inertials["link1"]["mass"] == 5.0
    assert override.joint_transforms["shoulder"]["xyz"] == "1 2 3"


def test_config_empty():
    override = ConfigOverride()
    assert override.joint_limits == {}
    assert override.inertials == {}
    assert override.joint_transforms == {}


def test_config_merge():
    override = ConfigOverride(
        joint_limits={"shoulder": {"lower": -1.0}},
        inertials={"link1": {"mass": 5.0}},
        dynamics={"shoulder": {"damping": 0.2}},
        joint_transforms={"shoulder": {"xyz": "1 2 3"}},
    )

    assert override.get_joint_limit(
        "shoulder", {"lower": -2.0, "upper": 2.0, "damping": 0.0}
    ) == {"lower": -1.0, "upper": 2.0, "damping": 0.2}
    assert override.get_inertial("link1", {"mass": 1.0, "origin": {}}) == {
        "mass": 5.0,
        "origin": {},
    }
    assert override.get_joint_transform(
        "shoulder", {"xyz": "0 0 0", "rpy": "0 0 0"}
    ) == {"xyz": "1 2 3", "rpy": "0 0 0"}
