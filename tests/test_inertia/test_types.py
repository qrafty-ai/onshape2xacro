"""Tests for inertia types."""

from onshape2xacro.inertia.types import InertialProperties


def test_inertial_properties_to_yaml_dict():
    props = InertialProperties(
        mass=1.5,
        com=(0.1, 0.2, 0.3),
        ixx=0.01,
        iyy=0.02,
        izz=0.03,
        ixy=0.001,
        ixz=0.002,
        iyz=0.003,
    )
    result = props.to_yaml_dict()

    assert result["mass"] == 1.5
    assert result["origin"]["xyz"] == "0.1 0.2 0.3"
    assert result["origin"]["rpy"] == "0 0 0"
    assert result["inertia"]["ixx"] == 0.01
    assert result["inertia"]["iyy"] == 0.02
    assert result["inertia"]["izz"] == 0.03
