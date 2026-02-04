import sys
import pytest
from unittest.mock import MagicMock
from types import ModuleType
from importlib.abc import Loader, MetaPathFinder
from importlib.machinery import ModuleSpec


class MockLoader(Loader):
    def exec_module(self, module):
        def getattr_mock(name):
            return MagicMock()

        module.__getattr__ = getattr_mock


class MockFinder(MetaPathFinder):
    def find_spec(self, fullname, path, target=None):
        if fullname.startswith("OCP") or fullname in ["trimesh", "coacd"]:
            return ModuleSpec(fullname, MockLoader())
        return None


import yaml


@pytest.fixture
def modular_classes():
    finder = MockFinder()
    sys.meta_path.insert(0, finder)
    try:
        from onshape2xacro.serializers.modular import ModularXacroSerializer
        from onshape2xacro.config import ConfigOverride
        from onshape2xacro.condensed_robot import LinkRecord, JointRecord

        yield ModularXacroSerializer, ConfigOverride, LinkRecord, JointRecord
    finally:
        if finder in sys.meta_path:
            sys.meta_path.remove(finder)
        for module_name in [
            "onshape2xacro.serializers.modular",
            "onshape2xacro.serializers",
            "onshape2xacro.condensed_robot",
            "onshape2xacro.module_boundary",
        ]:
            sys.modules.pop(module_name, None)


def test_generate_module_configs(tmp_path, modular_classes):
    serializer_cls, config_cls, link_cls, joint_cls = modular_classes
    serializer = serializer_cls()
    config_dir = tmp_path / "config"
    config = config_cls()
    computed_inertials = {
        "link1": {
            "mass": 0.5,
            "origin": {"xyz": "0 0 0", "rpy": "0 0 0"},
            "inertia": {
                "ixx": 0.01,
                "iyy": 0.01,
                "izz": 0.01,
                "ixy": 0,
                "ixz": 0,
                "iyz": 0,
            },
        }
    }

    # GIVEN
    link1 = link_cls(
        name="link1", part_ids=["part1"], occurrences=[], part_names=[], keys=[]
    )
    link2 = link_cls(name="link2", part_ids=[], occurrences=[], part_names=[], keys=[])

    joint1 = joint_cls(
        name="joint_revolute",
        joint_type="REVOLUTE",
        parent="link1",
        child="link2",
        axis=(0, 0, 1),
        limits=MagicMock(min=-1.0, max=1.0, effort=10.0, velocity=1.0),
    )

    elements = {"links": [link1, link2], "joints": [joint1]}

    # WHEN
    serializer._generate_module_configs(
        elements, config_dir, config, computed_inertials
    )

    # THEN
    joint_limits_path = config_dir / "joint_limits.yaml"
    assert joint_limits_path.exists()
    with open(joint_limits_path, "r") as f:
        data = yaml.safe_load(f)
        assert "joint_limits" in data
        assert "revolute" in data["joint_limits"]
        assert data["joint_limits"]["revolute"]["lower"] == -1.0
        assert data["joint_limits"]["revolute"]["upper"] == 1.0

    inertials_path = config_dir / "inertials.yaml"
    assert inertials_path.exists()
    with open(inertials_path, "r") as f:
        data = yaml.safe_load(f)
        assert "inertials" in data
        assert "link1" in data["inertials"]
        assert data["inertials"]["link1"]["mass"] == 0.5
        assert "link2" in data["inertials"]
        assert data["inertials"]["link2"]["mass"] == 1e-9
