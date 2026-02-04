import os
import sys
import shutil
import pytest
import types
from pathlib import Path
from unittest.mock import MagicMock, patch
import lxml.etree as ET


@pytest.fixture
def modular_serializer_class():
    orig_mesh_exporters = sys.modules.get("onshape2xacro.mesh_exporters")
    orig_mesh_exporters_step = sys.modules.get("onshape2xacro.mesh_exporters.step")

    sys.modules["onshape2xacro.mesh_exporters"] = MagicMock()
    sys.modules["onshape2xacro.mesh_exporters.step"] = MagicMock()

    try:
        from onshape2xacro.serializers.modular import ModularXacroSerializer

        yield ModularXacroSerializer
    finally:
        if orig_mesh_exporters is not None:
            sys.modules["onshape2xacro.mesh_exporters"] = orig_mesh_exporters
        else:
            sys.modules.pop("onshape2xacro.mesh_exporters", None)

        if orig_mesh_exporters_step is not None:
            sys.modules["onshape2xacro.mesh_exporters.step"] = orig_mesh_exporters_step
        else:
            sys.modules.pop("onshape2xacro.mesh_exporters.step", None)


class MockLink:
    def __init__(self, name, parent=None):
        self.name = name
        self.parent = parent
        self.part_ids = ["part1"]


class MockSubAssembly:
    def __init__(self, name, did, eid):
        self.name = name
        self.documentId = did
        self.elementId = eid
        self.documentMicroversion = "mv1"


@pytest.fixture
def mock_robot():
    robot = MagicMock()
    robot.name = "test_robot"

    sub_def = MockSubAssembly("wheel_module", "doc1", "el1")

    inst1_key = MagicMock()
    inst1_key.name = "left_wheel"
    inst1_key.parent = None

    inst2_key = MagicMock()
    inst2_key.name = "right_wheel"
    inst2_key.parent = None

    robot.cad.subassemblies = {inst1_key: sub_def, inst2_key: sub_def}

    base_link = MockLink("base_link", parent=None)
    l_link = MockLink("wheel_link", parent=inst1_key)
    r_link = MockLink("wheel_link", parent=inst2_key)

    joint_l = MagicMock()
    joint_l.name = "joint_left_wheel"
    joint_l.parent = "base_link"
    joint_l.child = "l_node"
    joint_l.joint_type = "revolute"
    joint_l.limits = {"min": -3.14, "max": 3.14}
    joint_l.mate = MagicMock()

    joint_r = MagicMock()
    joint_r.name = "joint_right_wheel"
    joint_r.parent = "base_link"
    joint_r.child = "r_node"
    joint_r.joint_type = "revolute"
    joint_r.limits = {"min": -3.14, "max": 3.14}
    joint_r.mate = MagicMock()

    robot.nodes = {
        "base_node": {"link": base_link},
        "l_node": {"link": l_link},
        "r_node": {"link": r_link},
    }

    module_groups = {
        None: {"links": [base_link], "joints": [joint_l, joint_r]},
        inst1_key: {"links": [l_link], "joints": []},
        inst2_key: {"links": [r_link], "joints": []},
    }
    robot.module_groups = module_groups

    interface_info = MagicMock()
    interface_info.parent_link = "base_link"
    interface_info.child_root_link = "wheel_link"
    interface_info.origin = MagicMock()
    interface_info.origin.xyz = [0, 0, 0]
    interface_info.origin.rpy = [0, 0, 0]
    interface_info.axis = [0, 0, 1]

    robot.interface_transforms = {inst1_key: interface_info, inst2_key: interface_info}

    return robot, inst1_key, inst2_key


def test_subassembly_consolidation(tmp_path, mock_robot, modular_serializer_class):
    robot, inst1_key, inst2_key = mock_robot
    serializer = modular_serializer_class()

    output_dir = tmp_path / "output"

    with (
        patch.object(
            modular_serializer_class,
            "_group_by_subassembly",
            return_value=robot.module_groups,
        ),
        patch(
            "onshape2xacro.serializers.modular.calculate_interface_transforms",
            return_value=robot.interface_transforms,
        ),
        patch("onshape2xacro.serializers.modular.detect_module_boundaries"),
        patch.object(
            modular_serializer_class, "_export_meshes", return_value=({}, {}, None)
        ),
        patch.object(modular_serializer_class, "_joint_to_xacro"),
        patch.object(modular_serializer_class, "_link_to_xacro"),
        patch.object(modular_serializer_class, "_generate_module_configs"),
    ):
        serializer.save(robot, str(output_dir), download_assets=False)

    urdf_dir = output_dir / "urdf"

    sub_module_dirs = [
        d.name
        for d in urdf_dir.iterdir()
        if d.is_dir() and d.name not in ["config", "meshes"]
    ]
    assert "wheel_module" in sub_module_dirs

    assert len(sub_module_dirs) == 1

    module_xacro = urdf_dir / "wheel_module" / "wheel_module.xacro"
    assert module_xacro.exists()

    root_xacro = urdf_dir / "test_robot.xacro"
    root_content = root_xacro.read_text()
    assert "wheel_module/wheel_module.xacro" in root_content
    assert 'wheel_module prefix="${prefix}left_wheel_"' in root_content
    assert 'wheel_module prefix="${prefix}right_wheel_"' in root_content
