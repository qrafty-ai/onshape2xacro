import pytest
import sys
from unittest.mock import MagicMock


from importlib.abc import Loader, MetaPathFinder
from importlib.machinery import ModuleSpec


class MockLoader(Loader):
    def exec_module(self, module):
        if module.__name__ == "onshape_robotics_toolkit.formats.base":

            class RobotSerializer:
                def save(self, *args, **kwargs):
                    pass

                def serialize(self, *args, **kwargs):
                    return ""

            setattr(module, "RobotSerializer", RobotSerializer)
        else:
            module.__getattr__ = lambda name: MagicMock()


class MockFinder(MetaPathFinder):
    def find_spec(self, fullname, path, target=None):
        if any(
            fullname.startswith(p)
            for p in ["OCP", "onshape_robotics_toolkit", "trimesh", "coacd", "keyring"]
        ):
            return ModuleSpec(fullname, MockLoader())
        return None


from unittest.mock import patch


@pytest.fixture
def serializer():
    finder = MockFinder()
    sys.meta_path.insert(0, finder)
    try:
        from onshape2xacro.serializers.modular import ModularXacroSerializer

        yield ModularXacroSerializer()
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


class MockLink:
    def __init__(self, name, parent=None, part_ids=None):
        self.name = name
        self.parent = parent
        self.part_ids = part_ids or ["part_1"]
        self.keys = ["part_1"]


class MockJoint:
    def __init__(self, name, parent, child, axis=(0, 0, 1), joint_type="revolute"):
        self.name = name
        self.parent = parent
        self.child = child
        self.axis = axis
        self.joint_type = joint_type
        self.limits = None


@pytest.fixture
def mock_robot():
    robot = MagicMock()
    robot.name = "test_robot"
    robot.cad = MagicMock()
    robot.kinematic_graph = MagicMock()

    root_key = None
    sub1_key = MagicMock()
    sub1_key.name = "sub1"
    sub1_key.parent = root_key

    subasm = MagicMock()
    subasm.name = "sub1"
    subasm.documentId = "doc1"
    subasm.elementId = "el1"
    robot.cad.subassemblies = {sub1_key: subasm}

    link_root = MockLink("link_root", parent=root_key)
    link_sub1 = MockLink("link_sub1", parent=sub1_key)

    nodes_data = {
        "link_root": {"link": link_root},
        "link_sub1": {"link": link_sub1},
    }
    robot.nodes = MagicMock()
    robot.nodes.side_effect = (
        lambda data=False: nodes_data.items() if data else nodes_data.keys()
    )
    robot.nodes.__getitem__.side_effect = lambda n: nodes_data[n]
    robot.nodes.__iter__.side_effect = lambda: iter(nodes_data.keys())

    joint1 = MockJoint("joint_sub1", "link_root", "link_sub1")

    robot.edges = [
        ("link_root", "link_sub1"),
    ]

    def get_edge_data(u, v):
        if u == "link_root" and v == "link_sub1":
            return {"joint": joint1}
        return {}

    robot.get_edge_data = get_edge_data

    robot.module_boundaries = None

    # Mock group_by_subassembly behavior
    def mock_group_by_subassembly(r):
        return {
            root_key: {"links": [link_root], "joints": [joint1]},
            sub1_key: {"links": [link_sub1], "joints": []},
        }

    return robot


def test_modular_xacro_serializer_save(mock_robot, tmp_path, serializer):
    with patch.object(type(serializer), "_export_meshes", return_value=({}, {}, None)):
        serializer.save(mock_robot, str(tmp_path), download_assets=False)

    assert (tmp_path / "urdf").exists()
    assert (tmp_path / "urdf" / "test_robot.xacro").exists()
    assert (tmp_path / "urdf" / "sub1").exists()
    assert (tmp_path / "urdf" / "sub1" / "sub1.xacro").exists()
    assert (tmp_path / "urdf" / "sub1" / "meshes").exists()
    assert (tmp_path / "urdf" / "config").exists()
    assert (tmp_path / "urdf" / "sub1" / "config").exists()

    root_xacro = (tmp_path / "urdf" / "test_robot.xacro").read_text()
    assert '<xacro:macro name="test_robot" params="prefix:=\'\'">' in root_xacro

    sub1_xacro = (tmp_path / "urdf" / "sub1" / "sub1.xacro").read_text()
    assert '<xacro:macro name="sub1" params="prefix:=\'\'">' in sub1_xacro

    assert (tmp_path / "urdf" / "config" / "test_robot_joint_limits.yaml").exists()
    assert (tmp_path / "urdf" / "sub1" / "config" / "sub1_inertials.yaml").exists()
