import pytest
import networkx as nx
from types import SimpleNamespace
from onshape2xacro.naming import sanitize_name
from onshape2xacro.serializers import XacroSerializer
from onshape2xacro.condensed_robot import LinkRecord, JointRecord


def test_sanitize_name():
    assert sanitize_name("My Part (v2.1)") == "my_part_v21"
    assert sanitize_name("Link 1") == "link_1"
    assert sanitize_name("Joint-A") == "joint_a"
    assert sanitize_name("123name") == "_123name"  # Should start with letter/underscore


@pytest.mark.parametrize(
    "input_name,expected",
    [
        ("Space Part", "space_part"),
        ("Special!@#Char", "specialchar"),
        ("_already_safe", "_already_safe"),
        ("Capitals", "capitals"),
    ],
)
def test_name_sanitization_variants(input_name, expected):
    assert sanitize_name(input_name) == expected


def test_xacro_uses_link_mesh_map(tmp_path):
    # Mock a node that has a 'parent' attribute
    class MockNode:
        def __init__(self, name, parent=None):
            self.name = name
            self.parent = parent

    node_a = MockNode("link_a")
    node_b = MockNode("link_b")

    robot = nx.DiGraph()
    robot.name = "r"
    robot.add_node(node_a, data=LinkRecord("link_a", ["pA"], [["A"]], ["A"]))
    robot.add_node(node_b, data=LinkRecord("link_b", ["pB"], [["B"]], ["B"]))

    joint_mate = SimpleNamespace(name="joint_revolute", mateType="REVOLUTE")
    joint = JointRecord(
        "joint_revolute", "REVOLUTE", "link_a", "link_b", None, joint_mate
    )
    robot.add_edge(node_a, node_b, data=joint)

    serializer = XacroSerializer()
    # Mock _export_meshes to avoid API calls and StepMeshExporter requirements
    serializer._export_meshes = lambda robot, mesh_dir: {
        "link_a": "link_a.stl",
        "link_b": "link_b.stl",
    }

    out = tmp_path / "output"
    serializer.save(robot, str(out), download_assets=True)

    assert (out / "urdf" / "r.urdf.xacro").exists()
    assert (out / "urdf" / "r.xacro").exists()

    with open(out / "urdf" / "r.xacro", "r") as f:
        content = f.read()
        assert "link_a.stl" in content
        assert "link_b.stl" in content
        assert 'name="${prefix}revolute"' in content
