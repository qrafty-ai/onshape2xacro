import pytest
import networkx as nx
import numpy as np
from types import SimpleNamespace
from unittest.mock import patch
from onshape_robotics_toolkit.models.link import Origin
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
    robot.add_node(
        node_a, data=LinkRecord("link_a", ["pA"], [["A"]], ["A"], keys=["pA"])
    )
    robot.add_node(
        node_b, data=LinkRecord("link_b", ["pB"], [["B"]], ["B"], keys=["pB"])
    )

    joint_mate = SimpleNamespace(name="joint_revolute", mateType="REVOLUTE")
    joint = JointRecord(
        "joint_revolute", "REVOLUTE", "link_a", "link_b", (0, 0, 1), joint_mate
    )
    robot.add_edge(node_a, node_b, data=joint)

    serializer = XacroSerializer()
    # Provide dummy client/cad to satisfy the check in save()
    robot.client = SimpleNamespace()
    robot.cad = SimpleNamespace(document_id="doc", element_id="elem")

    # Mock _export_meshes - we still mock this because StepMeshExporter will be used
    # But wait, save() calls StepMeshExporter.export_link_meshes directly.
    # So we need to mock StepMeshExporter.
    with patch("onshape2xacro.serializers.StepMeshExporter") as mock_exporter_cls:
        mock_exporter = mock_exporter_cls.return_value
        mock_exporter.export_link_meshes.return_value = (
            {"link_a": "link_a.stl", "link_b": "link_b.stl"},
            {},
            None,
        )

        out = tmp_path / "output"
        serializer.save(robot, str(out), download_assets=True)

    assert (out / "urdf" / "r.urdf.xacro").exists()
    assert (out / "urdf" / "r.xacro").exists()

    with open(out / "urdf" / "r.xacro", "r") as f:
        content = f.read()
        assert "link_a.stl" in content
        assert "link_b.stl" in content
        assert 'name="${prefix}revolute"' in content


def test_xacro_joint_origin(tmp_path):
    class MockNode:
        def __init__(self, name, parent=None):
            self.name = name
            self.parent = parent

    node_a = MockNode("link_a")
    node_b = MockNode("link_b")

    robot = nx.DiGraph()
    robot.name = "r"
    robot.add_node(
        node_a, data=LinkRecord("link_a", ["pA"], [["A"]], ["A"], keys=["pA"])
    )
    robot.add_node(
        node_b, data=LinkRecord("link_b", ["pB"], [["B"]], ["B"], keys=["pB"])
    )

    # Create specific origin (x=1, y=2, z=3, 90 deg rotation around X)
    mat = np.eye(4)
    mat[0, 3] = 1.0
    mat[1, 3] = 2.0
    mat[2, 3] = 3.0
    mat[1, 1] = 0
    mat[1, 2] = -1
    mat[2, 1] = 1
    mat[2, 2] = 0

    origin = Origin.from_matrix(mat)

    joint_mate = SimpleNamespace(name="joint_revolute", mateType="REVOLUTE")
    joint = JointRecord(
        "joint_revolute",
        "REVOLUTE",
        "link_a",
        "link_b",
        (0, 0, 1),
        joint_mate,
        origin=origin,
    )
    robot.add_edge(node_a, node_b, data=joint)

    serializer = XacroSerializer()
    robot.client = SimpleNamespace()
    robot.cad = SimpleNamespace(document_id="doc", element_id="elem")

    with patch("onshape2xacro.serializers.StepMeshExporter") as mock_exporter_cls:
        mock_exporter = mock_exporter_cls.return_value
        mock_exporter.export_link_meshes.return_value = (
            {"link_a": "link_a.stl", "link_b": "link_b.stl"},
            {},
            None,
        )

        out = tmp_path / "output"
        serializer.save(robot, str(out), download_assets=True)

        with open(out / "urdf" / "r.xacro", "r") as f:
            content = f.read()
            (tmp_path / "last_content.xacro").write_text(content)
            assert "1.0 2.0 3.0" in content or "1 2 3" in content
            assert 'rpy="1.57079' in content

        # Check visual/collision origins for links are "0 0 0" when mesh_map is present
        # Both link_a and link_b should have identity origins for their visuals/collisions
        assert content.count('<origin xyz="0 0 0" rpy="0 0 0"/>') >= 4


def test_xacro_plumbs_collision_options(tmp_path):
    from onshape2xacro.config.export_config import CollisionOptions, CoACDOptions

    robot = nx.DiGraph()
    robot.name = "r"
    robot.client = SimpleNamespace()
    robot.cad = SimpleNamespace(document_id="doc", element_id="elem")

    serializer = XacroSerializer()
    collision_option = CollisionOptions(
        method="coacd", coacd=CoACDOptions(threshold=0.123)
    )

    with patch("onshape2xacro.serializers.StepMeshExporter") as mock_exporter_cls:
        mock_exporter = mock_exporter_cls.return_value
        mock_exporter.export_link_meshes.return_value = ({}, {}, None)

        out = tmp_path / "output"
        serializer.save(
            robot, str(out), download_assets=True, collision_option=collision_option
        )

        args, kwargs = mock_exporter.export_link_meshes.call_args
        assert kwargs["collision_option"] == collision_option
        assert kwargs["collision_option"].coacd.threshold == 0.123
