import pytest
from onshape2xacro.serializers import is_joint, get_joint_name, is_module_boundary


def test_is_joint():
    assert is_joint("joint_shoulder") is True
    assert is_joint("joint_elbow_1") is True
    assert is_joint("hinge_door") is False
    assert is_joint("fastener_bolt") is False
    assert is_joint("joint") is False  # Too short/no underscore? Or should it pass?
    # According to spec: "starts with name 'joint_'"
    assert is_joint("joint_") is True


def test_get_joint_name():
    assert get_joint_name("joint_shoulder") == "shoulder"
    assert get_joint_name("joint_elbow_1") == "elbow_1"
    assert get_joint_name("joint_") == ""


def test_is_module_boundary_logic():
    # Mocking Robot/Graph structures will be complex,
    # but we can test the logic that checks for joint_* edges

    class MockEdge:
        def __init__(self, name):
            self.name = name

    class MockGraph:
        def __init__(self, edges):
            self.edges = edges

    # Subassembly with joints -> boundary
    edges = [MockEdge("joint_a"), MockEdge("fixed_b")]
    assert is_module_boundary(MockGraph(edges)) is True

    # Subassembly without joints -> not boundary
    edges = [MockEdge("fixed_a"), MockEdge("hinge_b")]
    assert is_module_boundary(MockGraph(edges)) is False
