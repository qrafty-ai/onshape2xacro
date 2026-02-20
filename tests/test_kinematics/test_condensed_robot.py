import numpy as np
import pytest
from unittest.mock import MagicMock
from onshape2xacro.condensed_robot import (
    CondensedRobot,
    is_joint_mate,
    occ_match,
    to_id_tuple,
)


def test_to_id_tuple():
    assert to_id_tuple(None) == ()
    assert to_id_tuple(["a", "b"]) == ("a", "b")
    assert to_id_tuple("a/b/c") == ("a", "b", "c")
    assert to_id_tuple("a b c") == ("a", "b", "c")
    assert to_id_tuple("a") == ("a",)

    pk = MagicMock()
    pk.path = ["p1", "p2"]
    assert to_id_tuple(pk) == ("p1", "p2")


def test_occ_match():
    assert occ_match(None, None) is False
    assert occ_match(["a", "b"], ["b"]) is True
    assert occ_match(["a", "b"], ["a", "b"]) is True
    assert occ_match(["b"], ["a", "b"]) is True
    assert occ_match(["a"], ["b"]) is False
    assert occ_match([], []) is True


def test_is_joint_mate():
    def make_mate(name):
        m = MagicMock()
        m.name = name
        return m

    assert is_joint_mate(make_mate("joint_revolute")) is True
    assert is_joint_mate(make_mate("joint_prism")) is True
    assert is_joint_mate(make_mate("revolute")) is False
    assert is_joint_mate(make_mate("fixed")) is False
    assert is_joint_mate(make_mate("fastened")) is False
    assert is_joint_mate(make_mate("JOINT_upper")) is False
    assert is_joint_mate(None) is False
    assert is_joint_mate({"name": "joint_revolute"}) is True
    assert is_joint_mate({"name": "fixed"}) is False
    assert is_joint_mate({}) is False


def create_mock_mate_with_entities(name, parent_node_occ, mate_type="REVOLUTE"):
    m = MagicMock()
    m.name = name
    m.mateType = mate_type
    m.limits = None
    m.id = f"id_{name}"

    e1 = MagicMock()
    e1.matedOccurrence = [parent_node_occ]
    e1.matedCS.to_tf = np.eye(4)

    e2 = MagicMock()
    e2.matedOccurrence = ["some_child_occ"]
    e2.matedCS.to_tf = np.eye(4)

    m.matedEntities = [e1, e2]
    return m


def test_condensed_robot_from_graph_simple():
    graph = MagicMock()

    class MockNode:
        def __init__(self, id, name):
            self.id = id
            self.part_id = f"p_{id}"
            self.occurrence = f"occ_{id}"
            self.part_name = name
            self.parent = None

    node_a = MockNode("A", "PartA")
    node_b = MockNode("B", "PartB")
    node_c = MockNode("C", "PartC")

    graph.nodes = [node_a, node_b, node_c]

    class MockEdge:
        def __init__(self, u, v, name, mate_type="REVOLUTE"):
            self.u = u
            self.v = v
            if mate_type == "REVOLUTE":
                self.mate = create_mock_mate_with_entities(
                    name, u.occurrence, mate_type
                )
            else:
                self.mate = MagicMock()
                self.mate.name = name
                self.mate.mateType = mate_type
                self.mate.limits = None

    edges = [
        MockEdge(node_a, node_b, "fixed_mate", "FASTENED"),
        MockEdge(node_b, node_c, "joint_1", "REVOLUTE"),
    ]
    graph.edges = edges

    cad = MagicMock()
    cad.get_transform.return_value = np.eye(4)

    mate_values = {"id_joint_1": {"rotationZ": 0.0}}
    robot = CondensedRobot.from_graph(
        graph, cad=cad, name="TestRobot", mate_values=mate_values
    )

    nodes = list(robot.nodes(data=True))
    assert len(nodes) == 2

    link_names = [data["link"].name for _, data in nodes]
    assert "parta" in link_names or "partb" in link_names
    assert "partc" in link_names

    edges = list(robot.edges)
    assert len(edges) == 1
    edge_data = robot.get_edge_data(edges[0][0], edges[0][1])
    assert edge_data["joint"].name == "joint_1"


def test_condensed_robot_name_collision():
    graph = MagicMock()

    class MockNode:
        def __init__(self, name):
            self.part_name = name
            self.part_id = name
            self.occurrence = name

    node1 = MockNode("Link")
    node2 = MockNode("Link")

    edge = MagicMock()
    edge.u = node1
    edge.v = node2
    edge.mate = create_mock_mate_with_entities("joint_1", node1.occurrence)

    graph.nodes = [node1, node2]
    graph.edges = [edge]

    cad = MagicMock()
    cad.get_transform.return_value = np.eye(4)

    mate_values = {"id_joint_1": {"rotationZ": 0.0}}

    robot = CondensedRobot.from_graph(graph, cad=cad, mate_values=mate_values)
    nodes = list(robot.nodes(data=True))
    assert len(nodes) == 2
    names = sorted([data["link"].name for _, data in nodes])
    assert names == ["link", "link_1"]


def test_condensed_robot_self_loop():
    graph = MagicMock()

    class MockNode:
        def __init__(self, name):
            self.part_name = name
            self.part_id = name
            self.occurrence = name

    node_a = MockNode("A")
    node_b = MockNode("B")
    graph.nodes = [node_a, node_b]

    class MockEdge:
        def __init__(self, u, v, name, is_joint=False):
            self.u = u
            self.v = v
            if is_joint:
                self.mate = create_mock_mate_with_entities(name, u.occurrence)
            else:
                self.mate = MagicMock()
                self.mate.name = name
                self.mate.mateType = "FASTENED"
                self.mate.limits = None

    graph.edges = [
        MockEdge(node_a, node_b, "fixed_mate", is_joint=False),
        MockEdge(node_a, node_b, "joint_1", is_joint=True),
    ]

    cad = MagicMock()
    cad.get_transform.return_value = np.eye(4)

    robot = CondensedRobot.from_graph(graph, cad=cad)
    assert len(list(robot.nodes)) == 1
    assert len(list(robot.edges)) == 0


def test_condensed_robot_fail_fast():
    graph = MagicMock()
    node_a = MagicMock()
    node_a.part_id = "A"
    node_a.occurrence = "occ_A"
    node_a.part_name = "PartA"

    node_b = MagicMock()
    node_b.part_id = "B"
    node_b.occurrence = "occ_B"
    node_b.part_name = "PartB"

    graph.nodes = [node_a, node_b]

    mate = create_mock_mate_with_entities("joint_1", "occ_A")

    class MockEdge:
        def __init__(self, u, v, m):
            self.u, self.v, self.mate = u, v, m

    graph.edges = [MockEdge(node_a, node_b, mate)]

    cad = MagicMock()
    cad.get_transform.return_value = None

    with pytest.raises(RuntimeError, match="Could not find part"):
        CondensedRobot.from_graph(graph, cad=cad, fail_fast=True)


def test_condensed_robot_virtual_frames():
    """Test generation of virtual links from frame_* mate connectors."""
    graph = MagicMock()

    # Setup simple graph with one link "base"
    class MockNode:
        def __init__(self, id, name):
            self.id = id
            self.part_id = f"p_{id}"
            self.path = [f"occ_{id}"]  # Add path for to_id_tuple
            self.occurrence = [f"occ_{id}"]  # List to match typical path
            self.part_name = name
            self.parent = None

    node_base = MockNode("base", "BasePart")
    graph.nodes = [node_base]
    graph.edges = []  # No joints initially

    # Mock CAD
    cad = MagicMock()
    cad.get_transform.return_value = np.eye(4)

    # Mock PathKey
    pk_base = MagicMock()
    pk_base.path = tuple(node_base.occurrence)

    # Mock keys_by_id
    cad.keys_by_id = {tuple(node_base.occurrence): pk_base}

    # Mock Mate Connectors
    mc_valid = MagicMock()
    mc_valid.name = "frame_tcp"
    mc_valid.occurrence = node_base.occurrence
    mc_valid.mateConnectorCS.to_tf = np.eye(4)
    mc_valid.mateConnectorCS.to_tf[0, 3] = 0.1

    mc_ignored_name = MagicMock()
    mc_ignored_name.name = "mate_connector_1"  # No frame_ prefix

    mc_no_occ = MagicMock()
    mc_no_occ.name = "frame_broken"
    mc_no_occ.occurrence = None

    mc_unknown_occ = MagicMock()
    mc_unknown_occ.name = "frame_unknown"
    mc_unknown_occ.occurrence = ["occ_unknown"]

    # Add a mock link that won't be in the graph to test "Parent link not found" (harder to mock with current setup, skipping)

    cad.mate_connectors = [mc_valid, mc_ignored_name, mc_no_occ, mc_unknown_occ]

    # Run
    robot = CondensedRobot.from_graph(graph, cad=cad, name="TestRobot")

    # Verify
    nodes = list(robot.nodes(data=True))
    link_names = [data["link"].name for _, data in nodes]
    assert "basepart" in link_names
    assert "frame_tcp" in link_names
    assert "mate_connector_1" not in link_names
    assert "frame_broken" not in link_names
    assert "frame_unknown" not in link_names

    # Verify Joint for valid frame
    edges = list(robot.edges(data=True))
    assert len(edges) == 1
    u, v, data = edges[0]

    joint = data["joint"]
    assert joint.parent == "basepart"
    assert joint.child == "frame_tcp"
    assert joint.joint_type == "fixed"
    assert joint.name == "fixed_frame_tcp"
    assert np.allclose(joint.origin.xyz, [0.1, 0.0, 0.0])


def test_condensed_robot_invert_direction_flips_axis_sign():
    graph = MagicMock()

    class MockNode:
        def __init__(self, name):
            self.part_name = name
            self.part_id = name
            self.occurrence = f"occ_{name}"

    node_a = MockNode("A")
    node_b = MockNode("B")

    edge = MagicMock()
    edge.u = node_a
    edge.v = node_b
    edge.mate = create_mock_mate_with_entities("joint_1", node_a.occurrence)

    graph.nodes = [node_a, node_b]
    graph.edges = [edge]

    cad = MagicMock()
    cad.get_transform.return_value = np.eye(4)
    cad.mates = {}
    cad.keys_by_id = {}

    robot_default = CondensedRobot.from_graph(
        graph,
        cad=cad,
        mate_values={"id_joint_1": {"rotationZ": 0.0}},
    )
    default_joint = list(robot_default.edges(data=True))[0][2]["joint"]

    robot_inverted = CondensedRobot.from_graph(
        graph,
        cad=cad,
        mate_values={"id_joint_1": {"rotationZ": 0.0, "invert_direction": True}},
    )
    inverted_joint = list(robot_inverted.edges(data=True))[0][2]["joint"]

    assert default_joint.axis[2] == -1.0
    assert inverted_joint.axis[2] == 1.0
    assert inverted_joint.axis[2] == -default_joint.axis[2]


def test_condensed_robot_invert_direction_keeps_pose_correction():
    graph = MagicMock()

    class MockNode:
        def __init__(self, name):
            self.part_name = name
            self.part_id = name
            self.occurrence = f"occ_{name}"

    node_a = MockNode("A")
    node_b = MockNode("B")

    edge = MagicMock()
    edge.u = node_a
    edge.v = node_b
    edge.mate = create_mock_mate_with_entities("joint_1", node_a.occurrence)

    graph.nodes = [node_a, node_b]
    graph.edges = [edge]

    cad = MagicMock()
    cad.get_transform.return_value = np.eye(4)
    cad.mates = {}
    cad.keys_by_id = {}

    robot_default = CondensedRobot.from_graph(
        graph,
        cad=cad,
        mate_values={"id_joint_1": {"rotationZ": 0.5}},
    )
    default_joint = list(robot_default.edges(data=True))[0][2]["joint"]
    default_child_link = next(
        data["link"]
        for _, data in robot_default.nodes(data=True)
        if data["link"].name == default_joint.child
    )

    robot_inverted = CondensedRobot.from_graph(
        graph,
        cad=cad,
        mate_values={"id_joint_1": {"rotationZ": 0.5, "invert_direction": True}},
    )
    inverted_joint = list(robot_inverted.edges(data=True))[0][2]["joint"]
    inverted_child_link = next(
        data["link"]
        for _, data in robot_inverted.nodes(data=True)
        if data["link"].name == inverted_joint.child
    )

    np.testing.assert_allclose(
        default_child_link.frame_transform,
        inverted_child_link.frame_transform,
        atol=1e-7,
    )
    assert inverted_joint.axis[2] == -default_joint.axis[2]


def test_condensed_robot_invert_direction_flips_joint_limits():
    graph = MagicMock()

    class MockNode:
        def __init__(self, name):
            self.part_name = name
            self.part_id = name
            self.occurrence = f"occ_{name}"

    node_a = MockNode("A")
    node_b = MockNode("B")

    edge = MagicMock()
    edge.u = node_a
    edge.v = node_b
    edge.mate = create_mock_mate_with_entities("joint_1", node_a.occurrence)
    edge.mate.limits = {"min": -0.2, "max": 1.0, "effort": 5.0, "velocity": 2.0}

    graph.nodes = [node_a, node_b]
    graph.edges = [edge]

    cad = MagicMock()
    cad.get_transform.return_value = np.eye(4)
    cad.mates = {}
    cad.keys_by_id = {}

    robot_default = CondensedRobot.from_graph(
        graph,
        cad=cad,
        mate_values={"id_joint_1": {"rotationZ": 0.0}},
    )
    default_joint = list(robot_default.edges(data=True))[0][2]["joint"]

    robot_inverted = CondensedRobot.from_graph(
        graph,
        cad=cad,
        mate_values={"id_joint_1": {"rotationZ": 0.0, "invert_direction": True}},
    )
    inverted_joint = list(robot_inverted.edges(data=True))[0][2]["joint"]

    assert default_joint.limits["min"] == -0.2
    assert default_joint.limits["max"] == 1.0
    assert inverted_joint.limits["min"] == -1.0
    assert inverted_joint.limits["max"] == 0.2
    assert inverted_joint.limits["effort"] == 5.0
    assert inverted_joint.limits["velocity"] == 2.0
