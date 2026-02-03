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
