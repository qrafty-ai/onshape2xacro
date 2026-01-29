from unittest.mock import MagicMock
from onshape2xacro.condensed_robot import CondensedRobot, is_joint_mate


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
    assert is_joint_mate(make_mate("JOINT_upper")) is False  # case sensitive


def test_condensed_robot_from_graph_simple():
    # A --(fixed)--> B --(joint_1)--> C
    # Should result in 2 links: Link(A, B) and Link(C)

    graph = MagicMock()

    # Mocking nodes
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

    # Mocking edges
    class MockEdge:
        def __init__(self, u, v, name, mate_type="REVOLUTE"):
            self.u = u
            self.v = v
            self.mate = MagicMock()
            self.mate.name = name
            self.mate.mateType = mate_type
            # Mocking limits and other attributes if needed
            self.mate.limits = None

    edges = [
        MockEdge(node_a, node_b, "fixed_mate", "FASTENED"),
        MockEdge(node_b, node_c, "joint_1", "REVOLUTE"),
    ]
    graph.edges = edges

    robot = CondensedRobot.from_graph(graph, name="TestRobot")

    # Check nodes in robot
    # robot should have 2 nodes now
    nodes = list(robot.nodes(data=True))
    assert len(nodes) == 2

    # One node should contain PartA and PartB
    # Another node should contain PartC

    link_names = [data["link"].name for _, data in nodes]
    assert "parta_partb" in link_names or "partb_parta" in link_names
    assert "partc" in link_names

    # Check edges in robot
    edges = list(robot.edges)
    assert len(edges) == 1

    # The edge should be joint_1
    edge_data = robot.get_edge_data(edges[0][0], edges[0][1])
    assert edge_data["joint"].name == "joint_1"


def test_is_joint_mate_robust():
    assert is_joint_mate(None) is False
    assert is_joint_mate({"name": "joint_revolute"}) is True
    assert is_joint_mate({"name": "fixed"}) is False
    assert is_joint_mate({}) is False


def test_condensed_robot_name_collision():
    graph = MagicMock()

    class MockNode:
        def __init__(self, name):
            self.part_name = name
            self.part_id = name
            self.occurrence = name

    node1 = MockNode("Link")
    node2 = MockNode("Link")  # Different object, same name
    graph.nodes = [node1, node2]
    graph.edges = []

    robot = CondensedRobot.from_graph(graph)
    nodes = list(robot.nodes(data=True))
    assert len(nodes) == 2
    names = sorted([data["link"].name for _, data in nodes])
    assert names == ["link", "link_1"]


def test_condensed_robot_self_loop():
    # A --(fixed)--> B, A --(joint_1)--> B
    # Since A and B are grouped, joint_1 should be skipped
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
        def __init__(self, u, v, name):
            self.u = u
            self.v = v
            self.mate = MagicMock()
            self.mate.name = name
            self.mate.mateType = "REVOLUTE"
            self.mate.limits = None

    graph.edges = [
        MockEdge(node_a, node_b, "fixed_mate"),
        MockEdge(node_a, node_b, "joint_1"),
    ]

    robot = CondensedRobot.from_graph(graph)
    assert len(list(robot.nodes)) == 1
    assert len(list(robot.edges)) == 0


def test_condensed_robot_networkx_style():
    # Test using a structure that mimics NetworkX
    class MockGraph:
        def __init__(self):
            m = MagicMock()
            m.name = "joint_1"
            m.mateType = "REVOLUTE"
            self.nodes = {
                "n1": {"data": MagicMock(part_name="P1")},
                "n2": {"data": MagicMock(part_name="P2")},
            }
            self._edges = [("n1", "n2", {"data": m})]

        def edges(self, data=False):
            if data:
                return self._edges
            return [(u, v) for u, v, d in self._edges]

    graph = MockGraph()
    # Mocking needed attributes
    graph.nodes["n1"]["data"].part_id = "p1"
    graph.nodes["n1"]["data"].occurrence = "o1"
    graph.nodes["n2"]["data"].part_id = "p2"
    graph.nodes["n2"]["data"].occurrence = "o2"

    robot = CondensedRobot.from_graph(graph)
    assert len(list(robot.nodes)) == 2
    assert len(list(robot.edges)) == 1
    edge_data = list(robot.edges(data=True))[0][2]
    assert edge_data["joint"].name == "joint_1"
