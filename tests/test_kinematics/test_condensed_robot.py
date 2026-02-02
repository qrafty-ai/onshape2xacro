import numpy as np
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


def create_mock_mate_with_entities(name, parent_node_occ, mate_type="REVOLUTE"):
    m = MagicMock()
    m.name = name
    m.mateType = mate_type
    m.limits = None
    m.id = f"id_{name}"

    # Entity matching parent
    e1 = MagicMock()
    e1.matedOccurrence = [parent_node_occ]
    e1.matedCS.to_tf = np.eye(4)

    # Entity matching child (generic)
    e2 = MagicMock()
    e2.matedOccurrence = ["some_child_occ"]
    e2.matedCS.to_tf = np.eye(4)

    m.matedEntities = [e1, e2]
    return m


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
            if mate_type == "REVOLUTE":
                # For joint, we need proper entities to match parent
                # Here parent is B (for joint_1)
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

    # Check nodes in robot
    # robot should have 2 nodes now
    nodes = list(robot.nodes(data=True))
    assert len(nodes) == 2

    # One node should contain PartA and PartB
    # Another node should contain PartC

    link_names = [data["link"].name for _, data in nodes]
    # Check that we have a link for the A-B group and a link for C
    # Names are derived from heavy parts. PartA or PartB.
    # sanitize_name("PartA") -> "parta"
    group_name_found = "parta" in link_names or "partb" in link_names
    assert group_name_found
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

    # Add edge to avoid multiple roots error (must be joint to keep links separate)
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


def test_condensed_robot_networkx_style():
    # Test using a structure that mimics NetworkX
    class MockGraph:
        def __init__(self):
            m = create_mock_mate_with_entities("joint_1", "o1")
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

    cad = MagicMock()
    cad.get_transform.return_value = np.eye(4)

    mate_values = {"id_joint_1": {"rotationZ": 0.0}}

    robot = CondensedRobot.from_graph(graph, cad=cad, mate_values=mate_values)
    assert len(list(robot.nodes)) == 2
    assert len(list(robot.edges)) == 1
    edge_data = list(robot.edges(data=True))[0][2]
    assert edge_data["joint"].name == "joint_1"


class SimpleMockNode:
    def __init__(self, id, name, occurrence):
        self.part_id = id
        self.part_name = name
        self.occurrence = occurrence

    def __repr__(self):
        return f"Node({self.part_name})"


def test_condensed_robot_correction():
    # A --(joint_1)--> B
    # Joint 1 has value pi/2 (90 deg)
    graph = MagicMock()

    # Nodes
    node_a = SimpleMockNode("A", "PartA", "occ_A")
    node_b = SimpleMockNode("B", "PartB", "occ_B")

    graph.nodes = [node_a, node_b]

    # Edge
    mate = MagicMock()
    mate.name = "joint_1"
    mate.mateType = "REVOLUTE"
    mate.id = "feature_1"
    mate.limits = None

    # Mock mated entities for transform calculation
    # Parent (A) is at origin. Joint is at (0,0,1).
    # T_PJ (Parent -> Joint)
    T_PJ = np.eye(4)
    T_PJ[2, 3] = 1.0

    # Mated Entity
    entity_parent = MagicMock()
    entity_parent.matedOccurrence = ["occ_A"]  # Belongs to parent
    entity_parent.matedCS.to_tf = T_PJ

    entity_child = MagicMock()
    entity_child.matedOccurrence = ["occ_B"]  # Belongs to child

    # Set parent as the SECOND entity (index 1) to simulate standard "Part to Base" mate
    # where Base (Parent) is target (Index 1) and Part (Child) is Mover (Index 0).
    # Child (0) = Parent (1) * Value. -> Positive sign.
    mate.matedEntities = [entity_child, entity_parent]

    class MockEdge:
        def __init__(self, u, v, m):
            self.u = u
            self.v = v
            self.mate = m

    graph.edges = [MockEdge(node_a, node_b, mate)]

    # CAD Mock
    cad = MagicMock()
    # Mock get_transform. Returns Identity for A and B.
    cad.get_transform.return_value = np.eye(4)
    # Mock keys_by_id for resolution
    # parent_occ (occ_A) -> key_A
    key_a = MagicMock()
    key_b = MagicMock()
    # Mock PathKey behavior for equality check
    key_a.__eq__ = lambda s, o: o is key_a
    key_b.__eq__ = lambda s, o: o is key_b

    cad.keys_by_id = {("occ_A",): key_a, ("occ_B",): key_b}

    # Mock cad.mates for original mate lookup
    # Mate id="feature_1". Original entities: [key_b (child), key_a (parent)]
    # This corresponds to: Child (0) moves relative to Parent (1).
    # Since Parent is Entity 1, sign should be +1.0.
    # Key structure in CAD.mates is (assembly_key, parent_key, child_key)
    mock_mate_data = MagicMock()
    mock_mate_data.id = "feature_1"
    # Set up mates such that URDF Parent (key_a) is Original Parent (key_a)
    # This simulates Standard Traversal (Parent->Child)
    cad.mates = {(None, key_a, key_b): mock_mate_data}

    # Mock cad.parts for rigid root lookup
    part_a = MagicMock()
    part_a.rigidAssemblyKey = None
    part_b = MagicMock()
    part_b.rigidAssemblyKey = None
    cad.parts = {key_a: part_a, key_b: part_b}

    # Mate Values: 90 degrees rotation around Z
    mate_values = {"feature_1": {"rotationZ": np.pi / 2, "translationZ": 0.0}}

    robot = CondensedRobot.from_graph(
        graph, cad=cad, name="TestRobot", mate_values=mate_values
    )

    # Verify Child Link Frame (for Part B)
    # T_WJ = T_A @ T_PJ = I @ T_PJ = T_PJ (0,0,1)
    # Correction = RotZ(90)
    # T_target = T_WJ @ Correction

    # Find link name for B
    links = {data["link"].name: data["link"] for _, data in robot.nodes(data=True)}
    # Assuming PartA -> LinkA, PartB -> LinkB (names are sanitized part names)
    link_b = None
    for link in links.values():
        if any("B" in pid for pid in link.part_ids):
            link_b = link
            break

    assert link_b is not None

    T_actual = link_b.frame_transform

    # Expected:
    # T_WJ: translation (0,0,1)
    # Correction: Rotation 90 deg around Z
    # Result: Translation (0,0,1), Rotation 90 deg Z

    c, s = 0.0, 1.0  # cos(90)=0, sin(90)=1
    T_expected = np.eye(4)
    T_expected[:3, :3] = np.array([[c, -s, 0], [s, c, 0], [0, 0, 1]])
    T_expected[2, 3] = 1.0

    np.testing.assert_allclose(T_actual, T_expected, atol=1e-6)
