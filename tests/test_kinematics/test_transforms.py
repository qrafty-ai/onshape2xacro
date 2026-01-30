import numpy as np
from unittest.mock import MagicMock
from onshape2xacro.condensed_robot import CondensedRobot


def test_condensed_robot_transforms():
    """
    Verifies that CondensedRobot correctly calculates link and joint transforms.

    Setup:
    - 2 links: parentlink and childlink.
    - parentlink is root.
    - One joint (joint_1) connecting parentlink to childlink.
    - Parent Part is translated by [0, 1, 0] in world.
    - Joint is translated by [0.1, 0, 0] relative to Parent Part.

    Expected:
    - parentlink.frame_transform = T_WP_parent = Translation([0, 1, 0])
    - childlink.frame_transform = T_WP_parent @ T_PJ = Translation([0.1, 1.0, 0.0])
    - Joint.origin = inv(parentlink.frame_transform) @ childlink.frame_transform = T_PJ = Translation([0.1, 0, 0])
    """
    # 1. Create a simple 2-link robot (Parent -> Joint -> Child).
    graph = MagicMock()

    class Node:
        def __init__(self, name, part_id, occurrence):
            self.part_name = name
            self.part_id = part_id
            self.occurrence = occurrence
            self.parent = None

    node_parent = Node("parentlink", "p_parent", "occ_parent")
    node_child = Node("childlink", "p_child", "occ_child")

    # Mocking graph.nodes
    graph.nodes = {
        node_parent: {"data": node_parent},
        node_child: {"data": node_child},
    }

    # 2. Mock 'mate' with 'matedEntities' and 'matedCS.to_tf'.
    class MockCS:
        def __init__(self, tf):
            self.to_tf = tf

    class MockEntity:
        def __init__(self, occurrence, tf):
            self.matedOccurrence = occurrence
            self.matedCS = MockCS(tf)

    class MockMate:
        def __init__(self, name, parent_occ, T_PJ):
            self.name = name
            self.mateType = "REVOLUTE"
            self.limits = None
            self.matedEntities = [MockEntity(parent_occ, T_PJ)]

    # Transform from Parent Part to Joint: 10cm in X
    T_PJ = np.eye(4)
    T_PJ[0, 3] = 0.1

    mate = MockMate("joint_1", "occ_parent", T_PJ)

    # Mocking edges for _iter_edges
    class MockEdge:
        def __init__(self, u, v, mate):
            self.u = u
            self.v = v
            self.mate = mate

    graph.edges = [MockEdge(node_parent, node_child, mate)]

    # 3. Mock 'cad' with parts having 'worldToPartTF.to_tf'.
    # Parent Part World Transform: 1m in Y
    T_WP_parent = np.eye(4)
    T_WP_parent[1, 3] = 1.0

    class MockPart:
        def __init__(self, occurrence, T_WP):
            self.occurrence = occurrence
            self.worldToPartTF = MagicMock()
            self.worldToPartTF.to_tf = T_WP

    cad = MagicMock()
    cad.parts = {"p_parent": MockPart("occ_parent", T_WP_parent)}
    cad.occurrences = {}
    cad.get_transform.return_value = T_WP_parent

    # 4. Run CondensedRobot.from_graph
    robot = CondensedRobot.from_graph(graph, cad=cad)

    # Find links
    links = {data["link"].name: data["link"] for _, data in robot.nodes(data=True)}
    assert "parentlink" in links
    assert "childlink" in links

    parent_link = links["parentlink"]
    child_link = links["childlink"]

    # - LinkRecord.frame_transform is correctly calculated.
    # Root link (ParentLink) should now have the part's world transform
    np.testing.assert_allclose(parent_link.frame_transform, T_WP_parent)

    # Child link frame_transform should be T_WJ = T_WP_parent @ T_PJ
    T_WJ = T_WP_parent @ T_PJ
    np.testing.assert_allclose(child_link.frame_transform, T_WJ)

    # - JointRecord.origin is correctly calculated (parent frame to joint frame).
    edges = list(robot.edges(data=True))
    assert len(edges) == 1
    joint_rec = edges[0][2]["joint"]

    # Joint origin translation should be [0.1, 0, 0] (local offset)
    expected_xyz = np.array([0.1, 0.0, 0.0])
    np.testing.assert_allclose(joint_rec.origin.xyz, expected_xyz)
    np.testing.assert_allclose(joint_rec.origin.rpy, [0.0, 0.0, 0.0], atol=1e-7)
