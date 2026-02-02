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
            self.id = f"id_{name}"
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
    # Parent Part World Transform: 1m in Y, Rotated 90 deg around X
    theta = np.pi / 2
    Rx_90 = np.array(
        [
            [1, 0, 0, 0],
            [0, np.cos(theta), -np.sin(theta), 0],
            [0, np.sin(theta), np.cos(theta), 0],
            [0, 0, 0, 1],
        ]
    )
    T_WP_parent = np.eye(4)
    T_WP_parent[:3, :3] = Rx_90[:3, :3]
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
    mate_values = {"id_joint_1": {"rotationZ": 0.0}}
    robot = CondensedRobot.from_graph(graph, cad=cad, mate_values=mate_values)

    # Find links
    links = {data["link"].name: data["link"] for _, data in robot.nodes(data=True)}
    assert "parentlink" in links
    assert "childlink" in links

    parent_link = links["parentlink"]
    child_link = links["childlink"]

    # Root link frame should match the CAD API part world transform
    # (unified handling with child links - no special rotation correction)
    np.testing.assert_allclose(parent_link.frame_transform, T_WP_parent, atol=1e-7)

    # Child link frame_transform should be T_WJ = T_WP_parent @ T_PJ
    T_WJ = T_WP_parent @ T_PJ
    np.testing.assert_allclose(child_link.frame_transform, T_WJ, atol=1e-7)

    # Joint origin should be inv(parent_frame) @ child_frame
    edges = list(robot.edges(data=True))
    assert len(edges) == 1
    joint_rec = edges[0][2]["joint"]

    # With unified root/child handling:
    # expected_origin = inv(T_WP_parent) @ T_WJ
    expected_origin = np.linalg.inv(T_WP_parent) @ T_WJ

    # Construct actual matrix from Origin object
    from scipy.spatial.transform import Rotation

    actual_matrix = np.eye(4)
    actual_matrix[:3, 3] = joint_rec.origin.xyz
    r = Rotation.from_euler("xyz", joint_rec.origin.rpy)
    actual_matrix[:3, :3] = r.as_matrix()

    np.testing.assert_allclose(actual_matrix, expected_origin, atol=1e-7)
