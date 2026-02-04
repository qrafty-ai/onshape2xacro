import numpy as np
import pytest
from scipy.spatial.transform import Rotation
from unittest.mock import MagicMock

from onshape2xacro.condensed_robot import CondensedRobot
from onshape2xacro.module_boundary import (
    InterfaceMateInfo,
    ModuleBoundaryInfo,
    calculate_interface_transforms,
)


class MockNode:
    def __init__(self, name, part_id, occurrence):
        self.part_name = name
        self.part_id = part_id
        self.occurrence = occurrence
        self.parent = None


class MockEdge:
    def __init__(self, u, v, mate):
        self.u = u
        self.v = v
        self.mate = mate


class MockCS:
    def __init__(self, tf):
        self.to_tf = tf


class MockEntity:
    def __init__(self, occurrence, tf):
        self.matedOccurrence = occurrence
        self.matedCS = MockCS(tf)


class MockMate:
    def __init__(self, name, parent_occ, child_occ, T_PJ):
        self.name = name
        self.mateType = "REVOLUTE"
        self.id = f"id_{name}"
        self.limits = None
        self.matedEntities = [
            MockEntity(parent_occ, T_PJ),
            MockEntity(child_occ, np.eye(4)),
        ]


def build_fixture(T_WP_parent, T_PJ, mate_name="joint_1"):
    graph = MagicMock()

    parent_node = MockNode("parentlink", "p_parent", "occ_parent")
    child_node = MockNode("childlink", "p_child", "occ_child")

    mate = MockMate(mate_name, "occ_parent", "occ_child", T_PJ)

    graph.nodes = [parent_node, child_node]
    graph.edges = [MockEdge(parent_node, child_node, mate)]

    parent_key = "parent_key"
    child_key = "child_key"

    cad = MagicMock()
    cad.get_transform.return_value = T_WP_parent
    cad.keys_by_id = {("occ_parent",): parent_key, ("occ_child",): child_key}
    cad.parts = {
        parent_key: MagicMock(rigidAssemblyKey=None),
        child_key: MagicMock(rigidAssemblyKey=None),
    }
    cad.mate_connectors = []
    cad.mates = {(None, parent_key, child_key): mate}

    child_module = MagicMock()
    boundaries = ModuleBoundaryInfo(
        subassembly_parts={child_module: {child_node}},
        interface_mates={
            mate.id: InterfaceMateInfo(
                owner_module=None,
                parent_part=parent_node,
                child_part=child_node,
                child_module=child_module,
                mate=mate,
            )
        },
        root_parts={parent_node},
    )

    mate_values = {mate.id: {"rotationZ": 0.0}}
    robot = CondensedRobot.from_graph(graph, cad=cad, mate_values=mate_values)

    return cad, robot, boundaries, child_module, parent_node, child_node


def test_interface_transform_origin_matches_parent_mate():
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

    psi = np.pi / 2
    Rz_90 = np.array(
        [
            [np.cos(psi), -np.sin(psi), 0],
            [np.sin(psi), np.cos(psi), 0],
            [0, 0, 1],
        ]
    )
    T_PJ = np.eye(4)
    T_PJ[:3, :3] = Rz_90
    T_PJ[0, 3] = 0.1

    cad, robot, boundaries, child_module, _, _ = build_fixture(T_WP_parent, T_PJ)

    transforms = calculate_interface_transforms(cad, robot, boundaries)
    info = transforms[child_module]

    assert info.parent_link == "parentlink"
    assert info.child_root_link == "childlink"
    assert info.axis == (0.0, 0.0, 1.0)

    actual_matrix = np.eye(4)
    actual_matrix[:3, 3] = info.origin.xyz
    r = Rotation.from_euler("xyz", info.origin.rpy)
    actual_matrix[:3, :3] = r.as_matrix()

    np.testing.assert_allclose(actual_matrix, T_PJ, atol=1e-7)


def test_interface_transform_skips_duplicate_child_modules():
    T_WP_parent = np.eye(4)
    T_PJ = np.eye(4)

    cad, robot, boundaries, child_module, parent_node, child_node = build_fixture(
        T_WP_parent, T_PJ
    )

    other_mate = MockMate("joint_2", "occ_parent", "occ_child", np.eye(4))
    boundaries.interface_mates[other_mate.id] = InterfaceMateInfo(
        owner_module=None,
        parent_part=parent_node,
        child_part=child_node,
        child_module=child_module,
        mate=other_mate,
    )

    transforms = calculate_interface_transforms(cad, robot, boundaries)
    assert list(transforms.keys()) == [child_module]
