import numpy as np
from unittest.mock import MagicMock

from onshape2xacro.condensed_robot import CondensedRobot
from onshape2xacro.module_boundary import InterfaceMateInfo, ModuleBoundaryInfo


class MockNode:
    def __init__(self, part_id, part_name):
        self.part_id = part_id
        self.part_name = part_name
        self.occurrence = f"occ_{part_id}"
        self.parent = None


class MockEdge:
    def __init__(self, u, v, mate):
        self.u = u
        self.v = v
        self.mate = mate


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
    e2.matedOccurrence = ["child_occ"]
    e2.matedCS.to_tf = np.eye(4)

    m.matedEntities = [e1, e2]
    return m


def create_fixed_mate(name, mate_id=None):
    m = MagicMock()
    m.name = name
    m.mateType = "FASTENED"
    m.limits = None
    m.id = mate_id or f"id_{name}"
    return m


def make_cad():
    cad = MagicMock()
    cad.get_transform.return_value = np.eye(4)
    cad.mates = {}
    cad.keys_by_id = {}
    cad.mate_connectors = []
    return cad


def test_boundary_condensing_respects_subassemblies():
    graph = MagicMock()

    node_a1 = MockNode("A1", "PartA1")
    node_a2 = MockNode("A2", "PartA2")
    node_b1 = MockNode("B1", "PartB1")
    graph.nodes = [node_a1, node_a2, node_b1]

    fixed_a = create_fixed_mate("fixed_a")
    fixed_cross = create_fixed_mate("fixed_cross")
    joint_ab = create_mock_mate_with_entities("joint_ab", node_a2.occurrence)

    graph.edges = [
        MockEdge(node_a1, node_a2, fixed_a),
        MockEdge(node_a2, node_b1, fixed_cross),
        MockEdge(node_a2, node_b1, joint_ab),
    ]

    module_a = MagicMock()
    module_b = MagicMock()
    boundaries = ModuleBoundaryInfo(
        subassembly_parts={module_a: {node_a1, node_a2}, module_b: {node_b1}},
        interface_mates={},
        root_parts=set(),
    )

    mate_values = {"id_joint_ab": {"rotationZ": 0.0}}
    robot = CondensedRobot.from_graph(
        graph,
        cad=make_cad(),
        mate_values=mate_values,
        module_boundaries=boundaries,
    )

    link_part_sets = [
        set(data["link"].part_names) for _, data in robot.nodes(data=True)
    ]
    assert {"PartA1", "PartA2"} in link_part_sets
    assert {"PartB1"} in link_part_sets

    edges = list(robot.edges(data=True))
    assert len(edges) == 1
    assert edges[0][2]["joint"].name == "joint_ab"


def test_interface_mate_skips_union():
    graph = MagicMock()

    node_x1 = MockNode("X1", "PartX1")
    node_x2 = MockNode("X2", "PartX2")
    graph.nodes = [node_x1, node_x2]

    fixed_x = create_fixed_mate("fixed_x", mate_id="fixed_x")
    joint_x = create_mock_mate_with_entities("joint_x", node_x1.occurrence)
    graph.edges = [
        MockEdge(node_x1, node_x2, fixed_x),
        MockEdge(node_x1, node_x2, joint_x),
    ]

    module_x = MagicMock()
    boundaries = ModuleBoundaryInfo(
        subassembly_parts={module_x: {node_x1, node_x2}},
        interface_mates={
            "fixed_x": InterfaceMateInfo(
                owner_module=module_x,
                parent_part=node_x1,
                child_part=node_x2,
                child_module=module_x,
                mate=fixed_x,
            )
        },
        root_parts=set(),
    )

    mate_values = {"id_joint_x": {"rotationZ": 0.0}}
    robot = CondensedRobot.from_graph(
        graph,
        cad=make_cad(),
        mate_values=mate_values,
        module_boundaries=boundaries,
    )

    assert len(list(robot.nodes)) == 2
    assert len(list(robot.edges)) == 1
