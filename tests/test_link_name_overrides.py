import pytest
import numpy as np
import networkx as nx
from unittest.mock import MagicMock
from onshape2xacro.condensed_robot import CondensedRobot


@pytest.fixture
def mock_cad():
    cad = MagicMock()
    cad.mates = {}
    cad.get_transform.return_value = np.eye(4)
    cad.keys_by_id = {}
    return cad


def test_link_name_override_applied(mock_cad):
    # Setup graph with two parts fixed together (one link)
    graph = nx.Graph()
    graph.add_node(
        "part1", data=MagicMock(part_name="Part 1", mass_properties=MagicMock(mass=1.0))
    )
    graph.add_node(
        "part2", data=MagicMock(part_name="Part 2", mass_properties=MagicMock(mass=0.5))
    )
    graph.add_edge("part1", "part2", data={"mate": MagicMock(name="fixed_mate")})

    # The generated name would be "part_1" (sanitized "Part 1")
    overrides = {"part_1": "base_link"}

    robot = CondensedRobot.from_graph(
        graph, cad=mock_cad, link_name_overrides=overrides
    )

    assert "base_link" in robot.nodes
    assert "part_1" not in robot.nodes


def test_multiple_overrides(mock_cad):
    # Setup graph with two separate links
    graph = nx.Graph()
    graph.add_node(
        "part1", data=MagicMock(part_name="Part 1", mass_properties=MagicMock(mass=1.0))
    )
    graph.add_node(
        "part2", data=MagicMock(part_name="Part 2", mass_properties=MagicMock(mass=1.0))
    )

    # Mock a joint between them
    joint_mate = MagicMock()
    joint_mate.name = "joint_1"
    joint_mate.mateType = "REVOLUTE"
    joint_mate.id = "mate_id_1"
    joint_mate.matedEntities = []
    # Add as 'mate' attribute directly
    graph.add_edge("part1", "part2", mate=joint_mate)

    overrides = {"part_1": "link_a", "part_2": "link_b"}

    # We need mate_values for the joint
    mate_values = {"mate_id_1": {"rotationZ": 0.0}}

    robot = CondensedRobot.from_graph(
        graph, cad=mock_cad, link_name_overrides=overrides, mate_values=mate_values
    )

    assert "link_a" in robot.nodes
    assert "link_b" in robot.nodes


def test_duplicate_target_names_raise_error(mock_cad):
    # Setup graph with two separate links
    graph = nx.Graph()
    graph.add_node(
        "part1", data=MagicMock(part_name="Part 1", mass_properties=MagicMock(mass=1.0))
    )
    graph.add_node(
        "part2", data=MagicMock(part_name="Part 2", mass_properties=MagicMock(mass=1.0))
    )

    joint_mate = MagicMock()
    joint_mate.name = "joint_1"
    joint_mate.mateType = "REVOLUTE"
    joint_mate.id = "mate_id_1"
    joint_mate.matedEntities = []
    graph.add_edge("part1", "part2", mate=joint_mate)

    overrides = {"part_1": "duplicate_name", "part_2": "duplicate_name"}

    mate_values = {"mate_id_1": {"rotationZ": 0.0}}

    with pytest.raises(RuntimeError) as excinfo:
        CondensedRobot.from_graph(
            graph, cad=mock_cad, link_name_overrides=overrides, mate_values=mate_values
        )

    assert "Link name collision" in str(excinfo.value)
    assert "duplicate_name" in str(excinfo.value)


def test_no_overrides_works_as_before(mock_cad):
    graph = nx.Graph()
    graph.add_node(
        "part1", data=MagicMock(part_name="Part 1", mass_properties=MagicMock(mass=1.0))
    )

    robot = CondensedRobot.from_graph(graph, cad=mock_cad)

    assert "part_1" in robot.nodes
