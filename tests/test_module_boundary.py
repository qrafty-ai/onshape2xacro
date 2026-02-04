import pytest
from unittest.mock import MagicMock
from onshape2xacro.module_boundary import (
    detect_module_boundaries,
    ModuleBoundaryInfo,
    InterfaceMateInfo,
)


class MockNode:
    def __init__(self, path):
        self.path = tuple(path)

    def __repr__(self):
        return f"Node({self.path})"


def test_module_boundary_detection_simple():
    # Setup CAD mock
    cad = MagicMock()

    # Define subassemblies
    # Root
    #  - Part P1, P2
    #  - SubAsm A (instance A1)
    #    - Part AP1
    #    - SubAsm B (instance B1)
    #      - Part BP1

    a1_key = MagicMock()
    a1_key.path = ("A1",)

    b1_key = MagicMock()
    b1_key.path = ("A1", "B1")

    cad.subassemblies = {a1_key: MagicMock(), b1_key: MagicMock()}

    # Parts
    p1 = MockNode(["P1"])
    p2 = MockNode(["P2"])
    ap1 = MockNode(["A1", "AP1"])
    bp1 = MockNode(["A1", "B1", "BP1"])

    cad.parts = {
        MockNode(["P1"]): MagicMock(),
        MockNode(["P2"]): MagicMock(),
        MockNode(["A1", "AP1"]): MagicMock(),
        MockNode(["A1", "B1", "BP1"]): MagicMock(),
    }

    # Kinematic graph mock
    graph = MagicMock()
    graph.nodes = [p1, p2, ap1, bp1]

    # Mates
    # joint_R1: P1 -> AP1 (defined in Root)
    mate_r1 = MagicMock()
    mate_r1.id = "mate_r1"
    mate_r1.name = "joint_r1"

    # joint_A1: AP1 -> BP1 (defined in SubAsm A)
    mate_a1 = MagicMock()
    mate_a1.id = "mate_a1"
    mate_a1.name = "joint_a1"

    # fixed_P1_P2: P1 -> P2 (defined in Root)
    mate_fixed = MagicMock()
    mate_fixed.id = "mate_fixed"
    mate_fixed.name = "fixed_p1_p2"

    cad.mates = {
        (None, "P1", "A1"): mate_r1,
        (a1_key, "AP1", "B1"): mate_a1,
        (None, "P1", "P2"): mate_fixed,
    }

    # We need to structure cad.mates correctly for the lookup in detect_module_boundaries
    # In my implementation: for (asm_key, _, _), mate in cad.mates.items():
    cad.mates = {
        (None, "P1", "A1"): mate_r1,
        (a1_key, "AP1", "B1"): mate_a1,
        (None, "P1", "P2"): mate_fixed,
    }

    graph.edges.return_value = [
        (p1, ap1, {"mate": mate_r1}),
        (ap1, bp1, {"mate": mate_a1}),
        (p1, p2, {"mate": mate_fixed}),
    ]

    # Run
    info = detect_module_boundaries(cad, graph)

    # Verify root parts
    assert p1 in info.root_parts
    assert p2 in info.root_parts
    assert ap1 not in info.root_parts
    assert bp1 not in info.root_parts

    # Verify subassembly parts
    assert a1_key in info.subassembly_parts
    assert ap1 in info.subassembly_parts[a1_key]
    assert b1_key in info.subassembly_parts
    assert bp1 in info.subassembly_parts[b1_key]
    # Check that BP1 is ONLY in B1 (direct parent)
    assert bp1 not in info.subassembly_parts[a1_key]

    # Verify interface mates
    assert "mate_r1" in info.interface_mates
    info_r1 = info.interface_mates["mate_r1"]
    assert info_r1.owner_module is None
    assert info_r1.child_module == a1_key

    assert "mate_a1" in info.interface_mates
    info_a1 = info.interface_mates["mate_a1"]
    assert info_a1.owner_module == a1_key
    assert info_a1.child_module == b1_key

    assert "mate_fixed" not in info.interface_mates  # Not a joint_


def test_module_boundary_sibling_subassemblies():
    # Root
    #  - SubAsm A (A1) -> Part AP1
    #  - SubAsm B (B1) -> Part BP1
    # joint_AB: AP1 -> BP1 (defined in Root)

    cad = MagicMock()

    a1_key = MagicMock()
    a1_key.path = ("A1",)
    b1_key = MagicMock()
    b1_key.path = ("B1",)

    cad.subassemblies = {a1_key: MagicMock(), b1_key: MagicMock()}

    ap1 = MockNode(["A1", "AP1"])
    bp1 = MockNode(["B1", "BP1"])

    graph = MagicMock()
    graph.nodes = [ap1, bp1]

    mate_ab = MagicMock()
    mate_ab.id = "mate_ab"
    mate_ab.name = "joint_ab"

    cad.mates = {(None, "A1", "B1"): mate_ab}

    graph.edges.return_value = [(ap1, bp1, {"mate": mate_ab})]

    info = detect_module_boundaries(cad, graph)

    assert "mate_ab" in info.interface_mates
    info_ab = info.interface_mates["mate_ab"]
    assert info_ab.owner_module is None
    # In sibling case, it picks one. Based on my implementation (v as child), it should be b1_key.
    assert info_ab.child_module == b1_key
