from onshape_robotics_toolkit.parse import CAD, PathKey
from onshape_robotics_toolkit.models.assembly import (
    PartInstance,
    Occurrence,
    Part,
    MateFeatureData,
)
from onshape2xacro.cad_utils import cad_to_serializable, serializable_to_cad


def test_cad_round_trip():
    pk1 = PathKey(("id1",), ("name1",))
    pk2 = PathKey(("id1", "id2"), ("name1", "name2"))

    # Create some dummy objects using model_construct to bypass validation
    instance1 = PartInstance.model_construct(
        id="id1",
        name="name1",
        partId="p1",
        documentId="d1" * 12,  # 24 chars
        elementId="e1" * 12,  # 24 chars
        type="Part",
    )
    occurrence1 = Occurrence.model_construct(
        path=["id1", "id2"],
        transform=[1.0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1],
    )
    part1 = Part.model_construct(
        partId="p1", name="part1", elementId="e1" * 12, documentId="d1" * 12
    )

    cad = CAD(
        document_id="doc" * 8,
        element_id="ele" * 8,
        wtype="w",
        workspace_id="ws" * 12,
        document_microversion="mv" * 12,
        name="test_cad",
        max_depth=2,
    )
    cad.keys_by_id = {("id1",): pk1, ("id1", "id2"): pk2}
    cad.keys_by_name = {("name1",): pk1, ("name1", "name2"): pk2}
    cad.instances = {pk1: instance1}
    cad.occurrences = {pk2: occurrence1}
    cad.subassemblies = {}
    cad.mates = {}
    cad.patterns = {}
    cad.parts = {pk1: part1}

    # Convert to serializable
    scad = cad_to_serializable(cad)

    assert scad.document_id == "doc" * 8
    assert "id1" in scad.keys_by_id
    assert "id1/id2" in scad.keys_by_id
    assert "id1" in scad.instances
    assert "id1/id2" in scad.occurrences

    # Convert back
    cad_back = serializable_to_cad(scad)

    assert cad_back.document_id == cad.document_id
    assert cad_back.element_id == cad.element_id
    assert cad_back.name == cad.name
    assert cad_back.max_depth == cad.max_depth

    assert pk1 in cad_back.instances
    assert pk2 in cad_back.occurrences
    assert pk1 in cad_back.parts

    assert cad_back.keys_by_id[("id1",)] == pk1
    assert cad_back.keys_by_name[("name1",)] == pk1


def test_mates_round_trip():
    pk_sub = PathKey(("sub",), ("Sub",))
    pk_mate = PathKey(("sub", "mate"), ("Sub", "Mate"))
    pk_owner = PathKey(("sub", "owner"), ("Sub", "Owner"))

    mate_key = (pk_sub, pk_mate, pk_owner)
    dummy_mate = MateFeatureData.model_construct(name="test_mate")

    cad = CAD(
        document_id="doc" * 8,
        element_id="ele" * 8,
        wtype="w",
        workspace_id="ws" * 12,
        document_microversion="mv" * 12,
        name="test_cad",
        max_depth=2,
    )
    cad.keys_by_id = {
        ("sub",): pk_sub,
        ("sub", "mate"): pk_mate,
        ("sub", "owner"): pk_owner,
    }
    cad.keys_by_name = {
        ("Sub",): pk_sub,
        ("Sub", "Mate"): pk_mate,
        ("Sub", "Owner"): pk_owner,
    }
    cad.instances = {}
    cad.occurrences = {}
    cad.subassemblies = {}
    cad.mates = {mate_key: dummy_mate}
    cad.patterns = {}
    cad.parts = {}

    scad = cad_to_serializable(cad)
    assert "sub::sub/mate::sub/owner" in scad.mates

    cad_back = serializable_to_cad(scad)
    assert mate_key in cad_back.mates
    assert cad_back.mates[mate_key].name == "test_mate"


def test_mate_with_none_round_trip():
    pk_mate = PathKey(("mate",), ("Mate",))
    pk_owner = PathKey(("owner",), ("Owner",))

    mate_key = (None, pk_mate, pk_owner)
    dummy_mate = MateFeatureData.model_construct(name="test_mate_none")

    cad = CAD(
        document_id="doc" * 8,
        element_id="ele" * 8,
        wtype="w",
        workspace_id="ws" * 12,
        document_microversion="mv" * 12,
        name="test_cad",
        max_depth=2,
    )
    cad.keys_by_id = {("mate",): pk_mate, ("owner",): pk_owner}
    cad.keys_by_name = {("Mate",): pk_mate, ("Owner",): pk_owner}
    cad.instances = {}
    cad.occurrences = {}
    cad.subassemblies = {}
    cad.mates = {mate_key: dummy_mate}
    cad.patterns = {}
    cad.parts = {}

    scad = cad_to_serializable(cad)
    assert "::mate::owner" in scad.mates

    cad_back = serializable_to_cad(scad)
    assert mate_key in cad_back.mates
    assert cad_back.mates[mate_key].name == "test_mate_none"
