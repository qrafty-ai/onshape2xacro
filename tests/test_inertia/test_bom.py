"""Tests for BOM parser."""

import pytest
from pathlib import Path
from onshape2xacro.inertia.bom import BOMParser, BOMEntry


@pytest.fixture
def bom_csv():
    return Path(__file__).parent.parent / "fixtures" / "test_bom.csv"


def test_parse_bom(bom_csv):
    parser = BOMParser()
    entries = parser.parse(bom_csv)

    assert len(entries) == 4

    # Part with mass
    assert entries["part_with_mass"].mass_kg == pytest.approx(0.3)
    assert entries["part_with_mass"].material is None

    # Part with material but no mass
    assert entries["part_with_material"].mass_kg is None
    assert entries["part_with_material"].material == "ppa-cf"

    # Part without either
    assert entries["part_without_either"].mass_kg is None
    assert entries["part_without_either"].material is None

    # Part with empty mass field
    assert entries["part_empty_mass"].mass_kg is None
    assert entries["part_empty_mass"].material == "aluminum"


def test_bom_entry_has_mass():
    entry = BOMEntry(name="test", material=None, mass_kg=1.0)
    assert entry.has_mass is True
    assert entry.has_material is False


def test_bom_entry_has_material():
    entry = BOMEntry(name="test", material="steel", mass_kg=None)
    assert entry.has_mass is False
    assert entry.has_material is True
