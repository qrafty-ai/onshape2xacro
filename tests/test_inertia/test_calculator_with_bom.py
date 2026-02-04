import pytest
from unittest.mock import MagicMock, patch
from pathlib import Path
from onshape2xacro.inertia.calculator import InertiaCalculator
from onshape2xacro.inertia.bom import BOMEntry


@pytest.fixture
def mock_cadquery():
    with patch("onshape2xacro.inertia.calculator.cq") as mock_cq:
        # Setup common mock behavior
        mock_model = MagicMock()
        mock_cq.importers.importStep.return_value = mock_model

        # Setup a solid with known properties
        mock_solid = MagicMock()
        # 1 cm cube
        mock_solid.Volume.return_value = 1000.0  # mm^3

        # COM at (5, 5, 5) mm
        mock_vec = MagicMock()
        mock_vec.x = 5.0
        mock_vec.y = 5.0
        mock_vec.z = 5.0
        mock_solid.centerOfMass.return_value = mock_vec

        # Inertia for 1x1x1 mm cube (simplified unit density)
        # I = 1/6 * m * s^2
        # Just return identity for simplicity of checking scaling
        mock_solid.matrixOfInertia.return_value = [
            [1.0, 0.0, 0.0],
            [0.0, 1.0, 0.0],
            [0.0, 0.0, 1.0],
        ]

        mock_model.solids.return_value.vals.return_value = [mock_solid]
        mock_model.val.return_value = mock_solid

        yield mock_cq, mock_solid


def test_compute_with_bom_mass_override(mock_cadquery):
    mock_cq, mock_solid = mock_cadquery
    calc = InertiaCalculator()

    bom_entries = {"part1": BOMEntry(name="part1", material="steel", mass_kg=2.0)}

    report = MagicMock()

    # Run
    props = calc.compute_from_step_with_bom(
        Path("test.step"),
        bom_entries,
        link_name="link1",
        report=report,
        part_metadata=[{"part_name": "part1", "part_id": "part1"}],
    )

    # Verify mass is overridden from BOM
    assert pytest.approx(props.mass) == 2.0

    # Verify effective density calculation
    # Volume = 1000 mm^3 = 1e-6 m^3
    # Density = 2.0 / 1e-6 = 2e6 kg/m^3
    # Ixx = 1.0 * (1e-3)^5 * 2e6 = 1e-15 * 2e6 = 2e-9
    # Plus parallel axis theorem shift? COM is at (0.005, 0.005, 0.005)
    # The solid COM is (5,5,5) mm.
    # Since there's only one solid, the link COM should be same as part COM.
    # So parallel axis shift should be zero.

    expected_ixx = 1.0 * (0.001**5) * (2.0 / 1e-6)
    assert pytest.approx(props.ixx) == expected_ixx


def test_compute_with_bom_material(mock_cadquery):
    mock_cq, mock_solid = mock_cadquery
    calc = InertiaCalculator()

    # Aluminum density ~2700
    bom_entries = {"part1": BOMEntry(name="part1", material="aluminum", mass_kg=None)}

    report = MagicMock()

    props = calc.compute_from_step_with_bom(
        Path("test.step"),
        bom_entries,
        link_name="link1",
        report=report,
        part_metadata=[{"part_name": "part1", "part_id": "part1"}],
    )

    # Mass = Volume * Density
    # 1e-6 * 2700 = 0.0027 kg
    assert pytest.approx(props.mass) == 0.0027


def test_compute_with_link_level_fuzzy_match(mock_cadquery):
    mock_cq, mock_solid = mock_cadquery
    calc = InertiaCalculator()

    bom_entries = {
        "My Assembly": BOMEntry(name="My Assembly", material="steel", mass_kg=None)
    }

    report = MagicMock()

    # Link name matches BOM entry fuzzily
    props = calc.compute_from_step_with_bom(
        Path("test.step"),
        bom_entries,
        link_name="My Assembly_1",
        report=report,
        part_metadata=[{"part_name": "part_unknown"}],  # No part match
    )

    # Should fallback to link-level material (Steel ~7850)
    # Mass = 1e-6 * 7850 = 0.00785 kg
    # Wait, check logic: Priority 3: Fallback to link-level material
    # "My Assembly" in "My Assembly_1" -> True

    assert pytest.approx(props.mass) == 0.00785


def test_compute_no_match_defaults(mock_cadquery):
    mock_cq, mock_solid = mock_cadquery
    calc = InertiaCalculator(default_density=1000.0)

    bom_entries = {}
    report = MagicMock()

    props = calc.compute_from_step_with_bom(
        Path("test.step"),
        bom_entries,
        link_name="unknown_link",
        report=report,
        part_metadata=[{"part_name": "unknown_part"}],
    )

    # Default density 1000.0
    # Mass = 1e-6 * 1000 = 0.001 kg
    assert pytest.approx(props.mass) == 0.001

    # Should report warning
    report.add_warning.assert_called()


def test_compute_multi_solid_aggregation(mock_cadquery):
    mock_cq, mock_solid = mock_cadquery

    # Setup 2 identical solids
    mock_model = mock_cq.importers.importStep.return_value
    mock_model.solids.return_value.vals.return_value = [mock_solid, mock_solid]

    calc = InertiaCalculator(default_density=1000.0)
    bom_entries = {}
    report = MagicMock()

    props = calc.compute_from_step_with_bom(
        Path("test.step"),
        bom_entries,
        link_name="link1",
        report=report,
        part_metadata=[{"part_name": "p1"}, {"part_name": "p2"}],
    )

    # Total mass should be double (2 * 0.001 = 0.002)
    assert pytest.approx(props.mass) == 0.002

    # COM should be same as individual solid (since they are identical and at same place in mock)
    # (5, 5, 5) mm -> (0.005, 0.005, 0.005) m
    assert pytest.approx(props.com[0]) == 0.005


def test_compute_empty_solids(mock_cadquery):
    mock_cq, _ = mock_cadquery
    mock_model = mock_cq.importers.importStep.return_value
    mock_model.solids.return_value.vals.return_value = []
    # Also fallback val() returns None or raises? Code catches Exception.
    # Let's verify empty list path

    calc = InertiaCalculator()
    props = calc.compute_from_step_with_bom(Path("test.step"), {}, "link1", MagicMock())

    assert props.mass == 0.0


def test_part_id_extraction():
    # This tests the regex logic inside compute_from_step_with_bom
    # We need to construct a scenario where part_metadata has complex names
    pass
    # Actually, logic is embedded. We can test via part_metadata inputs in existing tests
    # or rely on the integration tests. But let's add one specific case.


def test_part_id_parsing_logic(mock_cadquery):
    mock_cq, mock_solid = mock_cadquery
    calc = InertiaCalculator()
    bom_entries = {
        "square_plate": BOMEntry(name="square_plate", material="steel", mass_kg=None)
    }
    report = MagicMock()

    # "sub-asm-base_1_square_plate_1" -> should match "square_plate"
    props = calc.compute_from_step_with_bom(
        Path("test.step"),
        bom_entries,
        link_name="link1",
        report=report,
        part_metadata=[
            {
                "part_name": "sub-asm-base_1_square_plate_1",
                "part_id": "sub-asm-base_1_square_plate_1",
            }
        ],
    )

    # Steel density used?
    assert pytest.approx(props.mass) == 0.00785

    # Verify debug info captured correct match
    # We can inspect report calls
    args = report.add_link_parts.call_args[0]
    debug_infos = args[1]
    assert debug_infos[0].bom_match == "square_plate"
