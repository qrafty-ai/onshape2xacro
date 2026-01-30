"""Tests for inertia calculator."""

import pytest
from pathlib import Path
from onshape2xacro.inertia.calculator import InertiaCalculator


@pytest.fixture
def cube_step():
    return Path(__file__).parent.parent / "fixtures" / "test_cube.step"


def test_calculate_cube_volume(cube_step):
    """Test that a 1cm cube has correct volume."""
    calc = InertiaCalculator(default_density=1000.0)  # kg/m³ (water)
    props = calc.compute_from_step(cube_step, material="default")

    # 1cm cube = 0.01m x 0.01m x 0.01m = 1e-6 m³
    # Mass = 1e-6 m³ * 1000 kg/m³ = 0.001 kg = 1 gram
    assert pytest.approx(props.mass, rel=0.01) == 0.001


def test_calculate_cube_com(cube_step):
    """Test that COM is at origin for centered cube."""
    calc = InertiaCalculator(default_density=1000.0)
    props = calc.compute_from_step(cube_step, material="default")

    # Cube centered at origin
    assert pytest.approx(props.com[0], abs=1e-6) == 0.0
    assert pytest.approx(props.com[1], abs=1e-6) == 0.0
    assert pytest.approx(props.com[2], abs=1e-6) == 0.0


def test_calculate_cube_inertia(cube_step):
    """Test inertia tensor for uniform density cube."""
    calc = InertiaCalculator(default_density=1000.0)
    props = calc.compute_from_step(cube_step, material="default")

    # For a cube with side s and mass m: I = (1/6) * m * s²
    # s = 0.01m, m = 0.001 kg
    # I = (1/6) * 0.001 * 0.0001 = 1.667e-8 kg⋅m²
    expected_i = (1 / 6) * 0.001 * (0.01**2)

    assert pytest.approx(props.ixx, rel=0.05) == expected_i
    assert pytest.approx(props.iyy, rel=0.05) == expected_i
    assert pytest.approx(props.izz, rel=0.05) == expected_i


def test_material_density_lookup():
    calc = InertiaCalculator()

    # Direct match
    assert calc._get_density("aluminum") == 2700
    assert calc._get_density("ppa-cf") == 1400

    # Fuzzy match
    assert calc._get_density("Aluminum 6061") == 2700
    assert calc._get_density("STAINLESS STEEL 304") == 8000

    # None returns None
    assert calc._get_density(None) is None

    # Unknown returns default
    assert calc._get_density("unobtanium") == 1000
