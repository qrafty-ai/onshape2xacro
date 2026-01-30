"""Tests for STEP exporter inertia integration."""

import pytest
from pathlib import Path

from onshape2xacro.inertia import InertiaCalculator


@pytest.fixture
def waist_assembly():
    """Return path to waist assembly STEP file."""
    path = Path(__file__).parent.parent.parent / "waist" / "assembly.step"
    if not path.exists():
        pytest.skip("waist/assembly.step not found")
    return path


def test_compute_inertials_from_assembly(waist_assembly):
    """Smoke test: compute inertials from real assembly."""
    calc = InertiaCalculator(default_density=1000.0)
    props = calc.compute_from_step(waist_assembly)

    # Just verify we get sensible values
    assert props.mass > 0, "Mass should be positive"
    assert props.ixx > 0, "Ixx should be positive"
    assert props.iyy > 0, "Iyy should be positive"
    assert props.izz > 0, "Izz should be positive"

    # Print values for inspection
    print("\nWaist assembly properties:")
    print(f"  Mass: {props.mass:.4f} kg")
    print(f"  COM: ({props.com[0]:.4f}, {props.com[1]:.4f}, {props.com[2]:.4f}) m")
    print(f"  Ixx: {props.ixx:.6f} kg⋅m²")
    print(f"  Iyy: {props.iyy:.6f} kg⋅m²")
    print(f"  Izz: {props.izz:.6f} kg⋅m²")
