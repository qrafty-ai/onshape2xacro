"""Tests for STEP exporter inertia integration."""

from pathlib import Path
from unittest.mock import patch

from onshape2xacro.inertia import InertiaCalculator, InertialProperties


def test_compute_inertials_from_assembly():
    """Smoke test: compute inertials from mocked assembly."""
    mock_props = InertialProperties(
        mass=1.0,
        com=(0.1, 0.2, 0.3),
        ixx=0.01,
        iyy=0.02,
        izz=0.03,
        ixy=0.001,
        ixz=0.002,
        iyz=0.003,
    )

    with patch.object(
        InertiaCalculator, "compute_from_step", return_value=mock_props
    ) as mock_compute:
        calc = InertiaCalculator(default_density=1000.0)
        dummy_path = Path("dummy_path.step")
        props = calc.compute_from_step(dummy_path, material="default")

        mock_compute.assert_called_once_with(dummy_path, material="default")

        assert props.mass == 1.0
        assert props.com == (0.1, 0.2, 0.3)
        assert props.ixx == 0.01
        assert props.iyy == 0.02
        assert props.izz == 0.03

        print("\nMocked assembly properties:")
        print(f"  Mass: {props.mass:.4f} kg")
        print(f"  COM: ({props.com[0]:.4f}, {props.com[1]:.4f}, {props.com[2]:.4f}) m")
        print(f"  Ixx: {props.ixx:.6f} kg⋅m²")
        print(f"  Iyy: {props.iyy:.6f} kg⋅m²")
        print(f"  Izz: {props.izz:.6f} kg⋅m²")
