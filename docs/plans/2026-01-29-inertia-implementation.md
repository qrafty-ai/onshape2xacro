# Inertia Generation Implementation Plan

> **For Claude:** REQUIRED SUB-SKILL: Use superpowers:executing-plans to implement this plan task-by-task.

**Goal:** Add real mass/inertia computation from STEP geometry using CadQuery, replacing mock values.

**Architecture:** Extract per-link shapes from assembly STEP → compute mass properties via CadQuery → write computed values to `inertials.yaml`.

**Tech Stack:** CadQuery, OCP (OpenCASCADE), PyYAML

---

## Task 1: Create InertialProperties dataclass

**Files:**
- Create: `src/onshape2xacro/inertia/__init__.py`
- Create: `src/onshape2xacro/inertia/types.py`
- Test: `tests/test_inertia/test_types.py`

**Step 1: Create package and types module**

Create `src/onshape2xacro/inertia/__init__.py`:
```python
"""Inertia computation module."""
from .types import InertialProperties

__all__ = ["InertialProperties"]
```

Create `src/onshape2xacro/inertia/types.py`:
```python
"""Data types for inertia computation."""
from dataclasses import dataclass, asdict
from typing import Tuple


@dataclass
class InertialProperties:
    """Physical properties for a robot link."""

    mass: float  # kg
    com: Tuple[float, float, float]  # center of mass (m)
    ixx: float
    iyy: float
    izz: float
    ixy: float = 0.0
    ixz: float = 0.0
    iyz: float = 0.0

    def to_yaml_dict(self) -> dict:
        """Convert to inertials.yaml format."""
        return {
            "mass": self.mass,
            "origin": {
                "xyz": f"{self.com[0]} {self.com[1]} {self.com[2]}",
                "rpy": "0 0 0",
            },
            "inertia": {
                "ixx": self.ixx,
                "iyy": self.iyy,
                "izz": self.izz,
                "ixy": self.ixy,
                "ixz": self.ixz,
                "iyz": self.iyz,
            },
        }
```

**Step 2: Write test**

Create `tests/test_inertia/__init__.py` (empty file).

Create `tests/test_inertia/test_types.py`:
```python
"""Tests for inertia types."""
import pytest
from onshape2xacro.inertia.types import InertialProperties


def test_inertial_properties_to_yaml_dict():
    props = InertialProperties(
        mass=1.5,
        com=(0.1, 0.2, 0.3),
        ixx=0.01,
        iyy=0.02,
        izz=0.03,
        ixy=0.001,
        ixz=0.002,
        iyz=0.003,
    )
    result = props.to_yaml_dict()

    assert result["mass"] == 1.5
    assert result["origin"]["xyz"] == "0.1 0.2 0.3"
    assert result["origin"]["rpy"] == "0 0 0"
    assert result["inertia"]["ixx"] == 0.01
    assert result["inertia"]["iyy"] == 0.02
    assert result["inertia"]["izz"] == 0.03
```

**Step 3: Run test**

Run: `uv run pytest tests/test_inertia/test_types.py -v`
Expected: PASS

**Step 4: Commit**

```bash
git add src/onshape2xacro/inertia/ tests/test_inertia/
git commit -m "feat(inertia): add InertialProperties dataclass"
```

---

## Task 2: Implement InertiaCalculator

**Files:**
- Create: `src/onshape2xacro/inertia/calculator.py`
- Test: `tests/test_inertia/test_calculator.py`
- Test fixture: `tests/fixtures/test_cube.step` (1x1x1 cm cube)

**Step 1: Create test cube fixture**

Run this to create a 1cm cube STEP file:
```bash
uv run python -c "
import cadquery as cq
cube = cq.Workplane('XY').box(10, 10, 10)  # 10mm = 1cm cube
cq.exporters.export(cube, 'tests/fixtures/test_cube.step')
print('Created tests/fixtures/test_cube.step')
"
```

**Step 2: Write failing test**

Create `tests/test_inertia/test_calculator.py`:
```python
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
    props = calc.compute_from_step(cube_step)

    # 1cm cube = 0.01m x 0.01m x 0.01m = 1e-6 m³
    # Mass = 1e-6 m³ * 1000 kg/m³ = 0.001 kg = 1 gram
    assert pytest.approx(props.mass, rel=0.01) == 0.001


def test_calculate_cube_com(cube_step):
    """Test that COM is at origin for centered cube."""
    calc = InertiaCalculator(default_density=1000.0)
    props = calc.compute_from_step(cube_step)

    # Cube centered at origin
    assert pytest.approx(props.com[0], abs=1e-6) == 0.0
    assert pytest.approx(props.com[1], abs=1e-6) == 0.0
    assert pytest.approx(props.com[2], abs=1e-6) == 0.0


def test_calculate_cube_inertia(cube_step):
    """Test inertia tensor for uniform density cube."""
    calc = InertiaCalculator(default_density=1000.0)
    props = calc.compute_from_step(cube_step)

    # For a cube with side s and mass m: I = (1/6) * m * s²
    # s = 0.01m, m = 0.001 kg
    # I = (1/6) * 0.001 * 0.0001 = 1.667e-8 kg⋅m²
    expected_i = (1/6) * 0.001 * (0.01 ** 2)

    assert pytest.approx(props.ixx, rel=0.05) == expected_i
    assert pytest.approx(props.iyy, rel=0.05) == expected_i
    assert pytest.approx(props.izz, rel=0.05) == expected_i
```

**Step 3: Run test to verify it fails**

Run: `uv run pytest tests/test_inertia/test_calculator.py -v`
Expected: FAIL with "No module named 'onshape2xacro.inertia.calculator'"

**Step 4: Implement calculator**

Create `src/onshape2xacro/inertia/calculator.py`:
```python
"""Compute mass properties from STEP geometry using CadQuery."""
from pathlib import Path
from typing import Optional
import logging

import cadquery as cq

from .types import InertialProperties

logger = logging.getLogger(__name__)

# Material densities in kg/m³
DEFAULT_DENSITIES = {
    "aluminum": 2700,
    "steel": 7850,
    "abs": 1050,
    "pla": 1250,
    "default": 1000,
}


class InertiaCalculator:
    """Computes mass and inertia properties from STEP geometry."""

    def __init__(
        self,
        default_density: float = 1000.0,
        mm_to_m: float = 0.001,
    ):
        """
        Initialize calculator.

        Args:
            default_density: Default material density in kg/m³
            mm_to_m: Conversion factor from model units to meters
        """
        self.default_density = default_density
        self.mm_to_m = mm_to_m

    def compute_from_step(
        self,
        step_path: Path,
        material: Optional[str] = None,
    ) -> InertialProperties:
        """
        Compute inertial properties from a STEP file.

        Args:
            step_path: Path to STEP file
            material: Optional material name for density lookup

        Returns:
            InertialProperties with mass, COM, and inertia tensor
        """
        # Load STEP
        model = cq.importers.importStep(str(step_path))
        shape = model.val()

        # Get density
        density = self._get_density(material)

        # Volume in model units (mm³) -> m³
        volume_mm3 = shape.Volume()
        volume_m3 = volume_mm3 * (self.mm_to_m ** 3)

        # Mass
        mass = volume_m3 * density

        # Center of mass (mm -> m)
        com_vec = shape.centerOfMass(shape)
        com = (
            com_vec.x * self.mm_to_m,
            com_vec.y * self.mm_to_m,
            com_vec.z * self.mm_to_m,
        )

        # Inertia tensor (in model units, needs conversion)
        # CadQuery returns inertia assuming density=1, so we scale by density
        # Units: mm^5 -> m^5, then * density -> kg⋅m²
        inertia_matrix = shape.matrixOfInertia(shape)

        # Scale factor: mm^5 * (m/mm)^5 * density = m^5 * density
        # But inertia is volume*distance² = mm³ * mm² = mm^5
        # Convert to m^5: * (0.001)^5 = 1e-15
        # Then multiply by density to get kg⋅m²
        scale = (self.mm_to_m ** 5) * density

        # Matrix is 3x3, extract diagonal and off-diagonal
        ixx = inertia_matrix[0][0] * scale
        iyy = inertia_matrix[1][1] * scale
        izz = inertia_matrix[2][2] * scale
        ixy = inertia_matrix[0][1] * scale
        ixz = inertia_matrix[0][2] * scale
        iyz = inertia_matrix[1][2] * scale

        return InertialProperties(
            mass=mass,
            com=com,
            ixx=ixx,
            iyy=iyy,
            izz=izz,
            ixy=ixy,
            ixz=ixz,
            iyz=iyz,
        )

    def _get_density(self, material: Optional[str]) -> float:
        """Look up density for material name."""
        if material is None:
            return self.default_density

        key = material.lower()
        if key in DEFAULT_DENSITIES:
            return DEFAULT_DENSITIES[key]

        logger.warning(
            f"Unknown material '{material}', using default density {self.default_density}"
        )
        return self.default_density
```

**Step 5: Update __init__.py**

Edit `src/onshape2xacro/inertia/__init__.py`:
```python
"""Inertia computation module."""
from .types import InertialProperties
from .calculator import InertiaCalculator

__all__ = ["InertialProperties", "InertiaCalculator"]
```

**Step 6: Run tests**

Run: `uv run pytest tests/test_inertia/test_calculator.py -v`
Expected: PASS (may need to adjust scale factor based on actual CadQuery output)

**Step 7: Commit**

```bash
git add src/onshape2xacro/inertia/ tests/test_inertia/ tests/fixtures/
git commit -m "feat(inertia): add InertiaCalculator with CadQuery"
```

---

## Task 3: Implement InertiaConfigWriter

**Files:**
- Create: `src/onshape2xacro/inertia/writer.py`
- Test: `tests/test_inertia/test_writer.py`

**Step 1: Write failing test**

Create `tests/test_inertia/test_writer.py`:
```python
"""Tests for inertia config writer."""
import pytest
import yaml
from pathlib import Path
from tempfile import TemporaryDirectory

from onshape2xacro.inertia.types import InertialProperties
from onshape2xacro.inertia.writer import InertiaConfigWriter


def test_write_inertials_yaml():
    props = {
        "base_link": InertialProperties(
            mass=1.0, com=(0, 0, 0), ixx=0.01, iyy=0.01, izz=0.01
        ),
        "arm_link": InertialProperties(
            mass=0.5, com=(0.1, 0, 0), ixx=0.005, iyy=0.005, izz=0.002
        ),
    }

    with TemporaryDirectory() as tmpdir:
        config_dir = Path(tmpdir)
        writer = InertiaConfigWriter()
        writer.write(props, config_dir)

        # Verify file exists
        yaml_path = config_dir / "inertials.yaml"
        assert yaml_path.exists()

        # Verify content
        with open(yaml_path) as f:
            data = yaml.safe_load(f)

        assert "inertials" in data
        assert "base_link" in data["inertials"]
        assert data["inertials"]["base_link"]["mass"] == 1.0
        assert "arm_link" in data["inertials"]
```

**Step 2: Run test to verify it fails**

Run: `uv run pytest tests/test_inertia/test_writer.py -v`
Expected: FAIL

**Step 3: Implement writer**

Create `src/onshape2xacro/inertia/writer.py`:
```python
"""Write computed inertial properties to config files."""
from pathlib import Path
from typing import Dict

import yaml

from .types import InertialProperties


class InertiaConfigWriter:
    """Writes inertial properties to YAML config files."""

    def write(
        self,
        properties: Dict[str, InertialProperties],
        config_dir: Path,
    ) -> Path:
        """
        Write inertial properties to inertials.yaml.

        Args:
            properties: Map of link name to InertialProperties
            config_dir: Directory to write config file

        Returns:
            Path to written file
        """
        config_dir.mkdir(parents=True, exist_ok=True)

        inertials = {
            name: props.to_yaml_dict()
            for name, props in properties.items()
        }

        output_path = config_dir / "inertials.yaml"
        with open(output_path, "w") as f:
            yaml.dump({"inertials": inertials}, f, default_flow_style=False)

        return output_path
```

**Step 4: Update __init__.py**

Edit `src/onshape2xacro/inertia/__init__.py`:
```python
"""Inertia computation module."""
from .types import InertialProperties
from .calculator import InertiaCalculator
from .writer import InertiaConfigWriter

__all__ = ["InertialProperties", "InertiaCalculator", "InertiaConfigWriter"]
```

**Step 5: Run test**

Run: `uv run pytest tests/test_inertia/test_writer.py -v`
Expected: PASS

**Step 6: Commit**

```bash
git add src/onshape2xacro/inertia/ tests/test_inertia/
git commit -m "feat(inertia): add InertiaConfigWriter for YAML output"
```

---

## Task 4: Integrate into StepMeshExporter

**Files:**
- Modify: `src/onshape2xacro/mesh_exporters/step.py`
- Test: `tests/test_mesh_exporters/test_step_inertia.py`

**Step 1: Write integration test**

Create `tests/test_mesh_exporters/test_step_inertia.py`:
```python
"""Tests for STEP exporter inertia integration."""
import pytest
from pathlib import Path
from tempfile import TemporaryDirectory
import yaml


# This test requires the waist/assembly.step fixture
@pytest.fixture
def waist_assembly():
    path = Path(__file__).parent.parent.parent / "waist" / "assembly.step"
    if not path.exists():
        pytest.skip("waist/assembly.step not found")
    return path


def test_compute_inertials_from_assembly(waist_assembly):
    """Smoke test: compute inertials from real assembly."""
    from onshape2xacro.inertia import InertiaCalculator

    calc = InertiaCalculator(default_density=1000.0)
    props = calc.compute_from_step(waist_assembly)

    # Just verify we get sensible values
    assert props.mass > 0
    assert props.ixx > 0
    assert props.iyy > 0
    assert props.izz > 0
```

**Step 2: Run test**

Run: `uv run pytest tests/test_mesh_exporters/test_step_inertia.py -v`
Expected: PASS

**Step 3: Commit**

```bash
git add tests/test_mesh_exporters/
git commit -m "test(inertia): add integration test for assembly STEP"
```

---

## Task 5: Wire up pipeline (compute_link_inertials method)

**Files:**
- Modify: `src/onshape2xacro/mesh_exporters/step.py` (add method)
- Modify: `src/onshape2xacro/pipeline.py` (call the method)

**Step 1: Add compute_link_inertials to StepMeshExporter**

Add to `src/onshape2xacro/mesh_exporters/step.py` after the `export_link_meshes` method:

```python
def compute_link_inertials(
    self,
    link_records: List[LinkMeshRecord],
    output_dir: Path,
    density: float = 1000.0,
) -> Dict[str, "InertialProperties"]:
    """
    Compute inertial properties for each link.

    Args:
        link_records: Link mesh records with shape information
        output_dir: Directory for config output
        density: Default material density in kg/m³

    Returns:
        Dict mapping link name to InertialProperties
    """
    from onshape2xacro.inertia import InertiaCalculator, InertiaConfigWriter

    calc = InertiaCalculator(default_density=density)
    writer = InertiaConfigWriter()

    results = {}
    for record in link_records:
        if record.compound is None:
            logger.warning(f"No shape for link {record.name}, skipping inertia")
            continue

        # Export link compound to temp STEP for CadQuery processing
        # (CadQuery needs file path, not TopoDS_Shape directly)
        temp_step = output_dir / "links" / f"{record.name}.step"
        temp_step.parent.mkdir(parents=True, exist_ok=True)

        # Use OCP STEP writer
        from OCP.STEPControl import STEPControl_Writer, STEPControl_AsIs
        step_writer = STEPControl_Writer()
        step_writer.Transfer(record.compound, STEPControl_AsIs)
        step_writer.Write(str(temp_step))

        # Compute inertia
        try:
            props = calc.compute_from_step(temp_step)
            results[record.name] = props
            logger.info(f"Computed inertia for {record.name}: mass={props.mass:.4f} kg")
        except Exception as e:
            logger.error(f"Failed to compute inertia for {record.name}: {e}")

    # Write config
    config_dir = output_dir / "config"
    writer.write(results, config_dir)
    logger.info(f"Wrote inertials.yaml to {config_dir}")

    return results
```

**Step 2: Commit**

```bash
git add src/onshape2xacro/mesh_exporters/step.py
git commit -m "feat(inertia): add compute_link_inertials to StepMeshExporter"
```

---

## Task 6: Final integration and cleanup

**Step 1: Run all tests**

Run: `uv run pytest tests/test_inertia/ -v`
Expected: All PASS

**Step 2: Run full test suite**

Run: `uv run pytest --tb=short -q`
Expected: Same failure count as baseline (8 pre-existing failures)

**Step 3: Commit any fixes**

```bash
git add -A
git commit -m "chore(inertia): cleanup and fixes"
```

---

## Verification Checklist

- [ ] `InertialProperties` dataclass with `to_yaml_dict()`
- [ ] `InertiaCalculator` computing mass, COM, inertia tensor
- [ ] `InertiaConfigWriter` writing `inertials.yaml`
- [ ] Unit tests passing for all components
- [ ] Integration test with `waist/assembly.step`
- [ ] Per-link STEP export working
- [ ] All commits atomic and descriptive
