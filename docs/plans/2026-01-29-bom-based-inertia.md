# BOM-Based Inertia Calculation Enhancement

**Date:** 2026-01-29
**Status:** Draft
**Depends on:** Initial inertia implementation (completed)

## Problem Statement

The current `InertiaCalculator` uses a single default density for all parts. This is inaccurate because:
1. Sub-STEP files contain multiple parts with different materials
2. Onshape knows the material and sometimes the calculated mass, but we're not using it
3. Some parts have no material assigned and should be treated as zero-mass

## Solution

Use manually-exported BOM CSV from Onshape to get per-part material and mass data, then apply priority-based calculation:

1. **Onshape-calculated mass** → Use directly (highest priority)
2. **Material assigned** → Estimate from geometry volume × material density
3. **Neither** → Zero mass + warning

## BOM CSV Format

```csv
Item,Quantity,Part number,Description,Material,Mass,Name
1,1,,,- None -,0.3 kg,4310
2,1,,,ppa-cf,No value,J8_B
3,1,,,- None -,No value,left_soft
```

| Column | Values | Interpretation |
|--------|--------|----------------|
| Material | `- None -` | No material assigned |
| Material | `ppa-cf` | Material name for density lookup |
| Mass | `0.3 kg` | Onshape-calculated mass |
| Mass | `No value` or empty | No mass calculated |
| Name | `J8_B` | Part name for matching to geometry |

## Architecture

```
BOM.csv ──► BOMParser ──► Dict[part_name, BOMEntry]
                                    │
assembly.step ──► StepMeshExporter ─┼──► per-link compounds
                                    │
                                    ▼
                          EnhancedInertiaCalculator
                          (iterates solids in compound,
                           matches to BOM by name,
                           applies priority logic)
                                    │
                                    ▼
                          InertiaReport
                          (link properties + warnings)
```

---

## Implementation Tasks

### Task 1: Create BOMEntry and BOMParser

**Files:**
- Create: `src/onshape2xacro/inertia/bom.py`
- Test: `tests/test_inertia/test_bom.py`
- Fixture: `tests/fixtures/test_bom.csv`

**Step 1: Create test fixture**

Create `tests/fixtures/test_bom.csv`:
```csv
Item,Quantity,Part number,Description,Material,Mass,Name
1,1,,,- None -,0.3 kg,part_with_mass
2,1,,,ppa-cf,No value,part_with_material
3,1,,,- None -,No value,part_without_either
4,1,,,aluminum,,part_empty_mass
```

**Step 2: Write test**

Create `tests/test_inertia/test_bom.py`:
```python
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
```

**Step 3: Implement BOM parser**

Create `src/onshape2xacro/inertia/bom.py`:
```python
"""BOM (Bill of Materials) parser for Onshape CSV export."""
import csv
import re
from dataclasses import dataclass
from pathlib import Path
from typing import Dict, Optional


@dataclass
class BOMEntry:
    """Single part entry from BOM."""
    name: str
    material: Optional[str]  # None if "- None -"
    mass_kg: Optional[float]  # None if "No value" or empty

    @property
    def has_mass(self) -> bool:
        return self.mass_kg is not None

    @property
    def has_material(self) -> bool:
        return self.material is not None


class BOMParser:
    """Parse Onshape BOM CSV export."""

    def parse(self, csv_path: Path) -> Dict[str, BOMEntry]:
        """
        Parse BOM CSV file.

        Args:
            csv_path: Path to CSV file exported from Onshape

        Returns:
            Dict mapping part name to BOMEntry
        """
        entries = {}

        with open(csv_path, newline='', encoding='utf-8') as f:
            reader = csv.DictReader(f)
            for row in reader:
                name = row["Name"].strip()
                material = self._parse_material(row["Material"])
                mass_kg = self._parse_mass(row["Mass"])

                entries[name] = BOMEntry(
                    name=name,
                    material=material,
                    mass_kg=mass_kg,
                )

        return entries

    def _parse_material(self, value: str) -> Optional[str]:
        """Parse material field, return None if not assigned."""
        value = value.strip()
        if value == "- None -" or not value:
            return None
        return value

    def _parse_mass(self, value: str) -> Optional[float]:
        """Parse mass field, return None if not calculated."""
        value = value.strip()
        if value == "No value" or not value:
            return None

        # Extract numeric value from "0.3 kg"
        match = re.match(r"([\d.]+)\s*kg", value, re.IGNORECASE)
        if match:
            return float(match.group(1))

        return None
```

**Step 4: Update __init__.py**

Add to `src/onshape2xacro/inertia/__init__.py`:
```python
from .bom import BOMParser, BOMEntry
```

**Step 5: Run tests**
```bash
uv run pytest tests/test_inertia/test_bom.py -v
```

**Step 6: Commit**
```bash
git add src/onshape2xacro/inertia/ tests/
git commit -m "feat(inertia): add BOM parser for Onshape CSV export"
```

---

### Task 2: Create InertiaReport for warnings

**Files:**
- Create: `src/onshape2xacro/inertia/report.py`
- Test: `tests/test_inertia/test_report.py`

**Step 1: Write test**

Create `tests/test_inertia/test_report.py`:
```python
"""Tests for inertia report."""
import pytest
from onshape2xacro.inertia.report import InertiaReport, PartWarning
from onshape2xacro.inertia.types import InertialProperties


def test_report_add_warning():
    report = InertiaReport()
    report.add_warning("link1", "part_a", "No mass or material assigned")

    assert len(report.warnings) == 1
    assert report.warnings[0].link_name == "link1"
    assert report.warnings[0].part_name == "part_a"


def test_report_summary():
    report = InertiaReport()
    report.add_warning("link1", "part_a", "No mass or material")
    report.add_warning("link1", "part_b", "No mass or material")
    report.add_warning("link2", "part_c", "Unknown material 'custom'")

    summary = report.get_summary()
    assert "3 warnings" in summary.lower() or "3 parts" in summary.lower()


def test_report_no_warnings():
    report = InertiaReport()
    summary = report.get_summary()
    assert "no warnings" in summary.lower() or summary == ""
```

**Step 2: Implement report**

Create `src/onshape2xacro/inertia/report.py`:
```python
"""Inertia calculation report with warnings."""
from dataclasses import dataclass, field
from typing import Dict, List
import logging

from .types import InertialProperties

logger = logging.getLogger(__name__)


@dataclass
class PartWarning:
    """Warning for a specific part."""
    link_name: str
    part_name: str
    message: str


@dataclass
class InertiaReport:
    """Report collecting inertia results and warnings."""

    link_properties: Dict[str, InertialProperties] = field(default_factory=dict)
    warnings: List[PartWarning] = field(default_factory=list)

    def add_warning(self, link_name: str, part_name: str, message: str) -> None:
        """Add a warning for a part."""
        self.warnings.append(PartWarning(
            link_name=link_name,
            part_name=part_name,
            message=message,
        ))

    def get_summary(self) -> str:
        """Get summary of warnings."""
        if not self.warnings:
            return "No warnings - all parts have mass properties."

        lines = [f"⚠️  {len(self.warnings)} parts with missing mass properties:"]

        # Group by link
        by_link: Dict[str, List[PartWarning]] = {}
        for w in self.warnings:
            by_link.setdefault(w.link_name, []).append(w)

        for link_name, link_warnings in by_link.items():
            lines.append(f"\n  {link_name}:")
            for w in link_warnings:
                lines.append(f"    - {w.part_name}: {w.message}")

        return "\n".join(lines)

    def print_summary(self) -> None:
        """Print warning summary to logger."""
        summary = self.get_summary()
        if self.warnings:
            logger.warning(summary)
        else:
            logger.info(summary)
```

**Step 3: Update __init__.py**
```python
from .report import InertiaReport, PartWarning
```

**Step 4: Run tests and commit**
```bash
uv run pytest tests/test_inertia/test_report.py -v
git add src/onshape2xacro/inertia/ tests/
git commit -m "feat(inertia): add InertiaReport for warning collection"
```

---

### Task 3: Expand material density lookup

**Files:**
- Modify: `src/onshape2xacro/inertia/calculator.py`

**Step 1: Expand DEFAULT_DENSITIES**

Update the dictionary in `calculator.py`:
```python
# Material densities in kg/m³
# Sources: MatWeb, manufacturer datasheets
MATERIAL_DENSITIES = {
    # Metals
    "aluminum": 2700,
    "aluminium": 2700,
    "steel": 7850,
    "stainless steel": 8000,
    "brass": 8500,
    "copper": 8960,
    "titanium": 4500,

    # Plastics
    "abs": 1050,
    "pla": 1250,
    "petg": 1270,
    "nylon": 1150,
    "polycarbonate": 1200,
    "pc": 1200,
    "acrylic": 1180,
    "pmma": 1180,
    "hdpe": 970,
    "pp": 905,
    "pom": 1410,
    "delrin": 1410,

    # Composites / Reinforced
    "ppa-cf": 1400,  # Carbon fiber reinforced PPA
    "nylon-cf": 1200,
    "carbon fiber": 1600,
    "fiberglass": 1900,

    # Rubber/Elastomers
    "rubber": 1100,
    "silicone": 1100,
    "tpu": 1200,

    # Default
    "default": 1000,
}
```

**Step 2: Improve material matching**

Add fuzzy matching method:
```python
def _get_density(self, material: Optional[str]) -> Optional[float]:
    """
    Look up density for material name.

    Returns None if material is None (triggers zero-mass warning).
    """
    if material is None:
        return None

    # Normalize: lowercase, strip whitespace
    key = material.lower().strip()

    # Direct match
    if key in MATERIAL_DENSITIES:
        return MATERIAL_DENSITIES[key]

    # Partial match (e.g., "Aluminum 6061" matches "aluminum")
    for mat_name, density in MATERIAL_DENSITIES.items():
        if mat_name in key or key in mat_name:
            logger.info(f"Matched material '{material}' to '{mat_name}'")
            return density

    # Unknown material - log warning, return default
    logger.warning(
        f"Unknown material '{material}', using default density {self.default_density} kg/m³"
    )
    return self.default_density
```

**Step 3: Commit**
```bash
git add src/onshape2xacro/inertia/calculator.py
git commit -m "feat(inertia): expand material density lookup with fuzzy matching"
```

---

### Task 4: Enhance InertiaCalculator for per-part BOM data

**Files:**
- Modify: `src/onshape2xacro/inertia/calculator.py`
- Test: `tests/test_inertia/test_calculator_bom.py`

**Step 1: Add method for BOM-aware calculation**

Add to `InertiaCalculator`:
```python
def compute_from_step_with_bom(
    self,
    step_path: Path,
    bom_entries: Dict[str, BOMEntry],
    link_name: str,
    report: InertiaReport,
) -> InertialProperties:
    """
    Compute inertial properties using BOM data for per-part mass.

    Priority:
    1. BOM mass (Onshape-calculated) → use directly
    2. BOM material → estimate from geometry × density
    3. Neither → zero mass, add warning

    Args:
        step_path: Path to link STEP file
        bom_entries: Dict of part_name -> BOMEntry from BOM CSV
        link_name: Name of this link (for warnings)
        report: InertiaReport to collect warnings

    Returns:
        Aggregated InertialProperties for the link
    """
    import cadquery as cq

    model = cq.importers.importStep(str(step_path))

    # Get all solids in the compound
    solids = model.solids().vals()

    total_mass = 0.0
    weighted_com = [0.0, 0.0, 0.0]
    total_ixx = 0.0
    total_iyy = 0.0
    total_izz = 0.0
    total_ixy = 0.0
    total_ixz = 0.0
    total_iyz = 0.0

    for i, solid in enumerate(solids):
        # Try to get part name from solid
        # Note: STEP may not preserve names, use index as fallback
        part_name = self._get_solid_name(solid, i)

        bom_entry = bom_entries.get(part_name)

        if bom_entry and bom_entry.has_mass:
            # Priority 1: Use Onshape-calculated mass
            mass = bom_entry.mass_kg
            # Still need COM and inertia from geometry (scaled by mass/volume ratio)
            props = self._compute_with_known_mass(solid, mass)

        elif bom_entry and bom_entry.has_material:
            # Priority 2: Estimate from material density
            density = self._get_density(bom_entry.material)
            props = self._compute_from_geometry(solid, density)

        elif bom_entry:
            # Priority 3: No mass, no material
            report.add_warning(link_name, part_name, "No mass or material assigned")
            props = InertialProperties(
                mass=0.0, com=(0, 0, 0),
                ixx=0, iyy=0, izz=0
            )
        else:
            # Part not in BOM - use default density with warning
            report.add_warning(link_name, part_name, f"Part not found in BOM")
            props = self._compute_from_geometry(solid, self.default_density)

        # Aggregate
        total_mass += props.mass
        if props.mass > 0:
            for j in range(3):
                weighted_com[j] += props.com[j] * props.mass
        total_ixx += props.ixx
        total_iyy += props.iyy
        total_izz += props.izz
        total_ixy += props.ixy
        total_ixz += props.ixz
        total_iyz += props.iyz

    # Finalize COM
    if total_mass > 0:
        com = tuple(c / total_mass for c in weighted_com)
    else:
        com = (0.0, 0.0, 0.0)

    return InertialProperties(
        mass=total_mass,
        com=com,
        ixx=total_ixx,
        iyy=total_iyy,
        izz=total_izz,
        ixy=total_ixy,
        ixz=total_ixz,
        iyz=total_iyz,
    )
```

**Step 2: Add helper methods**
```python
def _get_solid_name(self, solid, index: int) -> str:
    """Extract name from solid or use index."""
    # CadQuery doesn't preserve STEP names easily
    # This is a known limitation - may need OCP-level access
    return f"solid_{index}"

def _compute_with_known_mass(self, solid, known_mass: float) -> InertialProperties:
    """Compute properties when mass is known from BOM."""
    # Get volume to compute density
    volume_mm3 = solid.Volume()
    volume_m3 = volume_mm3 * (self.mm_to_m ** 3)

    if volume_m3 > 0:
        effective_density = known_mass / volume_m3
    else:
        effective_density = self.default_density

    return self._compute_from_geometry(solid, effective_density)

def _compute_from_geometry(self, solid, density: float) -> InertialProperties:
    """Compute properties from geometry and density."""
    volume_mm3 = solid.Volume()
    volume_m3 = volume_mm3 * (self.mm_to_m ** 3)
    mass = volume_m3 * density

    com_vec = solid.centerOfMass(solid)
    com = (
        com_vec.x * self.mm_to_m,
        com_vec.y * self.mm_to_m,
        com_vec.z * self.mm_to_m,
    )

    inertia_matrix = solid.matrixOfInertia(solid)
    scale = (self.mm_to_m ** 5) * density

    return InertialProperties(
        mass=mass,
        com=com,
        ixx=inertia_matrix[0][0] * scale,
        iyy=inertia_matrix[1][1] * scale,
        izz=inertia_matrix[2][2] * scale,
        ixy=inertia_matrix[0][1] * scale,
        ixz=inertia_matrix[0][2] * scale,
        iyz=inertia_matrix[1][2] * scale,
    )
```

**Step 3: Write test and commit**
```bash
uv run pytest tests/test_inertia/ -v
git add src/onshape2xacro/inertia/ tests/
git commit -m "feat(inertia): add BOM-aware per-part calculation"
```

---

### Task 5: Update StepMeshExporter integration

**Files:**
- Modify: `src/onshape2xacro/mesh_exporters/step.py`

Update `compute_link_inertials` to accept optional BOM path and use the enhanced calculator.

---

## Verification Checklist

- [ ] BOMParser correctly parses CSV with all edge cases
- [ ] Material density lookup handles known + unknown materials
- [ ] Priority logic: mass > material > zero correctly applied
- [ ] Warnings collected and summarized at end
- [ ] Integration with StepMeshExporter works
- [ ] All tests passing
