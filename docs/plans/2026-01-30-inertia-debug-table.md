# Inertia Debug Table Implementation

**Date:** 2026-01-30
**Status:** Draft
**Depends on:** BOM-based inertia calculation (completed)

## Problem Statement

When debugging inertia calculation issues, it's difficult to see:
1. Which parts were found in each link's STEP file
2. How each part matched (or didn't match) to BOM entries
3. What mass/material was used for each part
4. Why the final link mass might be wrong

We need visibility into the per-part breakdown within each link.

## Solution

Generate a debug table showing all parts discovered in each link, their BOM matching status, and computed properties.

**Example Output:**

```markdown
## Link: link_base

| Part | BOM Match | Mass Source | Material | Volume (cm³) | Mass (kg) | Status |
|------|-----------|-------------|----------|--------------|-----------|--------|
| solid_0 | link_base | BOM Mass | - | 312.5 | 0.852 | ✓ |

**Link Total:** 0.852 kg

## Link: link_arm

| Part | BOM Match | Mass Source | Material | Volume (cm³) | Mass (kg) | Status |
|------|-----------|-------------|----------|--------------|-----------|--------|
| solid_0 | link_arm | Material Density | aluminum | 508.1 | 1.372 | ✓ |
| solid_1 | (fuzzy: link_arm) | Material Density | aluminum | 508.1 | 1.372 | ⚠️ Fuzzy match |

**Link Total:** 2.744 kg
**Warnings:** 1 part with fuzzy BOM matching
```

## Architecture

```
InertiaCalculator.compute_from_step_with_bom()
    │
    ├─► For each solid:
    │       ├─ Match to BOM
    │       ├─ Compute properties
    │       └─ Record to PartDebugInfo
    │
    └─► InertiaReport.add_link_parts(link_name, part_infos)

InertiaReport.generate_debug_table()
    │
    └─► Markdown table output
```

---

## Implementation Tasks

### Task 1: Add PartDebugInfo dataclass

**Files:**
- Modify: `src/onshape2xacro/inertia/report.py`

**Step 1: Add PartDebugInfo**

```python
@dataclass
class PartDebugInfo:
    """Debug information for a single part."""
    part_id: str                    # e.g., "solid_0"
    bom_match: Optional[str]        # BOM entry name if matched
    match_type: str                 # "exact", "fuzzy", "none"
    mass_source: str                # "BOM Mass", "Material Density", "Default Density", "Zero (no data)"
    material: Optional[str]         # Material name from BOM
    volume_cm3: float               # Volume in cm³
    mass_kg: float                  # Computed mass
    warnings: List[str]             # Any warnings for this part
```

**Step 2: Update InertiaReport**

Add field and method:
```python
@dataclass
class InertiaReport:
    """Report collecting inertia results and warnings."""

    link_properties: Dict[str, InertialProperties] = field(default_factory=dict)
    warnings: List[PartWarning] = field(default_factory=list)
    link_parts: Dict[str, List[PartDebugInfo]] = field(default_factory=dict)  # NEW

    def add_link_parts(self, link_name: str, parts: List[PartDebugInfo]) -> None:
        """Add part breakdown for a link."""
        self.link_parts[link_name] = parts
```

**Step 3: Add table generation method**

```python
def generate_debug_table(self) -> str:
    """Generate markdown table of parts breakdown per link."""
    if not self.link_parts:
        return "No part debug information available."

    lines = ["# Inertia Calculation Debug Report\n"]

    for link_name in sorted(self.link_parts.keys()):
        parts = self.link_parts[link_name]
        props = self.link_properties.get(link_name)

        lines.append(f"## Link: {link_name}\n")

        # Table header
        lines.append("| Part | BOM Match | Mass Source | Material | Volume (cm³) | Mass (kg) | Status |")
        lines.append("|------|-----------|-------------|----------|--------------|-----------|--------|")

        # Table rows
        link_warnings = []
        for part in parts:
            status = "✓"
            if part.warnings:
                status = "⚠️ " + part.warnings[0]
                link_warnings.extend(part.warnings)

            bom_match = part.bom_match or "-"
            if part.match_type == "fuzzy" and part.bom_match:
                bom_match = f"(fuzzy: {part.bom_match})"

            material = part.material or "-"

            lines.append(
                f"| {part.part_id} | {bom_match} | {part.mass_source} | "
                f"{material} | {part.volume_cm3:.2f} | {part.mass_kg:.4f} | {status} |"
            )

        # Link total
        total_mass = props.mass if props else sum(p.mass_kg for p in parts)
        lines.append(f"\n**Link Total:** {total_mass:.4f} kg")

        if link_warnings:
            lines.append(f"**Warnings:** {len(link_warnings)} parts with issues")

        lines.append("\n---\n")

    return "\n".join(lines)

def save_debug_table(self, output_path: Path) -> None:
    """Save debug table to file."""
    table = self.generate_debug_table()
    with open(output_path, 'w') as f:
        f.write(table)
    logger.info(f"Debug table saved to {output_path}")
```

---

### Task 2: Populate PartDebugInfo in InertiaCalculator

**Files:**
- Modify: `src/onshape2xacro/inertia/calculator.py`

**Update `compute_from_step_with_bom()`:**

```python
def compute_from_step_with_bom(
    self,
    step_path: Path,
    bom_entries: Dict[str, "BOMEntry"],
    link_name: str,
    report: "InertiaReport",
) -> InertialProperties:
    """
    Compute inertial properties using BOM data for per-part mass.

    Now also populates debug information in report.link_parts.
    """
    from .report import PartDebugInfo  # Import here to avoid circular

    model = cq.importers.importStep(str(step_path))

    try:
        solids = model.solids().vals()
    except Exception:
        solids = [model.val()]

    if not solids:
        logger.warning(f"No solids found in {step_path}")
        return InertialProperties(
            mass=0.0, com=(0.0, 0.0, 0.0), ixx=0.0, iyy=0.0, izz=0.0
        )

    # Aggregate properties
    total_mass = 0.0
    weighted_com = [0.0, 0.0, 0.0]
    total_ixx = 0.0
    total_iyy = 0.0
    total_izz = 0.0
    total_ixy = 0.0
    total_ixz = 0.0
    total_iyz = 0.0

    part_debug_infos = []  # NEW: Collect debug info

    for i, solid in enumerate(solids):
        part_id = f"solid_{i}"
        part_warnings = []

        # Try to find matching BOM entry
        bom_entry = bom_entries.get(link_name)
        match_type = "none"

        if bom_entry:
            match_type = "exact"
        else:
            # Try fuzzy match
            for bom_name in bom_entries:
                if (
                    bom_name.lower() in link_name.lower()
                    or link_name.lower() in bom_name.lower()
                ):
                    bom_entry = bom_entries[bom_name]
                    match_type = "fuzzy"
                    part_warnings.append(f"Fuzzy matched to BOM entry '{bom_name}'")
                    break

        # Compute volume
        volume_mm3 = solid.Volume()
        volume_cm3 = volume_mm3 / 1000.0  # mm³ to cm³

        # Determine mass source and compute
        if bom_entry and bom_entry.has_mass:
            # Priority 1: BOM mass
            mass_source = "BOM Mass"
            material = bom_entry.material
            solid_mass = bom_entry.mass_kg / len(solids)
            props = self._compute_with_known_mass(solid, solid_mass)

        elif bom_entry and bom_entry.has_material:
            # Priority 2: Material density
            mass_source = "Material Density"
            material = bom_entry.material
            density = self._get_density(material)
            if density is None:
                density = self.default_density
                mass_source = "Default Density"
                part_warnings.append(f"Material '{material}' unknown, used default density")
            props = self._compute_solid_properties(solid, density)

        elif bom_entry:
            # Priority 3: No mass, no material
            mass_source = "Zero (no data)"
            material = None
            part_warnings.append("No mass or material in BOM")
            report.add_warning(link_name, part_id, "No mass or material assigned in BOM")
            props = InertialProperties(
                mass=0.0, com=(0.0, 0.0, 0.0), ixx=0.0, iyy=0.0, izz=0.0
            )
        else:
            # No BOM entry
            mass_source = "Default Density"
            material = None
            part_warnings.append("Not found in BOM")
            report.add_warning(link_name, part_id, "Part not found in BOM, using default density")
            props = self._compute_solid_properties(solid, self.default_density)

        # Record debug info
        debug_info = PartDebugInfo(
            part_id=part_id,
            bom_match=bom_entry.name if bom_entry else None,
            match_type=match_type,
            mass_source=mass_source,
            material=material,
            volume_cm3=volume_cm3,
            mass_kg=props.mass,
            warnings=part_warnings,
        )
        part_debug_infos.append(debug_info)

        # Aggregate (existing code)
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

    # Store debug info in report
    report.add_link_parts(link_name, part_debug_infos)

    # Finalize COM (existing code)
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

---

### Task 3: Save debug table in XacroSerializer

**Files:**
- Modify: `src/onshape2xacro/serializers/__init__.py`

**After `report.print_summary()`:**

```python
if report:
    report.print_summary()

    # Save debug table if parts info is available
    if report.link_parts:
        debug_table_path = out_dir / "inertia_debug.md"
        report.save_debug_table(debug_table_path)
```

---

### Task 4: Add tests

**Files:**
- Create: `tests/test_inertia/test_debug_table.py`

```python
"""Tests for inertia debug table generation."""
import pytest
from onshape2xacro.inertia.report import InertiaReport, PartDebugInfo
from onshape2xacro.inertia.types import InertialProperties


def test_part_debug_info_creation():
    info = PartDebugInfo(
        part_id="solid_0",
        bom_match="link_base",
        match_type="exact",
        mass_source="BOM Mass",
        material="aluminum",
        volume_cm3=312.5,
        mass_kg=0.852,
        warnings=[],
    )
    assert info.part_id == "solid_0"
    assert info.mass_source == "BOM Mass"


def test_generate_debug_table_empty():
    report = InertiaReport()
    table = report.generate_debug_table()
    assert "No part debug information" in table


def test_generate_debug_table_with_parts():
    report = InertiaReport()

    # Add link properties
    report.link_properties["link_base"] = InertialProperties(
        mass=0.852, com=(0.1, 0.0, 0.0), ixx=0.001, iyy=0.001, izz=0.001
    )

    # Add part debug info
    parts = [
        PartDebugInfo(
            part_id="solid_0",
            bom_match="link_base",
            match_type="exact",
            mass_source="BOM Mass",
            material=None,
            volume_cm3=312.5,
            mass_kg=0.852,
            warnings=[],
        )
    ]
    report.add_link_parts("link_base", parts)

    table = report.generate_debug_table()

    assert "## Link: link_base" in table
    assert "solid_0" in table
    assert "BOM Mass" in table
    assert "0.852 kg" in table or "0.8520 kg" in table


def test_debug_table_with_warnings():
    report = InertiaReport()

    parts = [
        PartDebugInfo(
            part_id="solid_0",
            bom_match=None,
            match_type="none",
            mass_source="Default Density",
            material=None,
            volume_cm3=100.0,
            mass_kg=0.1,
            warnings=["Not found in BOM"],
        )
    ]
    report.add_link_parts("link_test", parts)

    table = report.generate_debug_table()

    assert "⚠️" in table
    assert "Not found in BOM" in table
```

---

### Task 5: Update __init__.py exports

**Files:**
- Modify: `src/onshape2xacro/inertia/__init__.py`

```python
from .report import InertiaReport, PartWarning, PartDebugInfo

__all__ = [
    "InertiaCalculator",
    "InertialProperties",
    "BOMParser",
    "BOMEntry",
    "InertiaReport",
    "PartWarning",
    "PartDebugInfo",
    "InertiaConfigWriter",
]
```

---

## Expected Output

After running `onshape2xacro export waist --output waist_xacro --bom bom.csv`:

```
waist_xacro/
├── urdf/
├── meshes/
├── config/
│   ├── joint_limits.yaml
│   └── inertials.yaml
└── inertia_debug.md    # NEW: Debug table
```

**inertia_debug.md content:**
```markdown
# Inertia Calculation Debug Report

## Link: sub_asm_base_1_square_base_plate_1

| Part | BOM Match | Mass Source | Material | Volume (cm³) | Mass (kg) | Status |
|------|-----------|-------------|----------|--------------|-----------|--------|
| solid_0 | sub_asm_base_1_square_base_plate_1 | BOM Mass | - | 312.50 | 0.8520 | ✓ |

**Link Total:** 0.8520 kg

---

## Link: sub_asm_vert_1_j_waist_yaw_a_cnc_1

| Part | BOM Match | Mass Source | Material | Volume (cm³) | Mass (kg) | Status |
|------|-----------|-------------|----------|--------------|-----------|--------|
| solid_0 | (fuzzy: waist_yaw) | Material Density | aluminum | 508.15 | 1.3720 | ⚠️ Fuzzy matched to BOM entry 'waist_yaw' |

**Link Total:** 1.3720 kg
**Warnings:** 1 parts with issues

---
```

---

## Verification Checklist

- [ ] PartDebugInfo dataclass created with all fields
- [ ] InertiaReport.add_link_parts() method works
- [ ] InertiaReport.generate_debug_table() produces markdown
- [ ] InertiaCalculator populates debug info during calculation
- [ ] Debug table saved to `inertia_debug.md` in output directory
- [ ] Console shows "Debug table saved to..." message
- [ ] Tests pass for debug table generation
- [ ] Manual test with real BOM shows useful debugging info

---

## Benefits

1. **Debugging BOM Matching:** See exactly which parts matched which BOM entries
2. **Mass Validation:** Verify computed masses match expectations per-part
3. **Fuzzy Match Visibility:** Identify when fuzzy matching is being used
4. **Volume Sanity Check:** Spot unrealistic volumes that indicate geometry issues
5. **Audit Trail:** Understand how final link mass was calculated
