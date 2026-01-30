# Inertia Calculation: Parallel Axis Theorem Fix

## Context

### Original Request
Fix the moment of inertia calculation to correctly apply the parallel axis theorem (Steiner's theorem) when aggregating inertias from multiple parts into a single robot link.

### Interview Summary
**Key Discussions**:
- Current code simply sums inertia tensors from parts at different COM positions - this is mathematically incorrect
- Each part's inertia must be shifted to the link's combined COM before summing
- User wants inertia columns (ixx, iyy, izz) added to debug table for verification
- User confirmed using COM frame for URDF (standard practice)
- User wants basic sanity checks for inertia values

**Technical Finding**:
- CadQuery `matrixOfInertia()` returns inertia about part's own COM in mm^5 units
- Current code correctly computes mass-weighted link COM
- Current code incorrectly sums inertias without applying parallel axis theorem
- URDF `<origin>` inside `<inertial>` specifies COM position relative to link frame

---

## Work Objectives

### Core Objective
Implement correct inertia tensor aggregation using the parallel axis theorem so that multi-part links have physically accurate inertia values.

### Concrete Deliverables
- Modified `calculator.py` with parallel axis theorem implementation
- Enhanced `report.py` with per-part inertia columns in debug table
- Inertia validation function with sanity checks
- Updated debug table output showing per-part inertias

### Definition of Done
- [ ] Run export on `waist/` test data: `uv run python -c "..."` completes without errors
- [ ] Debug table shows per-part ixx, iyy, izz values
- [ ] Link total inertia values are larger than simple sum (Steiner terms add positive values)
- [ ] No validation warnings for well-formed parts

### Must Have
- Parallel axis theorem correctly applied for all 6 inertia tensor components (ixx, iyy, izz, ixy, ixz, iyz)
- Two-pass algorithm: first compute link COM, then shift and sum
- Per-part inertia in debug table
- Basic validation (positive diagonal, triangle inequality)

### Must NOT Have (Guardrails)
- Do NOT modify mass calculation logic (already fixed)
- Do NOT change BOM matching logic
- Do NOT add complex eigenvalue-based validation
- Do NOT refactor the overall file structure
- Do NOT add new dependencies

---

## Verification Strategy

### Test Decision
- **Infrastructure exists**: NO (no formal test suite for inertia)
- **User wants tests**: Manual verification via debug table and known physics
- **QA approach**: Manual verification with sanity checks

### Manual Verification Approach
Each TODO includes verification via:
1. Running export on test data (`waist/`)
2. Checking debug table for expected values
3. Comparing before/after inertia values

---

## Task Flow

```
Task 1 (Parallel Axis Helper)
    ↓
Task 2 (Two-Pass Algorithm)
    ↓
Task 3 (Debug Table Enhancement)
    ↓
Task 4 (Validation)
    ↓
Task 5 (Integration Test)
```

## Parallelization

| Task | Depends On | Reason |
|------|------------|--------|
| 1 | - | Independent helper function |
| 2 | 1 | Uses the helper function |
| 3 | 2 | Needs per-part shifted inertias |
| 4 | 2 | Validates aggregated results |
| 5 | 2,3,4 | Final integration verification |

---

## TODOs

- [ ] 1. Create parallel axis theorem helper function

  **What to do**:
  - Add function `_apply_parallel_axis_theorem(props: InertialProperties, link_com: Tuple[float, float, float]) -> InertialProperties`
  - Compute displacement vector: `dx, dy, dz = part_com - link_com`
  - Apply Steiner's theorem formulas:
    - `ixx_shifted = ixx + m * (dy² + dz²)`
    - `iyy_shifted = iyy + m * (dx² + dz²)`
    - `izz_shifted = izz + m * (dx² + dy²)`
    - `ixy_shifted = ixy - m * dx * dy`
    - `ixz_shifted = ixz - m * dx * dz`
    - `iyz_shifted = iyz - m * dy * dz`
  - Return new InertialProperties with shifted values (same mass and COM)

  **Must NOT do**:
  - Do not modify the original props object
  - Do not change mass or COM values in returned object

  **Parallelizable**: NO (first task)

  **References**:
  - `src/onshape2xacro/inertia/calculator.py:191-220` - Existing `_compute_solid_properties` as pattern
  - `src/onshape2xacro/inertia/types.py:8-18` - InertialProperties dataclass structure
  - Physics reference: Parallel axis theorem (Steiner's theorem) - standard mechanics

  **Acceptance Criteria**:
  - [ ] Function exists in `calculator.py` after `_compute_solid_properties`
  - [ ] Function signature matches: `_apply_parallel_axis_theorem(self, props: InertialProperties, link_com: Tuple[float, float, float]) -> InertialProperties`
  - [ ] All 6 tensor components are shifted (ixx, iyy, izz, ixy, ixz, iyz)
  - [ ] Verify with mental math: for part 1m from COM with 1kg mass, ixx should increase by ~1 kg·m²

  **Commit**: NO (groups with 2)

---

- [ ] 2. Implement two-pass inertia aggregation algorithm

  **What to do**:
  - Modify `compute_from_step_with_bom()` in calculator.py
  - **First pass** (lines 408-467): Keep existing logic that computes per-part properties and link COM - already correct
  - **Second pass** (new): After link COM is computed (line 484), iterate through parts again:
    - For each part, call `_apply_parallel_axis_theorem(part_props, link_com)`
    - Sum the shifted inertia components
  - Store both original and shifted inertias for debug table
  - Replace current direct summation (lines 473-478) with shifted summation

  **Must NOT do**:
  - Do not change the mass calculation
  - Do not change the COM calculation (already correct)
  - Do not remove existing debug info collection

  **Parallelizable**: NO (depends on Task 1)

  **References**:
  - `src/onshape2xacro/inertia/calculator.py:408-497` - Current aggregation logic to modify
  - `src/onshape2xacro/inertia/calculator.py:469-478` - Lines to replace with shifted summation
  - `src/onshape2xacro/inertia/calculator.py:482-486` - Link COM calculation (keep as-is)

  **Acceptance Criteria**:
  - [ ] Two-pass algorithm implemented: first collect parts + compute COM, then shift and sum
  - [ ] Run: `cd /home/cc/codes/onshape2xacro/.worktrees/inertia && uv run python -c "from pathlib import Path; from onshape_robotics_toolkit import KinematicGraph; from onshape2xacro.serializers import XacroSerializer; from onshape2xacro.config import ConfigOverride; from onshape2xacro.condensed_robot import CondensedRobot; import pickle; cache_dir = Path('waist'); output_dir = Path('waist_xacro'); bom_path = cache_dir / 'bom.csv'; cad = pickle.load(open(cache_dir / 'cad.pickle', 'rb')); graph = KinematicGraph.from_cad(cad); robot = CondensedRobot.from_graph(graph, cad=cad, name='waist'); robot.cad = cad; robot.asset_path = cache_dir / 'assembly.step'; serializer = XacroSerializer(); serializer.save(robot, str(output_dir), download_assets=True, config=ConfigOverride(), bom_path=str(bom_path)); print('Done')"`
  - [ ] Check `waist_xacro/config/inertials.yaml` - inertia values should be LARGER than before (Steiner terms add positive values to diagonal)
  - [ ] Verify: `cat waist_xacro/config/inertials.yaml | head -15` shows ixx, iyy, izz values

  **Commit**: YES
  - Message: `fix(inertia): apply parallel axis theorem for multi-part links`
  - Files: `src/onshape2xacro/inertia/calculator.py`
  - Pre-commit: Export runs without error

---

- [ ] 3. Enhance debug table with per-part inertia columns

  **What to do**:
  - Modify `PartDebugInfo` in `report.py` to add `ixx`, `iyy`, `izz` fields
  - Update `save_debug_table()` to include inertia columns in markdown output
  - Pass per-part inertia values from calculator to debug info
  - Format inertia values in scientific notation for readability (e.g., `1.23e-04`)

  **Must NOT do**:
  - Do not remove existing columns
  - Do not change the table structure significantly

  **Parallelizable**: NO (depends on Task 2 for data)

  **References**:
  - `src/onshape2xacro/inertia/report.py:15-30` - PartDebugInfo dataclass to modify
  - `src/onshape2xacro/inertia/report.py:100-140` - `save_debug_table()` to modify
  - `src/onshape2xacro/inertia/calculator.py:457-467` - Where debug info is created

  **Acceptance Criteria**:
  - [ ] `PartDebugInfo` has new fields: `ixx: float`, `iyy: float`, `izz: float`
  - [ ] Run export, then: `head -10 waist_xacro/inertia_debug.md`
  - [ ] Debug table header includes: `| Part | ... | Ixx | Iyy | Izz | ... |`
  - [ ] Inertia values displayed in scientific notation

  **Commit**: YES
  - Message: `feat(inertia): add per-part inertia columns to debug table`
  - Files: `src/onshape2xacro/inertia/report.py`, `src/onshape2xacro/inertia/calculator.py`
  - Pre-commit: Export runs without error

---

- [ ] 4. Add inertia validation with sanity checks

  **What to do**:
  - Add function `_validate_inertia(props: InertialProperties) -> List[str]` in calculator.py
  - Check 1: Diagonal elements must be positive (ixx > 0, iyy > 0, izz > 0)
  - Check 2: Triangle inequality: `ixx + iyy >= izz`, `ixx + izz >= iyy`, `iyy + izz >= ixx`
  - Check 3: Warn if any diagonal is exactly zero
  - Return list of warning messages (empty if valid)
  - Call validation after computing final link inertia
  - Add warnings to report if validation fails

  **Must NOT do**:
  - Do not fail/raise on invalid inertia - just warn
  - Do not add eigenvalue checks (too complex)
  - Do not block export on validation failure

  **Parallelizable**: NO (depends on Task 2)

  **References**:
  - `src/onshape2xacro/inertia/calculator.py:488-497` - Where final InertialProperties is created
  - `src/onshape2xacro/inertia/report.py:40-60` - `add_warning()` method pattern
  - Physics: Inertia tensor must be positive semi-definite for physical objects

  **Acceptance Criteria**:
  - [ ] Function `_validate_inertia` exists and returns `List[str]`
  - [ ] Validation called after line 497 (final return)
  - [ ] For valid parts: empty warning list
  - [ ] Test validation logic manually: `python -c "from onshape2xacro.inertia.calculator import InertiaCalculator; ..."`
  - [ ] Warnings appear in report if inertia is invalid

  **Commit**: YES
  - Message: `feat(inertia): add validation checks for inertia tensor`
  - Files: `src/onshape2xacro/inertia/calculator.py`
  - Pre-commit: Export runs without error

---

- [ ] 5. Integration test and verification

  **What to do**:
  - Run full export on test data
  - Compare before/after inertia values
  - Verify debug table shows per-part inertias
  - Check that no validation warnings appear for good data
  - Document the expected behavior

  **Must NOT do**:
  - Do not create formal test files (manual verification only)

  **Parallelizable**: NO (final integration)

  **References**:
  - `waist/` - Test data directory
  - `waist_xacro/config/inertials.yaml` - Output to verify
  - `waist_xacro/inertia_debug.md` - Debug table to verify

  **Acceptance Criteria**:
  - [ ] Run export command (from Task 2) successfully
  - [ ] `cat waist_xacro/config/inertials.yaml` shows reasonable inertia values
  - [ ] `head -15 waist_xacro/inertia_debug.md` shows inertia columns with values
  - [ ] No "validation" warnings in output logs
  - [ ] Inertia values are larger than before the fix (Steiner terms added)
  - [ ] Compare: previous ixx=0.00684 vs new ixx (should be larger)

  **Commit**: NO (verification only)

---

## Commit Strategy

| After Task | Message | Files | Verification |
|------------|---------|-------|--------------|
| 2 | `fix(inertia): apply parallel axis theorem for multi-part links` | calculator.py | Export runs |
| 3 | `feat(inertia): add per-part inertia columns to debug table` | report.py, calculator.py | Debug table has columns |
| 4 | `feat(inertia): add validation checks for inertia tensor` | calculator.py | No false warnings |

---

## Success Criteria

### Verification Commands
```bash
# Run export
cd /home/cc/codes/onshape2xacro/.worktrees/inertia
uv run python -c "from pathlib import Path; from onshape_robotics_toolkit import KinematicGraph; from onshape2xacro.serializers import XacroSerializer; from onshape2xacro.config import ConfigOverride; from onshape2xacro.condensed_robot import CondensedRobot; import pickle; cache_dir = Path('waist'); output_dir = Path('waist_xacro'); bom_path = cache_dir / 'bom.csv'; cad = pickle.load(open(cache_dir / 'cad.pickle', 'rb')); graph = KinematicGraph.from_cad(cad); robot = CondensedRobot.from_graph(graph, cad=cad, name='waist'); robot.cad = cad; robot.asset_path = cache_dir / 'assembly.step'; serializer = XacroSerializer(); serializer.save(robot, str(output_dir), download_assets=True, config=ConfigOverride(), bom_path=str(bom_path)); print('Done')"

# Check inertia values (should be larger than before)
cat waist_xacro/config/inertials.yaml | head -15

# Check debug table has inertia columns
head -10 waist_xacro/inertia_debug.md
```

### Final Checklist
- [ ] Parallel axis theorem correctly applied
- [ ] Link inertia values are larger than simple sum
- [ ] Debug table shows per-part ixx, iyy, izz
- [ ] Validation passes for well-formed parts
- [ ] No regressions in mass calculation
- [ ] All commits follow Conventional Commits format
