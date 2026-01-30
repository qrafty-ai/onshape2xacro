# Plan: Fix Part Instance Swapping in STEP Export

## TL;DR

> **Quick Summary**: Replace the current index-based part matching in `StepMeshExporter` with distance-based matching to ensure identical part instances are correctly assigned to robot links based on their physical world position.
>
> **Deliverables**:
> - Modified `src/onshape2xacro/mesh_exporters/step.py` with `_pick_match_by_distance` logic.
> - Verification report using `openarm_left` model.
>
> **Estimated Effort**: Short
> **Parallel Execution**: NO - sequential changes to one core file.
> **Critical Path**: Update matching logic → Verify with openarm_left.

---

## Context

### Original Request
Fix an issue where identical part instances (like motors or screws) are swapped between robot links due to arbitrary link processing order and index-based matching.

### Interview Summary
**Key Discussions**:
- **Matching Metric**: Use Euclidean distance between 3D translations.
- **Threshold**: Use a 10mm "soft" threshold for warnings, but always pick the closest available instance.
- **Orientation**: Ignore orientation; translation-only matching is sufficient.
- **Units**: Handle meters (Onshape) vs. millimeters (STEP/OCP) conversion.

**Research Findings**:
- `StepMeshExporter` currently uses a simple counter per key.
- `part_locations` stores `TopLoc_Location` (OCP) for each instance.
- `_part_world_matrix` provides the Onshape world transform.

---

## Work Objectives

### Core Objective
Implement a robust, position-aware part matching algorithm in the STEP mesh exporter.

### Concrete Deliverables
- `src/onshape2xacro/mesh_exporters/step.py`: Updated `StepMeshExporter` class.

### Definition of Done
- [ ] Multiple identical parts in different links are correctly assigned to their nearest STEP instance.
- [ ] Log warnings are emitted if the distance exceeds 10mm.
- [ ] Export of `openarm_left` succeeds and `inertia_debug.md` shows correct assignments.

---

## Verification Strategy

### Test Decision
- **Infrastructure exists**: YES (pytest)
- **User wants tests**: Manual-only (verification via model export)
- **QA approach**: Manual verification using `openarm_left`.

### Automated Verification Procedure

```bash
# 1. Run export for openarm_left (requires Onshape API keys)
onshape2xacro export "https://cad.onshape.com/documents/57444395e4b010c282539097/w/0376180376483664d603a1d3/e/192f15e85501b4c93510e120" --output ./openarm_left_export

# 2. Check inertia_debug.md for part assignments
grep "mesh_match" ./openarm_left_export/inertia_debug.md
```

---

## TODOs

- [ ] 1. Implement `_pick_match_by_distance` in `StepMeshExporter`

  **What to do**:
  - Add `self.assigned_instances = set()` to `__init__` or clear it at start of `export_link_meshes`.
  - Implement `_pick_match_by_distance(self, mkey, world_matrix)`:
    - Extract translation from `world_matrix` (multiply by 1000.0 for mm).
    - Iterate over `part_locations[mkey]`.
    - For each instance `(idx, loc)`, calculate Euclidean distance to `world_matrix` translation.
    - Select the closest instance that is NOT in `self.assigned_instances`.
    - Record the selection in `self.assigned_instances`.
    - Log warning if `dist > 10.0`.
  - Update `export_link_meshes` to pass the part's world matrix to the matching logic.

  **Recommended Agent Profile**:
  - **Category**: `unspecified-high`
    - Reason: Requires precise coordinate math and OCP API knowledge.
  - **Skills**: [`python-programmer`]

  **Acceptance Criteria**:
  - [ ] `_pick_match_by_distance` correctly finds the closest unassigned instance.
  - [ ] Log contains "Distance warning" if parts are far.

- [ ] 2. Verify with `openarm_left`

  **What to do**:
  - Run the export command for the `openarm_left` document.
  - Inspect `inertia_debug.md` to ensure motors/parts are matched to the expected STEP instances.

  **Acceptance Criteria**:
  - [ ] Export completes without errors.
  - [ ] `inertia_debug.md` contains valid `mesh_match` metadata for all parts.

---

## Commit Strategy

| After Task | Message | Files |
|------------|---------|-------|
| 1 | `feat(mesh): implement distance-based part matching for STEP` | `src/onshape2xacro/mesh_exporters/step.py` |
| 2 | `test: verify part matching with openarm_left` | N/A |
