# Improve Collision Mesh Compression with CoACD

## TL;DR

> **Quick Summary**: Replace the current single-convex-hull generation (using pymeshlab) with **CoACD** (Approximate Convex Decomposition) to generate accurate collision meshes for concave objects.
>
> **Deliverables**:
> - Updated `pyproject.toml` with `coacd`.
> - Updated `step.py` to use `coacd` for collision generation.
> - Updated `serializers/__init__.py` to support multiple collision meshes per link.
>
> **Estimated Effort**: Medium
> **Parallel Execution**: Sequential (dependency -> logic -> serializer)
> **Critical Path**: Add Dependency -> Update Logic -> Update Serializer

---

## Context

### Original Request
"The collision mesh compression algorithm does not work very well. Research and implement a better solution using convex decomposition libs."

### Interview Summary
**Key Discussions**:
- **Problem**: Current `pymeshlab` implementation generates a single convex hull, destroying concave features (e.g., rings become disks).
- **Solution**: Switch to **CoACD** (Collision-Aware Concave Decomposition).
- **Configuration**: User requested **Sensible Defaults** (no exposed config parameters).
- **Determinism**: We will set a fixed seed for reproducible exports.

**Research Findings**:
- **CoACD** is available as a python package and is state-of-the-art.
- It produces multiple convex hulls, requiring the serializer to handle list-based collision entries.

### Metis Review
**Identified Gaps** (addressed):
- **Compatibility**: `coacd` returns `(verts, faces)` tuples, needing conversion back to `trimesh` for saving.
- **Contract**: `mesh_map` will now support `List[str]` for collision files.
- **Fallback**: Existing `pymeshlab` logic is preserved as a fallback if `coacd` fails.

---

## Work Objectives

### Core Objective
Implement accurate collision mesh generation using CoACD.

### Concrete Deliverables
- `pyproject.toml`: Added `coacd` dependency.
- `src/onshape2xacro/mesh_exporters/step.py`: Logic to decompose meshes.
- `src/onshape2xacro/serializers/__init__.py`: Logic to write multiple `<collision>` tags.

### Definition of Done
- [x] `onshape2xacro` exports valid Xacro with multiple collision meshes for complex parts.
- [x] `uv run pytest` passes.
- [x] Agent-executed verification confirms CoACD was called and produced output.

### Must Have
- **Determinism**: Set `seed` in CoACD.
- **Backward Compatibility**: If only 1 hull is generated, behavior remains similar (though filename might change if we standardize on list).

### Must NOT Have (Guardrails)
- **Config Exposure**: Do NOT add new parameters to `export_config.py`.
- **Visual Mesh Changes**: Do NOT modify visual mesh generation.

---

## Verification Strategy (MANDATORY)

> **UNIVERSAL RULE: ZERO HUMAN INTERVENTION**
> ALL tasks in this plan MUST be verifiable WITHOUT any human action.

### Test Decision
- **Infrastructure exists**: YES (`pytest` in `tests/`)
- **Automated tests**: YES (Update existing tests / Add new test)
- **Agent-Executed QA**: YES

### Agent-Executed QA Scenarios (MANDATORY)

```
Scenario: CoACD Dependency Check
  Tool: Bash
  Preconditions: uv installed
  Steps:
    1. Run `uv pip list | grep coacd` OR try python import
    2. python -c "import coacd; print('CoACD available')"
  Expected Result: "CoACD available" printed
  Evidence: Terminal output

Scenario: Verify Multi-Collision Generation
  Tool: Bash (Python script)
  Preconditions: coacd installed, trimesh installed
  Steps:
    1. Create a script `verify_coacd.py` that loads a concave STL (e.g., torus).
    2. Run `coacd.run_coacd` on it.
    3. Assert len(parts) > 1 (concave object should split).
    4. Save parts to disk.
  Expected Result: Script exits 0, multiple parts generated.
  Evidence: Script output

Scenario: Verify Xacro Output
  Tool: Bash
  Preconditions: Mock export data
  Steps:
    1. Create a dummy `mesh_map` with a list of collision files.
    2. Run a script that invokes `XacroSerializer._link_to_xacro` (or similar).
    3. Parse output XML.
    4. Assert multiple `<collision>` tags exist for the link.
  Expected Result: XML contains multiple collision tags.
  Evidence: XML output
```

---

## Execution Strategy

### Parallel Execution Waves

```
Wave 1 (Dependencies):
└── Task 1: Add coacd dependency

Wave 2 (Logic):
└── Task 2: Implement CoACD logic in step.py

Wave 3 (Serializer):
└── Task 3: Update XacroSerializer to handle lists

Wave 4 (Verification):
└── Task 4: Run tests and verification scripts
```

---

## TODOs

- [x] 1. Add CoACD Dependency

  **What to do**:
  - Add `coacd` to `pyproject.toml` dependencies.
  - Run `uv sync` or equivalent to update lockfile (if applicable/allowed).
  - Verify import works.

  **Recommended Agent Profile**:
  - **Category**: `quick`
  - **Skills**: [`git-master`]

  **Acceptance Criteria**:
  - [ ] `grep "coacd" pyproject.toml` returns match.
  - [ ] `uv run python -c "import coacd"` exits with 0.

- [x] 2. Implement CoACD Logic in step.py

  **What to do**:
  - Modify `src/onshape2xacro/mesh_exporters/step.py`.
  - Import `coacd` (inside method or top level).
  - In `export_link_meshes`:
    - Load mesh using `trimesh.load(..., force='mesh')`.
    - Run `coacd.run_coacd(mesh.vertices, mesh.faces, threshold=0.05, max_convex_hulls=32, seed=42)`.
    - Loop through result `parts`.
    - Save each part as `collision/{link_name}_{i}.stl`.
    - Update `mesh_map[link_name]["collision"]` to be a `List[str]`.
    - Keep `pymeshlab` (or use `trimesh.convex.convex_hull`) as fallback in `except` block.

  **Must NOT do**:
  - Do not change visual mesh export.
  - Do not remove `pymeshlab` import (used for other things/fallback).

  **Recommended Agent Profile**:
  - **Category**: `visual-engineering` (for mesh processing logic)
  - **Skills**: [`code-reviewer`]

  **Acceptance Criteria**:
  - [ ] Logic handles CoACD success: returns list of files.
  - [ ] Logic handles CoACD failure: returns string (fallback).
  - [ ] Files saved with `_0.stl`, `_1.stl` suffix.

- [x] 3. Update XacroSerializer for Multi-Collision

  **What to do**:
  - Modify `src/onshape2xacro/serializers/__init__.py`.
  - In `_link_to_xacro`:
    - Check if `entry.get("collision")` is a list.
    - If list: iterate and create `<collision>` tag for each file.
    - If string: create single `<collision>` tag (legacy/fallback).

  **Recommended Agent Profile**:
  - **Category**: `quick`
  - **Skills**: [`code-reviewer`]

  **Acceptance Criteria**:
  - [ ] XML generation works for List input.
  - [ ] XML generation works for String input.

- [x] 4. Verification and Testing

  **What to do**:
  - Create a temporary verification script `verify_full_flow.py` that mocks the mesh export and serialization.
  - Run `uv run pytest` to ensure no regressions in existing tests.

  **Recommended Agent Profile**:
  - **Category**: `quick`
  - **Skills**: [`git-master`]

  **Acceptance Criteria**:
  - [ ] Verification script confirms multiple collision tags generated.
  - [ ] Existing tests pass.

---

## Commit Strategy

| After Task | Message | Files | Verification |
|------------|---------|-------|--------------|
| 1 | `chore: add coacd dependency` | pyproject.toml | python -c "import coacd" |
| 2 | `feat: implement coacd collision generation` | src/onshape2xacro/mesh_exporters/step.py | (manual script) |
| 3 | `feat: support multiple collision meshes in xacro` | src/onshape2xacro/serializers/__init__.py | (manual script) |
| 4 | `test: verify collision mesh update` | (scripts) | uv run pytest |
