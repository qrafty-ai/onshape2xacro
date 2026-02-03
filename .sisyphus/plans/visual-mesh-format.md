# Add Visual Mesh Format Selection (DAE/OBJ/GLB)

> [!IMPORTANT]
> **DEPRECATED**: This plan has been marked as finished by user request. All tasks are considered complete.

## TL;DR

> **Quick Summary**: Add a new CLI argument `--visual-mesh-format` to allow users to select the export format for visual meshes (GLB, DAE, OBJ, STL). The default will remain GLB.
>
> **Deliverables**:
> - Updated `pyproject.toml` with `pycollada` dependency.
> - Updated CLI to accept `--visual-mesh-format`.
> - Updated pipeline and serializer to pass the format down.
> - Updated `StepMeshExporter` to export the selected format.
>
> **Estimated Effort**: Short
> **Parallel Execution**: Sequential (logic flow is linear)
> **Critical Path**: Config -> Pipeline -> Serializer -> Mesh Exporter

---

## Context

### Original Request
User wants to choose between DAE, OBJ, and GLB for visual mesh export. The current pipeline hardcodes GLB.

### Interview Summary
**Key Discussions**:
- **Selection**: CLI argument `--visual-mesh-format` (default: `glb`).
- **Replacement**: The selected format replaces the default. Only one visual mesh file per link.
- **Dependencies**: Add `pycollada` for DAE support.
- **Color**: Stick to geometry-only pipeline (STL intermediate) for now.

### Metis Review
**Identified Gaps** (addressed):
- **Collision Mesh**: Explicitly enforced to stay as STL for physics stability.
- **Optimization**: If `stl` is selected as visual format, skip `trimesh` overhead and use raw STL.
- **Validation**: Ensure `pycollada` dependency is present.

---

## Work Objectives

### Core Objective
Enable configurable visual mesh export formats.

### Concrete Deliverables
- [x] `pyproject.toml` updated
- [x] `src/onshape2xacro/cli/__init__.py` updated with new argument
- [x] `src/onshape2xacro/pipeline.py` updated
- [x] `src/onshape2xacro/serializers/__init__.py` updated
- [x] `src/onshape2xacro/mesh_exporters/step.py` updated

### Definition of Done
- [x] `onshape2xacro export ... --visual-mesh-format dae` produces `.dae` files in `meshes/` directory.
- [x] Generated `.xacro` files reference the correct `.dae` files.
- [x] `uv sync` installs `pycollada`.

### Must Have
- Support for `glb`, `dae`, `obj`, `stl`.
- Default to `glb` if unspecified.
- Collision meshes must remain `stl`.

### Must NOT Have (Guardrails)
- Do not attempt to preserve original CAD colors (out of scope, requires non-STL pipeline).
- Do not output multiple formats simultaneously (one visual format per run).

---

## Verification Strategy

### Test Decision
- **Infrastructure exists**: YES (pytest)
- **User wants tests**: Manual/Simple run (User asked for "Verification (manual test or simple run)")
- **Framework**: `pytest` (available) but we will use `interactive_bash` for end-to-end verification.

### Automated Verification Only (NO User Intervention)

**Config/Infra changes** (using Bash):
```bash
# Verify dependency addition
grep "pycollada" pyproject.toml
```

**CLI Verification** (using Bash):
```bash
# Verify help message shows new argument
uv run onshape2xacro export --help | grep "visual-mesh-format"
```

**Pipeline Verification** (using Bash - Dry Run if possible or Code Check):
```bash
# Check if code handles the argument (static check)
grep "visual_mesh_format" src/onshape2xacro/serializers/__init__.py
```

---

## Execution Strategy

### Parallel Execution Waves

```
Wave 1 (Configuration & Dependencies):
├── Task 1: Add pycollada dependency
└── Task 2: Update CLI configuration

Wave 2 (Pipeline Propagation):
├── Task 3: Update pipeline.py
└── Task 4: Update serializers/__init__.py

Wave 3 (Implementation):
└── Task 5: Implement export logic in StepMeshExporter
```

---

## TODOs

- [x] 1. Add pycollada dependency

  **What to do**:
  - Edit `pyproject.toml` to add `"pycollada>=0.8"` to the `dependencies` list.

  **Recommended Agent Profile**:
  - **Category**: `quick`
  - **Skills**: [`git-master`]

  **Parallelization**:
  - **Can Run In Parallel**: YES
  - **Parallel Group**: Wave 1

  **Verification**:
  ```bash
  grep "pycollada" pyproject.toml
  ```

- [x] 2. Update CLI configuration

  **What to do**:
  - Edit `src/onshape2xacro/cli/__init__.py`.
  - Import `Literal` from `typing`.
  - Add `visual_mesh_format: Literal["glb", "dae", "obj", "stl"] = "glb"` to `ExportConfig` dataclass.

  **Recommended Agent Profile**:
  - **Category**: `quick`

  **Parallelization**:
  - **Can Run In Parallel**: YES
  - **Parallel Group**: Wave 1

  **Verification**:
  ```bash
  uv run onshape2xacro export --help | grep "visual-mesh-format"
  ```

- [x] 3. Update pipeline.py

  **What to do**:
  - Edit `src/onshape2xacro/pipeline.py` in `run_export`.
  - Pass `visual_mesh_format=config.visual_mesh_format` to `serializer.save()`.

  **Recommended Agent Profile**:
  - **Category**: `quick`

  **Parallelization**:
  - **Can Run In Parallel**: YES
  - **Parallel Group**: Wave 2
  - **Blocked By**: Task 2

  **Verification**:
  ```bash
  # Static check
  grep "visual_mesh_format=config.visual_mesh_format" src/onshape2xacro/pipeline.py
  ```

- [x] 4. Update serializers/__init__.py

  **What to do**:
  - Edit `src/onshape2xacro/serializers/__init__.py`.
  - Update `XacroSerializer.save` method signature to accept `visual_mesh_format: str = "glb"`.
  - Update call to `exporter.export_link_meshes` to pass `visual_mesh_format=visual_mesh_format`.

  **Recommended Agent Profile**:
  - **Category**: `quick`

  **Parallelization**:
  - **Can Run In Parallel**: YES
  - **Parallel Group**: Wave 2
  - **Blocked By**: Task 3

  **Verification**:
  ```bash
  # Static check
  grep "visual_mesh_format=visual_mesh_format" src/onshape2xacro/serializers/__init__.py
  ```

- [x] 5. Implement export logic in StepMeshExporter

  **What to do**:
  - Edit `src/onshape2xacro/mesh_exporters/step.py`.
  - Update `export_link_meshes` signature to accept `visual_mesh_format: str = "glb"`.
  - Inside the mesh processing loop:
    - Determine file extension: `ext = visual_mesh_format`.
    - `vis_filename = f"{link_name}.{ext}"`.
    - If `visual_mesh_format == "stl"`:
      - Copy `temp_stl` to `vis_path`.
    - Else:
      - Use `mesh = trimesh.load(..., force="mesh")`.
      - `mesh.export(vis_path)`.
    - Update `mesh_map` to use `vis_filename`.

  **Recommended Agent Profile**:
  - **Category**: `ultrabrain`
  - **Skills**: [`simplify`]

  **Parallelization**:
  - **Can Run In Parallel**: NO
  - **Parallel Group**: Wave 3
  - **Blocked By**: Task 4

  **Verification**:
  ```bash
  # Check if code contains logic for extension handling
  grep "vis_filename =" src/onshape2xacro/mesh_exporters/step.py
  ```

---

## Commit Strategy

| After Task | Message | Files |
|------------|---------|-------|
| 1-5 | `feat(export): add --visual-mesh-format support (dae/obj/glb/stl)` | All modified files |
