# Make Collision Mesh Decomposition Optional

## TL;DR

> **Quick Summary**: Make the slow CoACD collision decomposition optional by adding a configuration toggle. Users can choose between "fast" (single convex hull) and "coacd" (accurate decomposition) methods.
>
> **Deliverables**:
> - Updated `src/onshape2xacro/config/export_config.py` with `collision_mesh_method` option.
> - Updated `src/onshape2xacro/schema.py` and `cli/__init__.py` for CLI support.
> - Updated `src/onshape2xacro/mesh_exporters/step.py` to support both "fast" and "coacd" methods.
> - Updated `src/onshape2xacro/pipeline.py` and `serializers/__init__.py` to plumb the option.
>
> **Estimated Effort**: Medium
> **Parallel Execution**: Sequential (plumbing -> logic)
> **Critical Path**: Config -> Pipeline -> Serializer -> Exporter Logic

---

## Context

### Original Request
"turns out that this process is way slower than the original method. Make this optional and add to configuration"

### Interview Summary (Metis Review)
- **Problem**: CoACD is too slow for all-the-time use.
- **Solution**: Implement a toggle between "fast" (original pymeshlab method) and "coacd".
- **Default**: "fast" (for speed).
- **Contract**: Exporter will now ALWAYS return `List[str]` for collision, even if it's just one file.
- **CLI**: Supports tri-state override to enable/disable via command line.

---

## Work Objectives

### Core Objective
Allow users to choose between fast (convex hull) and accurate (CoACD) collision generation.

### Concrete Deliverables
- `src/onshape2xacro/config/export_config.py`: Added `collision_mesh_method: Literal["fast", "coacd"] = "fast"`.
- `src/onshape2xacro/schema.py`: Added CLI flag `--collision-mesh-method`.
- `src/onshape2xacro/mesh_exporters/step.py`: Restored `pymeshlab` fast path and wrapped `coacd` in a conditional.
- `src/onshape2xacro/serializers/__init__.py`: Passing option to exporter.
- `src/onshape2xacro/pipeline.py`: Passing option from config to serializer.

### Definition of Done
- [x] `onshape2xacro export` uses fast method by default.
- [x] `onshape2xacro export --collision-mesh-method coacd` uses CoACD.
- [x] `uv run pytest` passes.

### Must Have
- **Normalization**: Exporter always returns `List[str]` for collision.
- **Lazy Import**: `coacd` should only be imported if needed.
- **Precedence**: CLI > configuration.yaml > default ("fast").

---

## Verification Strategy (MANDATORY)

### Agent-Executed QA Scenarios (MANDATORY)

```
Scenario: Verify Default (Fast) Method
  Tool: Bash (Python script)
  Steps:
    1. Run export on a test part (mocked).
    2. Verify `coacd.run_coacd` was NOT called.
    3. Verify `pymeshlab.MeshSet` was used.
    4. Verify `mesh_map` collision is `['collision/part_0.stl']`.
  Expected Result: Fast path used.

Scenario: Verify CoACD Method
  Tool: Bash (Python script)
  Steps:
    1. Run export with `--collision-mesh-method coacd`.
    2. Verify `coacd.run_coacd` WAS called.
  Expected Result: CoACD path used.
```

---

## Execution Strategy

### wave 1: Plumbing (Config & Schema)
- [x] 1. Update `ExportOptions` in `export_config.py`
- [x] 2. Update `ExportConfig` in `schema.py`
- [x] 3. Update CLI merge logic in `cli/__init__.py`

### wave 2: Implementation (Exporter & Pipeline)
- [x] 4. Update `pipeline.py` to pass option to serializer
- [x] 5. Update `serializers/__init__.py` to pass option to exporter
- [x] 6. Restore Fast Path in `step.py` and add toggle logic
- [x] 7. Update tests and run suite
  - Update existing CoACD tests to explicitly set the method.
  - Run `uv run pytest`.

---

## Commit Strategy
- `feat(config): add collision_mesh_method option`
- `feat(exporter): implement optional collision decomposition`
- `test: verify collision method toggle`
