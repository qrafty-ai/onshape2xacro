# Add CoACD Tuning Parameters to Configuration

## TL;DR

> **Quick Summary**: Add a tree-like configuration structure for collision options, allowing users to tune CoACD's speed and accuracy (threshold, resolution, etc.) through `configuration.yaml` or the CLI.
>
> **Deliverables**:
> - Tree-like configuration in `export_config.py` (e.g., `collision_option.coacd.threshold`).
> - CLI support for tuning parameters.
> - Exporter update to use these parameters.
>
> **Estimated Effort**: Medium
> **Parallel Execution**: Sequential (plumbing -> logic)
> **Critical Path**: Config -> Pipeline -> Serializer -> Exporter

---

## Context

### Original Request
"what options do I have to tune the coacd speed/accuracy? I don't actually need it to be this accurate and this much number of decompositions ... sure do that, and name the key in a tree-like way (e.g. collision_option.coacd....)"

### Parameters to Expose
- `threshold`: concave error allowed (default: 0.05, range: 0.02-0.1).
- `resolution`: voxel resolution (default: 2000).
- `max_convex_hull`: limit on number of hulls (default: 32).
- `preprocess`: whether to run manifold preprocessing (default: true).

---

## Work Objectives

### Core Objective
Expose CoACD tuning knobs in a structured way.

### Concrete Deliverables
- `src/onshape2xacro/config/export_config.py`: Nested `CoACDOptions` and `CollisionOptions`.
- `src/onshape2xacro/schema.py`: CLI-friendly versions of the above.
- `src/onshape2xacro/mesh_exporters/step.py`: Uses configured parameters instead of hardcoded defaults.

### Definition of Done
- [ ] `configuration.yaml` supports `collision_option.coacd.threshold`.
- [ ] `onshape2xacro export --collision-option.coacd.threshold 0.1` works.
- [ ] `uv run pytest` passes.

---

## TODOs

- [x] 1. Refactor `export_config.py` for tree-like collision options
  - Create `CoACDOptions` and `CollisionOptions` dataclasses.
  - Replace `collision_mesh_method` in `ExportOptions` with `collision_option: CollisionOptions`.
  - Update `load()` to recursively load nested dicts (or handle `collision_option` manually).
  - Update `merge_cli_overrides()` signature and logic.

- [x] 2. Update `schema.py` and `cli/__init__.py`
  - Mirror the nested structure in `ExportConfig`.
  - Ensure CLI overrides correctly propagate through to the final config object.

- [x] 3. Plumb `CollisionOptions` through Pipeline and Serializer
  - Update `run_export` to pass `export_configuration.export.collision_option`.
  - Update `serializer.save` and `_export_meshes` to accept and pass the options.

- [x] 4. Update `StepMeshExporter` to use tuned parameters
  - Update `export_link_meshes` to accept `collision_option`.
  - In the CoACD block, use `collision_option.coacd.threshold`, `resolution`, `max_convex_hull`, `preprocess`, and `seed`.

- [x] 5. Verification
  - Create a test robot config with very low resolution and high threshold.
  - Verify CoACD completes faster/simpler (manually or via logging).
  - Run full test suite.
