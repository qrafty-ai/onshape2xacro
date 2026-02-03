# Improve CLI Defaults Visibility

## TL;DR

> **Quick Summary**: Update `schema.py` to include default values for CoACD options so they appear in the CLI help text, and refine the merge logic in `cli/__init__.py` to handle these defaults gracefully.
>
> **Deliverables**:
> - Updated `src/onshape2xacro/schema.py` with explicit defaults.
> - Updated `src/onshape2xacro/cli/__init__.py` with refined merge logic.
>
> **Estimated Effort**: Low
> **Parallel Execution**: Sequential

---

## Context

### Original Request
"should set default values in cli prompt for better ux"

### Implementation Plan
- Set default values in `CoACDConfig` and `CollisionConfig` fields.
- This ensures `tyro` help shows `(default: 0.05)` instead of `(default: None)`.
- Refine the merge logic to ensure `configuration.yaml` still works as expected.
- We will adopt the priority: **CLI (if different from project default) > configuration.yaml > Project Default**.
- *Note*: If the user explicitly wants to set a value to the project default while `configuration.yaml` has something else, they will need to edit `configuration.yaml` or we accept this minor limitation for better UX.

---

## TODOs

- [x] 1. Update `src/onshape2xacro/schema.py`
- [x] 2. Update `src/onshape2xacro/cli/__init__.py`
- [x] 3. Verification
  - Run `onshape2xacro export --help` and verify output.
