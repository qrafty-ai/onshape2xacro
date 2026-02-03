# Draft: Deep Clean & Coverage Plan

## Requirements (Updated)
- **Goal**: Clean code + Expand test coverage.
- **Constraint**: Do NOT touch ignored files.
- **Coverage Goal**: Implement "all possible unit tests".

## Gap Analysis (Src vs Tests)
- `auth.py` -> NO TEST found.
- `condensed_robot.py` -> NO TEST found (but might be dead?).
- `config` -> `tests/test_config` (Exists).
- `inertia` -> `tests/test_inertia` (Exists).
- `mate_values.py` -> NO TEST found.
- `mesh_exporters` -> `tests/test_mesh_exporters` (Exists).
- `naming.py` -> NO TEST found.
- `pipeline.py` -> `tests/test_pipeline.py` (Exists).
- `serializers` -> `tests/test_serializers` (Exists).
- `validate.py` -> NO TEST found.
- `visualize_export.py` -> NO TEST found.

## Plan Updates
1.  **Cleanup Phase**: (Keep existing: prune pyproject, refactor inertia test).
2.  **Coverage Phase**: Add tests for:
    - `auth.py` (Mock keyring/input).
    - `mate_values.py` (Test parsing logic).
    - `naming.py` (Test string sanitization).
    - `validate.py` (Test validation rules).
    - `condensed_robot.py` (If not dead, add test. If dead, delete).

## Open Questions
- Should `visualize_export.py` be tested? (It's a script, maybe functional test only).
- `condensed_robot.py` usage? (Grep showed imports in `pipeline.py`, so it IS used. Needs tests).

## Next Steps
- Add TODOs for generating tests for `auth`, `mate`, `naming`, `validate`, `condensed_robot`.
