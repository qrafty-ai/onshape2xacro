# Deep Clean & Coverage Expansion Plan

## TL;DR

> **Quick Summary**: Clean up configuration/code while strictly preserving ignored files, and significantly expand unit test coverage.
>
> **Deliverables**:
> - **Clean Config**: `pyproject.toml` pruned.
> - **Refactored Test**: `test_step_inertia.py` mocked (no external file dependency).
> - **New Unit Tests**: Added coverage for `auth`, `mate_values`, `naming`, `validate`, `condensed_robot`.
>
> **Estimated Effort**: Medium
> **Parallel Execution**: Sequential

---

## Context

### Original Request
"clean code" + "mock data" + "strict gitignore" + "implement all possible unit tests"

### Analysis
- **Constraint**: `waist`, `openarm`, `logs` are ignored. **Touch nothing ignored.**
- **Existing Coverage**:
  - Covered: `config`, `inertia`, `mesh_exporters`, `pipeline`, `serializers`.
  - Missing: `auth.py`, `mate_values.py`, `naming.py`, `validate.py`, `condensed_robot.py`.
- **Strategy**:
  1.  Fix `test_step_inertia.py` to allow clean testing.
  2.  Prune `pyproject.toml`.
  3.  Implement unit tests for all uncovered modules using mocks where needed.

---

## Work Objectives

### Core Objective
Achieve high unit test coverage and a clean codebase without relying on local artifacts.

### Concrete Deliverables
- [ ] Refactored `test_step_inertia.py`
- [ ] Cleaned `pyproject.toml`
- [ ] New Test: `tests/test_auth.py`
- [ ] New Test: `tests/test_mate_values.py`
- [ ] New Test: `tests/test_naming.py`
- [ ] New Test: `tests/test_validate.py`
- [ ] New Test: `tests/test_kinematics/test_condensed_robot.py` (Expand/Create)

### Must NOT Have (Guardrails)
- Do NOT touch ignored files.
- Do NOT break existing tests.

---

## Verification Strategy

### Automated Verification
- `pytest` -> All tests (new and old) PASS.
- `grep "waist" tests/test_mesh_exporters/test_step_inertia.py` -> Empty/Clean.

---

## TODOs

### Phase 1: Cleanup & Refactor

- [x] 1. Refactor Test to Use Mock Data (Priority High)
  - **Task**: Mock `InertiaCalculator.compute_from_step` in `test_step_inertia.py`.
  - **Goal**: Remove dependency on `waist/assembly.step`.
  - **Acceptance**: Test passes without `waist` folder.

- [x] 2. Clean Config & Source (Priority Medium)
  - **Task**: Remove dead scripts from `pyproject.toml`.
  - **Task**: Remove unused imports in `src/`.
  - **Acceptance**: Valid `pyproject.toml`, clean code.

### Phase 2: Coverage Expansion

- [x] 3. Test Auth Module (Priority Medium)
  - **Target**: `src/onshape2xacro/auth.py`
  - **Strategy**: Mock `keyring` and `input` (or environment variables).
  - **Acceptance**: Verify `login`, `logout`, `status` logic.

- [x] 4. Test Mate Values (Priority Medium)
  - **Target**: `src/onshape2xacro/mate_values.py`
  - **Strategy**: Test parsing of JSON mate values and error handling.
  - **Acceptance**: JSON parsing covered.

- [x] 5. Test Naming (Priority Medium)
  - **Target**: `src/onshape2xacro/naming.py`
  - **Strategy**: Test string sanitization/slugification.
  - **Acceptance**: Edge cases (spaces, special chars) covered.

- [x] 6. Test Validation (Priority Medium)
  - **Target**: `src/onshape2xacro/validate.py`
  - **Strategy**: Test `main` or validation functions with mock Xacro content.
  - **Acceptance**: Valid/Invalid xacro detection covered.

- [x] 7. Test Condensed Robot (Priority Low)
  - **Target**: `src/onshape2xacro/condensed_robot.py`
  - **Strategy**: Test `CondensedRobot` class methods (add_link, add_joint).
  - **Acceptance**: Graph construction logic covered.


---

## Success Criteria

### Verification Commands
```bash
pytest
```

### Final Checklist
- [ ] All tests pass
- [ ] No ignored files touched
- [ ] Coverage significantly improved (5 new test files)
