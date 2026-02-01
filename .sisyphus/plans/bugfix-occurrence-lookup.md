# Fix Occurrence Lookup Bug & Add Fail Fast

## TL;DR

> **Quick Summary**: Fix a critical bug in `condensed_robot.py` where part transforms could not be found due to incorrect occurrence path lookup, and add a "fail fast" mechanism (RuntimeError + `--debug` flag) to prevent generating broken models.
>
> **Deliverables**:
> - `--debug` flag in CLI.
> - `RuntimeError` raised when parts are missing in `condensed_robot.py`.
> - Corrected occurrence lookup logic using full paths.
>
> **Estimated Effort**: Short
> **Parallel Execution**: Sequential
> **Critical Path**: CLI Config -> Pipeline -> CondensedRobot Logic

---

## Context

### Original Request
Fix the issue where `Could not find part for occurrence` warning appears, leading to broken visual meshes (parts at origin). Also, enable re-raising exceptions for better debugging.

### Interview Summary
**Key Discussions**:
- **Bug**: `parent_candidates` stored `(entity, entity.matedOccurrence)`, but `entity.matedOccurrence` is just a suffix. Lookup requires the full path from the link's occurrence list.
- **Fail Fast**: Warnings are ignored; we need `RuntimeError` to stop the pipeline when critical transforms are missing.
- **Debug Mode**: Add `--debug` to seeing full tracebacks.

### Metis Review
**Identified Gaps** (addressed):
- **Logic**: Use `next()` with `occ_match` to find the exact full path from `parent_occs`.
- **Propagation**: Ensure `debug` flag is passed down to the robot model builder.

---

## Work Objectives

### Core Objective
Ensure reliable part transform resolution and transparent error reporting.

### Concrete Deliverables
- [ ] `src/onshape2xacro/cli/__init__.py` updated
- [ ] `src/onshape2xacro/pipeline.py` updated
- [ ] `src/onshape2xacro/condensed_robot.py` updated

### Definition of Done
- [ ] `--debug` flag works (shows traceback on error).
- [ ] `Could not find part` error raises `RuntimeError` instead of logging warning.
- [ ] Occurrence lookup correctly resolves full path and finds the part transform.

### Must Have
- Fail fast on missing part transforms.
- Full path resolution for occurrences.

---

## Verification Strategy

### Test Decision
- **Infrastructure exists**: YES
- **User wants tests**: Manual verification of logic via code review and dry run check.
- **Framework**: `pytest` / `interactive_bash`

### Automated Verification Only (NO User Intervention)

**Code Check** (using Bash):
```bash
# Verify debug flag in CLI
grep "debug: bool" src/onshape2xacro/cli/__init__.py

# Verify RuntimeError in condensed_robot.py
grep "raise RuntimeError" src/onshape2xacro/condensed_robot.py
```

---

## Execution Strategy

### Parallel Execution Waves

```
Wave 1 (CLI & Pipeline):
├── Task 1: Add debug flag to CLI
└── Task 2: Pass debug flag in pipeline

Wave 2 (Core Logic):
└── Task 3: Fix lookup logic and implement fail-fast in condensed_robot.py
```

---

## TODOs

- [ ] 1. Add debug flag to CLI

  **What to do**:
  - Edit `src/onshape2xacro/cli/__init__.py`.
  - Add `debug: bool = False` to `ExportConfig`.
  - In `main()`, check `config.debug` and re-raise exceptions if True.

  **Recommended Agent Profile**:
  - **Category**: `quick`

  **Parallelization**:
  - **Can Run In Parallel**: YES
  - **Parallel Group**: Wave 1

  **Verification**:
  ```bash
  grep "debug: bool" src/onshape2xacro/cli/__init__.py
  ```

- [ ] 2. Pass debug flag in pipeline

  **What to do**:
  - Edit `src/onshape2xacro/pipeline.py`.
  - Update `run_export` to pass `debug=config.debug` to `CondensedRobot.from_graph`.

  **Recommended Agent Profile**:
  - **Category**: `quick`

  **Parallelization**:
  - **Can Run In Parallel**: YES
  - **Parallel Group**: Wave 1

  **Verification**:
  ```bash
  grep "debug=config.debug" src/onshape2xacro/pipeline.py
  ```

- [ ] 3. Fix lookup logic and fail-fast in condensed_robot.py

  **What to do**:
  - Edit `src/onshape2xacro/condensed_robot.py`.
  - Update `from_graph` signature to accept `debug: bool = False`.
  - Rewrite `parent_candidates` logic:
    - Iterate `parent_occs` and find the *exact* matching occurrence (full path).
    - Store `(entity, full_occ_path)` in `parent_candidates`.
  - Update usage: `parent_occ = candidate[1]`.
  - Change "Could not find part" warning to:
    ```python
    msg = f"Could not find part for occurrence {parent_occ}..."
    if debug:
        raise RuntimeError(msg)
    else:
        logger.error(msg) # Or just raise RuntimeError unconditionally if "fail fast" is the goal (User said "change... to a hard RuntimeError")
    ```
    *Correction*: User explicitly said: "Change the 'Could not find part' warning to a hard `RuntimeError`". So I will raise `RuntimeError` unconditionally (or maybe just logging error and raising is better, but I'll stick to `RuntimeError`).

  **Recommended Agent Profile**:
  - **Category**: `ultrabrain`
  - **Skills**: [`simplify`]

  **Parallelization**:
  - **Can Run In Parallel**: NO
  - **Parallel Group**: Wave 2
  - **Blocked By**: Task 2

  **Verification**:
  ```bash
  grep "raise RuntimeError" src/onshape2xacro/condensed_robot.py
  ```

---

## Commit Strategy

| After Task | Message | Files |
|------------|---------|-------|
| 1-3 | `fix(core): correct occurrence lookup and add fail-fast debug mode` | All modified files |
