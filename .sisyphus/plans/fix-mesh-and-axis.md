# Plan: Fix Missing Meshes and Joint Axis Inversion

## TL;DR

> **Quick Summary**: Fix mesh matching failures for non-ASCII parts by implementing STEP-specific Unicode decoding (`\X2\HHHH\X0\`) and resolve joint axis/limit inversion by reverting to standard conventions.
>
> **Deliverables**:
> - Updated `src/onshape2xacro/mesh_exporters/step.py` with Unicode decoding.
> - Updated `src/onshape2xacro/serializers/__init__.py` with corrected axis/limit logic.
>
> **Estimated Effort**: Short (Tiki-Taka)
> **Parallel Execution**: NO - sequential logic fixes.
> **Critical Path**: Unicode Matching Fix → Axis/Limit Correction → Simulation Verification

---

## Context

### Original Request
The user reported two issues:
1. Two links are missing meshes because their names contain Japanese characters. The STEP file encodes these using `\X2\` sequences.
2. Revolute joints have inverted axis limits. A previous fix "inverted twice," causing continued misalignment.

### Interview Summary
**Key Discussions**:
- **Missing Meshes**: Confirmed that `assembly.step` uses ISO 10303-21 `\X2\HHHH\X0\` encoding for Extended Characters.
- **Axis/Limits**: Identified that current logic hardcodes `0 0 1` and uses negated limits in some places, leading to confusion.

### Metis Review
**Identified Gaps** (addressed):
- **Decoding Logic**: The decoding must handle multiple characters in a single `\X2\` block (e.g., `\X2\FF76FF6F\X0\`).

---

## Work Objectives

### Core Objective
Ensure 100% mesh export success for non-ASCII parts and align joint motion with Onshape assembly behavior.

### Concrete Deliverables
- `src/onshape2xacro/mesh_exporters/step.py`: Added `_decode_step_name` utility and integrated it into matching.
- `src/onshape2xacro/serializers/__init__.py`: Corrected axis and limit assignment.

### Definition of Done
- [ ] `export` command completes with zero missing meshes in `MISSING_MESHES.md`.
- [ ] `simulate-robot` shows joints rotating in the correct direction relative to Onshape.

---

## Verification Strategy

### Test Decision
- **Infrastructure exists**: YES (Simulation)
- **User wants tests**: NO (Manual Verification via simulation tool)
- **QA approach**: Manual verification using `onshape2xacro simulate-robot` and visual inspection.

### Automated Verification (Agent-Executable)
Each task includes executable verification procedures using the project's own tools.

---

## TODOs

- [ ] 1. Implement STEP Unicode Decoding in `step.py`

  **What to do**:
  - Add a helper function `_decode_step_name(s: str) -> str` that detects `\X2\HHHH...\X0\` and converts it to Unicode.
  - Update `_label_name` to wrap the return value of `ToExtString()` with this decoder.
  - Update `_parse_export_id` if necessary to ensure it handles decoded strings.

  **Recommended Agent Profile**:
  - **Category**: `quick`
  - **Skills**: [`simplify`]

  **References**:
  - `src/onshape2xacro/mesh_exporters/step.py:47` - `_label_name` implementation.
  - ISO 10303-21 Annex B - STEP Extended Character Encoding.

  **Acceptance Criteria**:
  - [ ] Python test script `test_decoding.py` (temporary) confirms `\X2\FF76\X0\` maps to `ｶ`.
  - [ ] `onshape2xacro export` on the target URL (if available) results in no missing meshes.

- [ ] 2. Correct Joint Axis and Limits in Serializer

  **What to do**:
  - Revert the hardcoded `xyz="0 0 1"` in `_joint_to_xacro`.
  - Ensure `axis` is calculated or extracted from the `mate` data provided by the toolkit.
  - Re-evaluate the limit negation logic in `_generate_default_configs` and `_add_robot_macro`. It should match Onshape's positive rotation convention.

  **Recommended Agent Profile**:
  - **Category**: `unspecified-high`
  - **Skills**: [`code-reviewer`]

  **References**:
  - `src/onshape2xacro/serializers/__init__.py:483` - `_joint_to_xacro` implementation.
  - `src/onshape2xacro/serializers/__init__.py:314` - `_add_robot_macro` limit logic.

  **Acceptance Criteria**:
  - [ ] `onshape2xacro simulate-robot` shows joint limits are consistent with Onshape.
  - [ ] Visual inspection of `joint_limits.yaml` shows expected min/max values.

- [ ] 3. End-to-End Verification

  **What to do**:
  - Run the full export pipeline.
  - Run `uv run simulate-robot [output_dir]`.

  **Acceptance Criteria**:
  - [ ] Simulation test report shows all joints pass movement checks.
  - [ ] `MISSING_MESHES.md` is empty or not created.

---

## Commit Strategy

| After Task | Message | Files | Verification |
|------------|---------|-------|--------------|
| 1 | `fix(mesh): decode STEP Unicode labels for part matching` | step.py | Export test |
| 2 | `fix(joint): align joint axis and limits with Onshape convention` | serializers/__init__.py | Simulation |

---

## Success Criteria

### Verification Commands
```bash
onshape2xacro export [URL] --output ./test_export
uv run simulate-robot ./test_export
```
