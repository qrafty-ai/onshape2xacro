# Export CLI Configuration System

> [!IMPORTANT]
> **DEPRECATED**: This plan has been marked as finished by user request. All tasks are considered complete.

## TL;DR

> **Quick Summary**: Create a unified configuration system for the export command that loads from `configuration.yaml` with CLI overrides via tyro, deprecates remote URL exports in favor of fetch-cad workflow, and enables link name overrides for clean URDF output.
>
> **Deliverables**:
> - New `ExportConfiguration` dataclass with export options, mate values, and link name mappings
> - Updated `fetch-cad` command that generates `configuration.yaml` (deprecating `mate_values.json`)
> - Updated `export` command that auto-loads config from local directory with CLI overrides
> - Link name override system integrated into the serialization pipeline
>
> **Estimated Effort**: Medium (3-5 days)
> **Parallel Execution**: YES - 3 waves
> **Critical Path**: Task 1 (schema) -> Task 2 (fetch-cad) -> Task 4 (export) -> Task 5 (link names)

---

## Context

### Original Request
Create a CLI configuration system for the export command that:
1. Loads from a config file with CLI overrides (using tyro)
2. Generates default configuration.yaml at fetch-cad stage (merging mate_values.json)
3. Enables link name overrides for clean URDF output

### Interview Summary
**Key Discussions**:
- Config architecture: NEW export config separate from existing runtime config (ConfigOverride)
- File format: YAML (human-friendly, supports comments)
- Link name mapping: auto_name -> custom_name format
- Joint name overrides: NOT included (future enhancement)
- Backward compatibility: STRICT MODE - only load configuration.yaml
- Remote exports: DEPRECATED - require fetch-cad first

**Research Findings**:
- tyro pattern: `tyro.cli(Config, default=loaded_instance)` for CLI overrides
- Current ExportConfig has: url, output, name, config, bom, max_depth, visual_mesh_format, debug
- Link names currently based on heaviest part name in `condensed_robot.py:217`
- `sanitize_name()` removes special chars including `/` - assembly paths are keys only

### Metis Review
**Identified Gaps** (addressed):
- Link name key stability: Using generated name (user accepts re-editing if masses change)
- Remote export behavior: Deprecated in favor of fetch-cad workflow
- Flag collision: Keep `--config` for runtime, auto-load `configuration.yaml` from dir
- Duplicate link names: Fail-fast with clear error
- Re-running fetch-cad: Don't overwrite existing configuration.yaml

---

## Work Objectives

### Core Objective
Replace the fragmented configuration approach (CLI args + mate_values.json + implicit defaults) with a unified `configuration.yaml` that is generated at fetch-cad time and auto-loaded at export time, with tyro enabling CLI overrides for export options.

### Concrete Deliverables
- `src/onshape2xacro/config/export_config.py` - New ExportConfiguration dataclass
- Updated `src/onshape2xacro/pipeline.py:run_fetch_cad()` - Generates configuration.yaml
- Updated `src/onshape2xacro/pipeline.py:run_export()` - Auto-loads config, applies link names
- Updated `src/onshape2xacro/cli/__init__.py` - New CLI parsing with config file support
- Updated `src/onshape2xacro/condensed_robot.py` - Apply link name overrides
- Test files: `tests/test_config/test_export_config.py`, `tests/test_cli/test_export_config_cli.py`, `tests/test_link_name_overrides.py`

### Definition of Done
- [x] `uv run pytest -q` exits with code 0
- [x] `fetch-cad` generates `configuration.yaml` with mate_values and link_names sections
- [x] `export <local-dir>` auto-loads `configuration.yaml` and applies link name mappings
- [x] CLI args like `--export.name foo` override config file values
- [x] Export fails with clear error when `configuration.yaml` missing from local dir
- [x] Duplicate link name mappings fail with clear error

### Must Have
- ExportConfiguration dataclass with: export options, mate_values, link_names
- Auto-generation of configuration.yaml at fetch-cad
- Auto-loading of configuration.yaml at export
- CLI override support via tyro for export options (name, output, visual_mesh_format)
- Link name override application during robot construction
- Fail-fast behavior for missing config and duplicate link names

### Must NOT Have (Guardrails)
- NO joint name overrides (explicitly out of scope)
- NO migration/loading of legacy mate_values.json (STRICT MODE)
- NO remote URL export support (deprecated)
- NO changes to existing ConfigOverride (runtime config for joint_limits/inertials/dynamics)
- NO CLI overrides for mate_values or link_names (file-only, too complex for CLI)
- NO automatic overwrite of existing configuration.yaml on re-running fetch-cad

---

## Verification Strategy (MANDATORY)

### Test Decision
- **Infrastructure exists**: YES (pytest)
- **User wants tests**: TDD
- **Framework**: pytest (existing)

### TDD Workflow

Each TODO follows RED-GREEN-REFACTOR:

**Task Structure:**
1. **RED**: Write failing test first
   - Test command: `uv run pytest -q <test_file>`
   - Expected: FAIL (test exists, implementation doesn't)
2. **GREEN**: Implement minimum code to pass
   - Command: `uv run pytest -q <test_file>`
   - Expected: PASS
3. **REFACTOR**: Clean up while keeping green
   - Command: `uv run pytest -q <test_file>`
   - Expected: PASS (still)

---

## Execution Strategy

### Parallel Execution Waves

```
Wave 1 (Start Immediately):
├── Task 1: Define ExportConfiguration schema
└── Task 3: Update fetch-cad tests (can mock schema)

Wave 2 (After Wave 1):
├── Task 2: Implement fetch-cad config generation
└── Task 4: Update export pipeline + CLI

Wave 3 (After Wave 2):
└── Task 5: Implement link name overrides
└── Task 6: Final integration testing
```

### Dependency Matrix

| Task | Depends On | Blocks | Can Parallelize With |
|------|------------|--------|---------------------|
| 1 | None | 2, 4, 5 | 3 |
| 2 | 1 | 4, 5, 6 | None |
| 3 | None | 2 | 1 |
| 4 | 1, 2 | 5, 6 | None |
| 5 | 4 | 6 | None |
| 6 | 2, 4, 5 | None | None (final) |

---

## TODOs

- [x] 1. Define ExportConfiguration Schema

- [x] 2. Update fetch-cad to Generate configuration.yaml

- [x] 3. Write Tests for Export Config CLI (TDD Red Phase)

- [x] 4. Implement Export Pipeline with Config Loading

- [x] 5. Implement Link Name Overrides

- [x] 6. Final Integration Testing
