# onshape2xacro PROJECT KNOWLEDGE BASE

**Generated:** Sat Feb 07 2026 21:46 EST
**Commit:** 0492da3
**Branch:** main

## OVERVIEW

Converts Onshape CAD assemblies to Xacro/URDF robot descriptions using **local STEP processing** to minimize API calls. Python 3.12, built with `uv`, heavy CAD stack (OCP, trimesh, cadquery).

## STRUCTURE

```
onshape2xacro/
├── src/onshape2xacro/          # Main package (src layout)
│   ├── cli/                    # Entry point (tyro CLI)
│   ├── mesh_exporters/         # STEP → mesh (OBJ/DAE/STL) with color extraction
│   ├── inertia/                # BOM parsing, mass/inertia calculation, reporting
│   ├── serializers/            # Robot → Xacro generation
│   └── *.py                    # Core: pipeline, condensed_robot, optimized_cad
├── tests/                      # Mirror source structure, heavy OCP mocking
├── docs/plans/                 # Design docs
├── examples/                   # Example scripts (load_pickle.py)
├── manual_test/                # Manual test data (outside standard tests/)
├── openarm_asm_v2.5/           # CAD export example
└── teaarm_xacro/              # Generated Xacro example (168 meshes)
```

## WHERE TO LOOK

| Task | Location | Notes |
|------|----------|-------|
| CLI entry | `src/onshape2xacro/cli/__init__.py` | Uses `tyro` for subcommands |
| Main workflow | `src/onshape2xacro/pipeline.py` | `run_export`, `run_fetch_cad`, `run_auth` |
| STEP parsing | `src/onshape2xacro/mesh_exporters/step.py` | 1209 lines, OCP C++ bindings, color extraction |
| Inertia calculation | `src/onshape2xacro/inertia/calculator.py` | 660 lines, BOM integration |
| Link merging | `src/onshape2xacro/condensed_robot.py` | Merges fixed parts into single links |
| Xacro output | `src/onshape2xacro/serializers/` | Converts Robot to Xacro files |
| Config | `src/onshape2xacro/config/export_config.py` | Export options (CoACD, collision, visual) |
| Auth | `src/onshape2xacro/auth.py` | Keyring storage for Onshape credentials |

## CODE MAP

| Symbol | Type | Location | Role |
|--------|------|----------|------|
| `StepMeshExporter` | Class | `mesh_exporters/step.py` | STEP → mesh conversion, 30+ methods |
| `InertiaCalculator` | Class | `inertia/calculator.py` | Mass properties, BOM matching |
| `CondensedRobot` | Class | `condensed_robot.py` | Merges fixed occurrences |
| `XacroSerializer` | Class | `serializers/__init__.py` | Robot → Xacro files |
| `OptimizedCAD` | Class | `optimized_cad.py` | Caches Onshape API calls |
| `run_export` | Function | `pipeline.py` | Main export orchestration (123 lines) |
| `RichExportUI` | Class | `ui.py` | Progress bars, stats display |

## CONVENTIONS

### Naming
- **Joint Mates**: Onshape mates MUST be prefixed with `joint_` to be recognized as URDF joints
- **Sanitization**: Link/joint names sanitized via `naming.sanitize_name()` (replace `/`, `-`, `.` with `_`)

### Architecture
- **Modular CLI**: `auth` → `fetch-cad` → `export` (allows manual `mate_values.json` editing between steps)
- **Local Processing**: Fetch assembly as single STEP file, process locally (vs. per-part API calls)
- **Auto-Link Merging**: Fixed occurrences merged into single URDF links
- **Lazy CAD Loading**: `OptimizedCAD` caches API responses to reduce calls

### Dependencies
- **Python**: Strict `>=3.12, <3.13`
- **Package Manager**: `uv` (not pip) for dev workflow
- **External Toolkit**: `onshape-robotics-toolkit` (custom fork via git)

### Testing
- **Mocking Strategy**: OCP classes are C++ bindings → ALWAYS use `unittest.mock.patch`, never instantiate directly
- **Cleanup**: `conftest.py` uses `autouse` fixtures + `atexit` to remove temp files
- **Structure**: Tests mirror `src/` structure (`test_mesh_exporters/`, `test_inertia/`, etc.)

## ANTI-PATTERNS (THIS PROJECT)

### Forbidden
- **NEVER use link-level mass**: Inertia calculation allows link-level material for density, but FORBIDS link-level mass (line 501 of `inertia/calculator.py`)
- **NO STEP export without Client**: Raises `RuntimeError` if client not initialized (line 494 of `mesh_exporters/step.py`)
- **NO skipping pose zeroing**: Robot MUST be in zero pose or have correct `mate_values.json` before export (README limitation)
- **NO instantiating OCP in tests**: ALWAYS mock OCP classes (`XCAFDoc_DocumentTool`, `Quantity_Color`, etc.)

### Gotchas
- **Recursive Mate Retrieval**: Onshape API `getMateValues` does NOT support recursive retrieval → manually specify mate values for sub-assembly joints
- **STL ignores color**: For color preservation, use `obj`, `dae`, or `glb` formats (not `stl`)
- **Short name collisions**: BOM matching skips very short strings ("a", "b", "no") to avoid false positives (lines 455-460 of `inertia/calculator.py`)
- **Only revolute mates supported**: Other mate types (slider, etc.) not yet implemented

## UNIQUE STYLES

- **Conventional Commits**: Lowercase preferred (`feat:`, `fix:`) but some variation exists
- **Dependency Groups**: `[dependency-groups]` in `pyproject.toml` for `dev`, `visualize`, `simulate` (not standard `[project.optional-dependencies]`)
- **Visual Mesh Compression**: Uses `pymeshlab` for quadric edge collapse decimation
- **CoACD Integration**: Convex decomposition via `coacd` library for collision meshes
- **Debug Reporting**: Exports `inertia_debug.md` comparing calculated vs. expected mass properties

## COMMANDS

```bash
# Setup
uv sync                                  # Install dependencies
uv run pytest                            # Run tests
uv run pytest --cov --cov-branch         # Coverage

# Usage
onshape2xacro auth login                 # Store credentials
onshape2xacro fetch-cad <url> <dir>      # Download assembly as STEP
onshape2xacro export <dir> --output <out> [--bom <csv>]  # Generate Xacro
onshape2xacro visualize <output_dir>     # Visualize exported robot

# Dev utilities
python src/onshape2xacro/validate.py     # Validate Xacro files
python src/onshape2xacro/visualize_export.py  # Debug visualization
```

## NOTES

### BOM Integration
- Export BOM from Onshape assembly page with `Name`, `Material`, `Mass` columns
- BOM matching is fuzzy (handles case variations, substring matching)
- Inertia assumes uniform mass distribution (true for metals, NOT for 3D-printed parts)
- Check `inertia_debug.md` after export to verify calculations

### Data Artifacts in Root
- `openarm_asm_v2.5/` and `teaarm_xacro/`: Example CAD exports (should be in `examples/` per standard Python layout)
- `manual_test/`: Manual test assets (non-standard location)
- `ORT.yaml`, `ORT.log`: Onshape Robotics Toolkit artifacts (consider moving to `logs/` or `.gitignore`)

### Subdirectory AGENTS.md
- `src/onshape2xacro/mesh_exporters/AGENTS.md` - STEP processing details
- `src/onshape2xacro/inertia/AGENTS.md` - Inertia calculation internals
- `src/onshape2xacro/serializers/AGENTS.md` - Xacro generation patterns
- `tests/AGENTS.md` - Testing conventions and mocking strategies
- `tests/test_mesh_exporters/AGENTS.md` - OCP mocking specifics
- `tests/test_inertia/AGENTS.md` - Inertia test patterns
