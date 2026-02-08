# tests DIRECTORY

## OVERVIEW
Comprehensive test suite mirroring source structure with heavy OCP mocking, aggressive cleanup, and coverage-focused edge case testing.

## STRUCTURE
```
tests/
├── conftest.py              # Autouse fixtures, cleanup via atexit
├── fixtures/                # Static test assets (STEP, BOM CSV)
├── test_cli/                # CLI command testing
├── test_config/             # Config validation
├── test_inertia/            # BOM parsing, inertia calculation
├── test_integration/        # End-to-end workflow tests
├── test_kinematics/         # Robot condensing, joint handling
├── test_mesh_exporters/     # STEP processing, OCP mocking
└── test_serializers/        # Xacro generation
```

## WHERE TO LOOK

| Task | Location | Notes |
|------|----------|-------|
| Fixture setup | `conftest.py` | `autouse` cleanup, `atexit` hooks |
| OCP mocking patterns | `test_mesh_exporters/` | Extensive C++ binding mocks |
| BOM testing | `test_inertia/test_calculator_with_bom.py` | Fuzzy matching edge cases |
| UI mocking | `test_ui.py` | Rich console/progress mocking |
| Coverage tests | `*_coverage.py` files | Branch coverage for error paths |

## CONVENTIONS

### Test Organization
- **Mirror Structure**: Tests mirror `src/` structure exactly
- **Static Fixtures**: `tests/fixtures/` contains sample STEP files, BOM CSVs
- **Coverage Files**: `*_coverage.py` suffix for dedicated branch coverage tests

### Mocking Strategy
- **OCP Classes**: ALWAYS `unittest.mock.patch`, NEVER instantiate directly
- **Dynamic Side Effects**: Use `side_effect` functions for recursive CAD tree traversal
- **UI Components**: Mock `rich.console.Console` and progress bars for CI compatibility
- **Dependency Injection**: Insert `MagicMock` into `sys.modules` for optional deps (pymeshlab)

### Cleanup
- **Autouse Fixtures**: `conftest.py` registers cleanup functions
- **Atexit Hooks**: Ensures temp files removed even on test failures
- **Defensive Approach**: Every test creating files must clean up

## ANTI-PATTERNS

- **NO instantiating OCP objects**: Tests will fail with C++ errors
- **NO skipping cleanup**: Temp files pollute test environment
- **NO relying on test order**: Each test must be independent

## UNIQUE PATTERNS

### Recursive Mock Breaking
- `test_step_coverage.py` line 132: Manually break recursion in mocks to avoid infinite loops

### Monkeypatching CLI
- Heavy use of `monkeypatch.setattr('sys.argv', [...])` to simulate CLI invocation
- Environment variable mocking for credentials

### Subdirectory AGENTS.md
- `test_mesh_exporters/AGENTS.md` - OCP mocking specifics
- `test_inertia/AGENTS.md` - Inertia test patterns
