# Tasks: Add Xacro Exporter

> **Development Pattern**: Test-Driven Development (TDD)
> - Write tests first based on spec requirements
> - Implement to pass tests
> - Run and pass all tests before completing each stage

## 1. Project Setup
- [x] 1.1 Add dependencies to pyproject.toml (onshape-robotics-toolkit, tyro, lxml, pyyaml, pytest, OCP)
- [ ] 1.2 Create package structure: `src/onshape2xacro/{cli,serializers,config}/`
- [ ] 1.3 Create test structure: `tests/{test_cli,test_serializers,test_config}/`
- [x] 1.4 Configure uv and verify imports work
- [x] 1.5 **Safeguard**: Run `pytest` to ensure test infrastructure works

## 2. CLI Foundation
### 2.1 Tests First
- [ ] 2.1.1 Write tests for ExportConfig dataclass validation
- [ ] 2.1.2 Write tests for CLI argument parsing (url, output, name, config, max_depth)
- [ ] 2.1.3 Write tests for --help and --version output
- [ ] 2.1.4 Write tests for missing/invalid URL error handling

### 2.2 Implementation
- [ ] 2.2.1 Create ExportConfig dataclass with URL, output, name, config, max_depth fields
- [ ] 2.2.2 Implement CLI entry point using tyro.cli()
- [ ] 2.2.3 Add console script entry point in pyproject.toml
- [ ] 2.2.4 Add --help and --version support

### 2.3 Safeguard
- [ ] 2.3.1 **Run all CLI tests and ensure 100% pass**

## 3. Joint Detection & Module Boundary Logic
### 3.1 Tests First
- [ ] 3.1.1 Write tests for `joint_` prefix detection (`joint_shoulder` → joint, `hinge_door` → fixed)
- [ ] 3.1.2 Write tests for joint name extraction (strip `joint_` prefix)
- [ ] 3.1.3 Write tests for module boundary detection (subassembly with `joint_*` → module)
- [ ] 3.1.4 Write tests for nested module detection
- [ ] 3.1.5 Write tests for `CondensedRobot` graph reduction (merging links via non-joint edges)

### 3.2 Implementation
- [ ] 3.2.1 Implement joint prefix filter function
- [ ] 3.2.2 Implement joint name extraction function
- [ ] 3.2.3 Implement module boundary detection for subassemblies
- [x] 3.2.4 Implement `CondensedRobot` parallel model
- [x] 3.2.5 Implement graph reduction logic to merge links connected by fixed/non-joint edges

### 3.3 Safeguard
- [ ] 3.3.1 **Run all joint detection tests and ensure 100% pass**

## 4. Xacro Serializer
### 4.1 Tests First
- [ ] 4.1.1 Write tests for single-module xacro generation
- [ ] 4.1.2 Write tests for multi-module hierarchical xacro generation
- [ ] 4.1.3 Write tests for prefix argument in macros (`${prefix}${name}`)
- [ ] 4.1.4 Write tests for entry-point xacro with includes
- [ ] 4.1.5 Write tests for name sanitization (spaces, special chars → valid ROS names)
- [ ] 4.1.6 Write tests for `mesh_map` key sanitization using `sanitize_name`

### 4.2 Implementation
- [ ] 4.2.1 Create XacroSerializer class following RobotSerializer pattern
- [ ] 4.2.2 Implement module-level xacro file generation with macros
- [ ] 4.2.3 Generate entry-point xacro that includes all modules
- [ ] 4.2.4 Add prefix argument support to all macros
- [ ] 4.2.5 Implement name sanitization utility
- [ ] 4.2.6 Ensure `mesh_map` keys are sanitized in serializers

### 4.3 Safeguard
- [ ] 4.3.1 **Run all serializer tests and ensure 100% pass**

## 5. Placeholder Inertials
### 5.1 Tests First
- [ ] 5.1.1 Write tests for placeholder inertial generation (mass=1.0, identity inertia)
- [ ] 5.1.2 Write tests that no mass property API calls are made
- [ ] 5.1.3 Write tests for valid URDF output with placeholder values

### 5.2 Implementation
- [ ] 5.2.1 Implement placeholder inertial generator
- [ ] 5.2.2 Integrate placeholder inertials into link generation

### 5.3 Safeguard
- [ ] 5.3.1 **Run all inertial tests and ensure 100% pass**

## 6. Mesh Export
### 6.1 Tests First
- [ ] 6.1.1 Write tests for mesh directory structure (meshes/<module>/<link>.stl)
- [ ] 6.1.2 Write tests for mesh path references in xacro
- [ ] 6.1.3 Write tests for mesh naming conflict resolution
- [x] 6.1.4 Write tests for `StepMeshExporter` and STEP splitting (local vs global coordinates)
- [x] 6.1.5 Write tests for robust Export-ID regex parsing from STEP metadata
- [x] 6.1.6 Write tests for fail-fast behavior on STEP parse/split errors

### 6.2 Implementation
- [x] 6.2.1 Implement mesh directory structure
- [x] 6.2.2 Add proper mesh path references in xacro
- [x] 6.2.3 Handle mesh naming conflicts (sanitize, deduplicate)
- [x] 6.2.4 Implement `StepMeshExporter` using OCP
- [x] 6.2.5 Implement single STEP translation and local split logic
- [x] 6.2.6 Implement regex-based Export-ID extraction
- [x] 6.2.7 Implement fail-fast error handling for STEP parsing and splitting

### 6.3 Safeguard
- [ ] 6.3.1 **Run all mesh export tests and ensure 100% pass**

## 7. Config Override System
### 7.1 Tests First
- [ ] 7.1.1 Write tests for YAML schema validation (joint_limits, inertials, dynamics)
- [ ] 7.1.2 Write tests for ConfigOverride loader
- [ ] 7.1.3 Write tests for override merging with defaults
- [ ] 7.1.4 Write tests for default config file generation
- [ ] 7.1.5 Write tests for invalid YAML error handling

### 7.2 Implementation
- [ ] 7.2.1 Define YAML schema for joint_limits, inertials, dynamics
- [ ] 7.2.2 Implement ConfigOverride loader with validation
- [ ] 7.2.3 Apply overrides during serialization
- [ ] 7.2.4 Generate default config files with placeholder values

### 7.3 Safeguard
- [ ] 7.3.1 **Run all config tests and ensure 100% pass**

## 8. Export Pipeline Integration
### 8.1 Tests First
- [ ] 8.1.1 Write integration test for full export pipeline (mock Onshape API)
- [ ] 8.1.2 Write tests for URL parsing and validation
- [ ] 8.1.3 Write tests for output directory structure creation
- [ ] 8.1.4 Write tests for error handling (API failures, invalid assemblies)

### 8.2 Implementation
- [x] 8.2.1 Implement pipeline: Client → CAD → KinematicGraph → CondensedRobot
- [ ] 8.2.2 Add Onshape URL parsing and validation
- [ ] 8.2.3 Integrate XacroSerializer with pipeline
- [ ] 8.2.4 Add error handling for API failures with helpful messages

### 8.3 Safeguard
- [ ] 8.3.1 **Run all integration tests and ensure 100% pass**

## 9. Final Validation
- [ ] 9.1 **Run complete test suite**: `pytest -v`
- [ ] 9.2 Test with real Onshape document (manual validation)
- [ ] 9.3 Validate generated xacro with `xacro` CLI tool
- [ ] 9.4 Validate resulting URDF with `check_urdf`

## 10. Documentation
- [ ] 10.1 Write README with installation and usage examples
- [ ] 10.2 Document config YAML format with examples
- [ ] 10.3 Add troubleshooting section for common issues

## Dependencies
- Stage 2, 3, 4, 5, 6, 7 can be developed in parallel (tests first)
- Stage 8 depends on 2-7 being complete
- Stage 9 depends on 8
- Stage 10 can start after 2, complete after 9

## Parallelizable Work
- Stages 3-7 can have tests written in parallel
- Within each stage, test writing and implementation are sequential
- Multiple developers can work on different stages simultaneously
