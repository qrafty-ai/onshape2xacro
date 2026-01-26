# Project Context

## Purpose
onshape2xacro is a CLI tool that exports Onshape CAD assemblies to ROS2 xacro description files. It provides seamless, automated export with hierarchical structure and parameter override capabilities.

## Tech Stack
- Python 3.12+
- uv for packaging and dependency management
- onshape-robotics-toolkit for Onshape API access
- tyro for CLI generation
- lxml for XML generation
- pyyaml for configuration

## Project Conventions

### Code Style
- Follow PEP 8
- Use type hints throughout
- Prefer dataclasses for configuration objects
- Maximum line length: 100 characters

### Architecture Patterns
- Extend onshape-robotics-toolkit patterns (RobotSerializer, etc.)
- Single responsibility: separate CLI, pipeline, serializer, config modules
- Reuse library utilities; don't reinvent the wheel

### Testing Strategy
**Test-Driven Development (TDD)** is the primary development pattern:

1. **Write tests first**: Before implementing any feature, write tests based on the spec requirements and scenarios
2. **Implement to pass tests**: Write the minimum code needed to make tests pass
3. **Run tests before completion**: Every task/stage MUST pass all related tests before being marked complete
4. **Test safeguard**: No implementation PR without passing tests

Test organization:
- `tests/` directory at project root
- pytest as test framework
- Unit tests for each module
- Integration tests for CLI and export pipeline

### Git Workflow
- Feature branches from main
- Conventional commits (feat:, fix:, test:, docs:)
- All tests must pass before merge

## Domain Context

### Onshape Concepts
- **Assembly**: Collection of parts and subassemblies with mates
- **Mate**: Constraint between parts (fastened, revolute, slider, etc.)
- **Subassembly**: Nested assembly within parent assembly

### ROS2/URDF Concepts
- **URDF**: Unified Robot Description Format (XML)
- **xacro**: XML macro language extending URDF
- **Link**: Rigid body with visual, collision, inertial properties
- **Joint**: Connection between links (fixed, revolute, prismatic, continuous)

### Joint Detection Convention
**Only mates with names starting with `joint_` are treated as robot joints.**
All other mates (including movable ones) are treated as fixed connections.
This allows explicit control over kinematic structure via naming convention.

## Important Constraints

### API Usage (Development Phase)
- **Skip mass property API calls** during development/testing to conserve API quota
- Use placeholder inertial values (mass=1.0, identity inertia matrix)
- Mass property fetching can be enabled later via flag

### Naming Convention
- Mates must be named `joint_<name>` to become robot joints
- Example: `joint_shoulder`, `joint_elbow`, `joint_gripper`
- Mates without `joint_` prefix are treated as fixed regardless of type

## External Dependencies

### Onshape API
- Requires `ONSHAPE_ACCESS_KEY` and `ONSHAPE_SECRET_KEY` environment variables
- API reference: https://onshape-public.github.io/docs/

### onshape-robotics-toolkit
- GitHub: https://github.com/neurobionics/onshape-robotics-toolkit
- Provides: Client, CAD, KinematicGraph, Robot, URDFSerializer
- We extend this library's patterns
