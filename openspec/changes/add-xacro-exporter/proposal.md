# Change: Add Xacro Exporter for Onshape Assemblies

## Why

ROS2 robotics development requires URDF/xacro description files for simulation and control. Currently, exporting Onshape CAD assemblies to xacro is manual and error-prone. This tool provides seamless, automated export with proper hierarchical structure and parameter override capabilities.

## What Changes

- Add CLI tool using tyro for exporting Onshape assemblies to xacro format
- Create XacroSerializer that extends onshape-robotics-toolkit's serialization pattern
- Implement smart subassembly→xacro module mapping based on movable mates
- Support YAML config overrides for joint limits, inertials, and dynamics
- Generate prefix-aware xacro macros for multi-robot deployments
- Auto-export STL meshes organized by module

## Impact

- Affected specs: cli-export (new), xacro-serializer (new), config-override (new)
- Affected code: New modules - cli.py, serializers/xacro.py, config.py
- Dependencies: onshape-robotics-toolkit, tyro, lxml, pyyaml

## References

### Core Library
- onshape-robotics-toolkit: https://github.com/neurobionics/onshape-robotics-toolkit
  - Export example: `examples/export/main.py` - Pipeline: Client→CAD→KinematicGraph→Robot→Serializer
  - URDF serializer: `onshape_robotics_toolkit/formats/urdf.py` - Pattern for XacroSerializer
  - Robot model: NetworkX DiGraph with Links as nodes, Joints as edges
  - Mass/inertia: Fetched async via `client.get_mass_properties`
  - Mesh export: Via Asset class using STL export API

### Xacro Structure Reference
- openarm_description: https://github.com/enactic/openarm_description
  - Hierarchical xacro: `urdf/robot/` → `urdf/arm|body|ee/`
  - Pattern: macro.xacro (defines macro) + wrapper.xacro (instantiates)
  - Config: `config/<component>/<version>/{joint_limits,inertials}.yaml`
  - Meshes: `meshes/<component>/<version>/[visual|collision]/`
  - Prefix naming: `${prefix}${name}` for multi-robot support

### CLI Framework
- tyro: https://brentyi.github.io/tyro/
  - Type-hint driven CLI generation
  - Dataclass configs with validation
  - Clean subcommand via Union types

### Tooling
- uv: Package and dependency management (already in pyproject.toml)
- lxml: XML generation (used by onshape-robotics-toolkit)
