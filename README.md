# onshape2xacro

A seamless exporter tool from Onshape to ROS2 xacro description files.

## Features
- **Hierarchical Xacro**: Maps Onshape subassemblies to xacro modules based on joint boundaries.
- **Joint Detection**: Explicit control via `joint_` naming convention in Onshape.
- **YAML Overrides**: Easily override joint limits, inertials, and dynamics.
- **Multi-Robot Support**: Generated xacro macros accept a `prefix` argument for namespace isolation.
- **Automatic Mesh Export**: Exports STL meshes organized by module.
- **Fast Development**: Skip mass property API calls by default (uses placeholders).

## Installation

```bash
uv sync
```

## Usage

### Environment Setup
Set your Onshape API keys:
```bash
export ONSHAPE_ACCESS_KEY="your_access_key"
export ONSHAPE_SECRET_KEY="your_secret_key"
```

### Exporting
```bash
onshape2xacro export "https://cad.onshape.com/documents/..." --output ./robot_description
```

### Arguments
- `url`: (Positional) The Onshape document URL.
- `--output`: Output directory (default: `.`).
- `--name`: Robot name (default: from Onshape).
- `--config`: Path to YAML override file.
- `--max-depth`: Max subassembly depth (default: 5).

## Configuration Overrides

Create a YAML file to override parameters:

```yaml
joint_limits:
  shoulder:
    lower: -3.14
    upper: 3.14
    velocity: 1.5
    effort: 100
    
inertials:
  link1:
    mass: 5.0
    inertia:
      ixx: 0.1
      iyy: 0.1
      izz: 0.1
      
dynamics:
  elbow:
    friction: 0.1
    damping: 0.5
```

## Joint Convention
Only mates in Onshape starting with `joint_` will be treated as robot joints. All other mates are treated as fixed connections.
- `joint_shoulder` -> Joint named `shoulder`
- `hinge_door` -> Fixed connection

## License
MIT
