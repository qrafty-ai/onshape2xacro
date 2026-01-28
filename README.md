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

## Project Structure

### Core Files

#### `src/onshape2xacro/cli/__init__.py`
- **Function**: CLI argument parsing and command routing
- **Defines**: `ExportConfig` and `VisualizeConfig` dataclasses
- **Commands**: `onshape2xacro export` and `onshape2xacro visualize`
- **Usage**: Entry point for all CLI commands

#### `src/onshape2xacro/pipeline.py`
- **Function**: Core export and visualization pipeline
- **Functions**:
  - `run_export()`: Fetches Onshape assembly, builds kinematic graph, creates robot model, and serializes to xacro
  - `run_visualize()`: Fetches Onshape assembly and visualizes kinematic graph
- **Dependencies**: Requires Onshape API credentials

#### `src/onshape2xacro/serializers/__init__.py`
- **Function**: Converts robot model to hierarchical xacro files
- **Class**: `XacroSerializer` - Main serializer that:
  - Groups components by subassembly
  - Generates xacro macros for each module
  - Exports STL meshes
  - Creates joint and link definitions
  - Handles mesh path resolution
- **Key Functions**:
  - `is_joint()`: Checks if mate name starts with `joint_`
  - `get_joint_name()`: Extracts joint name from mate name
  - `sanitize_name()`: Converts Onshape names to valid ROS identifiers
  - `is_module_boundary()`: Determines subassembly boundaries

#### `src/onshape2xacro/config/__init__.py`
- **Function**: Configuration override management
- **Class**: `ConfigOverride` - Loads and applies YAML configuration overrides
- **Supports**: Joint limits, inertials, and dynamics overrides

#### `src/onshape2xacro/validate.py`
- **Function**: Validates xacro file structure
- **Checks**: XML well-formedness, root element, link/joint counts, xacro includes/macros
- **Command**: `validate-xacro <xacro_file>`
- **Note**: Basic validation without ROS xacro (for full processing, install ROS)

#### `src/onshape2xacro/visualize_export.py`
- **Function**: Visualizes exported robot description structure
- **Features**:
  - Parses xacro files from export directory
  - Shows link/joint hierarchy in text format
  - Optional graph visualization (requires matplotlib)
  - Lists mesh files and their usage
- **Command**: `visualize-export <output_dir> [--graph <output.png>]`
- **Purpose**: Verify exported structure matches Onshape assembly

#### `src/onshape2xacro/xacro_to_urdf.py`
- **Function**: Converts xacro files to URDF files
- **Features**:
  - Tries ROS xacro command first (if available)
  - Falls back to basic xacro processing (expands includes and macros)
  - Handles nested includes and macro expansion
  - Replaces `${prefix}` variables
- **Command**: `xacro-to-urdf <xacro_file> [-o <output.urdf>]`
- **Use Case**: Convert xacro to URDF for tools that don't support xacro (e.g., urdf-viz)

#### `src/onshape2xacro/fix_xacro.py`
- **Function**: Fixes common issues in exported xacro files
- **Fixes**:
  - Joints with axis and limit but `type="fixed"` → changes to `revolute`
  - Missing axis elements for revolute/prismatic joints → adds default axis
- **Command**: `fix-xacro <xacro_file_or_dir> [-o <output>]`
- **Use Case**: Correct export errors or fix manually edited files

#### `src/onshape2xacro/simulate_robot.py`
- **Function**: Simulates robot using PyBullet and tests joint movements
- **Features**:
  - Automatically converts xacro to URDF
  - Loads robot into PyBullet
  - Tests all movable joints through their range of motion
  - Displays 3D visualization
  - Reports joint test results
- **Command**: `simulate-robot <output_dir>`
- **Dependencies**: Requires `pybullet` and `numpy` (install with `uv pip install pybullet numpy`)

## Usage Guide

### 1. Environment Setup

Set your Onshape API keys:
```bash
export ONSHAPE_ACCESS_KEY="your_access_key"
export ONSHAPE_SECRET_KEY="your_secret_key"
```

### 2. Export from Onshape

Export an Onshape assembly to xacro files:

```bash
onshape2xacro export "https://cad.onshape.com/documents/..." --output ./robot_description
```

**Arguments**:
- `url`: (Positional) The Onshape document URL pointing to an assembly
- `--output`: Output directory (default: `.`)
- `--name`: Robot name (default: from Onshape)
- `--config`: Path to YAML override file
- `--max-depth`: Max subassembly depth (default: 5)

**Output Structure**:
```
robot_description/
├── urdf/
│   ├── <robot_name>.urdf.xacro    # Entry point
│   ├── <robot_name>.xacro         # Main macro
│   └── <subassembly>/              # Subassembly modules
│       └── <subassembly>.xacro
├── meshes/                         # STL mesh files
│   └── *.stl
└── config/                         # Configuration files
    ├── joint_limits.yaml
    └── inertials.yaml
```

### 3. Validate Export

Check that the exported xacro file is well-formed:

```bash
uv run validate-xacro robot_description/urdf/<robot_name>.urdf.xacro
```

**Output**: Shows XML structure, link/joint counts, and xacro elements.

### 4. Visualize Export Structure

View the exported robot structure to verify it matches Onshape:

```bash
# Text-based visualization
uv run visualize-export robot_description

# With graph output (requires matplotlib: uv pip install matplotlib)
uv run visualize-export robot_description --graph robot_graph.png
```

**Shows**: Link hierarchy, joint connections, mesh file usage.

### 5. Visualize Onshape Assembly (Alternative)

Visualize the kinematic graph directly from Onshape (uses API):

```bash
onshape2xacro visualize "https://cad.onshape.com/documents/..." --output graph.png
```

**Note**: This fetches data from Onshape API, while `visualize-export` works on local files.

### 6. Convert Xacro to URDF

Convert xacro files to URDF for tools that don't support xacro:

```bash
uv run xacro-to-urdf robot_description/urdf/<robot_name>.urdf.xacro -o robot.urdf
```

**Use Cases**:
- Viewing with `urdf-viz` (Rust-based viewer)
- Using with tools that only support URDF
- Debugging xacro expansion issues

### 7. Fix Xacro Issues

Fix common problems in exported xacro files:

```bash
# Fix single file
uv run fix-xacro robot_description/urdf/<robot_name>.xacro

# Fix all xacro files in directory (recursive)
uv run fix-xacro robot_description/urdf/

# Fix and save to new file
uv run fix-xacro input.xacro -o fixed.xacro
```

**Fixes**:
- Joints with axis/limit but wrong type
- Missing axis elements

### 8. Simulate Robot Joints

Test that all joints rotate correctly using PyBullet:

```bash
# Install simulation dependencies
uv pip install pybullet numpy

# Run automated joint testing
uv run simulate-robot robot_description
```

**Features**:
- Automatically converts xacro to URDF
- Loads robot into PyBullet
- Tests each joint through its full range of motion
- Displays 3D visualization
- Reports success/failure for each joint

## Configuration Overrides

Create a YAML file to override robot parameters:

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

Use with export:
```bash
onshape2xacro export "https://..." --output ./robot --config overrides.yaml
```

## Joint Convention

Only mates in Onshape starting with `joint_` will be treated as robot joints. All other mates are treated as fixed connections.

- `joint_shoulder` → Joint named `shoulder`
- `hinge_door` → Fixed connection
- `fastened_1` → Fixed connection

## Complete Workflow Example

```bash
# 1. Export from Onshape
onshape2xacro export "https://cad.onshape.com/documents/..." --output ./my_robot

# 2. Validate export
uv run validate-xacro my_robot/urdf/my_robot.urdf.xacro

# 3. Visualize structure
uv run visualize-export my_robot --graph structure.png

# 4. Fix any issues
uv run fix-xacro my_robot/urdf/

# 5. Convert to URDF for viewing
uv run xacro-to-urdf my_robot/urdf/my_robot.urdf.xacro -o my_robot.urdf
urdf-viz my_robot.urdf  # If urdf-viz is installed

# 6. Test joints in simulation
uv run simulate-robot my_robot
```

## Dependencies

### Required
- `onshape-robotics-toolkit`: Onshape API integration
- `tyro`: CLI argument parsing
- `lxml`: XML processing
- `pyyaml`: YAML configuration
- `xacro>=2.1.1`: Xacro processing

### Optional
- `matplotlib`: For graph visualization (`uv pip install matplotlib`)
- `pybullet`, `numpy`: For simulation (`uv pip install pybullet numpy`)
- ROS xacro: For full xacro processing (`sudo apt install ros-humble-xacro`)

## Notes

- **Xacro Processing**: The tool includes a basic xacro processor, but for full support (property substitution, complex macros), install ROS xacro.
- **Mesh Paths**: Exported xacro files use relative paths (`../meshes/`). Tools like `urdf-viz` may need absolute paths - use `xacro-to-urdf` first.
- **API Usage**: `onshape2xacro export` and `onshape2xacro visualize` consume Onshape API requests. Other commands work on local files only.

## License

MIT
