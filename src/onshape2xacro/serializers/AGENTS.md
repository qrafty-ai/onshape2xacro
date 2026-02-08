# serializers MODULE

## OVERVIEW
Converts Robot objects to Xacro files with proper macro structure, material definitions, and mesh references.

## STRUCTURE
- `__init__.py`: XacroSerializer class, helper functions

## WHERE TO LOOK

| Task | Symbol | Notes |
|------|--------|-------|
| Main serializer | `XacroSerializer` | Extends RobotSerializer from onshape-robotics-toolkit |
| Joint detection | `is_joint()` | Checks for `joint_` prefix in mate names |
| Joint naming | `get_joint_name()` | Strips `joint_` prefix for clean URDF names |
| Module boundaries | `is_module_boundary()` | Determines if subassembly should be separate xacro |

## CONVENTIONS

### Xacro Structure
- **Macro Pattern**: Generates `<xacro:macro name="robot_name">` wrapper
- **Material Files**: Separate `materials.xacro` for color definitions
- **Mesh Paths**: Uses `$(find package_name)/meshes/` pattern for ROS integration

### Joint Handling
- **Name Sanitization**: Strips `joint_` prefix, sanitizes special chars
- **Only Revolute**: Currently only revolute joints supported

## UNIQUE PATTERNS

### Modular Assembly
- Detects subassembly boundaries
- Can generate separate xacro files for complex robots
- Links via xacro includes
