## ADDED Requirements

### Requirement: Joint Detection via Naming Convention
The system SHALL only treat Onshape mates as robot joints if their name starts with the prefix `joint_`.

All other mates, regardless of their mechanical type (revolute, slider, etc.), SHALL be treated as fixed connections.

The joint name in the output xacro SHALL be derived by removing the `joint_` prefix from the mate name.

#### Scenario: Mate with joint_ prefix becomes joint
- **GIVEN** an Onshape assembly with a revolute mate named `joint_shoulder`
- **WHEN** export is executed
- **THEN** a revolute joint named `shoulder` is created in the xacro output

#### Scenario: Movable mate without prefix becomes fixed
- **GIVEN** an Onshape assembly with a revolute mate named `hinge_panel`
- **WHEN** export is executed
- **THEN** the mate is treated as a fixed connection
- **AND** no movable joint is created for this mate

#### Scenario: Multiple joint-prefixed mates
- **GIVEN** an assembly with mates `joint_shoulder`, `joint_elbow`, `joint_wrist`
- **WHEN** export is executed
- **THEN** three joints are created: `shoulder`, `elbow`, `wrist`

### Requirement: Hierarchical Xacro Generation
The system SHALL generate hierarchical xacro files that mirror the robot's kinematic structure.

The system SHALL create xacro modules based on the following rule:
- A subassembly becomes a separate xacro module if and only if it contains mates with the `joint_` prefix
- Subassemblies without any `joint_*` mates are merged into the parent as a single rigid body

#### Scenario: Subassembly with joint_ mate
- **GIVEN** an Onshape assembly with a subassembly containing a mate named `joint_elbow`
- **WHEN** export is executed
- **THEN** the subassembly is exported as a separate xacro file with its own macro
- **AND** the parent xacro includes this module via xacro:include

#### Scenario: Subassembly with only non-joint mates
- **GIVEN** an Onshape assembly with a subassembly containing only mates like `fastener_bolt` or `hinge_panel`
- **WHEN** export is executed
- **THEN** the subassembly parts are merged into parent module
- **AND** no separate xacro file is created for the subassembly

#### Scenario: Nested subassemblies with joint_ mates
- **GIVEN** an assembly with subassembly A containing subassembly B, both with `joint_*` mates
- **WHEN** export is executed
- **THEN** both A and B become separate xacro modules
- **AND** module A includes module B

### Requirement: Entry Point Xacro
The system SHALL generate a root xacro file that serves as the robot entry point.

The entry point file SHALL:
- Include all top-level module xacro files
- Define a macro that instantiates the complete robot
- Accept a `prefix` argument for namespace isolation

#### Scenario: Entry point generation
- **GIVEN** a robot with arm and gripper modules
- **WHEN** export is executed with name "my_robot"
- **THEN** `urdf/my_robot.xacro` is generated
- **AND** it includes `arm/arm.xacro` and `gripper/gripper.xacro`
- **AND** it defines macro `my_robot` with `prefix` argument

### Requirement: Prefix-Aware Naming
The system SHALL generate xacro macros that support prefix-based naming for multi-robot scenarios.

All link and joint names SHALL use the pattern `${prefix}${name}` where:
- `prefix` is a xacro argument defaulting to empty string
- `name` is the sanitized Onshape part/mate name

#### Scenario: Single robot deployment
- **GIVEN** generated xacro files
- **WHEN** instantiated without prefix argument
- **THEN** all links and joints have their base names (e.g., `base_link`, `shoulder_joint`)

#### Scenario: Multi-robot deployment
- **GIVEN** generated xacro files
- **WHEN** instantiated with `prefix:=robot1_`
- **THEN** all links and joints are prefixed (e.g., `robot1_base_link`, `robot1_shoulder_joint`)

### Requirement: STL Mesh Export
The system SHALL export visual and collision meshes in STL format.

Meshes SHALL be organized as:
```
meshes/<module_name>/<link_name>.stl
```

Mesh references in xacro SHALL use relative paths from the xacro file location.

#### Scenario: Mesh export
- **GIVEN** an assembly with links containing geometry
- **WHEN** export is executed to `./output`
- **THEN** STL files are created under `./output/meshes/<module>/`
- **AND** xacro visual/collision elements reference correct relative paths

#### Scenario: Mesh naming conflicts
- **GIVEN** two parts with the same name in different subassemblies
- **WHEN** export is executed
- **THEN** meshes are named uniquely (e.g., `part.stl`, `part_1.stl`)

### Requirement: Output Directory Structure
The system SHALL generate files in a consistent directory structure:

```
<output>/
├── urdf/
│   ├── <robot_name>.xacro     # Entry point
│   └── <module>/
│       └── <module>.xacro     # Module macro
├── meshes/
│   └── <module>/
│       └── <link>.stl
└── config/
    ├── joint_limits.yaml
    ├── inertials.yaml
    └── dynamics.yaml
```

#### Scenario: Complete output structure
- **GIVEN** a robot assembly with modules
- **WHEN** export is executed
- **THEN** the complete directory structure is created
- **AND** all directories are created even if empty (except config which always has defaults)

### Requirement: Name Sanitization
The system SHALL sanitize Onshape names to valid ROS identifiers.

Sanitization rules:
- Convert to lowercase
- Replace spaces with underscores
- Remove special characters except underscores
- Ensure name starts with letter or underscore
- Limit length to 255 characters

#### Scenario: Name with spaces and special characters
- **GIVEN** an Onshape part named "My Part (v2.1)"
- **WHEN** exported
- **THEN** the link name becomes `my_part_v21`
