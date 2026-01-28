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

### Requirement: Graph Condensation
The system SHALL merge all nodes connected by non-`joint_` edges into single condensed links.

Each condensed link SHALL represent a group of parts that are rigidly fixed together. The resulting robot graph MUST only contain joints derived from `joint_`-prefixed mates.

#### Scenario: Merging parts into condensed link
- **GIVEN** two parts connected by a mate named `fastener_bolt` (no `joint_` prefix)
- **WHEN** the robot graph is condensed
- **THEN** both parts are merged into a single link in the `CondensedRobot` model
- **AND** the visual/collision elements for both parts are combined in that link

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

### Requirement: High-Fidelity Mesh Export (STEP)
The system SHALL export visual and collision meshes using a single STEP translation of the entire assembly for maximum fidelity.

The system SHALL:
- Use Open CASCADE (OCP) to parse the exported STEP file.
- Export one STL per condensed link, merged from all parts belonging to that link.
- Use Onshape occurrence IDs extracted from STEP metadata to map geometry to parts, then group them into links based on the `CondensedRobot` model.
- Fail-fast: Immediately abort and report an error if STEP translation or parsing fails.
- Organization: Meshes SHALL be organized as `meshes/<module_name>/<link_name>.stl`.

#### Scenario: Successful STEP-based export
- **GIVEN** a robot assembly
- **WHEN** export is executed
- **THEN** a single STEP file is fetched from Onshape
- **AND** it is processed into a single STL file for each condensed link by merging its constituent parts
- **AND** the STL files are placed in the correct module subdirectories

#### Scenario: STEP parsing failure
- **GIVEN** a corrupted STEP file is received
- **WHEN** parsing is attempted
- **THEN** the system SHALL immediately exit with a descriptive error message

### Requirement: STEP Local Geometry Splitting
The system SHALL split the parsed STEP geometry into individual shapes based on their occurrence IDs. Each part shape SHALL be transformed from the assembly-global frame into its corresponding condensed-link frame BEFORE being grouped or merged into condensed-link meshes.

This ensures that the resulting STL for a condensed link correctly represents the merged geometry relative to the link's origin, preserving the relative transforms between parts within that link. The transformation SHALL NOT collapse all geometry to the link origin; instead, it MUST maintain the spatial arrangement of parts as defined in the Onshape assembly relative to the link's frame.

#### Scenario: Split and transform to condensed-link frame
- **GIVEN** a STEP file with multiple parts in assembly-global coordinates
- **WHEN** the system processes the geometry
- **THEN** it identifies shapes by occurrence IDs
- **AND** it applies the transformation that maps assembly-global coordinates to the condensed-link coordinate system for each shape
- **AND** it groups these link-frame shapes by their condensed link assignment before merging

### Requirement: STEP Local Coordinate Frames
The system SHALL export merged link meshes in the coordinate frame of their corresponding condensed link.

Local grouping of parts within a condensed link MUST preserve the relative transforms between parts as defined in the Onshape assembly.

#### Scenario: Merged mesh exported in link frame
- **GIVEN** a condensed link containing multiple rigidly-connected parts
- **WHEN** the STEP file is processed
- **THEN** all parts belonging to the link are merged into a single STL
- **AND** the merged mesh is transformed into the link's coordinate frame
- **AND** the STL file correctly represents the combined geometry relative to the link origin

### Requirement: Robust Export-ID Parsing
The system SHALL use regular expressions to extract Onshape occurrence IDs from STEP metadata.

This ensures stable mapping between the STEP geometry and the `CondensedRobot` parts, allowing the system to correctly group geometry into link-level meshes even when Onshape's metadata format varies slightly.

#### Scenario: Extract ID from metadata
- **GIVEN** STEP metadata containing `OCCURRENCE_ID: [some_id] (Part 1)`
- **WHEN** parsing occurrence IDs
- **THEN** the system extracts exactly `some_id`

### Requirement: Mesh Filename Sanitization
All mesh filenames and keys in the `mesh_map` SHALL be sanitized using the `sanitize_name` utility.

This ensures compatibility with ROS2 launch systems and prevents issues with special characters in filenames.

#### Scenario: Sanitize mesh filename
- **GIVEN** a condensed link named "Upper Arm"
- **WHEN** exporting its merged mesh
- **THEN** the resulting file is named `upper_arm.stl`

#### Scenario: Mesh map key sanitization
- **GIVEN** a condensed link named "End Effector @ Assembly"
- **WHEN** generating the `mesh_map` and xacro references
- **THEN** the map key is sanitized to `end_effector_assembly`
- **AND** the xacro filename reference matches this sanitized key exactly (`end_effector_assembly.stl`)

### Requirement: Relative Mesh Paths
The system SHALL use relative paths for mesh references within the generated xacro files.

This ensures that the generated output directory is self-contained and meshes are correctly resolved regardless of where the output directory is placed in the filesystem.

#### Scenario: Relative path in module xacro
- **GIVEN** a module xacro file located at `urdf/arm/arm.xacro`
- **AND** a mesh file located at `meshes/arm/link1.stl`
- **WHEN** the xacro is generated
- **THEN** the mesh reference in the xacro uses the relative path `../../meshes/arm/link1.stl`

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
