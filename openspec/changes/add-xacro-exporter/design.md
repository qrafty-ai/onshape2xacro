# Design: Xacro Exporter Architecture

## Context

We're building a CLI tool that bridges Onshape CAD assemblies to ROS2 xacro files. The tool must:
1. Reuse onshape-robotics-toolkit's existing pipeline (Client→CAD→KinematicGraph→Robot)
2. Generate hierarchical xacro files that mirror kinematic structure
3. Support iterative development with YAML overrides
4. Enable multi-robot deployments via xacro prefix arguments

### Stakeholders
- Robot developers designing in Onshape
- ROS2 simulation engineers needing accurate URDFs
- Teams with multiple robot variants sharing components

## Goals / Non-Goals

### Goals
- Seamless export: One command from Onshape URL to xacro files
- Smart hierarchy: Subassemblies with `joint_*` mates → xacro modules
- Override support: YAML configs for tuning without re-export
- Multi-robot ready: Prefix arguments for namespace isolation
- Maintainable output: Clean, readable xacro that humans can modify

### Non-Goals
- Full ROS2 package generation (CMakeLists.txt, package.xml)
- ros2_control or Gazebo plugin configuration (future enhancement)
- Bidirectional sync (changes back to Onshape)
- Support for non-STL mesh formats initially

## Decisions

### Decision 1: Extend URDFSerializer Pattern
**Choice**: Create XacroSerializer following onshape-robotics-toolkit's RobotSerializer pattern

**Why**:
- Consistent with library architecture
- Reuses Robot model traversal logic
- Link/Joint to_xml() methods already generate valid URDF elements

**Alternative considered**: Generate xacro from scratch
**Rejected**: Would duplicate graph traversal and XML generation logic

### Decision 2: Joint Detection via Naming Convention
**Choice**: Only mates with names starting with `joint_` are treated as robot joints. All other mates become fixed connections.

**Why**:
- Explicit control over kinematic structure
- Onshape assemblies often have many movable mates for CAD purposes that aren't robot joints
- Clear naming convention is self-documenting
- Avoids accidental joint creation from fastener mates, etc.

**Implementation**: Filter `KinematicGraph.edges` to only include mates where `mate.name.startswith("joint_")`

**Naming examples**:
- `joint_shoulder` → revolute joint named `shoulder`
- `joint_gripper_left` → joint named `gripper_left`
- `hinge_door` → fixed connection (no `joint_` prefix)

### Decision 2b: Module Boundary Heuristic
**Choice**: A subassembly becomes a xacro module IFF it contains mates prefixed with `joint_`

**Why**:
- Aligns with kinematic chain boundaries
- Only explicit joints define module structure
- Reduces file explosion for complex assemblies

**Implementation**: Check if subassembly subtree contains any edges matching `joint_*` naming pattern

### Decision 3: YAML Override Architecture
**Choice**: Separate YAML files per parameter type (joint_limits.yaml, inertials.yaml, dynamics.yaml)

**Why**:
- Matches openarm_description pattern
- Allows selective override of specific parameters
- Clean separation of concerns

**Structure**:
```yaml
# joint_limits.yaml
shoulder_joint:
  lower: -3.14
  upper: 3.14
  velocity: 2.0
  effort: 100.0
```

### Decision 4: Prefix via Xacro Arguments
**Choice**: Generate macros accepting `prefix` argument, all link/joint names use `${prefix}${name}`

**Why**:
- Standard ROS pattern (e.g., ur_description, openarm_description)
- No modification needed for single robot (prefix="")
- Enables multi-robot without regeneration

### Decision 5: CLI with tyro
**Choice**: Single `export` command using tyro with dataclass config

**Why**:
- Type-safe argument parsing
- Self-documenting via type hints
- Dataclass reusable for programmatic API

**Structure**:
```python
@dataclass
class ExportConfig:
    url: str                    # Onshape document URL
    output: Path = Path(".")    # Output directory
    name: str | None = None     # Robot name (default: from Onshape)
    config: Path | None = None  # Override config path
    max_depth: int = 5          # Max subassembly depth
```

## Architecture

```
┌─────────────────────────────────────────────────────────────────┐
│                         CLI (tyro)                               │
│  onshape2xacro export <url> --output ./out --config overrides   │
└─────────────────────────────────────────────────────────────────┘
                              │
                              ▼
┌─────────────────────────────────────────────────────────────────┐
│                    ExportPipeline                                │
│  1. Client(env) → connect to Onshape API                        │
│  2. CAD.from_url(url) → fetch assembly structure                │
│  3. KinematicGraph.from_cad(cad) → build kinematic tree         │
│  4. Robot.from_graph(graph) → create robot model                │
│  5. ConfigOverride.load(yaml) → load user overrides             │
│  6. XacroSerializer.save(robot, config, output) → generate      │
└─────────────────────────────────────────────────────────────────┘
                              │
                              ▼
┌─────────────────────────────────────────────────────────────────┐
│                   XacroSerializer                                │
│  - Analyze Robot graph for module boundaries                    │
│  - Generate xacro files per module                              │
│  - Apply config overrides to parameters                         │
│  - Export meshes to organized directory                         │
└─────────────────────────────────────────────────────────────────┘
                              │
                              ▼
┌─────────────────────────────────────────────────────────────────┐
│                     Output Structure                             │
│  output/                                                         │
   |-- robot.urdf.xacro # entry point
│  ├── modules/                                                       │
       |--
│  │   ├── arm/arm.xacro        # Module macro                    │
│  │   └── gripper/gripper.xacro                                  │
│  ├── meshes/                                                     │
│  │   ├── arm/*.stl                                              │
│  │   └── gripper/*.stl                                          │
│  └── config/                                                     │
│      ├── joint_limits.yaml    # Defaults (editable)             │
│      └── inertials.yaml                                         │
└─────────────────────────────────────────────────────────────────┘
```

## Risks / Trade-offs

### Risk: Onshape API rate limits during mesh export
**Mitigation**: Use async batch requests (already in toolkit), add retry with backoff

### Risk: Complex nested subassemblies may produce unwieldy hierarchy
**Mitigation**: `--max-depth` flag to flatten beyond certain level; module boundary heuristic reduces noise

### Trade-off: STL-only limits visual fidelity
**Accepted**: STL is universally supported; DAE can be added later as enhancement

### Trade-off: No automatic ros2_control setup
**Accepted**: Out of scope for v1; xacro structure supports easy manual addition

### Trade-off: Placeholder inertials during development
**Accepted**: Skip mass property API calls to conserve quota during development. Use placeholder values (mass=1.0, identity inertia). Can be enabled later via `--fetch-mass-properties` flag.

## Migration Plan

N/A - New project, no existing users to migrate.

## Open Questions

1. Should we generate a default `config/` with Onshape values for reference, even when no overrides provided?
   - **Tentative**: Yes, helps users discover tunable parameters

2. Should module names derive from Onshape subassembly names or be sanitized?
   - **Tentative**: Sanitize to valid ROS names (lowercase, underscores, no spaces)

3. Do we need `--dry-run` to preview structure without API calls?
   - **Tentative**: Not for v1, can add based on user feedback
