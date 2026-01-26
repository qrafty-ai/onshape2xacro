## ADDED Requirements

### Requirement: YAML Configuration Override
The system SHALL support YAML configuration files for overriding exported parameters.

Supported override categories:
- Joint limits (position, velocity, effort)
- Inertial properties (mass, center of mass, inertia tensor)
- Dynamics parameters (friction, damping)

#### Scenario: Override joint limits
- **GIVEN** an override YAML with joint limits
- **WHEN** export is executed with `--config` pointing to the file
- **THEN** generated xacro uses override values instead of Onshape values

#### Scenario: Partial override
- **GIVEN** an override YAML specifying only some joints
- **WHEN** export is executed
- **THEN** specified joints use override values
- **AND** unspecified joints use placeholder values (development mode) or Onshape-derived values (when mass properties API is enabled)

#### Scenario: Override file not found
- **GIVEN** `--config` pointing to non-existent file
- **WHEN** export is executed
- **THEN** CLI exits with error and helpful message

### Requirement: Joint Limits Configuration
The system SHALL support joint limits override in YAML format:

```yaml
joint_limits:
  <joint_name>:
    lower: <float>       # Lower position limit (rad or m)
    upper: <float>       # Upper position limit (rad or m)
    velocity: <float>    # Max velocity (rad/s or m/s)
    effort: <float>      # Max effort (Nm or N)
```

#### Scenario: Valid joint limits override
- **GIVEN** YAML with valid joint limits
- **WHEN** export is executed
- **THEN** xacro joint elements contain specified limit values

#### Scenario: Joint name not found
- **GIVEN** YAML referencing non-existent joint name
- **WHEN** export is executed
- **THEN** warning is logged
- **AND** export continues with remaining valid overrides

### Requirement: Inertial Properties Configuration
The system SHALL support inertial properties override in YAML format:

```yaml
inertials:
  <link_name>:
    mass: <float>                    # Mass in kg
    origin:
      xyz: [<x>, <y>, <z>]          # Center of mass offset
      rpy: [<r>, <p>, <y>]          # Orientation (optional)
    inertia:
      ixx: <float>
      ixy: <float>
      ixz: <float>
      iyy: <float>
      iyz: <float>
      izz: <float>
```

#### Scenario: Override link mass only
- **GIVEN** YAML specifying only mass for a link
- **WHEN** export is executed
- **THEN** mass is overridden
- **AND** inertia values use placeholder values (development mode)

### Requirement: Dynamics Configuration
The system SHALL support dynamics parameters override in YAML format:

```yaml
dynamics:
  <joint_name>:
    friction: <float>    # Static friction
    damping: <float>     # Viscous damping
```

#### Scenario: Override dynamics
- **GIVEN** YAML with dynamics parameters
- **WHEN** export is executed
- **THEN** xacro joint dynamics elements contain specified values

### Requirement: Default Configuration Generation
The system SHALL generate default configuration files containing Onshape-derived values.

Generated config files SHALL serve as:
- Reference for available tunable parameters
- Starting point for user customization

#### Scenario: Default config generation
- **GIVEN** an export without `--config` flag
- **WHEN** export completes
- **THEN** `config/joint_limits.yaml` contains limits derived from Onshape mates (or placeholders)
- **AND** `config/inertials.yaml` contains placeholder mass properties (development mode)
- **AND** `config/dynamics.yaml` contains default dynamics values

### Requirement: Placeholder Inertials (Development Mode)
The system SHALL use placeholder inertial values by default to avoid mass property API calls.

Placeholder values:
- mass: 1.0 kg
- origin: (0, 0, 0)
- inertia: identity matrix (ixx=iyy=izz=0.01, ixy=ixz=iyz=0)

The system MAY provide a `--fetch-mass-properties` flag for future implementation to enable actual API calls.

#### Scenario: Default export uses placeholders
- **GIVEN** an export without mass property flag
- **WHEN** export is executed
- **THEN** all links have placeholder inertial values
- **AND** no mass property API calls are made

#### Scenario: Placeholder values are valid URDF
- **GIVEN** generated xacro with placeholder inertials
- **WHEN** xacro is processed and validated with check_urdf
- **THEN** the resulting URDF is valid

### Requirement: Configuration Validation
The system SHALL validate configuration YAML against expected schema.

#### Scenario: Invalid YAML syntax
- **GIVEN** malformed YAML file
- **WHEN** export is executed with `--config`
- **THEN** CLI exits with parse error and line number

#### Scenario: Invalid value type
- **GIVEN** YAML with string where number expected
- **WHEN** export is executed
- **THEN** CLI exits with validation error indicating field and expected type
