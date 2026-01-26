## ADDED Requirements

### Requirement: CLI Export Command
The system SHALL provide a CLI command `onshape2xacro export` that exports an Onshape assembly to xacro files.

The command SHALL accept the following arguments:
- `url` (required): Onshape document URL pointing to an assembly
- `--output` (optional): Output directory path, defaults to current directory
- `--name` (optional): Robot name, defaults to assembly name from Onshape
- `--config` (optional): Path to YAML override configuration file
- `--max-depth` (optional): Maximum subassembly traversal depth, defaults to 5

#### Scenario: Basic export with URL only
- **GIVEN** a valid Onshape assembly URL
- **WHEN** user runs `onshape2xacro export <url>`
- **THEN** xacro files are generated in current directory
- **AND** meshes are exported to `meshes/` subdirectory
- **AND** default config files are generated in `config/` subdirectory

#### Scenario: Export with custom output directory
- **GIVEN** a valid Onshape assembly URL
- **WHEN** user runs `onshape2xacro export <url> --output ./my_robot`
- **THEN** all files are generated under `./my_robot/`

#### Scenario: Export with config overrides
- **GIVEN** a valid Onshape assembly URL and override YAML file
- **WHEN** user runs `onshape2xacro export <url> --config overrides.yaml`
- **THEN** override values are applied to generated xacro files

#### Scenario: Invalid Onshape URL
- **GIVEN** an invalid or inaccessible Onshape URL
- **WHEN** user runs `onshape2xacro export <url>`
- **THEN** CLI exits with non-zero status
- **AND** error message explains the issue (invalid URL, authentication, permissions)

### Requirement: Onshape Authentication
The system SHALL authenticate with Onshape API using environment variables.

The system SHALL support authentication via:
- `ONSHAPE_ACCESS_KEY` and `ONSHAPE_SECRET_KEY` for API key authentication

#### Scenario: Valid API credentials
- **GIVEN** valid Onshape API credentials in environment variables
- **WHEN** user runs export command
- **THEN** system successfully connects to Onshape API

#### Scenario: Missing credentials
- **GIVEN** missing Onshape API credentials
- **WHEN** user runs export command
- **THEN** CLI exits with non-zero status
- **AND** error message instructs user to set required environment variables

### Requirement: Help and Version
The system SHALL provide `--help` and `--version` flags.

#### Scenario: Help flag
- **WHEN** user runs `onshape2xacro --help` or `onshape2xacro export --help`
- **THEN** CLI displays usage information with all available options

#### Scenario: Version flag
- **WHEN** user runs `onshape2xacro --version`
- **THEN** CLI displays current version number
