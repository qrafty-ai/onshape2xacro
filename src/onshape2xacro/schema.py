from dataclasses import dataclass, field
from pathlib import Path
from typing import Annotated, Union, Literal
import tyro


@dataclass
class CoACDConfig:
    """Configuration for CoACD collision mesh generation."""

    threshold: float | None = None
    """Threshold for CoACD."""
    resolution: int | None = None
    """Resolution for CoACD."""
    max_convex_hull: int | None = None
    """Maximum convex hull for CoACD."""
    preprocess: bool | None = None
    """Preprocess for CoACD."""
    seed: int | None = None
    """Seed for CoACD."""


@dataclass
class CollisionConfig:
    """Configuration for collision mesh generation."""

    method: Literal["fast", "coacd"] | None = None
    """Method for collision mesh generation (fast, coacd). Defaults to fast."""
    coacd: CoACDConfig = field(default_factory=CoACDConfig)
    """CoACD specific configuration."""


@dataclass
class ExportConfig:
    """Export an Onshape assembly to xacro."""

    path: tyro.conf.Positional[Path]
    """Local directory containing cad.pickle and configuration.yaml."""
    output: Path | None = None
    """Output directory path."""
    name: str | None = None
    """Robot name, defaults to assembly name from Onshape."""
    config: Path | None = None
    """Path to YAML override configuration file."""
    bom: Path | None = None
    """Path to BOM CSV file for density lookup."""
    max_depth: int = 5
    """Maximum subassembly traversal depth."""
    visual_mesh_format: Literal["glb", "dae", "obj", "stl"] | None = None
    """Format for visual meshes (glb, dae, obj, stl). Defaults to obj."""
    collision_option: CollisionConfig = field(default_factory=CollisionConfig)
    """Configuration for collision mesh generation."""
    debug: bool = False
    """Enable debug mode with full tracebacks."""


@dataclass
class VisualizeConfig:
    """Visualize the kinematic graph of an Onshape assembly."""

    url: tyro.conf.Positional[str]
    """Onshape document URL pointing to an assembly."""
    output: Path
    """Output path to save the graph image (e.g. graph.png)."""
    max_depth: int = 5
    """Maximum subassembly traversal depth."""


@dataclass
class FetchCadConfig:
    """Fetch CAD data and assets from Onshape and save to a directory."""

    url: tyro.conf.Positional[str]
    """Onshape document URL pointing to an assembly."""
    output: Path
    """Output directory to save the CAD data (cad.pickle and assembly.zip)."""
    bom: Path | None = None
    """Path to BOM CSV file to copy to output directory for inertia calculation."""
    max_depth: int = 5
    """Maximum subassembly traversal depth."""


@dataclass
class AuthLoginConfig:
    """Store Onshape API credentials in system keyring."""

    pass


@dataclass
class AuthStatusConfig:
    """Check if credentials are stored."""

    pass


@dataclass
class AuthLogoutConfig:
    """Delete stored credentials from system keyring."""

    pass


@dataclass
class AuthConfig:
    """Manage Onshape API credentials stored in system keyring."""

    command: Annotated[
        Union[
            Annotated[AuthLoginConfig, tyro.conf.subcommand("login")],
            Annotated[AuthStatusConfig, tyro.conf.subcommand("status")],
            Annotated[AuthLogoutConfig, tyro.conf.subcommand("logout")],
        ],
        tyro.conf.arg(name=""),
    ]
