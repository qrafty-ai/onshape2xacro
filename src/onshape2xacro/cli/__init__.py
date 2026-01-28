import importlib.metadata
import sys
from typing import Annotated, Union
from dataclasses import dataclass
from pathlib import Path
import tyro


@dataclass
class ExportConfig:
    """Export an Onshape assembly to xacro."""

    url: tyro.conf.Positional[str]
    """Onshape document URL pointing to an assembly."""
    output: Path = Path(".")
    """Output directory path."""
    name: str | None = None
    """Robot name, defaults to assembly name from Onshape."""
    config: Path | None = None
    """Path to YAML override configuration file."""
    max_depth: int = 5
    """Maximum subassembly traversal depth."""


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


def parse_args() -> Union[ExportConfig, VisualizeConfig, AuthConfig]:
    """Parse CLI arguments using tyro."""
    # Add version support
    if "--version" in sys.argv:
        try:
            version = importlib.metadata.version("onshape2xacro")
        except importlib.metadata.PackageNotFoundError:
            version = "unknown"
        print(f"onshape2xacro {version}")
        sys.exit(0)

    return tyro.extras.subcommand_cli_from_dict(
        {
            "export": ExportConfig,
            "visualize": VisualizeConfig,
            "auth": AuthConfig,
        }
    )


def main():
    """Main entry point for the CLI."""
    try:
        config = parse_args()

        # Import pipeline here to avoid side effects (like ORT.yaml logging) during --help
        from onshape2xacro.pipeline import run_auth, run_export, run_visualize

        if isinstance(config, ExportConfig):
            run_export(config)
        elif isinstance(config, VisualizeConfig):
            run_visualize(config)
        elif isinstance(config, AuthConfig):
            run_auth(config)
    except SystemExit:
        raise
    except Exception as e:
        print(f"Error: {e}")
        sys.exit(1)


if __name__ == "__main__":
    main()
