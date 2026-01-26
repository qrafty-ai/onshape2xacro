import importlib.metadata
import sys
from typing import Union
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
    output: Path | None = None
    """Optional output path to save the graph image (e.g. graph.png)."""
    max_depth: int = 5
    """Maximum subassembly traversal depth."""


def parse_args() -> Union[ExportConfig, VisualizeConfig]:
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
        }
    )


def main():
    """Main entry point for the CLI."""
    from onshape2xacro.pipeline import run_export, run_visualize

    try:
        config = parse_args()
        if isinstance(config, ExportConfig):
            run_export(config)
        elif isinstance(config, VisualizeConfig):
            run_visualize(config)
    except SystemExit:
        raise
    except Exception as e:
        print(f"Error: {e}")
        sys.exit(1)


if __name__ == "__main__":
    main()
