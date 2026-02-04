import importlib.metadata
import sys
from typing import Union
import tyro


from onshape2xacro.schema import (
    AuthConfig,
    ExportConfig,
    FetchCadConfig,
    VisualizeConfig,
)


def parse_args() -> Union[ExportConfig, VisualizeConfig, FetchCadConfig, AuthConfig]:
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
            "fetch-cad": FetchCadConfig,
            "auth": AuthConfig,
        }
    )


def _confirm_export_config(cli_config: ExportConfig, export_config):
    """Ask user to confirm export configuration using rich."""
    if cli_config.skip_confirmation:
        return

    try:
        from rich.console import Console
        from rich.table import Table
        from rich.panel import Panel
        from rich.prompt import Confirm
    except ImportError:
        # Fallback if rich is missing (though it should be installed)
        print("Export Configuration:")
        print(f"  Input: {cli_config.path}")
        print(f"  Output: {export_config.export.output}")
        print(f"  Name: {export_config.export.name}")
        response = input("Proceed with export? [y/N] ")
        if response.lower() not in ("y", "yes"):
            print("Export cancelled.")
            sys.exit(0)
        return

    console = Console()
    table = Table(title=None, show_header=False, box=None, padding=(0, 2))
    table.add_column("Key", style="cyan")
    table.add_column("Value", style="magenta")

    table.add_row("Input Path", str(cli_config.path))
    table.add_row("Output Path", str(export_config.export.output))
    table.add_row("Robot Name", export_config.export.name)

    # Handle optional visual mesh formats
    formats = export_config.export.visual_mesh_formats
    if formats:
        table.add_row("Visual Mesh Formats", ", ".join(formats))
    else:
        table.add_row("Visual Mesh Formats", "obj (default)")

    col_method = export_config.export.collision_option.method
    table.add_row("Collision Method", col_method)

    if col_method == "coacd":
        coacd = export_config.export.collision_option.coacd
        table.add_row("  Threshold", str(coacd.threshold))
        table.add_row("  Resolution", str(coacd.resolution))
        table.add_row("  Max Convex Hull", str(coacd.max_convex_hull))
        # Add other useful coacd options if needed

    if export_config.export.bom:
        table.add_row("BOM Path", str(export_config.export.bom))
    else:
        table.add_row("BOM Path", "[yellow]None (mass will be default)[/yellow]")

    table.add_row("Max Depth", str(cli_config.max_depth))

    console.print(
        Panel(
            table, title="[bold green]Configuration Review[/bold green]", expand=False
        )
    )

    if not Confirm.ask("Proceed with export?"):
        console.print("[red]Export cancelled.[/red]")
        sys.exit(0)


def main():
    """Main entry point for the CLI."""
    try:
        config = parse_args()

        if isinstance(config, ExportConfig):
            if not config.path.is_dir() or not (config.path / "cad.pickle").exists():
                raise RuntimeError(
                    f"Local directory {config.path} is invalid or missing cad.pickle. "
                    "Remote URL exports are deprecated for 'export' command. "
                    "Use 'fetch-cad' first."
                )

            config_path = config.path / "configuration.yaml"
            if not config_path.exists():
                raise RuntimeError(f"configuration.yaml not found in {config.path}")

            from onshape2xacro.config.export_config import ExportConfiguration
            from onshape2xacro.schema import CoACDConfig, CollisionConfig

            export_config = ExportConfiguration.load(config_path)

            ExportConfig(path=config.path)

            CollisionConfig()
            CoACDConfig()

            # Generalized CLI override application
            from dataclasses import fields

            # Override ExportOptions (name, output, visual_mesh_formats, bom, etc.)
            for field in fields(ExportConfig):
                field_name = field.name
                if field_name == "path":  # path is not in ExportOptions
                    continue
                if field_name == "collision_option":  # handled separately
                    continue

                # Check if this field exists in export_config.export
                if not hasattr(export_config.export, field_name):
                    continue

                cli_val = getattr(config, field_name)

                # If CLI provided a value (not None), override config
                if cli_val is not None:
                    setattr(export_config.export, field_name, cli_val)
                else:
                    # Otherwise, update config object with value from file so pipeline sees effective config
                    setattr(
                        config, field_name, getattr(export_config.export, field_name)
                    )

            if config.bom is None:
                # Default behavior: search for .csv file in the input directory
                csv_files = list(config.path.glob("*.csv"))
                if len(csv_files) == 1:
                    config.bom = csv_files[0]
                    export_config.export.bom = csv_files[0]
                    from loguru import logger

                    logger.info(f"Auto-detected BOM file: {config.bom}")
                elif len(csv_files) > 1:
                    from loguru import logger

                    logger.warning(
                        f"Multiple CSV files found in {config.path}, skipping auto-detection. "
                        "Please specify BOM file explicitly."
                    )

            # Override collision method
            if config.collision_option.method is not None:
                export_config.export.collision_option.method = (
                    config.collision_option.method
                )
            else:
                config.collision_option.method = (
                    export_config.export.collision_option.method
                )

            # Override CoACD options
            for field in fields(CoACDConfig):
                field_name = field.name
                cli_val = getattr(config.collision_option.coacd, field_name)

                if cli_val is not None:
                    setattr(
                        export_config.export.collision_option.coacd, field_name, cli_val
                    )
                else:
                    setattr(
                        config.collision_option.coacd,
                        field_name,
                        getattr(
                            export_config.export.collision_option.coacd, field_name
                        ),
                    )

            from onshape2xacro.pipeline import run_export

            _confirm_export_config(config, export_config)

            run_export(config, export_configuration=export_config)
        elif isinstance(config, VisualizeConfig):
            from onshape2xacro.pipeline import run_visualize

            run_visualize(config)
        elif isinstance(config, FetchCadConfig):
            from onshape2xacro.pipeline import run_fetch_cad

            run_fetch_cad(config)
        elif isinstance(config, AuthConfig):
            from onshape2xacro.pipeline import run_auth

            run_auth(config)
    except SystemExit:
        raise
    except Exception as e:
        raise RuntimeError(f"Error: {e}") from e


if __name__ == "__main__":
    main()
