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


def main():
    """Main entry point for the CLI."""
    try:
        config = parse_args()

        if isinstance(config, ExportConfig):
            config_path = config.path / "configuration.yaml"
            if not config_path.exists():
                raise RuntimeError(f"configuration.yaml not found in {config.path}")

            from onshape2xacro.config.export_config import ExportConfiguration

            export_config = ExportConfiguration.load(config_path)
            export_config.merge_cli_overrides(
                name=config.name,
                output=config.output,
                visual_mesh_format=config.visual_mesh_format,
            )

            config.name = export_config.export.name
            config.output = export_config.export.output
            config.visual_mesh_format = export_config.export.visual_mesh_format

            from onshape2xacro.pipeline import run_export

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
