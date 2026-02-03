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

            schema_collision_defaults = CollisionConfig()
            schema_coacd_defaults = CoACDConfig()

            if config.name is not None:
                export_config.export.name = config.name
            if config.output is not None:
                export_config.export.output = config.output
            if config.visual_mesh_format is not None:
                export_config.export.visual_mesh_format = config.visual_mesh_format

            if config.collision_option.method != schema_collision_defaults.method:
                export_config.export.collision_option.method = (
                    config.collision_option.method
                )
            else:
                config.collision_option.method = (
                    export_config.export.collision_option.method
                )

            for field_name in [
                "threshold",
                "resolution",
                "max_convex_hull",
                "preprocess",
                "seed",
            ]:
                cli_val = getattr(config.collision_option.coacd, field_name)
                schema_default = getattr(schema_coacd_defaults, field_name)

                if cli_val != schema_default:
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
