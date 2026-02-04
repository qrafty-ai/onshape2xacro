import os
from pathlib import Path
from typing import Dict, Any
import numpy as np
from onshape_robotics_toolkit import Client, CAD, KinematicGraph
from onshape2xacro.optimized_cad import OptimizedCAD, OptimizedClient
from onshape_robotics_toolkit.models.assembly import Occurrence
from onshape2xacro.condensed_robot import CondensedRobot
from onshape2xacro.serializers import XacroSerializer
from onshape2xacro.config.export_config import ExportConfiguration
from onshape2xacro.config import ConfigOverride
from onshape2xacro.schema import (
    AuthConfig,
    AuthLogoutConfig,
    AuthStatusConfig,
    ExportConfig,
    FetchCadConfig,
    VisualizeConfig,
)
from onshape2xacro.auth import (
    get_credentials,
    store_credentials,
    delete_credentials,
    has_stored_credentials,
)
from onshape2xacro.naming import sanitize_name


# Monkeypatch Occurrence.tf to handle Onshape's column-major transformation arrays.
# The toolkit currently uses .reshape(4, 4) which defaults to row-major,
# transposing the matrix and losing the translation components.
def _fixed_occurrence_tf(self):
    return np.array(self.transform).reshape(4, 4, order="F")


# Use setattr to bypass property setter checks if needed,
# though re-assigning the property on the class should work.
setattr(Occurrence, "tf", property(_fixed_occurrence_tf))


def _setup_credentials():
    access_key, secret_key = get_credentials()
    if access_key and secret_key:
        os.environ["ONSHAPE_ACCESS_KEY"] = access_key
        os.environ["ONSHAPE_SECRET_KEY"] = secret_key
    return access_key, secret_key


def _get_client_and_cad(url: str, max_depth: int) -> tuple[Client, CAD]:
    """Setup client and fetch CAD assembly."""
    if not _setup_credentials()[0]:
        raise ValueError(
            "Onshape credentials not found. Either:\n"
            "  1. Set ONSHAPE_ACCESS_KEY and ONSHAPE_SECRET_KEY environment variables, or\n"
            "  2. Run 'onshape2xacro auth login' to store credentials in system keyring"
        )

    client = OptimizedClient(env=None, base_url="https://cad.onshape.com")
    print(f"Fetching assembly from {url}...")
    cad = OptimizedCAD.from_url(
        url, client=client, max_depth=max_depth, fetch_mass_properties=False
    )
    return client, cad


def _try_get_client() -> Client | None:
    if _setup_credentials()[0]:
        return OptimizedClient(env=None, base_url="https://cad.onshape.com")
    return None


def _generate_default_mate_values(cad: CAD) -> Dict[str, Any]:
    """
    Generate default mate values (zeros) for all mates in the CAD.
    This replaces the API fetch which is unreliable/unsupported.
    """
    mate_values = {}
    if hasattr(cad, "mates") and cad.mates:
        for m in cad.mates.values():
            if (
                hasattr(m, "id")
                and m.id
                and getattr(m, "name", "").startswith("joint_")
            ):
                mate_values[m.id] = {
                    "featureId": m.id,
                    "mateName": getattr(m, "name", "unknown"),
                    "rotationZ": 0.0,
                    "translationZ": 0.0,
                    "rotationX": 0.0,
                    "rotationY": 0.0,
                    "translationX": 0.0,
                    "translationY": 0.0,
                }
    return mate_values


def run_export(
    config: ExportConfig, export_configuration: ExportConfiguration | None = None
):
    """Run the complete export pipeline."""
    local_dir = config.path
    bom_path = config.bom

    if not local_dir.is_dir() or not (local_dir / "cad.pickle").exists():
        raise RuntimeError(
            f"Local directory {local_dir} is invalid or missing cad.pickle. "
            "Remote URL exports are deprecated for 'export' command. "
            "Use 'fetch-cad' first."
        )

    if export_configuration is None:
        config_path = local_dir / "configuration.yaml"
        if not config_path.exists():
            raise RuntimeError(f"configuration.yaml not found in {local_dir}")
        export_configuration = ExportConfiguration.load(config_path)
        export_configuration.merge_cli_overrides(
            name=config.name,
            output=config.output,
            format=config.format,
            visual_mesh_format=config.visual_mesh_format,
        )

    import pickle

    print(f"Loading pre-fetched CAD data from {local_dir}...")
    with open(local_dir / "cad.pickle", "rb") as f:
        cad = pickle.load(f)

    # In pre-fetched mode, we stay local unless assembly.step is missing.
    if (local_dir / "assembly.step").exists():
        asset_path = local_dir / "assembly.step"
        client = None
    elif (local_dir / "assembly.zip").exists():
        asset_path = local_dir / "assembly.zip"
        client = None
    else:
        asset_path = None
        client = _try_get_client()
        if client:
            print(
                "Warning: 'assembly.step' not found in local directory. Will attempt to download using API."
            )
        else:
            print(
                "Warning: 'assembly.step' not found in local directory and no API credentials found."
            )

    if bom_path is None and (local_dir / "bom.csv").exists():
        bom_path = local_dir / "bom.csv"

    mate_values = export_configuration.mate_values

    # 3. Build Kinematic Graph
    print("Building kinematic graph...")
    graph = KinematicGraph.from_cad(cad)

    format = export_configuration.export.format
    module_boundaries = None
    if format == "xacro_module":
        from onshape2xacro.module_boundary import detect_module_boundaries

        module_boundaries = detect_module_boundaries(cad, graph)

    # 4. Create Robot Model
    print("Creating robot model...")
    robot_name = export_configuration.export.name
    # Fallback to directory name if configured name is invalid/empty/placeholder

    if not robot_name or sanitize_name(robot_name) == "_":
        logger_func = print
        try:
            from loguru import logger

            logger_func = logger.warning
        except ImportError:
            pass

        old_name = robot_name
        robot_name = local_dir.name
        logger_func(
            f"Robot name '{old_name}' is invalid/empty. Using directory name '{robot_name}' instead."
        )
        # Update configuration so it persists
        export_configuration.export.name = robot_name

    robot = CondensedRobot.from_graph(
        graph,
        cad=cad,
        name=robot_name,
        mate_values=mate_values,
        link_name_overrides=export_configuration.link_names,
        # Pass fail_fast parameter derived from debug configuration
        fail_fast=getattr(config, "debug", False),
        module_boundaries=module_boundaries,
    )
    # Set client and cad for serializer's mesh export

    robot.client = client
    robot.cad = cad
    robot.asset_path = asset_path
    if module_boundaries is not None:
        robot.module_boundaries = module_boundaries

    # 5. Load Config Overrides
    print("Loading configuration overrides...")
    override = ConfigOverride.load(config.config)

    # 6. Serialize and Save
    output_path = export_configuration.export.output
    visual_mesh_format = export_configuration.export.visual_mesh_format

    print(f"Serializing to {output_path}...")
    if format == "xacro_module":
        try:
            from onshape2xacro.serializers.modular import ModularXacroSerializer

            serializer = ModularXacroSerializer()
        except ImportError:
            raise NotImplementedError(
                f"Format '{format}' is not yet implemented. "
                "ModularXacroSerializer not found."
            )
    else:
        serializer = XacroSerializer()

    serializer.save(
        robot,
        str(output_path),
        download_assets=True,
        config=override,
        bom_path=bom_path,
        visual_mesh_format=visual_mesh_format,
        collision_option=export_configuration.export.collision_option,
    )
    print("Export complete!")


def run_visualize(config: VisualizeConfig):
    """Visualize the kinematic graph."""
    _, cad = _get_client_and_cad(config.url, config.max_depth)

    print("Building kinematic graph...")
    graph = KinematicGraph.from_cad(cad)

    print("Visualizing graph...")
    output_path = str(config.output)
    graph.show(file_name=output_path)
    print(f"Graph saved to {output_path}")


def run_fetch_cad(config: FetchCadConfig):
    """Fetch CAD data and save to a directory."""
    import pickle
    import shutil
    from onshape2xacro.mesh_exporters.step import StepMeshExporter
    from onshape2xacro.config.export_config import ExportConfiguration, ExportOptions

    client, cad = _get_client_and_cad(config.url, config.max_depth)

    output_dir = Path(config.output)
    output_dir.mkdir(parents=True, exist_ok=True)

    print(f"Saving CAD data to {output_dir / 'cad.pickle'}...")
    with open(output_dir / "cad.pickle", "wb") as f:
        pickle.dump(cad, f)

    print(f"Exporting STEP assembly to {output_dir / 'assembly.step'}...")
    exporter = StepMeshExporter(client, cad)
    exporter.export_step(output_dir / "assembly.step")

    print("Generating default mate values...")
    mate_values = _generate_default_mate_values(cad)

    # Build temporary robot to discover auto-generated link names
    print("Building kinematic graph...")
    graph = KinematicGraph.from_cad(cad)
    print("Creating temporary robot model to discover link names...")
    robot = CondensedRobot.from_graph(graph, cad=cad, mate_values=mate_values)
    link_names = {name: name for name in robot.nodes}

    config_path = output_dir / "configuration.yaml"
    if config_path.exists():
        print(f"Warning: {config_path} already exists. Skipping generation.")
    else:
        export_config = ExportConfiguration(
            export=ExportOptions(name=cad.name or "robot"),
            mate_values=mate_values,
            link_names=link_names,
        )
        export_config.save(config_path)
        print(f"Saved configuration to {config_path}")

    if config.bom:
        if not config.bom.exists():
            print(f"Warning: BOM file not found: {config.bom}")
        else:
            dest_bom = output_dir / "bom.csv"
            shutil.copy(config.bom, dest_bom)
            print(f"Copied BOM to {dest_bom}")

    print(f"Fetch CAD complete! Data saved to {output_dir}")


def run_auth(config: AuthConfig):
    """Manage Onshape API credentials."""
    import getpass

    cmd = config.command

    if isinstance(cmd, AuthStatusConfig):
        # Just check status
        if has_stored_credentials():
            print("✓ Credentials are stored in system keyring")
        else:
            access_key = os.environ.get("ONSHAPE_ACCESS_KEY")
            secret_key = os.environ.get("ONSHAPE_SECRET_KEY")
            if access_key and secret_key:
                print("✓ Credentials found in environment variables")
            else:
                print("✗ No credentials found")
        return

    if isinstance(cmd, AuthLogoutConfig):
        if not has_stored_credentials():
            print("No credentials stored in keyring")
            return
        delete_credentials()
        print("Credentials deleted from system keyring")
        return

    # AuthLoginConfig: Interactive credential entry
    if has_stored_credentials():
        response = (
            input("Credentials already exist in keyring. Overwrite? [y/N]: ")
            .strip()
            .lower()
        )
        if response not in ("y", "yes"):
            print("Aborted")
            return

    print("Enter your Onshape API credentials.")
    print("Get them from: https://dev-portal.onshape.com/keys")
    print()

    access_key = input("Access Key: ").strip()
    if not access_key:
        print("Error: Access key cannot be empty")
        return

    secret_key = getpass.getpass("Secret Key: ").strip()
    if not secret_key:
        print("Error: Secret key cannot be empty")
        return

    store_credentials(access_key, secret_key)
    print("✓ Credentials stored in system keyring")
