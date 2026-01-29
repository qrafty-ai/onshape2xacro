import os
from pathlib import Path
import numpy as np
from onshape_robotics_toolkit import Client, CAD, KinematicGraph
from onshape_robotics_toolkit.models.assembly import Occurrence
from onshape2xacro.condensed_robot import CondensedRobot
from onshape2xacro.serializers import XacroSerializer
from onshape2xacro.config import ConfigOverride
from onshape2xacro.cli import (
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


# Monkeypatch Occurrence.tf to handle Onshape's column-major transformation arrays.
# The toolkit currently uses .reshape(4, 4) which defaults to row-major,
# transposing the matrix and losing the translation components.
def _fixed_occurrence_tf(self):
    return np.array(self.transform).reshape(4, 4, order="F")


# Use setattr to bypass property setter checks if needed,
# though re-assigning the property on the class should work.
setattr(Occurrence, "tf", property(_fixed_occurrence_tf))


def _get_client_and_cad(url: str, max_depth: int) -> tuple[Client, CAD]:
    """Setup client and fetch CAD assembly."""
    access_key, secret_key = get_credentials()

    if not access_key or not secret_key:
        raise ValueError(
            "Onshape credentials not found. Either:\n"
            "  1. Set ONSHAPE_ACCESS_KEY and ONSHAPE_SECRET_KEY environment variables, or\n"
            "  2. Run 'onshape2xacro auth login' to store credentials in system keyring"
        )

    # Set env vars for the toolkit's Client which reads from os.environ
    os.environ["ONSHAPE_ACCESS_KEY"] = access_key
    os.environ["ONSHAPE_SECRET_KEY"] = secret_key

    client = Client(
        env=None,
        base_url="https://cad.onshape.com",
    )

    print(f"Fetching assembly from {url}...")
    cad = CAD.from_url(url, client=client, max_depth=max_depth)
    return client, cad


def _try_get_client() -> Client | None:
    access_key, secret_key = get_credentials()
    if not access_key or not secret_key:
        return None

    os.environ["ONSHAPE_ACCESS_KEY"] = access_key
    os.environ["ONSHAPE_SECRET_KEY"] = secret_key

    return Client(
        env=None,
        base_url="https://cad.onshape.com",
    )


def run_export(config: ExportConfig):
    """Run the complete export pipeline."""
    url_path = Path(config.url)
    if url_path.is_dir() and (url_path / "cad.pickle").exists():
        import pickle

        print(f"Loading pre-fetched CAD data from {url_path}...")
        with open(url_path / "cad.pickle", "rb") as f:
            cad = pickle.load(f)
        client = _try_get_client()
        # Check for assembly.step (new format) or assembly.zip (old format)
        if (url_path / "assembly.step").exists():
            asset_path = url_path / "assembly.step"
        elif (url_path / "assembly.zip").exists():
            asset_path = url_path / "assembly.zip"
        else:
            asset_path = None
    else:
        client, cad = _get_client_and_cad(config.url, config.max_depth)
        asset_path = None

    # 3. Build Kinematic Graph
    print("Building kinematic graph...")
    graph = KinematicGraph.from_cad(cad)

    # 4. Create Robot Model
    print("Creating robot model...")
    robot_name = config.name or cad.name or "robot"
    robot = CondensedRobot.from_graph(graph, cad=cad, name=robot_name)
    # Set client and cad for serializer's mesh export
    robot.client = client
    robot.cad = cad
    robot.asset_path = asset_path

    # 5. Load Config Overrides
    print("Loading configuration overrides...")
    override = ConfigOverride.load(config.config)

    # 6. Serialize and Save
    print(f"Serializing to {config.output}...")
    serializer = XacroSerializer()
    serializer.save(robot, str(config.output), download_assets=True, config=override)
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
    from onshape2xacro.mesh_exporters.step import StepMeshExporter

    client, cad = _get_client_and_cad(config.url, config.max_depth)

    output_dir = Path(config.output)
    output_dir.mkdir(parents=True, exist_ok=True)

    print(f"Saving CAD data to {output_dir / 'cad.pickle'}...")
    with open(output_dir / "cad.pickle", "wb") as f:
        pickle.dump(cad, f)

    print(f"Exporting STEP assembly to {output_dir / 'assembly.step'}...")
    exporter = StepMeshExporter(client, cad)
    exporter.export_step(output_dir / "assembly.step")

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
