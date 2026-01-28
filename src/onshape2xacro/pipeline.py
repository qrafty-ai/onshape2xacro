import os
from onshape_robotics_toolkit import Client, CAD, KinematicGraph
from onshape_robotics_toolkit.robot import Robot
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


def run_export(config: ExportConfig):
    """Run the complete export pipeline."""
    client, cad = _get_client_and_cad(config.url, config.max_depth)

    # 3. Build Kinematic Graph
    print("Building kinematic graph...")
    graph = KinematicGraph.from_cad(cad)

    # 4. Create Robot Model
    print("Creating robot model...")
    robot_name = config.name or cad.name
    robot = Robot.from_graph(graph, client=client, name=robot_name)

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
    """Fetch CAD data and save to pickle."""
    import pickle

    _, cad = _get_client_and_cad(config.url, config.max_depth)

    print(f"Saving CAD data to {config.output}...")
    with open(config.output, "wb") as f:
        pickle.dump(cad, f)

    print("Fetch CAD complete!")


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
