import os
from pathlib import Path
from typing import Optional, Union
from onshape_robotics_toolkit import Client, CAD, KinematicGraph
from onshape_robotics_toolkit.robot import Robot
from onshape2xacro.serializers import XacroSerializer
from onshape2xacro.config import ConfigOverride
from onshape2xacro.cli import ExportConfig, VisualizeConfig


def _get_client_and_cad(url: str, max_depth: int) -> tuple[Client, CAD]:
    """Setup client and fetch CAD assembly."""
    access_key = os.environ.get("ONSHAPE_ACCESS_KEY")
    secret_key = os.environ.get("ONSHAPE_SECRET_KEY")

    if not access_key or not secret_key:
        raise ValueError(
            "ONSHAPE_ACCESS_KEY and ONSHAPE_SECRET_KEY must be set in environment"
        )

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
    output_path = str(config.output) if config.output else None
    graph.show(file_name=output_path)

    if output_path:
        print(f"Graph saved to {output_path}")
    else:
        # robotics-toolkit default filename is <assembly_name>_kinematic_graph.png
        print(f"Graph saved as {cad.name}_kinematic_graph.png")
