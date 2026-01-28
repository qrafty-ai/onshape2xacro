import pickle
import sys
from onshape_robotics_toolkit import KinematicGraph
from onshape_robotics_toolkit.parse import CAD
from onshape2xacro.condensed_robot import CondensedRobot


def load_cad(file_path: str):
    print(f"Loading CAD data from {file_path}...")
    try:
        with open(file_path, "rb") as f:
            cad = pickle.load(f)

        if not isinstance(cad, CAD):
            print(f"Error: Loaded object is not a CAD instance, but {type(cad)}")
            return

        print(f"✅ CAD object loaded successfully: {cad.name}")
        print(f"  - Document ID: {cad.document_id}")
        print(
            f"  - Elements: {len(cad.parts)} parts, {len(cad.subassemblies)} subassemblies"
        )

        print("\nBuilding Kinematic Graph...")
        graph = KinematicGraph.from_cad(cad)
        print("✅ Kinematic Graph built successfully")
        print(f"  - Nodes: {len(graph.nodes)}")
        print(f"  - Edges: {len(graph.edges)}")

        print("\nBuilding Condensed Robot...")
        robot = CondensedRobot.from_graph(graph, cad=cad, name=cad.name or "robot")
        print("✅ Condensed Robot built successfully")
        print(f"  - Links: {len(robot.nodes)}")
        print(f"  - Joints: {len(robot.edges)}")

        return cad, graph, robot

    except FileNotFoundError:
        print(f"Error: File not found: {file_path}")
    except Exception as e:
        print(f"Error loading CAD: {e}")
        import traceback

        traceback.print_exc()


if __name__ == "__main__":
    if len(sys.argv) < 2:
        print("Usage: python load_pickle.py <path_to_pickle_file>")
        sys.exit(1)

    load_cad(sys.argv[1])
