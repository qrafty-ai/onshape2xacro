import hashlib
from dataclasses import dataclass
from typing import List, Any, Dict, Iterable, Tuple, cast
from collections.abc import Iterable as IterableABC
from onshape_robotics_toolkit.robot import Robot
from onshape2xacro.naming import sanitize_name


@dataclass
class LinkRecord:
    name: str
    part_ids: List[str]
    occurrences: List[List[str]]
    part_names: List[str]
    keys: List[Any]
    parent: Any = None


@dataclass
class JointRecord:
    name: str
    joint_type: str
    parent: str  # link name
    child: str  # link name
    limits: Any = None
    mate: Any = None


def is_joint_mate(mate) -> bool:
    """Check if a mate name indicates a robot joint."""
    if mate is None:
        return False
    # Handle both object with .name and dict with "name" key
    name = getattr(mate, "name", None)
    if name is None and isinstance(mate, dict):
        name = mate.get("name")

    if name is None:
        return False
    return str(name).startswith("joint_")


def _iter_edges(graph) -> Iterable[Tuple[Any, Any, Any]]:
    edges_obj = getattr(graph, "edges", [])
    edge_iter: Iterable[Any]
    if callable(edges_obj):
        try:
            edge_iter = cast(Iterable[Any], edges_obj(data=True))
        except TypeError:
            edge_iter = cast(Iterable[Any], edges_obj())
    elif isinstance(edges_obj, IterableABC):
        edge_iter = cast(Iterable[Any], edges_obj)
    else:
        edge_iter = []

    for edge in edge_iter:
        if hasattr(edge, "u") and hasattr(edge, "v"):
            yield edge.u, edge.v, getattr(edge, "mate", None)
            continue

        if isinstance(edge, (list, tuple)) and len(edge) >= 2:
            u, v = edge[0], edge[1]
            data = edge[2] if len(edge) >= 3 else None
            if data is None and hasattr(graph, "get_edge_data"):
                data = graph.get_edge_data(u, v)

            mate = None
            if isinstance(data, dict):
                mate = data.get("data") or data.get("mate")
            else:
                mate = data

            yield u, v, mate


class CondensedRobot(Robot):
    """
    A Robot representation where fixed parts are grouped into single links.
    """

    client: Any = None
    cad: Any = None
    asset_path: Any = None

    @classmethod
    def from_graph(
        cls,
        kinematic_graph,
        client=None,
        name="robot",
        fetch_mass_properties=True,
        **kwargs,
    ):
        """
        Create a condensed robot from a kinematic graph.
        Groups all nodes connected by non-joint edges into single links.
        """
        nodes = list(kinematic_graph.nodes)
        parent_map = {node: node for node in nodes}

        def find(i):
            if parent_map[i] == i:
                return i
            parent_map[i] = find(parent_map[i])
            return parent_map[i]

        def union(i, j):
            root_i = find(i)
            root_j = find(j)
            if root_i != root_j:
                parent_map[root_i] = root_j

        # Union nodes connected by non-joint edges
        for u, v, mate in _iter_edges(kinematic_graph):
            if not is_joint_mate(mate):
                union(u, v)

        # Collect groups
        groups: Dict[Any, List[Any]] = {}
        for node in nodes:
            root = find(node)
            if root not in groups:
                groups[root] = []
            groups[root].append(node)

        # Create the robot instance
        # Based on toolkit.Robot signature: Robot(kinematic_graph, name)
        robot = cls(kinematic_graph, name=name)
        # Clear any nodes/edges added by the base constructor so we can add condensed ones
        robot.clear()

        # Map group root to LinkRecord
        root_to_link_node = {}
        used_names = set()

        for root, group_nodes in groups.items():

            def node_payload(node):
                node_data = None
                if hasattr(kinematic_graph, "nodes"):
                    try:
                        node_data = kinematic_graph.nodes[node]
                    except Exception:
                        node_data = None
                if isinstance(node_data, dict) and "data" in node_data:
                    node_data = node_data.get("data")
                if node_data is None and hasattr(node, "part_id"):
                    node_data = node
                return node_data

            payloads = [node_payload(n) for n in group_nodes]
            part_ids = [
                getattr(p, "partId", getattr(p, "part_id", str(n)))
                for p, n in zip(payloads, group_nodes)
            ]
            occurrences = [
                getattr(p, "occurrence", n) for p, n in zip(payloads, group_nodes)
            ]
            part_names = [
                getattr(p, "part_name", str(n)) for p, n in zip(payloads, group_nodes)
            ]

            # Create link name by joining sanitized part names
            sanitized_names = sorted(
                list(set(sanitize_name(n) for n in part_names if n))
            )
            link_name = "_".join(sanitized_names)
            if len(link_name) > 128:
                h = hashlib.md5(link_name.encode()).hexdigest()[:8]
                link_name = f"{link_name[:119]}_{h}"

            if not link_name:
                link_name = f"link_{len(used_names)}"

            if link_name in used_names:
                suffix = 1
                unique_name = f"{link_name}_{suffix}"
                while unique_name in used_names:
                    suffix += 1
                    unique_name = f"{link_name}_{suffix}"
                link_name = unique_name
            used_names.add(link_name)

            # Use parent of the root node (or first node in group)
            # This is important for hierarchical xacro export
            parent = None
            root_payload = node_payload(root)
            if root_payload is not None:
                parent = getattr(root_payload, "parent", None)
            if parent is None and payloads:
                parent = getattr(payloads[0], "parent", None)

            link_rec = LinkRecord(
                part_ids=part_ids,
                occurrences=occurrences,
                part_names=part_names,
                keys=group_nodes,
                parent=parent,
                name=link_name,
            )

            # Add node to robot.
            node_obj = link_name
            robot.add_node(node_obj, link=link_rec, data=link_rec)
            root_to_link_node[root] = node_obj

        # Add joint edges
        # Assumes kinematic graph edges are directed parent -> child.
        # This is required for a valid tree structure in the resulting robot.
        for u, v, mate in _iter_edges(kinematic_graph):
            if not is_joint_mate(mate) or mate is None:
                continue

            parent_node = root_to_link_node[find(u)]
            child_node = root_to_link_node[find(v)]
            if parent_node == child_node:
                continue

            # Extract mate properties robustly
            mate_name = getattr(mate, "name", None)
            if mate_name is None and isinstance(mate, dict):
                mate_name = mate.get("name")

            if mate_name is None:
                mate_name = f"joint_{parent_node}_{child_node}"

            mate_type = getattr(mate, "mateType", None)
            if mate_type is None and isinstance(mate, dict):
                mate_type = mate.get("mateType") or mate.get("joint_type")
            if mate_type is None:
                mate_type = "REVOLUTE"

            mate_limits = getattr(mate, "limits", None)
            if mate_limits is None and isinstance(mate, dict):
                mate_limits = mate.get("limits")

            joint_rec = JointRecord(
                name=mate_name,
                joint_type=mate_type,
                parent=parent_node,
                child=child_node,
                limits=mate_limits,
                mate=mate,
            )

            # Store in both 'joint' and 'data' attributes
            robot.add_edge(parent_node, child_node, joint=joint_rec, data=joint_rec)

        return robot
