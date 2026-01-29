import hashlib
import numpy as np
from dataclasses import dataclass
from typing import List, Any, Dict, Iterable, Tuple, cast, Optional
from collections.abc import Iterable as IterableABC
from onshape_robotics_toolkit.robot import Robot
from onshape_robotics_toolkit.models.link import Origin
from onshape2xacro.naming import sanitize_name


@dataclass
class LinkRecord:
    name: str
    part_ids: List[str]
    occurrences: List[List[str]]
    part_names: List[str]
    keys: List[Any]
    parent: Any = None
    frame_transform: Optional[np.ndarray] = None  # World to Link transform


@dataclass
class JointRecord:
    name: str
    joint_type: str
    parent: str  # link name
    child: str  # link name
    limits: Any = None
    mate: Any = None
    origin: Optional[Origin] = None


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


def _part_world_matrix(part: Any) -> np.ndarray:
    """Extract 4x4 world transform from a CAD part."""
    part_tf = getattr(part, "worldToPartTF", None)
    if part_tf is None:
        return np.eye(4)
    tf_value = getattr(part_tf, "to_tf", None)
    if callable(tf_value):
        return cast(np.ndarray, tf_value())
    if tf_value is not None:
        return cast(np.ndarray, tf_value)
    return np.eye(4)


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
        cad=None,
        **kwargs,
    ):
        """
        Create a condensed robot from a kinematic graph.
        Groups all nodes connected by non-joint edges into single links.
        """
        if cad is None and "cad" in kwargs:
            cad = kwargs["cad"]

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
        robot = cls(kinematic_graph, name=name)
        robot.clear()

        # Map group root to LinkRecord
        root_to_link_node = {}
        used_names = set()

        for group_root, group_nodes in groups.items():

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
            part_ids = [str(n) for n in group_nodes]
            occurrences = [
                getattr(p, "occurrence", n) for p, n in zip(payloads, group_nodes)
            ]
            part_names = [
                getattr(p, "part_name", str(n)) for p, n in zip(payloads, group_nodes)
            ]

            # Create link name
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

            parent = None
            root_payload = node_payload(group_root)
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

            robot.add_node(link_name, link=link_rec, data=link_rec)
            root_to_link_node[group_root] = link_name

        # Calculate Transforms and Add Joints
        # 1. Identify roots and build link-level graph
        link_to_rec = {
            data["link"].name: data["link"] for _, data in robot.nodes(data=True)
        }

        # We need to know which link is parent and which is child based on joints
        # LinkGraph: parent_link_name -> list of (child_link_name, mate)
        link_tree: Dict[str, List[Tuple[str, Any]]] = {name: [] for name in link_to_rec}
        has_parent = set()

        for u, v, mate in _iter_edges(kinematic_graph):
            if not is_joint_mate(mate) or mate is None:
                continue

            p_link_name = root_to_link_node[find(u)]
            c_link_name = root_to_link_node[find(v)]
            if p_link_name == c_link_name:
                continue

            link_tree[p_link_name].append((c_link_name, mate))
            has_parent.add(c_link_name)

        # 2. BFS to propagate transforms
        roots = [name for name in link_to_rec if name not in has_parent]
        queue = list(roots)
        for root_name in roots:
            # Root link frame defaults to identity (Assembly origin)
            link_to_rec[root_name].frame_transform = np.eye(4)

        visited = set(roots)
        while queue:
            curr_name = queue.pop(0)
            curr_link = link_to_rec[curr_name]
            T_WC = curr_link.frame_transform

            for next_name, mate in link_tree[curr_name]:
                if next_name in visited:
                    continue

                # Calculate World transform of joint (mate connector)
                # matedEntities[0] is the parent entity in the toolkit's KinematicGraph
                try:
                    parent_entity = mate.matedEntities[0]
                    # Find the part in cad.parts that matches the occurrence
                    parent_occ = parent_entity.matedOccurrence
                    # We need the world transform of this part
                    # In our case, payloads might already have it if cad was provided
                    parent_part_world = np.eye(4)
                    if cad and hasattr(cad, "parts"):
                        # Search for part with matching occurrence
                        for part in cad.parts.values():
                            if getattr(part, "occurrence", None) == parent_occ:
                                parent_part_world = _part_world_matrix(part)
                                break

                    T_PJ = parent_entity.matedCS.to_tf
                    T_WJ = parent_part_world @ T_PJ
                except (AttributeError, IndexError):
                    T_WJ = T_WC  # Fallback to identity if mate data missing

                # Joint origin is transform from parent link frame to joint frame
                T_origin_mat = np.linalg.inv(T_WC) @ T_WJ
                joint_origin = Origin.from_matrix(T_origin_mat)

                # Store joint
                mate_name = getattr(mate, "name", f"joint_{curr_name}_{next_name}")
                mate_type = getattr(mate, "mateType", "REVOLUTE")
                mate_limits = getattr(mate, "limits", None)

                joint_rec = JointRecord(
                    name=mate_name,
                    joint_type=str(mate_type),
                    parent=curr_name,
                    child=next_name,
                    limits=mate_limits,
                    mate=mate,
                    origin=joint_origin,
                )
                robot.add_edge(curr_name, next_name, joint=joint_rec, data=joint_rec)

                # Set child link frame to joint frame
                link_to_rec[next_name].frame_transform = T_WJ

                visited.add(next_name)
                queue.append(next_name)

        return robot
