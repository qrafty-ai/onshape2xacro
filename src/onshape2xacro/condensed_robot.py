import hashlib
import logging
import numpy as np
from dataclasses import dataclass
from typing import List, Any, Dict, Iterable, Tuple, cast, Optional
from collections.abc import Iterable as IterableABC
from onshape_robotics_toolkit.robot import Robot
from onshape_robotics_toolkit.models.link import Origin
from onshape2xacro.naming import sanitize_name

logger = logging.getLogger(__name__)


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
    mat = np.eye(4)
    if part_tf is not None:
        tf_value = getattr(part_tf, "to_tf", None)
        if callable(tf_value):
            mat = cast(np.ndarray, tf_value())
        elif tf_value is not None:
            mat = cast(np.ndarray, tf_value)

    # Check for transpose (Onshape often returns column-major which numpy reads as transposed row-major if not reshaped with order='F')
    # Standard rigid transform has [0,0,0,1] as last row.
    # If last column is [0,0,0,1] and last row is not, it's transposed.
    if not np.allclose(mat[3, :], [0, 0, 0, 1]) and np.allclose(
        mat[:, 3], [0, 0, 0, 1]
    ):
        return mat.T
    return mat


def occ_match(occ1, occ2):
    """
    Check if two Onshape occurrences match by checking their common suffix.
    Handles PathKey, strings (space or / separated), lists, and tuples.
    """
    if occ1 is None or occ2 is None:
        return False

    def to_id_tuple(occ) -> tuple[str, ...]:
        if hasattr(occ, "path") and isinstance(occ.path, (list, tuple)):
            return tuple(str(x) for x in occ.path)
        if isinstance(occ, (list, tuple)):
            return tuple(str(x) for x in occ)
        if isinstance(occ, str):
            if " " in occ:
                return tuple(occ.split(" "))
            if "/" in occ:
                return tuple(occ.split("/"))
            return (occ,) if occ else ()
        return (str(occ),)

    t1 = to_id_tuple(occ1)
    t2 = to_id_tuple(occ2)

    n1, n2 = len(t1), len(t2)
    min_n = min(n1, n2)
    if min_n == 0:
        return n1 == n2

    return t1[-min_n:] == t2[-min_n:]


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

            # Pick link name based on heaviest part
            max_mass = None
            heaviest_name = part_names[0] if part_names else ""
            for p, name in zip(payloads, part_names):
                mp = getattr(p, "mass_properties", None)
                if mp is None:
                    continue
                mass = getattr(mp, "mass", None)
                if mass is None:
                    continue
                try:
                    mass_value = float(mass)
                except (TypeError, ValueError):
                    continue
                if max_mass is None or mass_value > max_mass:
                    max_mass = mass_value
                    heaviest_name = name

            link_name = sanitize_name(heaviest_name)
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
        for root_name in roots:
            # Root link frame defaults to identity (Assembly origin)
            # Try to find representative part frame for root too
            root_link_frame = np.eye(4)
            if cad:
                link_rec = link_to_rec[root_name]
                for key, occ in zip(link_rec.keys, link_rec.occurrences):
                    found = False
                    res_occ, res_key = occ, key
                    if isinstance(occ, str) and hasattr(cad, "occurrences"):
                        for k in cad.occurrences:
                            if str(k) == occ:
                                res_occ = k
                                break
                    if isinstance(key, str) and hasattr(cad, "parts"):
                        for k in cad.parts:
                            if str(k) == key:
                                res_key = k
                                break

                    if hasattr(cad, "get_transform"):
                        tf = cad.get_transform(res_occ)
                        if tf is not None:
                            if not np.allclose(tf[3, :], [0, 0, 0, 1]) and np.allclose(
                                tf[:, 3], [0, 0, 0, 1]
                            ):
                                tf = tf.T
                            root_link_frame, found = tf, True

                    if not found and hasattr(cad, "parts"):
                        if res_key in cad.parts:
                            root_link_frame, found = (
                                _part_world_matrix(cad.parts[res_key]),
                                True,
                            )
                        if not found:
                            for p_key, p_part in cad.parts.items():
                                if occ_match(p_key.path, occ) or occ_match(
                                    getattr(p_key, "name_path", []), occ
                                ):
                                    root_link_frame, found = (
                                        _part_world_matrix(p_part),
                                        True,
                                    )
                                    break
                    if found:
                        break
            link_to_rec[root_name].frame_transform = root_link_frame

        queue = list(roots)
        visited = set(roots)
        while queue:
            curr_name = queue.pop(0)
            curr_link = link_to_rec[curr_name]
            T_WC = curr_link.frame_transform

            for next_name, mate in link_tree[curr_name]:
                if next_name in visited:
                    continue

                # Calculate World transform of joint (mate connector)
                # Choose the mate entity that belongs to the current (parent) link's occurrences
                try:
                    parent_entity = None
                    matched_world_occ = None
                    child_link = link_to_rec.get(next_name)
                    parent_occs = getattr(curr_link, "occurrences", None) or []
                    child_occs = getattr(child_link, "occurrences", None) or []
                    if parent_occs or child_occs:
                        parent_candidates = []
                        for entity in mate.matedEntities:
                            entity_occ = getattr(entity, "matedOccurrence", None)
                            parent_match = any(
                                occ_match(entity_occ, occ) for occ in parent_occs
                            )
                            child_match = any(
                                occ_match(entity_occ, occ) for occ in child_occs
                            )
                            if parent_match and not child_match:
                                # Strongest candidate: in parent link but not in child link
                                parent_candidates.insert(0, (entity, entity_occ))
                            elif parent_match:
                                # Second best: in parent link
                                parent_candidates.append((entity, entity_occ))

                        if parent_candidates:
                            parent_entity, entity_occ = parent_candidates[0]
                            for occ in parent_occs:
                                if occ_match(entity_occ, occ):
                                    matched_world_occ = occ
                                    break

                    if parent_entity is None:
                        # Fallback to first entity if matching fails
                        parent_entity = mate.matedEntities[0]

                    # Always use the mate entity's occurrence for finding the part transform
                    # This is the actual part the mate connector is attached to
                    parent_occ = getattr(
                        parent_entity, "matedOccurrence", matched_world_occ
                    )

                    # Resolve to a PathKey when possible (cad.parts keys are PathKey)
                    parent_key = None
                    if parent_occ is not None:
                        if hasattr(parent_occ, "path") and hasattr(
                            parent_occ, "name_path"
                        ):
                            parent_key = parent_occ
                        elif (
                            isinstance(parent_occ, (list, tuple))
                            and cad
                            and hasattr(cad, "keys_by_id")
                        ):
                            parent_key = cad.keys_by_id.get(tuple(parent_occ))
                        elif (
                            isinstance(parent_occ, str)
                            and cad
                            and hasattr(cad, "keys_by_name")
                        ):
                            for key in cad.keys_by_name.values():
                                if "_".join(key.name_path) == parent_occ:
                                    parent_key = key
                                    break

                    if parent_key is None and cad and hasattr(cad, "keys_by_name"):
                        for key in cad.keys_by_name.values():
                            if occ_match(key.path, parent_occ) or occ_match(
                                key.name_path, parent_occ
                            ):
                                parent_key = key
                                break

                    # We need the world transform of this part
                    parent_part_world = np.eye(4)
                    found_part = False
                    if cad and hasattr(cad, "get_transform"):
                        try:
                            parent_part_world = cad.get_transform(
                                parent_key or parent_occ
                            )
                            # Normalize
                            if not np.allclose(
                                parent_part_world[3, :], [0, 0, 0, 1]
                            ) and np.allclose(parent_part_world[:, 3], [0, 0, 0, 1]):
                                parent_part_world = parent_part_world.T
                            found_part = True
                        except Exception:
                            found_part = False

                    if not found_part and cad and hasattr(cad, "parts"):
                        # Search for part with matching occurrence (cad.parts keyed by PathKey)
                        if parent_key is not None and parent_key in cad.parts:
                            parent_part_world = _part_world_matrix(
                                cad.parts[parent_key]
                            )
                            found_part = True
                        else:
                            for key, part in cad.parts.items():
                                if occ_match(key.path, parent_occ) or occ_match(
                                    key.name_path, parent_occ
                                ):
                                    parent_part_world = _part_world_matrix(part)
                                    found_part = True
                                    break

                    if not found_part and cad:
                        logger.warning(
                            f"Could not find part for occurrence {parent_occ} "
                            f"(joint: {getattr(mate, 'name', 'unknown')}, links: {curr_name} -> {next_name})"
                        )

                    T_PJ = parent_entity.matedCS.to_tf
                    T_WJ = parent_part_world @ T_PJ
                except (AttributeError, IndexError):
                    T_WJ = T_WC  # Fallback to identity if mate data missing

                # =============================================================
                # KINEMATIC CHAIN APPROACH:
                # Child link frame = Joint world frame (T_WJ)
                # This ensures joint origins form a proper kinematic chain:
                #   joint_origin = inv(T_parent_link) @ T_WJ
                # Meshes are "baked" into link frame by StepMeshExporter.
                # =============================================================

                mate_name = getattr(mate, "name", f"joint_{curr_name}_{next_name}")

                # Child link frame IS the joint world frame
                # This is kinematically correct: the child link origin is at the joint axis
                T_target_child = T_WJ

                # Joint origin = transform from parent link frame to joint world frame
                T_origin_mat = np.linalg.inv(T_WC) @ T_WJ
                joint_origin = Origin.from_matrix(T_origin_mat)

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

                # Set child link frame
                link_to_rec[next_name].frame_transform = T_target_child

                visited.add(next_name)
                queue.append(next_name)

        return robot
