import hashlib
import logging
import numpy as np
from dataclasses import dataclass
from typing import List, Any, Dict, Iterable, Tuple, cast, Optional
from collections.abc import Iterable as IterableABC
from onshape_robotics_toolkit.models.assembly import MatedEntity
from onshape_robotics_toolkit.robot import Robot
from onshape_robotics_toolkit.models.link import Origin
from onshape_robotics_toolkit.parse import CAD, MateFeatureData
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
    axis: Tuple[float, float, float]
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


def to_id_tuple(occ) -> tuple[str, ...]:
    if occ is None:
        return ()
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


def occ_match(occ1, occ2):
    """
    Check if two Onshape occurrences match by checking their common suffix.
    Handles PathKey, strings (space or / separated), lists, and tuples.
    """
    if occ1 is None or occ2 is None:
        return False

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
    cad: CAD = None
    asset_path: Any = None

    @classmethod
    def from_graph(
        cls,
        kinematic_graph,
        cad: CAD,
        name="robot",
        mate_values: Optional[Dict[str, Any]] = None,
        fail_fast: bool = True,
    ):
        """
        Create a condensed robot from a kinematic graph.
        Groups all nodes connected by non-joint edges into single links.

        Args:
            mate_values: Dictionary of mate values keyed by featureId.
            fail_fast: If True, raise exception on missing part transforms instead of warning.
        """
        if mate_values is None:
            mate_values = {}

        # Build lookup for original mate entities to determine joint direction
        mate_id_to_orig = {}
        if hasattr(cad, "mates") and cad.mates:
            for (asm_key, k1, k2), m in cad.mates.items():
                if m.id:
                    mate_id_to_orig[m.id] = (k1, k2)

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
        link_to_rec: dict[str, LinkRecord] = {
            data["link"].name: data["link"] for _, data in robot.nodes(data=True)
        }

        # We need to know which link is parent and which is child based on joints
        # LinkGraph: parent_link_name -> list of (child_link_name, mate)
        link_tree: dict[str, list[tuple[str, MateFeatureData]]] = {
            name: [] for name in link_to_rec
        }
        has_parent = set()

        for u, v, mate in _iter_edges(kinematic_graph):
            if not is_joint_mate(mate) or mate is None:
                continue

            p_link_name = root_to_link_node[find(u)]
            c_link_name = root_to_link_node[find(v)]
            if p_link_name == c_link_name:
                continue

            # print(f"Edge {u}->{v} mapped to Link {p_link_name}->{c_link_name}")
            link_tree[p_link_name].append((c_link_name, mate))
            has_parent.add(c_link_name)

        # 2. BFS to propagate transforms
        roots = [name for name in link_to_rec if name not in has_parent]
        for root_name in roots:
            # Root link frame defaults to identity (Assembly origin)

            # Try to find representative part frame for root too
            root_link_frame = np.eye(4)
            link_rec = link_to_rec[root_name]
            for _, occ in zip(link_rec.keys, link_rec.occurrences):
                found = False

                tf = cad.get_transform(occ)
                if tf is not None:
                    if not np.allclose(tf[3, :], [0, 0, 0, 1]) and np.allclose(
                        tf[:, 3], [0, 0, 0, 1]
                    ):
                        tf = tf.T
                    root_link_frame, found = tf, True

                if found:
                    break
            link_to_rec[root_name].frame_transform = root_link_frame

        queue = list(roots)
        visited = set(roots)
        if len(queue) > 1:
            raise RuntimeError(
                f"Multiple root links found: {roots}. Robot must have a single root."
            )
        while queue:
            curr_name = queue.pop(0)
            curr_link = link_to_rec[curr_name]
            T_WC = curr_link.frame_transform

            for next_name, mate in link_tree[curr_name]:
                if next_name in visited:
                    continue

                mate_name = mate.name
                mate_type = mate.mateType
                mate_id = mate.id

                # Calculate World transform of joint (mate connector)
                # Choose the mate entity that belongs to the current (parent) link's occurrences

                parent_entity = None
                child_link = link_to_rec.get(next_name)
                parent_occs = curr_link.occurrences
                child_occs = child_link.occurrences if child_link else []
                parent_occ = None

                if parent_occs or child_occs:
                    parent_candidates: list[tuple[MatedEntity, list[str]]] = []
                    for entity in mate.matedEntities:
                        entity_occ = entity.matedOccurrence

                        matched_parent_occ = next(
                            (occ for occ in parent_occs if occ_match(entity_occ, occ)),
                            None,
                        )
                        matched_child_occ = next(
                            (occ for occ in child_occs if occ_match(entity_occ, occ)),
                            None,
                        )

                        if matched_parent_occ and not matched_child_occ:
                            parent_candidates.insert(0, (entity, matched_parent_occ))
                        elif matched_parent_occ:
                            parent_candidates.append((entity, matched_parent_occ))

                    if parent_candidates:
                        parent_entity, parent_occ = parent_candidates[0]
                    elif mate.matedEntities:
                        parent_entity = mate.matedEntities[0]
                        parent_occ = parent_entity.matedOccurrence
                        for occ in parent_occs:
                            if occ_match(parent_occ, occ):
                                parent_occ = occ
                                break

                if parent_entity is None:
                    logger.warning(
                        f"Could not find mated entity for joint {mate_name}. Skipping."
                    )
                    continue

                if parent_occ is None:
                    parent_occ = parent_entity.matedOccurrence

                parent_key = cad.keys_by_id.get(to_id_tuple(parent_occ))

                # We need the world transform of this part
                found_part = False
                parent_part_world = cad.get_transform(parent_key)
                if parent_part_world is not None:
                    # Normalize
                    if not np.allclose(
                        parent_part_world[3, :], [0, 0, 0, 1]
                    ) and np.allclose(parent_part_world[:, 3], [0, 0, 0, 1]):
                        parent_part_world = parent_part_world.T
                    found_part = True
                else:
                    parent_part_world = np.eye(4)

                if not found_part:
                    msg = (
                        f"Could not find part for occurrence {parent_occ} "
                        f"(joint: {mate_name}, links: {curr_name} -> {next_name})"
                    )
                    if fail_fast:
                        raise RuntimeError(msg)
                    logger.warning(msg)

                T_PJ = parent_entity.matedCS.to_tf
                T_WJ = parent_part_world @ T_PJ

                # Compensation for Non-Zero Configuration:
                # 1. Child link frame = Joint world frame (T_WJ) * Correction
                # 2. Joint origin = inv(T_parent_link) @ T_WJ (always relative to zero pose)
                #
                # If the robot is in a non-zero configuration (angle theta), the mesh
                # is "baked" at that angle. To fix this, we rotate the child link frame
                # by theta.
                # T_mesh_visual = inv(T_child_frame) @ T_part_world
                #               = inv(T_WJ @ T_mate) @ (T_WJ @ T_mate @ T_part_local)
                #               = inv(T_mate) @ inv(T_WJ) @ T_WJ @ T_mate @ T_part_local
                #               = T_part_local (Canonical/Zero Pose)

                # Calculate correction transform based on current mate values
                T_mate_correction = np.eye(4)

                # Determine correction sign based on whether the parent entity is the
                # first or second entity in the mate definition.

                if mate_id and mate_id in mate_values:
                    values = mate_values[mate_id]
                    if values is None:
                        raise RuntimeError(f"Mate values for mate ID {mate_id} is None")
                    # Onshape mates align Z axis for primary motion
                    if mate_type == "REVOLUTE":
                        # Default to -1.0 if we can't determine direction
                        sign = -1.0

                        if mate_id in mate_id_to_orig:
                            k0, k1 = mate_id_to_orig[mate_id]
                            # Check if parent matches Original Entity 1 (Fixed/Base) -> Sign +1
                            # Check if parent matches Original Entity 0 (Mover) -> Sign -1

                            def _get_root(k):
                                if hasattr(cad, "parts") and k in cad.parts:
                                    p = cad.parts[k]
                                    if getattr(p, "rigidAssemblyKey", None):
                                        return p.rigidAssemblyKey
                                return k

                            r0 = _get_root(k0)
                            r1 = _get_root(k1)

                            if parent_key == r1:
                                sign = -1.0
                            elif parent_key == r0:
                                sign = 1.0

                        axis = (0.0, 0.0, sign)
                        angle = values.get("rotationZ", 0.0) * sign
                        # print(f"Joint {mate_name} axis set to {axis}")

                        if abs(angle) > 1e-6:
                            c, s = np.cos(angle), np.sin(angle)
                            T_mate_correction[:3, :3] = np.array(
                                [[c, -s, 0], [s, c, 0], [0, 0, 1]]
                            )
                    elif mate_type == "PRISMATIC":
                        raise NotImplementedError(
                            "PRISMATIC mates not yet implemented."
                        )
                        # dist = values.get("translationZ", 0.0) * sign
                        # if abs(dist) > 1e-6:
                        #     T_mate_correction[2, 3] = dist
                    else:
                        raise NotImplementedError(
                            f"Mate type {mate_type} not supported"
                        )
                else:
                    raise RuntimeError(
                        f"Mate values for mate ID {mate_id} not provided for joint {mate_name}"
                    )

                # Child link frame adjusted for current mate value
                T_target_child = T_WJ @ T_mate_correction

                # Joint origin = transform from parent link frame to joint world frame (Zero Pose)
                # We use T_WJ (which represents the joint frame at zero pose on the parent side)
                if T_WC is None:
                    T_WC = np.eye(4)
                T_origin_mat = np.linalg.inv(T_WC) @ T_WJ

                joint_origin = Origin.from_matrix(T_origin_mat)

                mate_limits = getattr(mate, "limits", None)

                joint_rec = JointRecord(
                    name=mate_name,
                    joint_type=str(mate_type),
                    parent=curr_name,
                    child=next_name,
                    limits=mate_limits,
                    mate=mate,
                    origin=joint_origin,
                    axis=axis,
                )
                robot.add_edge(curr_name, next_name, joint=joint_rec, data=joint_rec)

                # Set child link frame
                link_to_rec[next_name].frame_transform = T_target_child

                visited.add(next_name)
                queue.append(next_name)

        return robot
