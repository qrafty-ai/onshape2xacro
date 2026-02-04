from dataclasses import dataclass, field
from typing import Any, Optional, Tuple

import numpy as np
from loguru import logger
from onshape_robotics_toolkit.models.link import Origin


@dataclass
class InterfaceMateInfo:
    """Information about a mate that crosses a module boundary."""

    owner_module: Optional[Any]  # The subassembly instance where the mate is defined
    parent_part: Any  # Part in the "parent" side of the joint
    child_part: Any  # Part in the "child" side of the joint
    child_module: Optional[Any]  # The module being attached (contains child_part)
    mate: Any


@dataclass
class InterfaceJointInfo:
    """Resolved interface joint information for modular export."""

    parent_link: Any
    child_root_link: Any
    origin: Origin
    axis: Tuple[float, float, float]


@dataclass
class ModuleBoundaryInfo:
    """Results of module boundary detection."""

    # Map of subassembly instance PathKey to set of part PathKeys it contains (directly or indirectly)
    # Root parts are NOT included here, they are in root_parts.
    subassembly_parts: dict[Any, set[Any]] = field(default_factory=dict)

    # Mates that cross boundaries, keyed by mate ID
    interface_mates: dict[str, InterfaceMateInfo] = field(default_factory=dict)

    # Parts that are directly in the root assembly
    root_parts: set[Any] = field(default_factory=set)


def to_path_tuple(occ: Any) -> Tuple[str, ...]:
    """Convert an occurrence or PathKey to a tuple of strings."""
    if occ is None:
        return ()
    if hasattr(occ, "path") and isinstance(occ.path, (list, tuple)):
        return tuple(str(x) for x in occ.path)
    if isinstance(occ, (list, tuple)):
        return tuple(str(x) for x in occ)
    if isinstance(occ, str):
        return (occ,) if occ else ()
    return (str(occ),)


def to_id_tuple(occ: Any) -> Tuple[str, ...]:
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


def occ_match(occ1: Any, occ2: Any) -> bool:
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


def get_direct_module(
    path: Tuple[str, ...], subassembly_keys: set[Tuple[str, ...]]
) -> Optional[Tuple[str, ...]]:
    """
    Find the PathKey of the direct parent subassembly instance for a given occurrence path.
    """
    if not path:
        return None
    for i in range(len(path) - 1, 0, -1):
        prefix = path[:i]
        if prefix in subassembly_keys:
            return prefix
    return None


def detect_module_boundaries(cad: Any, kinematic_graph: Any) -> ModuleBoundaryInfo:
    """
    Detect module boundaries based on Onshape subassembly hierarchy.
    """
    info = ModuleBoundaryInfo()
    subassembly_keys = {to_path_tuple(k) for k in cad.subassemblies.keys()}

    # 1. Map parts to their modules
    part_to_module: dict[Tuple[str, ...], Optional[Tuple[str, ...]]] = {}

    # We iterate over nodes in kinematic_graph because those are the parts we care about
    for node in kinematic_graph.nodes:
        # Convert node to tuple path
        path = to_path_tuple(node)

        module_path = get_direct_module(path, subassembly_keys)
        part_to_module[path] = module_path

        if module_path is None:
            info.root_parts.add(node)
        else:
            # Find the PathKey that matches this module_path
            module_key = next(
                (
                    k
                    for k in cad.subassemblies.keys()
                    if to_path_tuple(k) == module_path
                ),
                None,
            )
            if module_key:
                if module_key not in info.subassembly_parts:
                    info.subassembly_parts[module_key] = set()
                info.subassembly_parts[module_key].add(node)

    # 2. Identify interface mates
    # Build a lookup for mates by ID
    mate_lookup: dict[str, Tuple[Optional[Any], Any]] = {}
    if hasattr(cad, "mates"):
        for (asm_key, _, _), mate in cad.mates.items():
            mate_id = getattr(mate, "id", None)
            if mate_id:
                mate_lookup[mate_id] = (asm_key, mate)

    # Helper to check if a mate is a joint
    def is_joint_mate(mate: Any) -> bool:
        if mate is None:
            return False
        name = getattr(mate, "name", None)
        if name is None and isinstance(mate, dict):
            name = mate.get("name")
        return name is not None and str(name).startswith("joint_")

    for u, v, data in kinematic_graph.edges(data=True):
        mate = data.get("mate") or data.get("data")
        if not is_joint_mate(mate):
            continue

        mate_id = getattr(mate, "id", None)
        if not mate_id or mate_id not in mate_lookup:
            continue

        asm_key, mate_data = mate_lookup[mate_id]

        u_path = to_path_tuple(u)
        v_path = to_path_tuple(v)

        u_module = part_to_module.get(u_path)
        v_module = part_to_module.get(v_path)

        if u_module != v_module:
            # This is an interface mate.
            owner_path = to_path_tuple(asm_key) if asm_key else None

            def is_child(
                mod_path: Optional[Tuple[str, ...]],
                owner_path: Optional[Tuple[str, ...]],
            ) -> bool:
                if mod_path is None:
                    return False
                if owner_path is None:
                    return True
                return len(mod_path) > len(owner_path)

            u_is_child = is_child(u_module, owner_path)
            v_is_child = is_child(v_module, owner_path)

            if u_is_child and not v_is_child:
                child_part, parent_part = u, v
                child_module_path = u_module
            elif v_is_child and not u_is_child:
                child_part, parent_part = v, u
                child_module_path = v_module
            else:
                child_part, parent_part = v, u
                child_module_path = v_module

            child_module_key = next(
                (
                    k
                    for k in cad.subassemblies.keys()
                    if to_path_tuple(k) == child_module_path
                ),
                None,
            )

            info.interface_mates[mate_id] = InterfaceMateInfo(
                owner_module=asm_key,
                parent_part=parent_part,
                child_part=child_part,
                child_module=child_module_key,
                mate=mate_data,
            )

    return info


def calculate_interface_transforms(
    cad: Any, condensed_robot: Any, boundaries: Optional[ModuleBoundaryInfo]
) -> dict[Any, InterfaceJointInfo]:
    """
    Calculate interface joint transforms for modular export.
    """
    if boundaries is None:
        return {}

    link_to_rec = {
        data["link"].name: data["link"] for _, data in condensed_robot.nodes(data=True)
    }
    part_to_link: dict[Any, Any] = {}
    for link_name, link_rec in link_to_rec.items():
        for key in link_rec.keys:
            part_to_link[key] = link_name

    mate_id_to_orig: dict[str, Tuple[Any, Any]] = {}
    if hasattr(cad, "mates") and cad.mates:
        for (_, k1, k2), mate in cad.mates.items():
            mate_id = getattr(mate, "id", None)
            if mate_id:
                mate_id_to_orig[mate_id] = (k1, k2)

    def axis_for_mate(
        mate: Any, mate_id: Optional[str], parent_key: Any
    ) -> Tuple[float, float, float]:
        mate_type = getattr(mate, "mateType", None)
        if mate_type is None and isinstance(mate, dict):
            mate_type = mate.get("mateType")

        if mate_type != "REVOLUTE":
            raise NotImplementedError(f"Mate type {mate_type} not supported")

        sign = -1.0
        if mate_id and mate_id in mate_id_to_orig and parent_key is not None:
            k0, k1 = mate_id_to_orig[mate_id]

            def _get_root(key: Any) -> Any:
                if hasattr(cad, "parts") and key in cad.parts:
                    part = cad.parts[key]
                    root_key = getattr(part, "rigidAssemblyKey", None)
                    if root_key:
                        return root_key
                return key

            r0 = _get_root(k0)
            r1 = _get_root(k1)

            if parent_key == r1:
                sign = -1.0
            elif parent_key == r0:
                sign = 1.0

        return (0.0, 0.0, sign)

    interface_transforms: dict[Any, InterfaceJointInfo] = {}

    for mate_id, info in boundaries.interface_mates.items():
        child_module = info.child_module
        if child_module is None:
            raise RuntimeError("Interface mate is missing a child module")
        if child_module in interface_transforms:
            logger.warning(
                "Multiple interface mates found for child module "
                f"{child_module}, skipping: {mate_id}"
            )
            continue

        parent_link = part_to_link.get(info.parent_part)
        if parent_link is None:
            raise RuntimeError("Parent part not found in condensed robot links")

        child_root_link = part_to_link.get(info.child_part)
        if child_root_link is None:
            raise RuntimeError("Child part not found in condensed robot links")

        mate = info.mate
        if mate is None:
            raise RuntimeError("Interface mate data is missing")

        parent_occ = getattr(info.parent_part, "occurrence", None)
        child_occ = getattr(info.child_part, "occurrence", None)

        parent_entity = None
        parent_entity_occ = None
        entities = getattr(mate, "matedEntities", None)
        if entities:
            parent_candidates: list[tuple[Any, Any]] = []
            for entity in entities:
                entity_occ = getattr(entity, "matedOccurrence", None)
                matched_parent_occ = (
                    parent_occ
                    if parent_occ is not None and occ_match(entity_occ, parent_occ)
                    else None
                )
                matched_child_occ = (
                    child_occ
                    if child_occ is not None and occ_match(entity_occ, child_occ)
                    else None
                )

                if matched_parent_occ and not matched_child_occ:
                    parent_candidates.insert(0, (entity, matched_parent_occ))
                elif matched_parent_occ:
                    parent_candidates.append((entity, matched_parent_occ))

            if parent_candidates:
                parent_entity, parent_entity_occ = parent_candidates[0]
            else:
                parent_entity = entities[0]
                parent_entity_occ = getattr(parent_entity, "matedOccurrence", None)

        if parent_entity is None:
            raise RuntimeError(
                "Could not resolve parent mate entity for interface joint"
            )

        if parent_entity_occ is None:
            parent_entity_occ = parent_occ

        parent_key = None
        if parent_entity_occ is not None and hasattr(cad, "keys_by_id"):
            parent_key = cad.keys_by_id.get(to_id_tuple(parent_entity_occ))

        parent_part_world = None
        if parent_key is not None:
            parent_part_world = cad.get_transform(parent_key)
        if parent_part_world is None and parent_entity_occ is not None:
            parent_part_world = cad.get_transform(to_id_tuple(parent_entity_occ))
        if parent_part_world is None:
            parent_part_world = np.eye(4)
        else:
            if not np.allclose(parent_part_world[3, :], [0, 0, 0, 1]) and np.allclose(
                parent_part_world[:, 3], [0, 0, 0, 1]
            ):
                parent_part_world = parent_part_world.T

        parent_link_world = link_to_rec[parent_link].frame_transform
        if parent_link_world is None:
            parent_link_world = np.eye(4)

        parent_mate_local = parent_entity.matedCS.to_tf
        parent_mate_world = parent_part_world @ parent_mate_local

        T_interface = np.linalg.inv(parent_link_world) @ parent_mate_world
        origin = Origin.from_matrix(T_interface)
        axis = axis_for_mate(mate, mate_id, parent_key)

        interface_transforms[child_module] = InterfaceJointInfo(
            parent_link=parent_link,
            child_root_link=child_root_link,
            origin=origin,
            axis=axis,
        )

    return interface_transforms
