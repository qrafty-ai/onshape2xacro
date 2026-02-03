import asyncio
from functools import lru_cache
from typing import Optional, List, Any
from loguru import logger

from onshape_robotics_toolkit import CAD, Client
from onshape_robotics_toolkit.models.assembly import (
    RootAssembly,
    SubAssembly,
    MateFeatureData,
    MateType,
    Part,
    BodyType,
)
from onshape_robotics_toolkit.models.document import (
    WorkspaceType,
    parse_url,
    Document,
    DocumentMetaData,
)
from onshape_robotics_toolkit.parse import PathKey
from onshape_robotics_toolkit.utilities.helpers import (
    get_sanitized_name,
    parse_onshape_expression,
)
from onshape_robotics_toolkit.config import update_mate_limits


class OptimizedClient(Client):
    """
    Optimized version of Client class with cached metadata fetches.
    """

    @lru_cache(maxsize=128)
    def get_document_metadata(self, did: str) -> DocumentMetaData:
        logger.debug(f"Fetching document metadata for {did} (cached)")
        return super().get_document_metadata(did)


class OptimizedCAD(CAD):
    """
    Optimized version of CAD class with deduplicated API calls.
    """

    @classmethod
    def from_url(
        cls,
        url: str,
        *,
        client: Client,
        max_depth: int = 0,
        configuration: str = "default",
        log_response: bool = True,
        with_meta_data: bool = True,
        fetch_mass_properties: bool = True,
        fetch_mate_properties: bool = True,
    ) -> "CAD":
        """
        Create a CAD instance directly from an Onshape document URL.
        Exposes fetch_mass_properties and fetch_mate_properties control.
        """
        base_url, did, wtype, wid, eid = parse_url(url)
        if client is None:
            raise ValueError("client must be provided to load CAD from URL")

        document = Document(
            base_url=base_url, did=did, wtype=wtype, wid=wid, eid=eid, url=url
        )

        assembly = client.get_assembly(
            document.did,
            document.wtype,
            document.wid,
            document.eid,
            configuration=configuration,
            log_response=log_response,
            with_meta_data=with_meta_data,
        )

        return cls.from_assembly(
            assembly,
            max_depth=max_depth,
            client=client,
            fetch_mass_properties=fetch_mass_properties,
            fetch_mate_properties=fetch_mate_properties,
        )

    async def fetch_occurrences_for_subassemblies(self, client: Client) -> None:
        async def _fetch_and_populate(
            definition_id: tuple[str, str, str, str],
            instances: list[tuple[PathKey, SubAssembly]],
            client: Client,
        ) -> None:
            did, wtype, wid, eid = definition_id
            try:
                logger.debug(
                    f"Fetching root assembly for subassembly definition: {did}/{wid}/{eid} (used by {len(instances)} instances)"
                )

                _subassembly_data: RootAssembly = await asyncio.to_thread(
                    client.get_root_assembly,
                    did=did,
                    wtype=wtype,
                    wid=wid,
                    eid=eid,
                    with_mass_properties=False,
                    log_response=False,
                )
                _subassembly_occurrences = _subassembly_data.occurrences

                for key, subassembly in instances:
                    for occurrence in _subassembly_occurrences:
                        path_tuple = tuple(key.path) + tuple(occurrence.path)
                        occ_key = self.keys_by_id.get(path_tuple)
                        if occ_key:
                            if subassembly.RootOccurrences is None:
                                subassembly.RootOccurrences = {}
                            subassembly.RootOccurrences[occ_key] = occurrence
                        else:
                            is_suppressed_parent = False
                            for i in range(len(path_tuple), 0, -1):
                                partial_path = path_tuple[:i]
                                partial_key = self.keys_by_id.get(partial_path)
                                if (
                                    partial_key
                                    and self.instances.get(partial_key, None)
                                    and self.instances[partial_key].suppressed
                                ):
                                    is_suppressed_parent = True
                                    break

                            if not is_suppressed_parent:
                                logger.warning(
                                    f"No PathKey for subassembly occurrence {occurrence.path} in {key}"
                                )

            except Exception as e:
                logger.error(
                    f"Failed to fetch root assembly for subassembly definition {definition_id}: {e}"
                )

        instances_by_def: dict[
            tuple[str, str, str, str], list[tuple[PathKey, SubAssembly]]
        ] = {}

        for key, subassembly in self.subassemblies.items():
            if self.instances[key].suppressed:
                continue

            if subassembly.RootOccurrences is not None:
                logger.debug(f"Subassembly {key} already has RootOccurrences, skipping")
                continue

            if not subassembly.isRigid:
                continue

            def_id = (
                subassembly.documentId,
                WorkspaceType.M.value,
                subassembly.documentMicroversion,
                subassembly.elementId,
            )
            instances_by_def.setdefault(def_id, []).append((key, subassembly))

        tasks = []
        for def_id, instances in instances_by_def.items():
            tasks.append(_fetch_and_populate(def_id, instances, client))

        if tasks:
            await asyncio.gather(*tasks, return_exceptions=True)

    def fetch_mate_limits(self, client: Optional[Client]) -> None:
        """
        Fetch joint limits from Onshape features and populate mate data.
        Optimized to fetch features once per unique subassembly definition.
        """
        if client is None:
            logger.warning("No client provided for fetching mate limits, skipping")
            return

        logger.info("Fetching mate limits from assembly features")

        assembly_groups: dict[tuple[str, str, str, str], list[Optional[PathKey]]] = {}

        root_def = (self.document_id, self.wtype, self.workspace_id, self.element_id)
        assembly_groups.setdefault(root_def, []).append(None)

        for sub_key, subassembly in self.subassemblies.items():
            if self.instances[sub_key].suppressed:
                continue

            if not subassembly.isRigid:
                def_id = (
                    subassembly.documentId,
                    WorkspaceType.M.value,
                    subassembly.documentMicroversion,
                    subassembly.elementId,
                )
                assembly_groups.setdefault(def_id, []).append(sub_key)

        mate_lookup: dict[tuple[Optional[PathKey], str], list[MateFeatureData]] = {}
        for (asm_key, _, _), mate in self.mates.items():
            if mate.id:
                mate_lookup.setdefault((asm_key, mate.id), []).append(mate)

        limits_found_count = 0
        for (did, wtype, wid, eid), assembly_keys in assembly_groups.items():
            try:
                logger.debug(
                    f"Fetching features for assembly definition: {did}/{wid}/{eid} (used by {len(assembly_keys)} instances)"
                )
                features = client.get_features(did=did, wtype=wtype, wid=wid, eid=eid)

                for feature in features.features:
                    if feature.message.featureType != "mate":
                        continue

                    feature_id = feature.message.featureId

                    params = feature.message.parameter_dict()
                    limits_enabled = params.get("limitsEnabled")
                    if limits_enabled is None or not limits_enabled.get(
                        "message", {}
                    ).get("value", False):
                        continue

                    target_mates: List[MateFeatureData] = []
                    for assembly_key in assembly_keys:
                        found_mates = mate_lookup.get((assembly_key, feature_id))
                        if found_mates:
                            target_mates.extend(found_mates)

                    if not target_mates:
                        continue

                    first_mate = target_mates[0]
                    is_axial = first_mate.mateType in (
                        MateType.REVOLUTE,
                        MateType.CYLINDRICAL,
                    )

                    if is_axial:
                        min_param_name = "limitAxialZMin"
                        max_param_name = "limitAxialZMax"
                    else:
                        min_param_name = "limitZMin"
                        max_param_name = "limitZMax"

                    def get_param_expression(
                        param_dict: dict[str, Any], param_name: str
                    ) -> str | None:
                        param = param_dict.get(param_name)
                        if not isinstance(param, dict):
                            return None
                        if param.get("typeName") == "BTMParameterNullableQuantity":
                            message = param.get("message", {})
                            if not isinstance(message, dict) or message.get(
                                "isNull", True
                            ):
                                return None
                            expression = message.get("expression")
                            return expression if isinstance(expression, str) else None
                        return None

                    min_expression = get_param_expression(params, min_param_name)
                    max_expression = get_param_expression(params, max_param_name)

                    min_value = parse_onshape_expression(min_expression)
                    max_value = parse_onshape_expression(max_expression)

                    if min_value is not None and max_value is not None:
                        limits = {"min": min_value, "max": max_value}

                        for mate in target_mates:
                            mate.limits = limits
                            sanitized_mate_name = get_sanitized_name(mate.name)
                            update_mate_limits(sanitized_mate_name, limits)
                            limits_found_count += 1
                            logger.debug(
                                f"Set limits for mate '{mate.name}' ({mate.mateType}): "
                                f"min={min_value:.4f}, max={max_value:.4f}"
                            )

            except Exception as e:
                logger.warning(
                    f"Failed to fetch features for assembly definition {did}/{wid}/{eid}: {e}"
                )
                continue

        logger.info(
            f"Fetched limits for {limits_found_count} mates out of {len(self.mates)} total mates"
        )

    async def fetch_mass_properties_for_parts(self, client: Client) -> None:
        """
        Fetch mass properties for parts and rigid assemblies.
        Optimized to fetch once per unique Part/Configuration definition.
        """
        parts_by_def: dict[tuple[str, str, str, str, str], list[Part]] = {}

        for key, part in self.parts.items():
            if part.bodyType == BodyType.SHEET.value:
                continue
            if self.instances[key].suppressed:
                continue
            if part.MassProperty is not None:
                continue
            if part.rigidAssemblyToPartTF is not None:
                continue

            if part.isRigidAssembly:
                def_id = (
                    part.documentId,
                    WorkspaceType.W.value,
                    part.rigidAssemblyWorkspaceId or "",
                    part.elementId,
                    "rigid_assembly",
                )
            else:
                def_id = (
                    part.documentId,
                    WorkspaceType.M.value,
                    part.documentMicroversion
                    if part.documentMicroversion
                    else self.document_microversion,
                    part.elementId,
                    part.partId,
                )

            parts_by_def.setdefault(def_id, []).append(part)

        async def _fetch_and_assign(
            def_id: tuple[str, str, str, str, str], parts: list[Part], client: Client
        ) -> None:
            did, wtype, wid, eid, partID = def_id
            try:
                logger.debug(
                    f"Fetching mass properties for definition {def_id} (used by {len(parts)} parts)"
                )
                mass_prop = None

                if partID == "rigid_assembly":
                    mass_prop = await asyncio.to_thread(
                        client.get_assembly_mass_properties,
                        did=did,
                        wtype=wtype,
                        wid=wid,
                        eid=eid,
                    )
                else:
                    mass_prop = await asyncio.to_thread(
                        client.get_mass_property,
                        did=did,
                        wtype=wtype,
                        wid=wid,
                        eid=eid,
                        partID=partID,
                    )

                for part in parts:
                    part.MassProperty = mass_prop

            except Exception as e:
                logger.error(f"Failed to fetch mass properties for {def_id}: {e}")

        tasks = []
        for def_id, parts in parts_by_def.items():
            tasks.append(_fetch_and_assign(def_id, parts, client))

        if tasks:
            await asyncio.gather(*tasks, return_exceptions=True)
