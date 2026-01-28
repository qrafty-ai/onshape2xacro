from typing import Optional, Tuple
from onshape_robotics_toolkit.parse import CAD, PathKey
from onshape2xacro.models import SerializableCAD, PathKeyModel


def cad_to_serializable(cad: CAD) -> SerializableCAD:
    """
    Convert an ORT CAD object to a SerializableCAD model.
    """

    def pk_to_str(pk: PathKey) -> str:
        return "/".join(pk._path)

    def tuple_to_str(t: Tuple[str, ...]) -> str:
        return "/".join(t)

    keys_by_id = {
        tuple_to_str(k): PathKeyModel(path=v._path, name_path=v._name_path)
        for k, v in cad.keys_by_id.items()
    }
    keys_by_name = {
        tuple_to_str(k): PathKeyModel(path=v._path, name_path=v._name_path)
        for k, v in cad.keys_by_name.items()
    }

    instances = {pk_to_str(k): v for k, v in cad.instances.items()}
    occurrences = {pk_to_str(k): v for k, v in cad.occurrences.items()}
    subassemblies = {pk_to_str(k): v for k, v in cad.subassemblies.items()}

    # Mates keys are (Optional[PathKey], PathKey, PathKey)
    def mate_key_to_str(k: Tuple[Optional[PathKey], PathKey, PathKey]) -> str:
        parts = []
        for pk in k:
            if pk is None:
                parts.append("")
            else:
                parts.append(pk_to_str(pk))
        return "::".join(parts)

    mates = {mate_key_to_str(k): v for k, v in cad.mates.items()}
    patterns = cad.patterns
    parts = {pk_to_str(k): v for k, v in cad.parts.items()}

    return SerializableCAD(
        document_id=cad.document_id,
        element_id=cad.element_id,
        wtype=cad.wtype,
        workspace_id=cad.workspace_id,
        document_microversion=cad.document_microversion,
        name=cad.name or "",
        max_depth=cad.max_depth,
        keys_by_id=keys_by_id,
        keys_by_name=keys_by_name,
        instances=instances,
        occurrences=occurrences,
        subassemblies=subassemblies,
        mates=mates,
        patterns=patterns,
        parts=parts,
    )


def serializable_to_cad(scad: SerializableCAD) -> CAD:
    """
    Convert a SerializableCAD model back to an ORT CAD object.
    """

    def to_pk(pkm: PathKeyModel) -> PathKey:
        return PathKey(pkm.path, pkm.name_path)

    keys_by_id = {
        tuple(k.split("/")) if k else (): to_pk(v) for k, v in scad.keys_by_id.items()
    }
    keys_by_name = {
        tuple(k.split("/")) if k else (): to_pk(v) for k, v in scad.keys_by_name.items()
    }

    # Collect all PathKeys to reconstruct mappings
    all_pks = list(keys_by_id.values()) + list(keys_by_name.values())
    path_to_pk = {"/".join(pk._path): pk for pk in all_pks}

    def from_path(p: str) -> PathKey:
        if p not in path_to_pk:
            # Fallback: create a PathKey if not found (though it should be there)
            # We don't have the name_path here, so we use path parts as names
            path_parts = tuple(p.split("/")) if p else ()
            return PathKey(path_parts, path_parts)
        return path_to_pk[p]

    instances = {from_path(k): v for k, v in scad.instances.items()}
    occurrences = {from_path(k): v for k, v in scad.occurrences.items()}
    subassemblies = {from_path(k): v for k, v in scad.subassemblies.items()}

    def str_to_mate_key(s: str) -> Tuple[Optional[PathKey], PathKey, PathKey]:
        parts = s.split("::")
        res = []
        for p in parts:
            if p == "":
                res.append(None)
            else:
                res.append(from_path(p))
        return tuple(res)  # type: ignore

    mates = {str_to_mate_key(k): v for k, v in scad.mates.items()}
    patterns = scad.patterns
    parts = {from_path(k): v for k, v in scad.parts.items()}

    cad = CAD(
        document_id=scad.document_id,
        element_id=scad.element_id,
        wtype=scad.wtype,
        workspace_id=scad.workspace_id or "",
        document_microversion=scad.document_microversion or "",
        name=scad.name,
        max_depth=scad.max_depth,
    )
    cad.keys_by_id = keys_by_id
    cad.keys_by_name = keys_by_name
    cad.instances = instances
    cad.occurrences = occurrences
    cad.subassemblies = subassemblies
    cad.mates = mates
    cad.patterns = patterns
    cad.parts = parts

    return cad
