from typing import Dict, Tuple, Optional, Union
from pydantic import BaseModel
from onshape_robotics_toolkit.models.assembly import (
    PartInstance,
    AssemblyInstance,
    Occurrence,
    SubAssembly,
    MateFeatureData,
    Pattern,
    Part,
)


class PathKeyModel(BaseModel):
    path: Tuple[str, ...]
    name_path: Tuple[str, ...]


class SerializableCAD(BaseModel):
    document_id: str
    element_id: str
    wtype: str
    workspace_id: Optional[str] = None
    document_microversion: Optional[str] = None
    name: str
    max_depth: int

    # Store dictionaries where keys are strings (the PathKey.path joined by "/")
    keys_by_id: Dict[str, PathKeyModel]
    keys_by_name: Dict[str, PathKeyModel]
    instances: Dict[str, Union[PartInstance, AssemblyInstance]]
    occurrences: Dict[str, Occurrence]
    subassemblies: Dict[str, SubAssembly]
    mates: Dict[str, MateFeatureData]
    patterns: Dict[str, Pattern]
    parts: Dict[str, Part]
