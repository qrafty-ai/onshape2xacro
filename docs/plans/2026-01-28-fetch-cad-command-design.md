# [Fetch-CAD] Implementation Plan

> **For Claude:** REQUIRED SUB-SKILL: Use superpowers:executing-plans to implement this plan task-by-task.

**Goal:** Create a new CLI command `fetch-cad` that downloads Onshape assembly data (CAD object) and saves it to a Pydantic-serializable JSON file, enabling API-efficient development by allowing local reloading of CAD data.

**Architecture:**
- **Model Layer**: `src/onshape2xacro/models.py` defines Pydantic models for `CAD` and its components (wrapping `onshape-robotics-toolkit` models).
- **Utility Layer**: `src/onshape2xacro/cad_utils.py` handles conversion between `onshape-robotics-toolkit.CAD` and our Pydantic `SerializableCAD`.
- **CLI Layer**: New `fetch-cad` subcommand in `onshape2xacro` CLI.

**Tech Stack:** Python 3.12, Pydantic v2, onshape-robotics-toolkit.

---

### Task 1: Create Serializable Models

**Files:**
- Create: `src/onshape2xacro/models.py`

**Step 1: Define SerializableCAD and helper models**
We need to handle `PathKey` (dataclass) and various `onshape-robotics-toolkit` sub-models.

```python
from typing import Dict, List, Tuple, Optional, Union
from pydantic import BaseModel
from onshape_robotics_toolkit.models.assembly import (
    PartInstance, AssemblyInstance, Occurrence, SubAssembly,
    MateFeatureData, Pattern, Part
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
    instances: Dict[str, Union[PartInstance, AssemblyInstance]]
    occurrences: Dict[str, Occurrence]
    subassemblies: Dict[str, SubAssembly]
    mates: Dict[str, MateFeatureData]
    patterns: Dict[str, Pattern]
    parts: Dict[str, Part]
```

**Step 2: Commit**
```bash
git add src/onshape2xacro/models.py
git commit -m "feat: add SerializableCAD pydantic model"
```

### Task 2: Implement Conversion Utilities

**Files:**
- Create: `src/onshape2xacro/cad_utils.py`
- Test: `tests/test_cad_utils.py`

**Step 1: Write conversion functions**
`cad_to_serializable(cad: CAD) -> SerializableCAD`
`serializable_to_cad(scad: SerializableCAD) -> CAD`

**Step 2: Write tests for round-trip conversion**
Ensure `CAD -> SerializableCAD -> CAD` preserves all data.

**Step 3: Commit**
```bash
git add src/onshape2xacro/cad_utils.py tests/test_cad_utils.py
git commit -m "feat: implement CAD serialization utilities"
```

### Task 3: Add fetch-cad CLI Command

**Files:**
- Modify: `src/onshape2xacro/cli/__init__.py`
- Modify: `src/onshape2xacro/pipeline.py`

**Step 1: Add FetchCadConfig to CLI**
```python
@dataclass
class FetchCadConfig:
    """Fetch CAD data from Onshape and save to a JSON file."""
    url: tyro.conf.Positional[str]
    output: Path
    max_depth: int = 5
```

**Step 2: Implement run_fetch_cad in pipeline.py**
Fetch CAD using `CAD.from_url`, convert to `SerializableCAD`, and save to JSON.

**Step 3: Register subcommand in parse_args**

**Step 4: Commit**
```bash
git add src/onshape2xacro/cli/__init__.py src/onshape2xacro/pipeline.py
git commit -m "feat: add fetch-cad cli command"
```

### Task 4: Verify with Integration Test

**Files:**
- Create: `tests/test_cli/test_fetch_cad.py`

**Step 1: Write integration test**
Mock `CAD.from_url` and verify `fetch-cad` saves expected JSON.

**Step 2: Run tests**
`uv run pytest tests/test_cli/test_fetch_cad.py`

**Step 3: Commit**
```bash
git add tests/test_cli/test_fetch_cad.py
git commit -m "test: add fetch-cad integration test"
```
