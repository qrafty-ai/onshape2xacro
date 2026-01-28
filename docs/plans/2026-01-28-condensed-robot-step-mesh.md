# Condensed Robot + STEP Mesh Export Implementation Plan

> **For Claude:** REQUIRED SUB-SKILL: Use superpowers:executing-plans to implement this plan task-by-task.

**Goal:** Reduce Onshape API calls by building a condensed robot graph (links as fixed part groups) and exporting meshes from a single assembly STEP translation.

**Architecture:** Build `CondensedRobot` (parallel to toolkit `Robot`) by union-finding all non-`joint_` mates as fixed edges, producing `LinkRecord` and `JointRecord` objects. Update `XacroSerializer` to emit URDF/Xacro from these records and use a `StepMeshExporter` to export one assembly STEP and split solids into per-link STL files using OCP. Mesh filenames use `sanitize_name` to match serializer expectations, and export-id parsing is robust (regex). Fail fast on STEP parsing errors; no STL per-part fallback.

**Tech Stack:** Python, onshape_robotics_toolkit (Client, CAD, KinematicGraph), networkx, OCP/pythonocc, numpy, pytest.

---

### Task 1: Update OpenSpec change docs

**Files:**
- Modify: `openspec/changes/add-xacro-exporter/design.md`
- Modify: `openspec/changes/add-xacro-exporter/tasks.md`
- Modify: `openspec/changes/add-xacro-exporter/specs/xacro-serializer/spec.md`

**Step 1: Update the design doc**

Add a section describing:
- Non-`joint_` edges treated as fixed for union-find grouping
- `CondensedRobot` as a parallel implementation of toolkit `Robot`
- Single STEP translation + local split into per-link STL files
- Fail-fast behavior if STEP parse fails

**Step 2: Update the tasks doc**

Add tasks for:
- `CondensedRobot` implementation + tests
- OCP dependency + `StepMeshExporter`
- Xacro serializer changes
- Pipeline update

**Step 3: Update the serializer spec**

Add requirements:
- Mesh export uses a single STEP translation and local splitting
- No per-part STL downloads
- Fail-fast on STEP parsing errors

**Step 4: Validate OpenSpec change**

Run: `openspec validate add-xacro-exporter --strict`
Expected: `OK` (no validation errors)

**Step 5: Commit**

If requested:
```bash
git add openspec/changes/add-xacro-exporter/design.md openspec/changes/add-xacro-exporter/tasks.md openspec/changes/add-xacro-exporter/specs/xacro-serializer/spec.md
git commit -m "docs: update xacro exporter plan for STEP meshes"
```

---

### Task 2: Implement CondensedRobot + unit tests

**Files:**
- Create: `src/onshape2xacro/condensed_robot.py`
- Modify: `src/onshape2xacro/__init__.py`
- Test: `tests/test_kinematics/test_condensed_robot.py`

**Step 1: Write failing tests**

```python
import networkx as nx
from types import SimpleNamespace
from onshape2xacro.condensed_robot import CondensedRobot, is_joint_mate


def mate(name: str):
    return SimpleNamespace(name=name, mateType="FASTENED", limits=None, matedEntities=[])


def test_is_joint_mate_prefix():
    assert is_joint_mate(mate("joint_revolute")) is True
    assert is_joint_mate(mate("Fastened 1")) is False


def test_condensed_robot_groups_fixed_edges():
    g = nx.DiGraph()
    g.add_node("A", part_id="pA", part_name="A", occurrence=["A"], parent=None)
    g.add_node("B", part_id="pB", part_name="B", occurrence=["B"], parent=None)
    g.add_node("C", part_id="pC", part_name="C", occurrence=["C"], parent=None)

    g.add_edge("A", "B", data=mate("Fastened 1"))
    g.add_edge("B", "C", data=mate("joint_revolute"))

    robot = CondensedRobot.from_graph(g, cad=None, name="r")

    link_part_sets = [tuple(sorted(data["link"].part_ids)) for _, data in robot.nodes(data=True)]
    assert tuple(sorted(("pA", "pB"))) in link_part_sets
    assert ("pC",) in link_part_sets

    edges = list(robot.edges(data=True))
    assert len(edges) == 1
    assert edges[0][2]["joint"].name.startswith("joint_")
```

**Step 2: Run tests to verify failure**

Run: `pytest tests/test_kinematics/test_condensed_robot.py -v`
Expected: FAIL (CondensedRobot not implemented)

**Step 3: Implement minimal CondensedRobot**

```python
from dataclasses import dataclass
from collections import defaultdict
from typing import Any, Iterable
from onshape_robotics_toolkit.robot import Robot


def is_joint_mate(mate: Any) -> bool:
    name = getattr(mate, "name", "") or ""
    return name.startswith("joint_")


@dataclass
class LinkRecord:
    name: str
    part_ids: list[str]
    occurrences: list[list[str]]
    part_names: list[str]
    parent: Any = None


@dataclass
class JointRecord:
    name: str
    joint_type: str
    parent: str
    child: str
    limits: dict | None
    mate: Any


class CondensedRobot(Robot):
    @classmethod
    def from_graph(cls, graph: Any, cad: Any, name: str):
        robot = cls(kinematic_graph=None, name=name)

        # Union-find across all non-joint edges
        parent = {node: node for node in graph.nodes}

        def find(x):
            while parent[x] != x:
                parent[x] = parent[parent[x]]
                x = parent[x]
            return x

        def union(a, b):
            ra, rb = find(a), find(b)
            if ra != rb:
                parent[rb] = ra

        for u, v in graph.edges:
            mate = graph.get_edge_data(u, v).get("data")
            if not is_joint_mate(mate):
                union(u, v)

        groups = defaultdict(list)
        for node, attrs in graph.nodes(data=True):
            groups[find(node)].append((node, attrs))

        # Build link records
        group_to_link = {}
        for group_id, nodes in groups.items():
            part_ids = [attrs.get("part_id") for _, attrs in nodes]
            part_names = [attrs.get("part_name") for _, attrs in nodes]
            occurrences = [attrs.get("occurrence") for _, attrs in nodes]
            link_name = "_".join([n for n in part_names if n]) or str(group_id)
            link = LinkRecord(
                name=link_name,
                part_ids=part_ids,
                occurrences=occurrences,
                part_names=part_names,
                parent=nodes[0][1].get("parent"),
            )
            robot.add_node(link_name, link=link, parent=link.parent)
            group_to_link[group_id] = link_name

        # Build joint records
        for u, v in graph.edges:
            mate = graph.get_edge_data(u, v).get("data")
            if not is_joint_mate(mate):
                continue
            parent_link = group_to_link[find(u)]
            child_link = group_to_link[find(v)]
            if parent_link == child_link:
                continue
            joint = JointRecord(
                name=mate.name,
                joint_type=str(getattr(mate, "mateType", "fixed")),
                parent=parent_link,
                child=child_link,
                limits=getattr(mate, "limits", None),
                mate=mate,
            )
            robot.add_edge(parent_link, child_link, joint=joint)

        return robot
```

**Step 4: Run tests to verify pass**

Run: `pytest tests/test_kinematics/test_condensed_robot.py -v`
Expected: PASS

**Step 5: Commit**

If requested:
```bash
git add src/onshape2xacro/condensed_robot.py src/onshape2xacro/__init__.py tests/test_kinematics/test_condensed_robot.py
git commit -m "feat: add condensed robot graph"
```

---

### Task 3: Add OCP dependency + failing StepMeshExporter tests

**Files:**
- Modify: `pyproject.toml`
- Modify: `uv.lock`
- Test: `tests/test_serializers/test_step_mesh_exporter.py`

**Step 1: Add dependency**

Add to `pyproject.toml`:
```toml
OCP = ">=7.7.0"
```

Run: `uv lock`
Expected: lockfile updated without errors

**Step 2: Write failing tests**

```python
from pathlib import Path
from onshape2xacro.mesh_exporters.step import split_step_to_meshes
from OCP.BRepPrimAPI import BRepPrimAPI_MakeBox
from OCP.STEPCAFControl import STEPCAFControl_Writer
from OCP.TDocStd import TDocStd_Document
from OCP.TCollection import TCollection_ExtendedString
from OCP.XCAFDoc import XCAFDoc_DocumentTool
from OCP.TDataStd import TDataStd_Name
from OCP.STEPControl import STEPControl_AsIs


def make_step(tmp_path: Path) -> Path:
    doc = TDocStd_Document(TCollection_ExtendedString("step"))
    shape_tool = XCAFDoc_DocumentTool.ShapeTool(doc.Main())

    box1 = BRepPrimAPI_MakeBox(1, 1, 1).Shape()
    box2 = BRepPrimAPI_MakeBox(2, 1, 1).Shape()

    label1 = shape_tool.AddShape(box1)
    label2 = shape_tool.AddShape(box2)
    TDataStd_Name.Set(label1, TCollection_ExtendedString("export:pA"))
    TDataStd_Name.Set(label2, TCollection_ExtendedString("export:pB"))

    writer = STEPCAFControl_Writer()
    writer.Transfer(doc, STEPControl_AsIs)
    out = tmp_path / "assembly.step"
    assert writer.Write(str(out)) == 1
    return out


def test_split_step_to_meshes(tmp_path: Path):
    step_path = make_step(tmp_path)
    link_groups = {"link_ab": ["pA", "pB"]}

    mesh_map = split_step_to_meshes(step_path, link_groups, tmp_path)
    assert (tmp_path / "link_ab.stl").exists()
    assert mesh_map["link_ab"] == "link_ab.stl"
```

**Step 3: Run tests to verify failure**

Run: `pytest tests/test_serializers/test_step_mesh_exporter.py -v`
Expected: FAIL (module not implemented)

**Step 4: Commit**

If requested:
```bash
git add pyproject.toml uv.lock tests/test_serializers/test_step_mesh_exporter.py
git commit -m "test: add step mesh exporter tests"
```

---

### Task 4: Implement StepMeshExporter

**Files:**
- Create: `src/onshape2xacro/mesh_exporters/step.py`
- Create: `src/onshape2xacro/mesh_exporters/__init__.py`

**Step 1: Implement exporter helpers (including sanitize helper)**

```python
from pathlib import Path
import re
from collections import defaultdict
from typing import Any
from onshape_robotics_toolkit.connect import Client
from onshape2xacro.naming import sanitize_name
from OCP.STEPCAFControl import STEPCAFControl_Reader
from OCP.TDocStd import TDocStd_Document
from OCP.TCollection import TCollection_ExtendedString
from OCP.XCAFDoc import XCAFDoc_DocumentTool
from OCP.TDataStd import TDataStd_Name
from OCP.TDF import TDF_LabelSequence
from OCP.IFSelect import IFSelect_RetDone
from OCP.BRep import BRep_Builder
from OCP.TopoDS import TopoDS_Compound
from OCP.BRepMesh import BRepMesh_IncrementalMesh
from OCP.StlAPI import StlAPI_Writer


def _label_name(label) -> str:
    name_attr = TDataStd_Name()
    if label.FindAttribute(TDataStd_Name.GetID(), name_attr):
        return name_attr.Get().ToExtString()
    return ""


def _parse_export_id(label_name: str) -> str:
    # Try multiple formats (export:<id>, partId=<id>, id=<id>)
    match = re.search(r"(?:export:|partId=|id=)([A-Za-z0-9+/_-]+)", label_name)
    if match:
        return match.group(1)
    return label_name


def split_step_to_meshes(step_path: Path, link_groups: dict[str, list[str]], mesh_dir: Path) -> dict[str, str]:
    doc = TDocStd_Document(TCollection_ExtendedString("step"))
    reader = STEPCAFControl_Reader()
    reader.SetNameMode(True)
    status = reader.ReadFile(str(step_path))
    if status != IFSelect_RetDone:
        raise RuntimeError(f"STEP read failed: {status}")
    reader.Transfer(doc)

    shape_tool = XCAFDoc_DocumentTool.ShapeTool(doc.Main())
    labels = TDF_LabelSequence()
    shape_tool.GetFreeShapes(labels)

    part_shapes = {}
    for i in range(labels.Length()):
        label = labels.Value(i + 1)
        name = _parse_export_id(_label_name(label))
        shape = shape_tool.GetShape(label)
        if name:
            part_shapes[name] = shape

    mesh_dir.mkdir(parents=True, exist_ok=True)
    stl_writer = StlAPI_Writer()
    mesh_map = {}

    for link_name, part_ids in link_groups.items():
        safe_link_name = sanitize_name(link_name)
        compound = TopoDS_Compound()
        builder = BRep_Builder()
        builder.MakeCompound(compound)
        for part_id in part_ids:
            if part_id not in part_shapes:
                raise RuntimeError(f"Missing part id in STEP: {part_id}")
            builder.Add(compound, part_shapes[part_id])

        BRepMesh_IncrementalMesh(compound, 0.001)
        out_path = mesh_dir / f"{safe_link_name}.stl"
        stl_writer.Write(compound, str(out_path))
        mesh_map[safe_link_name] = out_path.name

    return mesh_map
```

**Step 2: Implement STEP translation (single API call)**

```python
import time


class StepMeshExporter:
    def __init__(self, client: Client, cad: Any):
        self.client = client
        self.cad = cad

    def export_step(self, output_path: Path) -> Path:
        did = self.cad.document_id
        wtype = self.cad.wvm
        wid = self.cad.wvm_id
        eid = self.cad.element_id

        payload = {
            "formatName": "STEP",
            "storeInDocument": False,
            "stepVersionString": "AP242",
            "extractAssemblyHierarchy": True,
            "includeExportIds": True,
            "flattenAssemblies": False,
        }
        response = self.client.request(
            method="POST",
            path=f"/api/assemblies/d/{did}/{wtype}/{wid}/e/{eid}/translations",
            body=payload,
        )
        translation_id = response.json()["id"]

        while True:
            status = self.client.request(
                method="GET",
                path=f"/api/translations/{translation_id}",
            ).json()
            state = status.get("requestState")
            if state == "DONE":
                break
            if state in {"FAILED", "CANCELED"}:
                raise RuntimeError(f"STEP translation failed: {state}")
            time.sleep(0.5)

        file_id = status["resultExternalDataIds"][0]
        download = self.client.request(
            method="GET",
            path=f"/api/documents/d/{did}/externaldata/{file_id}",
            headers={"Accept": "application/octet-stream"},
        )
        output_path.write_bytes(download.content)
        return output_path

    def export_link_meshes(self, link_groups: dict[str, list[str]], mesh_dir: Path) -> dict[str, str]:
        step_path = mesh_dir / "assembly.step"
        step_path = self.export_step(step_path)
        return split_step_to_meshes(step_path, link_groups, mesh_dir)
```

**Step 3: Run tests to verify pass**

Run: `pytest tests/test_serializers/test_step_mesh_exporter.py -v`
Expected: PASS

**Step 4: Commit**

If requested:
```bash
git add src/onshape2xacro/mesh_exporters/step.py src/onshape2xacro/mesh_exporters/__init__.py
git commit -m "feat: add STEP mesh exporter"
```

---

### Task 5: Update serializer + pipeline integration

**Files:**
- Modify: `src/onshape2xacro/serializers/__init__.py`
- Modify: `src/onshape2xacro/pipeline.py`
- Modify: `examples/load_pickle.py`
- Test: `tests/test_serializers/test_xacro.py`

**Step 1: Write failing serializer test**

```python
from onshape2xacro.condensed_robot import LinkRecord, JointRecord
from onshape2xacro.serializers import XacroSerializer
import networkx as nx


def test_xacro_uses_link_mesh_map(tmp_path):
    robot = nx.DiGraph()
    robot.name = "r"
    robot.add_node("link_a", link=LinkRecord("link_a", ["pA"], [["A"]], ["A"]))
    robot.add_node("link_b", link=LinkRecord("link_b", ["pB"], [["B"]], ["B"]))
    robot.add_edge("link_a", "link_b", joint=JointRecord("joint_revolute", "REVOLUTE", "link_a", "link_b", None, None))

    serializer = XacroSerializer()
    serializer._export_meshes = lambda robot, mesh_dir: {"link_a": "link_a.stl", "link_b": "link_b.stl"}

    out = tmp_path / "robot.xacro"
    serializer.save(robot, str(out), download_assets=True)
    assert out.exists()
```

**Step 2: Run test to verify failure**

Run: `pytest tests/test_serializers/test_xacro.py -v`
Expected: FAIL (serializer expects toolkit Link/Joint types)

**Step 3: Update serializer to use LinkRecord/JointRecord**

```python
# in XacroSerializer._link_to_xacro
link = data.get("link")
if hasattr(link, "to_xml"):
    # existing path
    ...
else:
    link_elem = Element("link", name=f"${{prefix}}{link.name}")
    for tag in ("visual", "collision"):
        geom = Element(tag)
        mesh = Element("mesh", filename=mesh_rel_path)
        geometry = Element("geometry")
        geometry.append(mesh)
        geom.append(geometry)
        link_elem.append(geom)

# in _joint_to_xacro
joint = data.get("joint")
if hasattr(joint, "to_xml"):
    ...
else:
    joint_elem = Element("joint", name=f"${{prefix}}{joint.name}")
    joint_elem.set("type", "revolute" if "REVOLUTE" in joint.joint_type else "fixed")
    joint_elem.append(Element("parent", link=f"${{prefix}}{joint.parent}"))
    joint_elem.append(Element("child", link=f"${{prefix}}{joint.child}"))
```

**Step 4: Replace mesh export with StepMeshExporter**

```python
from onshape2xacro.mesh_exporters.step import StepMeshExporter
from onshape2xacro.naming import sanitize_name

def _export_meshes(self, robot, mesh_dir: Path):
    link_groups = {
        sanitize_name(data["link"].name): data["link"].part_ids
        for _, data in robot.nodes(data=True)
        if "link" in data
    }
    exporter = StepMeshExporter(robot.client, robot.cad)
    return exporter.export_link_meshes(link_groups, mesh_dir)
```

**Step 5: Update pipeline + example script**

```python
# pipeline.py
from onshape2xacro.condensed_robot import CondensedRobot

robot = CondensedRobot.from_graph(graph, cad, name=robot_name)
robot.client = client
robot.cad = cad
```

```python
# examples/load_pickle.py
from onshape2xacro.condensed_robot import CondensedRobot
...
robot = CondensedRobot.from_graph(graph, cad, name=cad.name)
print(f"Condensed links: {len(robot.nodes)}")
print(f"Condensed joints: {len(robot.edges)}")
```

**Step 6: Run tests to verify pass**

Run: `pytest tests/test_serializers/test_xacro.py -v`
Expected: PASS

**Step 7: Commit**

If requested:
```bash
git add src/onshape2xacro/serializers/__init__.py src/onshape2xacro/pipeline.py examples/load_pickle.py tests/test_serializers/test_xacro.py
git commit -m "feat: use condensed robot and STEP mesh export"
```

---

### Task 6: Smoke test offline graph simplification

**Files:**
- None (uses existing pickle)

**Step 1: Run pickle-based script**

Run: `python examples/load_pickle.py test.pkl`
Expected: Prints condensed link/joint counts without API calls

**Step 2: Commit**

No commit required (no code changes)
