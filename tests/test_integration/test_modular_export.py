import pytest
import sys
import os
import pickle
import yaml
import numpy as np
import lxml.etree as ET
from pathlib import Path
from unittest.mock import MagicMock, patch
from importlib.abc import Loader, MetaPathFinder
from importlib.machinery import ModuleSpec


class MockLoader(Loader):
    def exec_module(self, module):
        if module.__name__ == "onshape_robotics_toolkit.formats.base":

            class RobotSerializer:
                def save(self, *args, **kwargs):
                    pass

                def serialize(self, *args, **kwargs):
                    return ""

            module.RobotSerializer = RobotSerializer
        elif module.__name__ == "onshape_robotics_toolkit.robot":

            class Robot:
                def __init__(self, *args, **kwargs):
                    self.kinematic_graph = args[0] if args else None
                    self._nodes = {}
                    self._edges = {}
                    self.name = kwargs.get("name", "robot")

                def clear(self):
                    self._nodes = {}
                    self._edges = {}

                def add_node(self, n, **attr):
                    self._nodes[n] = attr

                def add_edge(self, u, v, **attr):
                    self._edges[(u, v)] = attr

                @property
                def nodes(self):
                    class NodeView:
                        def __init__(self, nodes):
                            self._nodes = nodes

                        def __iter__(self):
                            return iter(self._nodes.keys())

                        def __call__(self, data=False):
                            if data:
                                return self._nodes.items()
                            return self._nodes.keys()

                        def __getitem__(self, n):
                            return self._nodes[n]

                    return NodeView(self._nodes)

                @property
                def edges(self):
                    return self._edges.keys()

                def get_edge_data(self, u, v):
                    return self._edges.get((u, v), {})

            module.Robot = Robot
        elif module.__name__ == "onshape_robotics_toolkit.models.link":

            class Origin:
                def __init__(self, xyz=(0, 0, 0), rpy=(0, 0, 0)):
                    self.xyz = xyz
                    self.rpy = rpy

                @classmethod
                def from_matrix(cls, matrix):
                    return cls()

            module.Origin = Origin
        else:
            module.__getattr__ = lambda name: MagicMock()


class MockFinder(MetaPathFinder):
    def find_spec(self, fullname, path, target=None):
        if any(
            fullname.startswith(p)
            for p in ["OCP", "onshape_robotics_toolkit", "trimesh", "coacd", "keyring"]
        ):
            return ModuleSpec(fullname, MockLoader())
        return None


sys.meta_path.insert(0, MockFinder())

from onshape2xacro.pipeline import run_export
from onshape2xacro.schema import ExportConfig
from onshape2xacro.config.export_config import ExportConfiguration
from onshape2xacro.serializers.modular import ModularXacroSerializer


class MockCAD:
    def __init__(self, name):
        self.name = name
        self.subassemblies = {}
        self.mates = {}
        self.keys_by_id = {}
        self.mate_connectors = []

    def get_transform(self, key):
        return np.eye(4)


class MockSubassembly:
    def __init__(self, name, doc_id, el_id, version="v1"):
        self.name = name
        self.documentId = doc_id
        self.elementId = el_id
        self.documentVersion = version
        self.workspaceId = ""
        self.documentMicroversion = ""


class MockNode:
    def __init__(self, name, parent=None):
        self.part_name = name
        self.part_id = name
        self.occurrence = [name]
        self.parent = parent
        self.mass_properties = MockMassProperties()


class MockMassProperties:
    def __init__(self):
        self.mass = 1.0


class MockMate:
    def __init__(self, name, id, parent_occ, child_occ, type="REVOLUTE"):
        self.name = name
        self.id = id
        self.mateType = type
        self.limits = None
        self.matedEntities = [MockMatedEntity(parent_occ), MockMatedEntity(child_occ)]


class MockMatedEntity:
    def __init__(self, occ):
        self.matedOccurrence = occ
        self.matedCS = MockCS()


class MockCS:
    @property
    def to_tf(self):
        return np.eye(4)


class MockKinematicGraph:
    def __init__(self, nodes, edges, edge_data_map):
        self.nodes = nodes
        self._edges = edges
        self.edge_data_map = edge_data_map

    def edges(self, data=False):
        if data:
            return [(u, v, self.edge_data_map.get((u, v), {})) for u, v in self._edges]
        return self._edges

    def get_edge_data(self, u, v):
        return self.edge_data_map.get((u, v), {})


def test_modular_export_integration(tmp_path):
    input_dir = tmp_path / "input"
    input_dir.mkdir()
    output_dir = tmp_path / "output"
    output_dir.mkdir()

    sub_def = MockSubassembly("SubMod", "doc1", "el1")

    inst1 = "inst1"
    inst2 = "inst2"

    cad = MockCAD("RootRobot")
    cad.subassemblies = {
        inst1: sub_def,
        inst2: sub_def,
    }

    node_root = MockNode("RootPart", parent=None)
    node_sub1 = MockNode("SubPart", parent=inst1)
    node_sub2 = MockNode("SubPart", parent=inst2)

    cad.keys_by_id[("RootPart",)] = node_root
    cad.keys_by_id[(inst1, "SubPart")] = node_sub1
    cad.keys_by_id[(inst2, "SubPart")] = node_sub2

    mate1 = MockMate("joint_j1", "m1", ["RootPart"], [inst1, "SubPart"])
    mate2 = MockMate("joint_j2", "m2", ["RootPart"], [inst2, "SubPart"])

    cad.mates = {
        ("root", "RootPart", (inst1, "SubPart")): mate1,
        ("root", "RootPart", (inst2, "SubPart")): mate2,
    }

    with open(input_dir / "cad.pickle", "wb") as f:
        pickle.dump(cad, f)

    config_data = {
        "export": {
            "name": "RootRobot",
            "output": str(output_dir),
            "format": "xacro_module",
        },
        "mate_values": {
            "m1": {"rotationZ": 0.0},
            "m2": {"rotationZ": 0.0},
        },
        "link_names": {},
    }
    with open(input_dir / "configuration.yaml", "w") as f:
        yaml.safe_dump(config_data, f)

    nodes = {
        node_root: {"data": node_root},
        node_sub1: {"data": node_sub1},
        node_sub2: {"data": node_sub2},
    }
    edges = [(node_root, node_sub1), (node_root, node_sub2)]
    edge_data_map = {
        (node_root, node_sub1): {"joint": mate1, "data": mate1},
        (node_root, node_sub2): {"joint": mate2, "data": mate2},
    }
    mock_graph = MockKinematicGraph(nodes, edges, edge_data_map)

    with (
        patch("onshape2xacro.pipeline.get_credentials", return_value=(None, None)),
        patch(
            "onshape2xacro.pipeline.KinematicGraph.from_cad", return_value=mock_graph
        ),
        patch.object(
            ModularXacroSerializer, "_export_meshes", return_value=({}, {}, None)
        ),
    ):
        config = ExportConfig(path=input_dir, format="xacro_module")
        run_export(config)

    urdf_dir = output_dir / "urdf"
    assert urdf_dir.exists()

    assert (urdf_dir / "rootrobot.xacro").exists()
    assert (urdf_dir / "rootrobot.urdf.xacro").exists()

    sub_mod_dir = urdf_dir / "submod"
    assert sub_mod_dir.exists()
    assert (sub_mod_dir / "submod.xacro").exists()

    existing_items = os.listdir(urdf_dir)
    assert "submod" in existing_items
    assert "rootrobot.xacro" in existing_items
    assert "rootrobot.urdf.xacro" in existing_items

    import subprocess

    result = subprocess.run(
        ["xacro", str(urdf_dir / "rootrobot.urdf.xacro")],
        capture_output=True,
        text=True,
    )
    assert result.returncode == 0, f"Xacro expansion failed: {result.stderr}"

    urdf_content = result.stdout
    root = ET.fromstring(urdf_content.encode())

    links = root.findall("link")
    link_names = [l.get("name") for l in links]
    assert "rootpart" in link_names
    assert "inst1_subpart" in link_names
    assert "inst2_subpart" in link_names
    assert len(links) == 3

    joints = root.findall("joint")
    joint_names = [j.get("name") for j in joints]
    assert "j1" in joint_names
    assert "j2" in joint_names
    assert len(joints) == 2
