import networkx as nx
from types import SimpleNamespace
from unittest.mock import patch, MagicMock

from onshape2xacro.serializers import (
    XacroSerializer,
    is_module_boundary,
    get_joint_name,
)
from onshape2xacro.condensed_robot import LinkRecord, JointRecord


def test_is_module_boundary():
    graph = MagicMock()
    # No edges
    graph.edges = []
    assert is_module_boundary(graph) is False

    # Edge without joint_ prefix
    edge1 = MagicMock()
    edge1.name = "not_a_joint"
    graph.edges = [edge1]
    assert is_module_boundary(graph) is False

    # Edge with joint_ prefix
    edge2 = MagicMock()
    edge2.name = "joint_arm"
    graph.edges = [edge2]
    assert is_module_boundary(graph) is True


def test_get_joint_name():
    assert get_joint_name("joint_revolute") == "revolute"
    assert get_joint_name("not_a_joint") == "not_a_joint"


def test_xacro_serializer_serialize_flattened():
    robot = nx.DiGraph()
    robot.name = "flat_robot"
    node_id = "base_link"
    robot.add_node(node_id, data=LinkRecord("base_link", [], [], [], keys=["pA"]))

    serializer = XacroSerializer()
    xacro_str = serializer.serialize(robot)
    assert 'name="flat_robot"' in xacro_str
    assert '<link name="${prefix}base_link">' in xacro_str
    assert "joint_transforms" in xacro_str


def test_xacro_serializer_serialize_joint_transform_override():
    from onshape2xacro.config import ConfigOverride

    robot = nx.DiGraph()
    robot.name = "flat_robot"
    node_a = "link_a"
    node_b = "link_b"
    robot.add_node(node_a, data=LinkRecord("link_a", [], [], [], keys=["pA"]))
    robot.add_node(node_b, data=LinkRecord("link_b", [], [], [], keys=["pB"]))

    joint = JointRecord(
        "joint_arm",
        "REVOLUTE",
        "link_a",
        "link_b",
        (0, 0, 1),
        SimpleNamespace(name="joint_arm"),
    )
    robot.add_edge(node_a, node_b, data=joint)

    serializer = XacroSerializer()
    xacro_str = serializer.serialize(
        robot,
        config=ConfigOverride(joint_transforms={"arm": {"xyz": "1 2 3"}}),
    )

    assert "joint_transforms" in xacro_str
    assert "'arm': {'xyz': '1 2 3', 'rpy': '0 0 0'}" in xacro_str
    assert "joint_transforms['arm']['xyz']" in xacro_str


def test_xacro_serializer_serialize_fixed_joint_transform_override():
    from onshape2xacro.config import ConfigOverride

    robot = nx.DiGraph()
    robot.name = "flat_robot"
    node_a = "link_a"
    node_b = "link_b"
    robot.add_node(node_a, data=LinkRecord("link_a", [], [], [], keys=["pA"]))
    robot.add_node(node_b, data=LinkRecord("link_b", [], [], [], keys=["pB"]))

    joint = JointRecord(
        "fixed_frame_right_oak",
        "FIXED",
        "link_a",
        "link_b",
        (0, 0, 1),
        SimpleNamespace(name="fixed_frame_right_oak"),
    )
    robot.add_edge(node_a, node_b, data=joint)

    serializer = XacroSerializer()
    xacro_str = serializer.serialize(
        robot,
        config=ConfigOverride(
            joint_transforms={"fixed_frame_right_oak": {"xyz": "4 5 6"}}
        ),
    )

    assert "'fixed_frame_right_oak': {'xyz': '4 5 6', 'rpy': '0 0 0'}" in xacro_str
    assert "joint_transforms['fixed_frame_right_oak']['xyz']" in xacro_str


def test_xacro_serializer_save_no_download(tmp_path):
    robot = nx.DiGraph()
    robot.name = "test_robot"
    node_id = "base_link"
    robot.add_node(node_id, data=LinkRecord("base_link", [], [], [], keys=["pA"]))

    serializer = XacroSerializer()
    out = tmp_path / "output"
    serializer.save(robot, str(out), download_assets=False)

    assert (out / "urdf" / "test_robot.urdf.xacro").exists()
    assert (out / "config" / "joint_limits.yaml").exists()
    assert (out / "config" / "inertials.yaml").exists()
    assert (out / "config" / "joint_transforms.yaml").exists()

    with open(out / "urdf" / "test_robot.urdf.xacro", "r") as f:
        content = f.read()
        assert "joint_transforms_file" in content
        assert "joint_transforms.yaml" in content
        assert "joint_transforms_file['joint_transforms']" in content


def test_xacro_serializer_subassembly_grouping(tmp_path):
    # Mock node IDs as strings or use a simple class with hash
    node1 = "link1"
    node2 = "link2"
    node3 = "link3"

    robot = nx.DiGraph()
    robot.name = "complex_robot"
    # Parent should be hashable (like a string or a namedtuple)
    parent_sub1 = "sub1"

    link1 = LinkRecord("link1", [], [], [], keys=["p1"], parent=None)
    link2 = LinkRecord("link2", [], [], [], keys=["p2"], parent=parent_sub1)
    link3 = LinkRecord("link3", [], [], [], keys=["p3"], parent=parent_sub1)

    robot.add_node(node1, data=link1)
    robot.add_node(node2, data=link2)
    robot.add_node(node3, data=link3)

    # Edge within sub1
    joint1 = JointRecord(
        "joint_1",
        "REVOLUTE",
        "link2",
        "link3",
        (0, 0, 1),
        SimpleNamespace(name="joint_1"),
    )
    robot.add_edge(node2, node3, data=joint1)

    serializer = XacroSerializer()
    # Provide dummy client/cad
    robot.client = MagicMock()
    robot.cad = MagicMock()

    out = tmp_path / "output"
    with patch("onshape2xacro.serializers.StepMeshExporter") as mock_exporter_cls:
        mock_exporter = mock_exporter_cls.return_value
        mock_exporter.export_link_meshes.return_value = ({}, {}, None)
        serializer.save(robot, str(out), download_assets=True)

    # Check if subassembly directories were created
    assert (out / "urdf" / "complex_robot.xacro").exists()
    assert (out / "urdf" / "sub1" / "sub1.xacro").exists()


def test_xacro_joint_types(tmp_path):
    robot = nx.DiGraph()
    robot.name = "joints_robot"
    node_a = "link_a"
    node_b = "link_b"
    node_c = "link_c"
    robot.add_node(node_a, data=LinkRecord("link_a", [], [], [], keys=["pA"]))
    robot.add_node(node_b, data=LinkRecord("link_b", [], [], [], keys=["pB"]))
    robot.add_node(node_c, data=LinkRecord("link_c", [], [], [], keys=["pC"]))

    # Prismatic joint
    joint_p = JointRecord(
        "joint_p",
        "PRISMATIC",
        "link_a",
        "link_b",
        (0, 0, 1),
        SimpleNamespace(name="joint_p"),
    )
    robot.add_edge(node_a, node_b, data=joint_p)

    # Continuous joint
    joint_c = JointRecord(
        "joint_c",
        "CONTINUOUS",
        "link_b",
        "link_c",
        (0, 0, 1),
        SimpleNamespace(name="joint_c"),
    )
    robot.add_edge(node_b, node_c, data=joint_c)

    serializer = XacroSerializer()
    out = tmp_path / "output"
    serializer.save(robot, str(out), download_assets=False)

    with open(out / "urdf" / "joints_robot.xacro", "r") as f:
        content = f.read()
        assert 'type="prismatic"' in content
        assert 'type="continuous"' in content


def test_fixed_joint_origin_uses_joint_transforms(tmp_path):
    import yaml

    robot = nx.DiGraph()
    robot.name = "fixed_joint_robot"
    node_a = "link_a"
    node_b = "link_b"
    robot.add_node(node_a, data=LinkRecord("link_a", [], [], [], keys=["pA"]))
    robot.add_node(node_b, data=LinkRecord("link_b", [], [], [], keys=["pB"]))

    joint = JointRecord(
        "fixed_frame_right_oak",
        "FIXED",
        "link_a",
        "link_b",
        (0, 0, 1),
        SimpleNamespace(name="fixed_frame_right_oak"),
    )
    robot.add_edge(node_a, node_b, data=joint)

    serializer = XacroSerializer()
    out = tmp_path / "output"
    serializer.save(robot, str(out), download_assets=False)

    with open(out / "config" / "joint_transforms.yaml", "r") as f:
        data = yaml.safe_load(f)
        assert "fixed_frame_right_oak" in data["joint_transforms"]
        assert data["joint_transforms"]["fixed_frame_right_oak"] == {
            "xyz": "0 0 0",
            "rpy": "0 0 0",
        }

    with open(out / "urdf" / "fixed_joint_robot.xacro", "r") as f:
        content = f.read()
        assert 'name="${prefix}fixed_frame_right_oak"' in content
        assert "joint_transforms['fixed_frame_right_oak']['xyz']" in content
        assert "joint_transforms['fixed_frame_right_oak']['rpy']" in content


def test_joint_edge_key_generates_joint_transforms(tmp_path):
    import yaml

    robot = nx.DiGraph()
    robot.name = "joint_edge_robot"
    node_a = "link_a"
    node_b = "link_b"
    robot.add_node(node_a, data=LinkRecord("link_a", [], [], [], keys=["pA"]))
    robot.add_node(node_b, data=LinkRecord("link_b", [], [], [], keys=["pB"]))

    joint = JointRecord(
        "fixed_frame_right_oak",
        "FIXED",
        "link_a",
        "link_b",
        (0, 0, 1),
        SimpleNamespace(name="fixed_frame_right_oak"),
    )
    robot.add_edge(node_a, node_b, joint=joint)

    serializer = XacroSerializer()
    out = tmp_path / "output"
    serializer.save(robot, str(out), download_assets=False)

    with open(out / "config" / "joint_transforms.yaml", "r") as f:
        data = yaml.safe_load(f)
        assert "fixed_frame_right_oak" in data["joint_transforms"]

    with open(out / "urdf" / "joint_edge_robot.xacro", "r") as f:
        content = f.read()
        assert "joint_transforms['fixed_frame_right_oak']['xyz']" in content


def test_xacro_missing_meshes_prompt(tmp_path):
    robot = nx.DiGraph()
    robot.name = "missing_mesh_robot"
    node_id = "bad_link"
    robot.add_node(node_id, data=LinkRecord("bad_link", [], [], [], keys=["p1"]))

    serializer = XacroSerializer()
    robot.client = MagicMock()
    robot.cad = MagicMock()

    out = tmp_path / "output"
    with patch("onshape2xacro.serializers.StepMeshExporter") as mock_exporter_cls:
        mock_exporter = mock_exporter_cls.return_value
        # Simulate missing meshes
        mock_exporter.export_link_meshes.return_value = (
            {},
            {
                "bad_link": [
                    {"part_name": "p1", "export_name": "p1.stl", "reason": "failed"}
                ]
            },
            None,
        )
        serializer.save(robot, str(out), download_assets=True)

    assert (out / "MISSING_MESHES.md").exists()
    with open(out / "MISSING_MESHES.md", "r") as f:
        content = f.read()
        assert "bad_link" in content
        assert "p1.stl" in content


def test_xacro_virtual_link_negligible_inertia(tmp_path):
    robot = nx.DiGraph()
    robot.name = "virtual_robot"
    # Link with NO part_ids
    node_id = "virtual_link"
    robot.add_node(
        node_id,
        data=LinkRecord(
            "virtual_link", part_ids=[], occurrences=[], part_names=[], keys=[]
        ),
    )

    serializer = XacroSerializer()
    out = tmp_path / "output"
    serializer.save(robot, str(out), download_assets=False)

    import yaml

    with open(out / "config" / "inertials.yaml", "r") as f:
        data = yaml.safe_load(f)
        # Should have very small mass
        assert data["inertials"]["virtual_link"]["mass"] < 1e-8


def test_xacro_link_mesh_map_dict(tmp_path):
    robot = nx.DiGraph()
    robot.name = "dict_mesh_robot"
    node_id = "link1"
    robot.add_node(node_id, data=LinkRecord("link1", [], [], [], keys=["p1"]))

    serializer = XacroSerializer()
    robot.client = MagicMock()
    robot.cad = MagicMock()

    out = tmp_path / "output"
    with patch("onshape2xacro.serializers.StepMeshExporter") as mock_exporter_cls:
        mock_exporter = mock_exporter_cls.return_value
        # mesh_map entry is a dict
        mock_exporter.export_link_meshes.return_value = (
            {"link1": {"visual": "v.stl", "collision": ["c1.stl", "c2.stl"]}},
            {},
            None,
        )
        serializer.save(robot, str(out), download_assets=True)

    with open(out / "urdf" / "dict_mesh_robot.xacro", "r") as f:
        content = f.read()
        assert "c1.stl" in content
        assert "c2.stl" in content
        # Visual uses dynamic extension anyway
        assert "visual/link1.${visual_mesh_ext}" in content
