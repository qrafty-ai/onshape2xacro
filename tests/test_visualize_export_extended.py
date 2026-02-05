import pytest
from unittest.mock import patch, MagicMock
import io
from onshape2xacro.visualize_export import (
    find_all_xacro_files,
    find_mesh_files,
    print_text_visualization,
    create_graph_visualization,
    main,
    build_robot_structure,
)


@pytest.fixture
def mock_export_dir(tmp_path):
    urdf_dir = tmp_path / "urdf"
    urdf_dir.mkdir(parents=True)

    meshes_dir = tmp_path / "meshes"
    meshes_dir.mkdir(parents=True)

    main_xacro = urdf_dir / "robot.urdf.xacro"
    main_xacro.write_text("""<?xml version="1.0"?>
<robot name="robot" xmlns:xacro="http://www.ros.org/wiki/xacro">
    <xacro:include filename="base.xacro"/>
    <xacro:macro name="test_macro" params="p1 p2"/>
</robot>
""")

    base_xacro = urdf_dir / "base.xacro"
    base_xacro.write_text("""<?xml version="1.0"?>
<robot name="robot">
    <link name="base_link">
        <visual><geometry><mesh filename="meshes/base.stl"/></geometry></visual>
        <collision><geometry><mesh filename="meshes/base.stl"/></geometry></collision>
    </link>
    <link name="arm_link"/>
    <joint name="joint_base_arm" type="revolute">
        <parent link="base_link"/>
        <child link="arm_link"/>
    </joint>
</robot>
""")

    mesh_file = meshes_dir / "base.stl"
    mesh_file.write_text("fake stl content")

    return tmp_path


def test_find_files(mock_export_dir):
    xacros = find_all_xacro_files(mock_export_dir)
    assert len(xacros) == 2

    meshes = find_mesh_files(mock_export_dir)
    assert len(meshes) == 1

    # Test non-existent dirs
    empty_dir = mock_export_dir / "empty"
    empty_dir.mkdir()
    assert find_all_xacro_files(empty_dir) == []
    assert find_mesh_files(empty_dir) == []


def test_print_text_visualization(mock_export_dir):
    structure = build_robot_structure(mock_export_dir)

    # Capture stdout
    f = io.StringIO()
    with patch("sys.stdout", f):
        print_text_visualization(structure, mock_export_dir)

    output = f.getvalue()
    assert "ROBOT DESCRIPTION VISUALIZATION" in output
    assert "base_link" in output
    assert "joint_base_arm" in output
    assert "base.stl" in output


def test_print_text_visualization_no_root(mock_export_dir):
    # Create structure with no links (so root_links will be empty)
    structure = {
        "links": {},
        "joints": [],
        "meshes": [],
        "mesh_map": {},
        "files": {},
        "entry_point": None,
    }

    f = io.StringIO()
    with patch("sys.stdout", f):
        print_text_visualization(structure, mock_export_dir)

    output = f.getvalue()
    assert "(No clear root link found)" in output


def test_create_graph_visualization(mock_export_dir):
    structure = build_robot_structure(mock_export_dir)
    output_path = mock_export_dir / "graph.png"

    with patch("onshape2xacro.visualize_export.plt") as mock_plt:
        mock_ax = MagicMock()
        mock_plt.subplots.return_value = (MagicMock(), mock_ax)
        create_graph_visualization(structure, output_path)
        assert mock_plt.subplots.called
        assert mock_plt.savefig.called


def test_create_graph_visualization_no_matplotlib(mock_export_dir):
    structure = build_robot_structure(mock_export_dir)
    output_path = mock_export_dir / "graph.png"

    with patch("onshape2xacro.visualize_export.HAS_MATPLOTLIB", False):
        # Should return early without error
        create_graph_visualization(structure, output_path)


def test_main_success(mock_export_dir):
    with (
        patch("sys.argv", ["visualize_export", str(mock_export_dir)]),
        patch("onshape2xacro.visualize_export.create_graph_visualization"),
    ):
        main()


def test_main_with_graph_arg(mock_export_dir):
    graph_path = mock_export_dir / "my_graph.png"
    with (
        patch(
            "sys.argv",
            ["visualize_export", str(mock_export_dir), "--graph", str(graph_path)],
        ),
        patch(
            "onshape2xacro.visualize_export.create_graph_visualization"
        ) as mock_graph,
    ):
        main()
        mock_graph.assert_called_once()


def test_main_error_dir_not_found():
    with (
        patch("sys.argv", ["visualize_export", "/non/existent/dir"]),
        pytest.raises(SystemExit) as excinfo,
    ):
        main()
    assert excinfo.value.code == 1
