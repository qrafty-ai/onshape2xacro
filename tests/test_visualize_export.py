import pytest
from pathlib import Path
from onshape2xacro.visualize_export import parse_xacro_file, build_robot_structure


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
</robot>
""")

    base_xacro = urdf_dir / "base.xacro"
    base_xacro.write_text("""<?xml version="1.0"?>
<robot name="robot">
    <link name="base_link">
        <visual><geometry><mesh filename="meshes/base.stl"/></geometry></visual>
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


def test_parse_xacro_file(mock_export_dir):
    base_xacro = mock_export_dir / "urdf" / "base.xacro"
    root, data = parse_xacro_file(base_xacro)

    assert "base_link" in data["links"]
    assert data["links"]["base_link"]["has_visual"] is True
    assert len(data["joints"]) == 1
    assert data["joints"][0]["name"] == "joint_base_arm"
    assert data["joints"][0]["parent"] == "base_link"


def test_build_robot_structure(mock_export_dir):
    structure = build_robot_structure(mock_export_dir)

    assert "base_link" in structure["links"]
    assert len(structure["joints"]) == 1
    assert len(structure["meshes"]) == 1
    assert structure["entry_point"] == Path("urdf/robot.urdf.xacro")
