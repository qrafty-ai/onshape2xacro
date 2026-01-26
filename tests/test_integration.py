import os
import pytest
from pathlib import Path
from onshape2xacro.pipeline import run_export
from onshape2xacro.cli import ExportConfig


@pytest.fixture
def onshape_creds():
    access = os.environ.get("ONSHAPE_ACCESS_KEY")
    secret = os.environ.get("ONSHAPE_SECRET_KEY")
    if not access or not secret:
        pytest.skip("Onshape credentials not set")
    return access, secret


def test_full_export_integration(onshape_creds, tmp_path):
    url = "https://cad.onshape.com/documents/1705d0f261fcd9a4be84bc11/w/20eeb5f726c3be4d0746a9ac/e/fa1b8966a541ae6953c4caae"
    output_dir = tmp_path / "export_test"

    config = ExportConfig(url=url, output=output_dir, name="test_robot", max_depth=5)

    # Run the actual pipeline
    run_export(config)

    # 1. Verify Directory Structure
    assert (output_dir / "urdf").exists()
    assert (output_dir / "meshes").exists()
    assert (output_dir / "config").exists()

    # 2. Verify Xacro Entry Point (.urdf.xacro)
    entry_point_path = output_dir / "urdf" / "test_robot.urdf.xacro"
    assert entry_point_path.exists()

    entry_content = entry_point_path.read_text()
    assert '<xacro:include filename="test_robot.xacro"/>' in entry_content
    assert '<xacro:test_robot prefix=""/>' in entry_content

    # Verify Xacro Macro File (.xacro)
    xacro_path = output_dir / "urdf" / "test_robot.xacro"
    assert xacro_path.exists()

    content = xacro_path.read_text()
    assert '<xacro:macro name="test_robot" params="prefix:=\'\'">' in content

    # Verify sub-module include (relative to test_robot.xacro)
    assert '<xacro:include filename="subasm_1/subasm_1.xacro"/>' in content
    assert '<xacro:subasm_1 prefix="${prefix}"/>' in content
    assert "$(find" not in content

    # Verify sub-module content
    sub_xacro_path = output_dir / "urdf" / "subasm_1" / "subasm_1.xacro"
    assert sub_xacro_path.exists()
    sub_content = sub_xacro_path.read_text()
    assert 'joint name="${prefix}revolute"' in sub_content
    assert "$(find" not in sub_content

    # 3. Verify Config Files
    assert (output_dir / "config" / "joint_limits.yaml").exists()
    assert (output_dir / "config" / "inertials.yaml").exists()

    # 4. Verify Meshes
    stl_files = list(output_dir.glob("meshes/**/*.stl"))
    assert len(stl_files) > 0
    # Verify mesh path in xacro (relative from urdf/subasm_1/subasm_1.xacro to meshes/)
    assert "../../meshes/" in sub_content
    assert "package://" not in sub_content
