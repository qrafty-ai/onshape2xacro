import sys
from unittest.mock import MagicMock

# Mock OCP before importing step
mock_ocp = MagicMock()
sys.modules["OCP"] = mock_ocp
sys.modules["OCP.STEPCAFControl"] = mock_ocp
sys.modules["OCP.TDocStd"] = mock_ocp
sys.modules["OCP.TCollection"] = mock_ocp
sys.modules["OCP.XCAFDoc"] = mock_ocp
sys.modules["OCP.TDataStd"] = mock_ocp
sys.modules["OCP.TDF"] = mock_ocp
sys.modules["OCP.Quantity"] = mock_ocp
sys.modules["OCP.IFSelect"] = mock_ocp
sys.modules["OCP.TopLoc"] = mock_ocp
sys.modules["OCP.BRep"] = mock_ocp
sys.modules["OCP.BRepBuilderAPI"] = mock_ocp
sys.modules["OCP.TopoDS"] = mock_ocp
sys.modules["OCP.BRepMesh"] = mock_ocp
sys.modules["OCP.StlAPI"] = mock_ocp
sys.modules["OCP.gp"] = mock_ocp
sys.modules["OCP.STEPControl"] = mock_ocp

# Mock other potentially missing deps
sys.modules["trimesh"] = MagicMock()
sys.modules["coacd"] = MagicMock()
sys.modules["pymeshlab"] = MagicMock()
sys.modules["cadquery"] = MagicMock()

import pytest
from unittest.mock import patch
from pathlib import Path
import numpy as np

from onshape2xacro.mesh_exporters.step import StepMeshExporter


@pytest.fixture
def mock_step_env(tmp_path):
    client = MagicMock()
    cad = MagicMock()
    exporter = StepMeshExporter(client, cad)

    # Create dummy STEP file
    asset_path = tmp_path / "assembly.step"
    asset_path.write_bytes(
        b"ISO-10303-21;\nHEADER;\nENDSEC;\nDATA;\nENDSEC;\nEND-ISO-10303-21;\n"
    )
    exporter.asset_path = asset_path

    mesh_dir = tmp_path / "base_meshes"
    mesh_dir.mkdir()

    return exporter, cad, mesh_dir, tmp_path


def test_export_link_meshes_modular(mock_step_env):
    exporter, cad, mesh_dir, tmp_path = mock_step_env

    # Setup modular directories
    module_a_dir = tmp_path / "module_a_meshes"
    module_mesh_dirs = {"module_a": module_a_dir}

    # Mocks for export_link_meshes
    with (
        patch("onshape2xacro.mesh_exporters.step.BRepMesh_IncrementalMesh"),
        patch("onshape2xacro.mesh_exporters.step.StlAPI_Writer") as mock_writer,
        patch(
            "onshape2xacro.mesh_exporters.step.STEPCAFControl_Reader"
        ) as mock_reader_cls,
        patch("onshape2xacro.mesh_exporters.step.TDocStd_Document"),
        patch("onshape2xacro.mesh_exporters.step._get_shape_tool"),
        patch("onshape2xacro.mesh_exporters.step.XCAFDoc_DocumentTool"),
        patch(
            "onshape2xacro.mesh_exporters.step._get_free_shape_labels"
        ) as mock_labels,
        patch("onshape2xacro.mesh_exporters.step.IFSelect_RetDone", new=1),
        patch("onshape2xacro.mesh_exporters.step.BRepBuilderAPI_Transform"),
        patch("onshape2xacro.mesh_exporters.step.BRep_Builder"),
        patch("onshape2xacro.mesh_exporters.step.TopoDS_Compound"),
        patch("trimesh.load") as mock_trimesh_load,
        patch("onshape2xacro.mesh_exporters.step.get_direct_module") as mock_get_module,
        patch("onshape2xacro.mesh_exporters.step.to_path_tuple") as mock_to_path,
        patch(
            "onshape2xacro.mesh_exporters.step._part_world_matrix",
            return_value=np.eye(4),
        ),
    ):
        # Mock file operations
        mock_writer.return_value.Write.side_effect = lambda shape, path: Path(
            path
        ).touch()

        # Mock reader
        mock_reader = mock_reader_cls.return_value
        mock_reader.ReadFile.return_value = 1
        mock_reader.Transfer.return_value = True

        # Mock labels
        mock_labels.return_value.Length.return_value = 1

        # Mock trimesh mesh
        mock_mesh = MagicMock()
        mock_trimesh_load.return_value = mock_mesh

        def export_side_effect(path, **kwargs):
            Path(path).touch()

        mock_mesh.export.side_effect = export_side_effect

        # Setup CAD and link records
        cad.subassemblies = {"module_a": MagicMock()}

        part_a_key = MagicMock()
        part_a_key.path = ["subasm", "part_a"]
        part_root_key = MagicMock()
        part_root_key.path = ["part_root"]

        cad.parts = {part_a_key: MagicMock(), part_root_key: MagicMock()}
        cad.parts[part_a_key].isRigidAssembly = False
        cad.parts[part_a_key].partId = "part_a"
        cad.parts[part_root_key].isRigidAssembly = False
        cad.parts[part_root_key].partId = "part_root"
        cad.instances = {}
        cad.occurrences = {}

        # Mock path and module detection
        def to_path_side_effect(key):
            if key == part_a_key:
                return ("subasm", "part_a")
            if key == part_root_key:
                return ("part_root",)
            if key == "module_a":
                return ("subasm",)
            if hasattr(key, "path"):
                return tuple(key.path)
            return (str(key),)

        mock_to_path.side_effect = to_path_side_effect

        def get_module_side_effect(path, subassembly_keys):
            if path == ("subasm", "part_a"):
                return ("subasm",)
            return None

        mock_get_module.side_effect = get_module_side_effect

        link_a = MagicMock()
        link_a.keys = [part_a_key]
        link_a.part_names = ["part_a"]
        link_a.frame_transform = np.eye(4)

        link_root = MagicMock()
        link_root.keys = [part_root_key]
        link_root.part_names = ["part_root"]
        link_root.frame_transform = np.eye(4)

        link_records = {"link_a": link_a, "link_root": link_root}

        # Mock collecting shapes
        def populate_shapes(st, ct, lbl, loc, shapes, locations, colors, path=()):
            shapes[("subasm", "part_a")] = [MagicMock()]
            locations[("subasm", "part_a")] = [MagicMock()]
            colors[("subasm", "part_a")] = [None]

            shapes[("part_root",)] = [MagicMock()]
            locations[("part_root",)] = [MagicMock()]
            colors[("part_root",)] = [None]

        with patch(
            "onshape2xacro.mesh_exporters.step._collect_shapes",
            side_effect=populate_shapes,
        ):
            # RUN
            mesh_map, missing, report = exporter.export_link_meshes(
                link_records, mesh_dir, module_mesh_dirs=module_mesh_dirs
            )

            # VERIFY
            # link_a should be in module_a_meshes
            assert ("module_a", "link_a") in mesh_map
            assert mesh_map[("module_a", "link_a")]["visual"] == "visual/link_a.obj"
            assert (module_a_dir / "visual" / "link_a.obj").exists()

            # link_root should be in base_meshes
            assert (None, "link_root") in mesh_map
            assert mesh_map[(None, "link_root")]["visual"] == "visual/link_root.obj"
            assert (mesh_dir / "visual" / "link_root.obj").exists()

            # Verify directories were created
            assert (module_a_dir / "visual").exists()
            assert (module_a_dir / "collision").exists()
            assert (mesh_dir / "visual").exists()
            assert (mesh_dir / "collision").exists()


def test_export_link_meshes_non_modular(mock_step_env):
    exporter, cad, mesh_dir, tmp_path = mock_step_env

    # Mocks for export_link_meshes
    with (
        patch("onshape2xacro.mesh_exporters.step.BRepMesh_IncrementalMesh"),
        patch("onshape2xacro.mesh_exporters.step.StlAPI_Writer") as mock_writer,
        patch(
            "onshape2xacro.mesh_exporters.step.STEPCAFControl_Reader"
        ) as mock_reader_cls,
        patch("onshape2xacro.mesh_exporters.step.TDocStd_Document"),
        patch("onshape2xacro.mesh_exporters.step._get_shape_tool"),
        patch("onshape2xacro.mesh_exporters.step.XCAFDoc_DocumentTool"),
        patch(
            "onshape2xacro.mesh_exporters.step._get_free_shape_labels"
        ) as mock_labels,
        patch("onshape2xacro.mesh_exporters.step.IFSelect_RetDone", new=1),
        patch("onshape2xacro.mesh_exporters.step.BRepBuilderAPI_Transform"),
        patch("onshape2xacro.mesh_exporters.step.BRep_Builder"),
        patch("onshape2xacro.mesh_exporters.step.TopoDS_Compound"),
        patch("trimesh.load") as mock_trimesh_load,
        patch(
            "onshape2xacro.mesh_exporters.step._part_world_matrix",
            return_value=np.eye(4),
        ),
    ):
        mock_writer.return_value.Write.side_effect = lambda shape, path: Path(
            path
        ).touch()
        mock_reader_cls.return_value.ReadFile.return_value = 1
        mock_reader_cls.return_value.Transfer.return_value = True
        mock_labels.return_value.Length.return_value = 1
        mock_mesh = MagicMock()
        mock_trimesh_load.return_value = mock_mesh

        def export_side_effect(path, **kwargs):
            Path(path).touch()

        mock_mesh.export.side_effect = export_side_effect

        cad.subassemblies = {}
        part_root_key = MagicMock()
        part_root_key.path = ["part_root"]

        cad.parts = {part_root_key: MagicMock()}
        cad.parts[part_root_key].isRigidAssembly = False
        cad.parts[part_root_key].partId = "part_root"

        class MockInstance:
            def __init__(self, name):
                self.name = name

        cad.instances = {part_root_key: MockInstance("part_root_instance")}
        cad.occurrences = {}

        link_root = MagicMock()
        link_root.keys = [part_root_key]
        link_root.part_names = ["part_root"]
        link_root.frame_transform = np.eye(4)

        link_records = {"link_root": link_root}

        def populate_shapes(st, ct, lbl, loc, shapes, locations, colors, path=()):
            shapes[("part_root",)] = [MagicMock()]
            locations[("part_root",)] = [MagicMock()]
            colors[("part_root",)] = [None]

        with patch(
            "onshape2xacro.mesh_exporters.step._collect_shapes",
            side_effect=populate_shapes,
        ):
            # RUN
            mesh_map, missing, report = exporter.export_link_meshes(
                link_records, mesh_dir
            )

            # VERIFY
            # Should be keyed by link_name (string)
            assert "link_root" in mesh_map
            assert mesh_map["link_root"]["visual"] == "visual/link_root.obj"
            assert (mesh_dir / "visual" / "link_root.obj").exists()
