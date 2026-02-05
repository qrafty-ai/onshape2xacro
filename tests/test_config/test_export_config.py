from pathlib import Path
from onshape2xacro.config.export_config import ExportConfiguration


def test_default_config():
    config = ExportConfiguration()
    assert config.export.name == "robot"
    assert config.export.visual_option.formats == ["obj"]
    assert config.export.visual_option.max_size_mb == 10.0
    assert config.export.collision_option.method == "fast"
    assert config.export.output == Path("output")
    assert config.mate_values == {}
    assert config.link_names == {}


def test_save_and_load(tmp_path):
    config_path = tmp_path / "config.yaml"
    config = ExportConfiguration()
    config.export.name = "my_robot"
    config.mate_values = {"mate1": {"rotation": 1.0}}
    config.link_names = {"link1": "custom1"}

    config.save(config_path)
    assert config_path.exists()

    loaded = ExportConfiguration.load(config_path)
    assert loaded.export.name == "my_robot"
    assert loaded.mate_values == {"mate1": {"rotation": 1.0}}
    assert loaded.link_names == {"link1": "custom1"}
    assert isinstance(loaded.export.output, Path)


def test_load_non_existent():
    config = ExportConfiguration.load(Path("non_existent.yaml"))
    assert config.export.name == "robot"


def test_merge_cli_overrides():
    config = ExportConfiguration()
    config.merge_cli_overrides(
        name="overridden", output=Path("new_output"), visual_mesh_formats=["stl"]
    )
    assert config.export.name == "overridden"
    assert config.export.output == Path("new_output")
    assert config.export.visual_option.formats == ["stl"]


def test_partial_merge():
    config = ExportConfiguration()
    config.merge_cli_overrides(name="only_name")
    assert config.export.name == "only_name"
    assert config.export.output == Path("output")
    assert config.export.visual_option.formats == ["obj"]
