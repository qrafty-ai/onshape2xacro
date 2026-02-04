from onshape2xacro.config.export_config import ExportConfiguration


def test_config_load_sanitization(tmp_path):
    # Create a config file with hyphenated keys
    config_path = tmp_path / "config.yaml"
    config_content = """
export:
  name: test_robot
  collision_option:
    method: coacd
    coacd:
      max-workers: 20
      max-convex-hull: 16
"""
    config_path.write_text(config_content)

    # Load config
    config = ExportConfiguration.load(config_path)

    # Verify sanitization (hyphens -> underscores)
    assert config.export.collision_option.coacd.max_workers == 20
    assert config.export.collision_option.coacd.max_convex_hull == 16


def test_config_load_unknown_keys(tmp_path):
    # Create a config file with unknown keys (should be filtered out)
    config_path = tmp_path / "config.yaml"
    config_content = """
export:
  collision_option:
    method: coacd
    coacd:
      unknown-key: 123
      max-workers: 10
"""
    config_path.write_text(config_content)

    # Load config
    config = ExportConfiguration.load(config_path)

    # Verify valid key exists
    assert config.export.collision_option.coacd.max_workers == 10
    # No error should be raised for unknown key
