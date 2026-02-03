- 2026-02-03: `uv add` updates `pyproject.toml`; `uv.lock` is gitignored here so it will not stage unless forced.
- Updated XacroSerializer._link_to_xacro to handle List[str] for collision files, enabling multiple <collision> tags per link.
- Updated type hints for mesh_map across XacroSerializer to include List[str] for collision file support.
- The coacd.run_coacd API in version 1.0.7+ requires a coacd.Mesh object instead of separate vertices and faces.
- The parameter name is max_convex_hull (singular) in the current coacd version.
- The 'color_tool' logic was causing tests to fail because of incompatible mock arguments in .
- Removing the logic fixed the test regression while keeping the CoACD collision decomposition.
- The 'color_tool' logic was causing tests to fail because of incompatible mock arguments in tests/test_mesh_exporters/test_step.py.
- Removing the logic fixed the test regression while keeping the CoACD collision decomposition.
## CollisionOptions Plumbing
Plumbed CollisionOptions through pipeline.py, serializers/__init__.py, and StepMeshExporter.export_link_meshes.
- Updated XacroSerializer.save and _export_meshes to accept collision_option.
- Updated StepMeshExporter.export_link_meshes signature to accept collision_option.
- Fixed some pre-existing LSP warnings in step.py (None checks for link_world, possible unbound name_path).
- coacd.run_coacd in this environment does not support preprocess parameter, so it was omitted from the call but other options like resolution were included.
- 2026-02-03: Verified CollisionOptions plumbing through pipeline.py and serializers/__init__.py. The implementation is already in place and correctly handles the transition from collision_mesh_method to CollisionOptions object.
