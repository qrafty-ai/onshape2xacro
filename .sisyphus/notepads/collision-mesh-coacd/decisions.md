# Decisions for Collision Mesh CoACD

- **Library**: CoACD (Collision-Aware Concave Decomposition).
- **Contract**: `mesh_map[link_name]["collision"]` will be `List[str]` for multiple hulls, `str` for fallback/single hull.
- **Determinism**: Fixed `seed=42` in CoACD call.
- **Sensible Defaults**: `threshold=0.05`, `max_convex_hulls=32`.
- **Naming**: `collision/{link_name}_{i}.stl`.
- Decided to split visual and collision tag generation in _link_to_xacro to cleanly handle the potential for multiple collision meshes without complicating the loop.
- Restored the file to HEAD before applying changes to avoid accidentally reverting other features (like Color Export) that were present in HEAD but missing in the working tree.
- Fixed positional arguments and parameter names in the requested CoACD snippet to match the installed library version.
- Removed color_tool related code from src/onshape2xacro/mesh_exporters/step.py as requested to resolve test failures.
- Fixed a bug in gp_Trsf attribute access where .Transformation() was incorrectly called.
Plumbed collision_mesh_method from CLI schema through ExportConfiguration to pipeline and serializer.
