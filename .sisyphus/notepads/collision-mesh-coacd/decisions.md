Added coacd (v1.0.7) as a dependency to support collision-aware concave decomposition. Verified successful import in the project environment.

# CoACD Integration (Implemented 2026-02-03)

## Decisions
- **API Adaptation**: Used `coacd.run_coacd` with `max_convex_hull=32` (singular) instead of `max_convex_hulls` (plural) after API inspection.
- **Fallback Strategy**: Implemented robust fallback: CoACD -> pymeshlab convex hull -> raw STL copy.
- **Data Structure**: Updated `mesh_map` schema to support `List[str]` for collision meshes, allowing multiple convex hulls per link.
- **Serializer Update**: Updated `XacroSerializer` to handle list of collision meshes by generating multiple `<collision>` elements.

## Verification
- **Test**: Created `tests/test_mesh_exporters/test_step_coacd.py` verifying full pipeline from STEP input to multiple collision mesh files.
- **Result**: Successfully generated decomposed meshes (verified via file existence and naming convention `link_name_{i}.stl`).
