# Draft: Fix Part Instance Swapping in STEP Export

## Requirements (confirmed)
- Fix the issue where identical part instances are swapped between robot links.
- Use physical location (world position) to match Onshape parts with STEP instances.
- Implementation should be in `src/onshape2xacro/mesh_exporters/step.py`.

## Technical Decisions
- Extract 3D translation from `gp_Trsf` (via `TopLoc_Location` in OpenCASCADE).
- Implement a `DistanceSelector` or modify `_pick_match` to use world coordinates.
- Maintain a set of used indices for each `mkey` to prevent double-assignment.
- Onshape part world position can be obtained from `_part_world_matrix(part)`.

## Research Findings
- `StepMeshExporter.export_link_meshes` iterates over links and then parts.
- `part_locations` stores `TopLoc_Location` for each STEP instance found in the document.
- `used_indices` is currently an integer counter per `mkey` (line 456-457).
- `_part_world_matrix(part)` extracts a 4x4 matrix from Onshape part objects (line 270).
- `link_matrix` is scaled by 1000.0 (line 417), confirming Onshape uses meters and STEP/OCP uses millimeters.
- `gp_Trsf` has a `Transforms()` or `TranslationPart()` equivalent method (in OCP it might be `Transforms` returning translation and rotation or just accessing the matrix). Actually, `gp_Trsf` has `TranslationPart()` returning a `gp_Vec` or `gp_XYZ`.

## Open Questions
- **Distance Threshold**: Should there be a maximum distance threshold to prevent matching a part to a completely wrong instance if something is missing?
- **Orientation Matching**: Is position sufficient, or should we also consider orientation? (Usually position is enough for distinct instances).
- **Unit Consistency**: I will ensure all positions are in millimeters during comparison.

## Scope Boundaries
- INCLUDE: Modifying `src/onshape2xacro/mesh_exporters/step.py`.
- INCLUDE: Distance-based matching logic.
- EXCLUDE: Changes to Onshape API or robotics toolkit.
- EXCLUDE: Global optimization (Hungarian algorithm) unless distance-based matching is insufficient.
