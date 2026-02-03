# Feature Documentation

## STEP Color Export (Implemented 2026-02-03)

### Overview
The `StepMeshExporter` now supports extracting material colors from STEP files and applying them to generated visual meshes (OBJ, DAE, GLB).

### Implementation Details
- **Extraction**: Uses `OCP.XCAFDoc.XCAFDoc_ColorTool` to retrieve color attributes from STEP labels and shapes.
  - Priority: Surface Color > Generic Color > Curve Color.
  - Checks both the assembly label and the underlying shape.
- **Application**:
  - For `stl` export: Fast path (geometry only), color is ignored.
  - For other formats (`obj`, `dae`):
    1. Iterates through all valid parts in a link.
    2. Exports each part to a temporary STL.
    3. Loads it back with `trimesh`.
    4. Applies the extracted color to `mesh.visual.face_colors`.
    5. Merges all parts using `trimesh.util.concatenate`.
    6. Exports the combined mesh.

### Testing Strategy
- **Mocking**: OCP classes are C++ bindings and hard to instantiate in tests. We use `unittest.mock` to patch:
  - `XCAFDoc_DocumentTool`
  - `Quantity_Color`
  - `STEPCAFControl_Reader`
- **Coverage**:
  - `tests/test_mesh_exporters/test_step_color.py`: Dedicated tests for color extraction logic (`_get_color`, `_collect_shapes`) and the full export flow.
