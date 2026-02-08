# mesh_exporters MODULE

## OVERVIEW
STEP → mesh conversion with color extraction. Primary module for processing CAD geometry into visual/collision meshes using OCP (Open CASCADE) C++ bindings.

## STRUCTURE
- `step.py` (1209 lines): StepMeshExporter class, 30+ methods
- `__init__.py`: Module exports

## WHERE TO LOOK

| Task | Symbol | Notes |
|------|--------|-------|
| Main exporter | `StepMeshExporter` | 30+ methods, handles STEP → OBJ/DAE/STL/GLB |
| Color extraction | `_get_color()` | Priority: Surface > Generic > Curve |
| Shape collection | `_collect_shapes()` | Recursive assembly tree traversal |
| CoACD decomposition | `_process_coacd_task()` | Multiprocess convex decomposition |
| Visual compression | `_compress_visual_mesh()` | pymeshlab quadric edge collapse |
| Transform math | `_matrix_to_trsf()` | numpy → OCP coordinate conversion |

## CONVENTIONS

### OCP Integration
- **C++ Bindings**: OCP classes (`XCAFDoc_DocumentTool`, `Quantity_Color`, `STEPCAFControl_Reader`) are C++ objects
- **Document Model**: STEP files loaded into XDE Document with labels forming assembly tree
- **Location Tracking**: Each part has `TopLoc_Location` defining position in world frame

### Color Extraction
- **Priority Order**: Surface color > Generic color > Curve color
- **RGB Conversion**: OCP `Quantity_Color` → RGB [0-1] → uint8 [0-255]
- **Format Support**: STL ignores color (fast path), OBJ/DAE/GLB preserve color

### Export Formats
- **STL**: Fast geometry-only export (no color, no merging)
- **OBJ**: Color via material library (`.mtl` file)
- **DAE**: Collada with embedded materials
- **GLB**: Binary glTF with embedded textures

## ANTI-PATTERNS

- **NO STEP export without Client**: Line 494 raises `RuntimeError` if `self.client` not set
- **NO direct OCP instantiation in tests**: ALWAYS mock (see `tests/test_mesh_exporters/AGENTS.md`)
- **NO skipping shape tool**: Every STEP operation requires `_get_shape_tool(doc)` first

## UNIQUE PATTERNS

### Multiprocess CoACD
- Launches worker pools for convex decomposition
- Each part processed independently
- Results merged into single collision mesh directory

### Recursive Assembly Traversal
- `_collect_shapes()` walks label tree depth-first
- Accumulates transforms as it descends
- Filters out empty/invalid shapes

### Trimesh Merging
- Exports each part to temp STL
- Loads with trimesh + applies color
- Concatenates all parts into single mesh
- Preserves per-face colors via `face_colors` array
