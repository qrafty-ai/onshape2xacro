# Analysis: `export_link_meshes` in `step.py`

## Overview
The `export_link_meshes` method is the core engine for converting Onshape assembly data into robot description assets (meshes and inertial properties). It handles the complex bridging between Onshape's metadata (Part IDs, transforms) and the geometric data in STEP files.

## Signature
```python
def export_link_meshes(
    self,
    link_records: Dict[str, Any],     # Mapping of link names to Onshape parts/frames
    mesh_dir: Path,                   # Output directory for meshes
    bom_path: Optional[Path] = None,  # Optional BOM for mass properties
) -> Tuple[
    Dict[str, str | Dict[str, str]],  # mesh_map: link -> {"visual": "x.glb", "collision": "y.stl"}
    Dict[str, List[Dict[str, str]]],  # missing_meshes: errors per link
    Optional["InertiaReport"],        # Calculated mass properties
]
```

## Key Workflows

### 1. STEP File Acquisition & Validation
- Checks for existing `assembly.step` or `asset_path`.
- If missing, calls `self.export_step()` to fetch from Onshape.
- **Self-Healing**: Validates that the STEP file contains "Occurrence Export IDs". If missing (common in generic exports), it forces a re-export with `includeExportIds=True` to ensure parts can be reliably identified.

### 2. Geometry Parsing (Open Cascade)
- Uses `STEPCAFControl_Reader` to load the STEP file.
- Harvests all shapes into a lookup table (`part_shapes`) indexed by multiple keys:
  - Full Occurrence Path (tuple of IDs)
  - Part ID
  - Instance Name (e.g., "Part <1>")
  - Label Name

### 3. Link Processing Loop
Iterates through each `link` in `link_records`:
- **Transform Normalization**: Calculates the relative transform from the **Link Frame** to each **Part Frame**. This ensures the generated mesh origin matches the URDF link origin.
- **Shape Matching (The "5-Strategy Fallback")**:
  Tries to find the geometry for each part using progressively looser criteria:
  1. `part_path`: Exact occurrence path match (most reliable).
  2. `inst_name`: Instance name (e.g., "Bracket <1>").
  3. `name_path`: Hierarchy of names.
  4. `leaf_name`: Normalized part name.
  5. `part_id`: Raw Part ID (fallback for single-instance parts).
- **Compound Creation**: Fuses all parts for a link into a single `TopoDS_Compound`.

### 4. Asset Generation
For each valid link compound:
- **Inertial Calculation**: If `bom_path` is present, exports a temp STEP of the link and computes mass/inertia using `InertiaCalculator`.
- **Visual Mesh (GLB)**:
  - Generates a high-res STL.
  - Uses `trimesh` to convert STL -> **GLB** (smaller, web-ready).
- **Collision Mesh (Convex Hull)**:
  - Uses `pymeshlab` to generate a **Convex Hull**.
  - Decimates the hull to ~200 faces for physics performance.
  - Fallback: Uses raw STL if hull generation fails.

## Returns
- **mesh_map**: Paths to generated assets.
- **missing_meshes**: Detailed report of any parts that existed in Onshape metadata but couldn't be found in the STEP file.
- **inertia_report**: Mass, center of mass, and inertia tensor for each link.
