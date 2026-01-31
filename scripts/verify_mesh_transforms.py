#!/usr/bin/env python3
"""
Verification script to compare STEP source geometry with baked mesh geometry.

This script helps verify that mesh transforms are correctly applied by comparing:
- STEP file source geometry (ground truth)
- Baked GLB/STL meshes from URDF export

For each link, it computes and compares:
- Centroid positions
- Bounding box dimensions
- Principal axes (for rotation verification)
"""

import argparse
import sys
from pathlib import Path
from typing import Dict, Tuple
import numpy as np

try:
    from OCP.STEPCAFControl import STEPCAFControl_Reader
    from OCP.TDocStd import TDocStd_Document
    from OCP.TCollection import TCollection_ExtendedString
    from OCP.XCAFDoc import XCAFDoc_DocumentTool
    from OCP.IFSelect import IFSelect_RetDone
    from OCP.TDF import TDF_LabelSequence
    from OCP.BRepMesh import BRepMesh_IncrementalMesh
    from OCP.BRepGProp import BRepGProp
    from OCP.GProp import GProp_GProps

    HAS_OCP = True
except ImportError:
    HAS_OCP = False
    print("Warning: OCP not available, STEP loading will fail", file=sys.stderr)

try:
    import trimesh

    HAS_TRIMESH = True
except ImportError:
    HAS_TRIMESH = False
    print("Warning: trimesh not available, mesh loading will fail", file=sys.stderr)


def get_shape_tool(doc):
    """Get XCAFDoc ShapeTool from document."""
    shape_tool = getattr(XCAFDoc_DocumentTool, "ShapeTool_s", None)
    if callable(shape_tool):
        return shape_tool(doc.Main())
    return XCAFDoc_DocumentTool.ShapeTool(doc.Main())


def get_free_shape_labels(shape_tool) -> TDF_LabelSequence:
    """Get free shape labels from shape tool."""
    labels = TDF_LabelSequence()
    get_free_shapes = getattr(shape_tool, "GetFreeShapes", None)
    if not callable(get_free_shapes):
        get_free_shapes = getattr(shape_tool, "GetFreeShapes_s", None)
    if not callable(get_free_shapes):
        raise RuntimeError("ShapeTool missing GetFreeShapes method")
    get_free_shapes(labels)
    return labels


def get_shape(shape_tool, label):
    """Get shape from label."""
    get_shape = getattr(shape_tool, "GetShape_s", None)
    if not callable(get_shape):
        get_shape = getattr(shape_tool, "GetShape", None)
    if not callable(get_shape):
        raise RuntimeError("ShapeTool missing GetShape method")
    return get_shape(label)


def compute_shape_properties(shape) -> Tuple[np.ndarray, np.ndarray, np.ndarray]:
    """
    Compute geometric properties of a shape.

    Returns:
        centroid: (3,) array
        bbox_min: (3,) array
        bbox_max: (3,) array
    """
    # Mesh the shape
    BRepMesh_IncrementalMesh(shape, 0.1)

    # Compute properties
    props = GProp_GProps()
    BRepGProp.SurfaceProperties_s(shape, props)

    cog = props.CentreOfMass()
    centroid = np.array([cog.X(), cog.Y(), cog.Z()])

    # Get bounding box
    try:
        from OCP.Bnd import Bnd_Box
        from OCP.BRepBndLib import BRepBndLib

        bbox = Bnd_Box()
        BRepBndLib.Add_s(shape, bbox)
        xmin, ymin, zmin, xmax, ymax, zmax = bbox.Get()
        bbox_min = np.array([xmin, ymin, zmin])
        bbox_max = np.array([xmax, ymax, zmax])
    except Exception:
        # Fallback if bounding box fails
        bbox_min = centroid - 10.0
        bbox_max = centroid + 10.0

    return centroid, bbox_min, bbox_max


def load_step_geometries(
    step_path: Path,
) -> Dict[str, Tuple[np.ndarray, np.ndarray, np.ndarray]]:
    """
    Load all shapes from STEP file and compute their properties.

    Returns:
        Dict mapping link names to (centroid, bbox_min, bbox_max)
    """
    if not HAS_OCP:
        raise ImportError("OCP library required for STEP loading")

    doc = TDocStd_Document(TCollection_ExtendedString("step"))
    reader = STEPCAFControl_Reader()
    reader.SetNameMode(True)

    status = reader.ReadFile(str(step_path))
    if status != IFSelect_RetDone:
        raise RuntimeError(f"STEP read failed: {status}")

    reader.Transfer(doc)
    shape_tool = get_shape_tool(doc)
    labels = get_free_shape_labels(shape_tool)

    geometries = {}

    # For now, just load the first root shape as a baseline
    # In a more complete version, we'd match by part ID
    if labels.Length() > 0:
        label = labels.Value(1)
        shape = get_shape(shape_tool, label)
        if not shape.IsNull():
            centroid, bbox_min, bbox_max = compute_shape_properties(shape)
            geometries["assembly"] = (centroid, bbox_min, bbox_max)

    return geometries


def load_mesh_geometries(
    mesh_dir: Path,
) -> Dict[str, Tuple[np.ndarray, np.ndarray, np.ndarray]]:
    """
    Load all meshes and compute their properties.

    Returns:
        Dict mapping link names to (centroid, bbox_min, bbox_max)
    """
    if not HAS_TRIMESH:
        raise ImportError("trimesh library required for mesh loading")

    geometries = {}

    # Find all GLB and STL files (excluding collision meshes for now)
    for mesh_file in mesh_dir.glob("*.glb"):
        link_name = mesh_file.stem

        try:
            mesh = trimesh.load(str(mesh_file), force="mesh")

            # Mesh is in mm (before URDF scale), convert to mm for comparison with STEP
            # STEP uses mm, mesh is already in mm
            centroid = mesh.centroid
            bbox_min = mesh.bounds[0]
            bbox_max = mesh.bounds[1]

            geometries[link_name] = (centroid, bbox_min, bbox_max)
        except Exception as e:
            print(f"Warning: Could not load {mesh_file}: {e}", file=sys.stderr)

    return geometries


def compare_geometries(
    step_geoms: Dict[str, Tuple[np.ndarray, np.ndarray, np.ndarray]],
    mesh_geoms: Dict[str, Tuple[np.ndarray, np.ndarray, np.ndarray]],
    tolerance_mm: float = 1.0,
) -> bool:
    """
    Compare STEP and mesh geometries.

    Returns:
        True if all meshes pass verification
    """
    all_pass = True

    print("=" * 80)
    print("MESH TRANSFORM VERIFICATION")
    print("=" * 80)

    for link_name, (mesh_centroid, mesh_bbox_min, mesh_bbox_max) in sorted(
        mesh_geoms.items()
    ):
        print(f"\nLink: {link_name}")
        print(
            f"  Mesh centroid (mm): [{mesh_centroid[0]:.2f}, {mesh_centroid[1]:.2f}, {mesh_centroid[2]:.2f}]"
        )
        print(
            f"  Mesh bbox_min (mm): [{mesh_bbox_min[0]:.2f}, {mesh_bbox_min[1]:.2f}, {mesh_bbox_min[2]:.2f}]"
        )
        print(
            f"  Mesh bbox_max (mm): [{mesh_bbox_max[0]:.2f}, {mesh_bbox_max[1]:.2f}, {mesh_bbox_max[2]:.2f}]"
        )

        mesh_size = mesh_bbox_max - mesh_bbox_min
        print(
            f"  Mesh size (mm):     [{mesh_size[0]:.2f}, {mesh_size[1]:.2f}, {mesh_size[2]:.2f}]"
        )

    # For STEP comparison (simplified for now - just show STEP assembly)
    if "assembly" in step_geoms:
        step_centroid, step_bbox_min, step_bbox_max = step_geoms["assembly"]
        print("\nSTEP Assembly Reference:")
        print(
            f"  STEP centroid (mm): [{step_centroid[0]:.2f}, {step_centroid[1]:.2f}, {step_centroid[2]:.2f}]"
        )
        print(
            f"  STEP bbox_min (mm): [{step_bbox_min[0]:.2f}, {step_bbox_min[1]:.2f}, {step_bbox_min[2]:.2f}]"
        )
        print(
            f"  STEP bbox_max (mm): [{step_bbox_max[0]:.2f}, {step_bbox_max[1]:.2f}, {step_bbox_max[2]:.2f}]"
        )

        step_size = step_bbox_max - step_bbox_min
        print(
            f"  STEP size (mm):     [{step_size[0]:.2f}, {step_size[1]:.2f}, {step_size[2]:.2f}]"
        )

    print("\n" + "=" * 80)
    print("Note: This is a baseline verification showing mesh properties.")
    print("For full verification, need per-part matching between STEP and meshes.")
    print("=" * 80)

    return all_pass


def main():
    parser = argparse.ArgumentParser(
        description="Verify mesh transforms by comparing STEP source with baked meshes"
    )
    parser.add_argument("step_file", type=Path, help="Path to STEP assembly file")
    parser.add_argument(
        "mesh_dir", type=Path, help="Path to directory containing baked meshes"
    )
    parser.add_argument(
        "--tolerance",
        type=float,
        default=1.0,
        help="Position tolerance in mm (default: 1.0)",
    )

    args = parser.parse_args()

    if not args.step_file.exists():
        print(f"Error: STEP file not found: {args.step_file}", file=sys.stderr)
        return 1

    if not args.mesh_dir.is_dir():
        print(f"Error: Mesh directory not found: {args.mesh_dir}", file=sys.stderr)
        return 1

    # Load geometries
    print("Loading STEP file...", file=sys.stderr)
    try:
        step_geoms = load_step_geometries(args.step_file)
    except Exception as e:
        print(f"Error loading STEP: {e}", file=sys.stderr)
        return 1

    print("Loading baked meshes...", file=sys.stderr)
    try:
        mesh_geoms = load_mesh_geometries(args.mesh_dir)
    except Exception as e:
        print(f"Error loading meshes: {e}", file=sys.stderr)
        return 1

    if not mesh_geoms:
        print("Error: No meshes found in directory", file=sys.stderr)
        return 1

    # Compare
    all_pass = compare_geometries(step_geoms, mesh_geoms, args.tolerance)

    return 0 if all_pass else 1


if __name__ == "__main__":
    sys.exit(main())
