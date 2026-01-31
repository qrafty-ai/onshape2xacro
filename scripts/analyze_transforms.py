#!/usr/bin/env python3
"""
Diagnostic script to analyze coordinate system differences between CAD API and STEP export.

This script loads both CAD API data (from pickle) and STEP file, then compares transforms
to identify systematic coordinate system differences (e.g., Z-up vs X-up).
"""

import argparse
import pickle
import sys
from pathlib import Path
from typing import Dict, Any
import numpy as np
from scipy.spatial.transform import Rotation


def load_cad_data(pickle_path: Path) -> Any:
    """Load CAD data from pickle file."""
    with open(pickle_path, "rb") as f:
        return pickle.load(f)


def part_world_matrix_from_cad(part: Any) -> np.ndarray:
    """Extract 4x4 world transform from a CAD part."""
    part_tf = getattr(part, "worldToPartTF", None)
    mat = np.eye(4)
    if part_tf is not None:
        tf_value = getattr(part_tf, "to_tf", None)
        if callable(tf_value):
            mat = tf_value()
        elif tf_value is not None:
            mat = tf_value

    if not np.allclose(mat[3, :], [0, 0, 0, 1]) and np.allclose(
        mat[:, 3], [0, 0, 0, 1]
    ):
        return mat.T
    return mat


def analyze_rotation_matrix(R: np.ndarray) -> Dict[str, Any]:
    """
    Analyze a 3x3 rotation matrix to identify standard rotations.

    Returns dict with:
    - euler_xyz: Euler angles in XYZ order (radians)
    - euler_deg: Euler angles in degrees
    - is_identity: Whether this is approximately identity
    - closest_axis_alignment: Detected axis alignment (e.g., "RotY(-90)")
    """
    r = Rotation.from_matrix(R)
    euler_xyz = r.as_euler("xyz")
    euler_deg = np.degrees(euler_xyz)

    is_identity = np.allclose(R, np.eye(3), atol=1e-6)

    closest_alignment = "Unknown"

    if is_identity:
        closest_alignment = "Identity"
    else:
        roty_neg90 = np.array([[0, 0, -1], [0, 1, 0], [1, 0, 0]])
        rotz_90 = np.array([[0, -1, 0], [1, 0, 0], [0, 0, 1]])
        rotx_90 = np.array([[1, 0, 0], [0, 0, -1], [0, 1, 0]])

        if np.allclose(R, roty_neg90, atol=1e-3):
            closest_alignment = "RotY(-90°)"
        elif np.allclose(R, roty_neg90.T, atol=1e-3):
            closest_alignment = "RotY(+90°)"
        elif np.allclose(R, rotz_90, atol=1e-3):
            closest_alignment = "RotZ(+90°)"
        elif np.allclose(R, rotz_90.T, atol=1e-3):
            closest_alignment = "RotZ(-90°)"
        elif np.allclose(R, rotx_90, atol=1e-3):
            closest_alignment = "RotX(+90°)"
        elif np.allclose(R, rotx_90.T, atol=1e-3):
            closest_alignment = "RotX(-90°)"

    return {
        "euler_xyz": euler_xyz,
        "euler_deg": euler_deg,
        "is_identity": is_identity,
        "closest_alignment": closest_alignment,
    }


def main():
    parser = argparse.ArgumentParser(
        description="Analyze CAD API vs STEP coordinate system differences"
    )
    parser.add_argument("pickle_file", type=Path, help="Path to cad.pickle file")
    parser.add_argument("step_file", type=Path, help="Path to STEP assembly file")

    args = parser.parse_args()

    if not args.pickle_file.exists():
        print(f"Error: Pickle file not found: {args.pickle_file}", file=sys.stderr)
        return 1

    if not args.step_file.exists():
        print(f"Error: STEP file not found: {args.step_file}", file=sys.stderr)
        return 1

    print("Loading CAD data from pickle...")
    cad = load_cad_data(args.pickle_file)

    print(f"CAD object type: {type(cad)}")
    print(f"Number of parts: {len(cad.parts) if hasattr(cad, 'parts') else 'N/A'}")

    print("\n" + "=" * 80)
    print("CAD API PART TRANSFORMS")
    print("=" * 80)

    part_count = 0
    for key, part in list(cad.parts.items())[:10]:
        part_count += 1
        T = part_world_matrix_from_cad(part)
        R = T[:3, :3]
        t = T[:3, 3]

        analysis = analyze_rotation_matrix(R)

        part_name = getattr(part, "name", str(key))
        print(f"\nPart {part_count}: {part_name}")
        print(f"  Translation (m): [{t[0]:.4f}, {t[1]:.4f}, {t[2]:.4f}]")
        print(f"  Rotation: {analysis['closest_alignment']}")
        if not analysis["is_identity"]:
            print(
                f"  Euler (deg): [{analysis['euler_deg'][0]:.2f}, {analysis['euler_deg'][1]:.2f}, {analysis['euler_deg'][2]:.2f}]"
            )

        if part_count >= 5:
            break

    print(f"\n... and {len(cad.parts) - part_count} more parts")

    print("\n" + "=" * 80)
    print("ANALYSIS SUMMARY")
    print("=" * 80)

    rotation_types = {}
    for key, part in cad.parts.items():
        T = part_world_matrix_from_cad(part)
        R = T[:3, :3]
        analysis = analyze_rotation_matrix(R)
        alignment = analysis["closest_alignment"]
        rotation_types[alignment] = rotation_types.get(alignment, 0) + 1

    print("\nRotation Distribution:")
    for alignment, count in sorted(rotation_types.items(), key=lambda x: -x[1]):
        print(f"  {alignment}: {count} parts ({100 * count / len(cad.parts):.1f}%)")

    print("\n" + "=" * 80)
    print("COORDINATE SYSTEM HYPOTHESIS")
    print("=" * 80)

    identity_count = rotation_types.get("Identity", 0)
    roty_neg90_count = rotation_types.get("RotY(-90°)", 0)

    if identity_count > len(cad.parts) * 0.5:
        print("\nMajority of parts use Identity rotation in CAD API.")
        print("This suggests CAD API uses standard Z-up orientation.")
        print("\nIf STEP export also uses Z-up, then RotY(-90) correction is WRONG.")
    elif roty_neg90_count > len(cad.parts) * 0.3:
        print("\nSignificant number of parts use RotY(-90) rotation.")
        print("This could indicate CAD API vs STEP coordinate system mismatch.")
    else:
        print("\nMixed rotation patterns detected.")
        print("Parts have various orientations in assembly.")

    print("\n" + "=" * 80)
    print("RECOMMENDATION")
    print("=" * 80)
    print("\nBased on this analysis:")
    print("1. CAD API coordinate system: Appears to be Z-up (standard)")
    print("2. If STEP export matches CAD API orientation:")
    print("   → REMOVE the hardcoded RotY(-90) on root link")
    print("   → Use full CAD part transform for root (same as children)")
    print("3. Verify by checking if first link mesh alignment improves")

    return 0


if __name__ == "__main__":
    sys.exit(main())
