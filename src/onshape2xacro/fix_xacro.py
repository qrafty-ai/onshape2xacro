#!/usr/bin/env python3
"""
Fix common issues in exported xacro files:
- Joints with axis and limit but type="fixed" -> change to "revolute"
- Missing axis elements for revolute joints
"""

import sys
import argparse
from pathlib import Path
import xml.etree.ElementTree as ET


def fix_joint_types(root: ET.Element) -> int:
    """Fix joints that have axis and limit but are marked as fixed."""
    fixed_count = 0

    for joint in root.iter("joint"):
        joint_type = joint.get("type", "fixed")
        axis_elem = joint.find("axis")
        limit_elem = joint.find("limit")

        # If joint has axis and limit but is marked as fixed, it should be revolute
        if joint_type == "fixed" and axis_elem is not None and limit_elem is not None:
            joint.set("type", "revolute")
            fixed_count += 1
            joint_name = joint.get("name", "unknown")
            print(f"  Fixed: {joint_name} (has axis+limit but was fixed -> revolute)")

    return fixed_count


def fix_missing_axes(root: ET.Element) -> int:
    """Add missing axis elements for revolute/prismatic joints."""
    added_count = 0

    for joint in root.iter("joint"):
        joint_type = joint.get("type", "fixed")
        axis_elem = joint.find("axis")

        # Revolute/prismatic joints need axis
        if joint_type in ["revolute", "prismatic"] and axis_elem is None:
            ET.SubElement(joint, "axis", xyz="0 0 1")
            added_count += 1
            joint_name = joint.get("name", "unknown")
            print(f"  Added axis: {joint_name} (revolute/prismatic without axis)")

    return added_count


def fix_xacro_file(xacro_path: Path, output_path: Path = None) -> bool:
    """Fix issues in a xacro file."""
    if not xacro_path.exists():
        print(f"Error: File not found: {xacro_path}")
        return False

    print(f"Reading: {xacro_path}")
    tree = ET.parse(xacro_path)
    root = tree.getroot()

    print("Fixing issues...")
    fixed_joints = fix_joint_types(root)
    added_axes = fix_missing_axes(root)

    if fixed_joints == 0 and added_axes == 0:
        print("  No issues found!")
    else:
        print("\nSummary:")
        print(f"  - Fixed {fixed_joints} joint type(s)")
        print(f"  - Added {added_axes} axis element(s)")

    # Write fixed file
    output = output_path or xacro_path
    tree.write(output, encoding="unicode", xml_declaration=True)
    print(f"\n✓ Fixed file saved to: {output}")

    return True


def fix_all_xacro_files(directory: Path, recursive: bool = True) -> int:
    """Fix all xacro files in a directory."""
    if recursive:
        xacro_files = list(directory.rglob("*.xacro"))
    else:
        xacro_files = list(directory.glob("*.xacro"))

    if not xacro_files:
        print(f"No xacro files found in {directory}")
        return 0

    print(f"Found {len(xacro_files)} xacro file(s)")
    print("=" * 70)

    fixed_count = 0
    for xacro_file in xacro_files:
        print(f"\nProcessing: {xacro_file.relative_to(directory)}")
        if fix_xacro_file(xacro_file):
            fixed_count += 1

    print("\n" + "=" * 70)
    print(f"Fixed {fixed_count} file(s)")
    return fixed_count


def main():
    """Main entry point."""
    parser = argparse.ArgumentParser(
        description="Fix common issues in exported xacro files"
    )
    parser.add_argument(
        "path", type=Path, help="Path to xacro file or directory containing xacro files"
    )
    parser.add_argument(
        "--output", "-o", type=Path, help="Output file path (only for single file mode)"
    )
    parser.add_argument(
        "--recursive",
        "-r",
        action="store_true",
        default=True,
        help="Recursively process directories (default: True)",
    )

    args = parser.parse_args()

    path = args.path.resolve()

    if not path.exists():
        print(f"Error: Path not found: {path}")
        sys.exit(1)

    if path.is_file():
        # Single file mode
        fix_xacro_file(path, args.output)
    elif path.is_dir():
        # Directory mode
        fix_all_xacro_files(path, args.recursive)
    else:
        print(f"Error: Invalid path: {path}")
        sys.exit(1)


if __name__ == "__main__":
    main()
