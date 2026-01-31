#!/usr/bin/env python3
"""
Visualize exported robot description to verify it matches Onshape assembly.
Shows links, joints, hierarchy, and mesh files.
"""

import sys
import xml.etree.ElementTree as ET
from pathlib import Path
from typing import Dict, List, Tuple
import argparse

try:
    import matplotlib.pyplot as plt
    import matplotlib.patches as mpatches
    from matplotlib.patches import FancyArrowPatch

    HAS_MATPLOTLIB = True
except ImportError:
    HAS_MATPLOTLIB = False
    print("Warning: matplotlib not available. Install with: uv pip install matplotlib")


def parse_xacro_file(file_path: Path) -> Tuple[ET.Element, Dict]:
    """Parse xacro file and extract structure."""
    tree = ET.parse(file_path)
    root = tree.getroot()

    # Extract links
    links = {}
    for link in root.iter("link"):
        name = link.get("name", "").replace("${prefix}", "")
        links[name] = {
            "name": name,
            "has_visual": len(list(link.iter("visual"))) > 0,
            "has_collision": len(list(link.iter("collision"))) > 0,
            "has_mesh": len(list(link.iter("mesh"))) > 0,
        }

    # Extract joints
    joints = []
    for joint in root.iter("joint"):
        joint_name = joint.get("name", "").replace("${prefix}", "")
        joint_type = joint.get("type", "unknown")
        parent = None
        child = None

        parent_elem = joint.find("parent")
        child_elem = joint.find("child")

        if parent_elem is not None:
            parent = parent_elem.get("link", "").replace("${prefix}", "")
        if child_elem is not None:
            child = child_elem.get("link", "").replace("${prefix}", "")

        joints.append(
            {
                "name": joint_name,
                "type": joint_type,
                "parent": parent,
                "child": child,
            }
        )

    # Extract includes
    includes = []
    for inc in root.iter("{http://www.ros.org/wiki/xacro}include"):
        filename = inc.get("filename", "")
        includes.append(filename)

    # Extract macros
    macros = []
    for macro in root.iter("{http://www.ros.org/wiki/xacro}macro"):
        name = macro.get("name", "")
        params = macro.get("params", "")
        macros.append({"name": name, "params": params})

    return root, {
        "links": links,
        "joints": joints,
        "includes": includes,
        "macros": macros,
    }


def find_all_xacro_files(output_dir: Path) -> List[Path]:
    """Find all xacro files in the output directory."""
    urdf_dir = output_dir / "urdf"
    if not urdf_dir.exists():
        return []

    xacro_files = list(urdf_dir.glob("**/*.xacro"))
    return sorted(xacro_files)


def find_mesh_files(output_dir: Path) -> List[Path]:
    """Find all mesh files in the output directory."""
    mesh_dir = output_dir / "meshes"
    if not mesh_dir.exists():
        return []

    mesh_files = list(mesh_dir.glob("**/*.stl"))
    return sorted(mesh_files)


def build_robot_structure(output_dir: Path) -> Dict:
    """Build complete robot structure from exported files."""
    xacro_files = find_all_xacro_files(output_dir)
    mesh_files = find_mesh_files(output_dir)

    all_links = {}
    all_joints = []
    file_structure = {}

    # Parse entry point first
    entry_point = None
    for xacro_file in xacro_files:
        if xacro_file.name.endswith(".urdf.xacro"):
            entry_point = xacro_file
            break

    if entry_point:
        root, data = parse_xacro_file(entry_point)
        file_structure["entry_point"] = {
            "file": str(entry_point.relative_to(output_dir)),
            "includes": data["includes"],
            "macros": data["macros"],
        }

    # Parse all xacro files
    for xacro_file in xacro_files:
        root, data = parse_xacro_file(xacro_file)
        rel_path = xacro_file.relative_to(output_dir)

        file_structure[str(rel_path)] = {
            "links": len(data["links"]),
            "joints": len(data["joints"]),
            "includes": len(data["includes"]),
            "macros": len(data["macros"]),
        }

        # Merge links and joints
        all_links.update(data["links"])
        all_joints.extend(data["joints"])

    # Map meshes to links
    mesh_map = {}
    for mesh_file in mesh_files:
        mesh_name = mesh_file.stem
        # Try to match mesh to link (simple heuristic)
        for link_name in all_links.keys():
            if mesh_name.lower().replace("_", "").replace(
                "-", ""
            ) in link_name.lower().replace("_", "").replace("-", ""):
                mesh_map[link_name] = str(mesh_file.relative_to(output_dir))
                break

    return {
        "links": all_links,
        "joints": all_joints,
        "meshes": [str(m.relative_to(output_dir)) for m in mesh_files],
        "mesh_map": mesh_map,
        "files": file_structure,
        "entry_point": entry_point.relative_to(output_dir) if entry_point else None,
    }


def print_text_visualization(structure: Dict, output_dir: Path):
    """Print text-based visualization."""
    print("\n" + "=" * 70)
    print("ROBOT DESCRIPTION VISUALIZATION")
    print("=" * 70)

    print(f"\n📁 Export Directory: {output_dir}")

    # Entry point
    if structure["entry_point"]:
        print(f"\n📄 Entry Point: {structure['entry_point']}")

    # File structure
    print("\n📋 File Structure:")
    for file_path, info in structure["files"].items():
        print(f"  {file_path}")
        if "links" in info:
            print(f"    - Links: {info['links']}")
            print(f"    - Joints: {info['joints']}")
            print(f"    - Includes: {info['includes']}")
            print(f"    - Macros: {info['macros']}")

    # Links
    print(f"\n🔗 Links ({len(structure['links'])}):")
    for link_name, link_info in sorted(structure["links"].items()):
        mesh_status = "✓" if link_info["has_mesh"] else "✗"
        visual_status = "✓" if link_info["has_visual"] else "✗"
        collision_status = "✓" if link_info["has_collision"] else "✗"
        mesh_file = structure["mesh_map"].get(link_name, "N/A")
        print(f"  {link_name}")
        print(f"    Mesh: {mesh_status} ({mesh_file})")
        print(f"    Visual: {visual_status} | Collision: {collision_status}")

    # Joints
    print(f"\n⚙️  Joints ({len(structure['joints'])}):")
    for joint in structure["joints"]:
        print(f"  {joint['name']} ({joint['type']})")
        print(f"    Parent: {joint['parent']}")
        print(f"    Child:  {joint['child']}")

    # Kinematic tree
    print("\n🌳 Kinematic Tree:")
    # Build tree structure
    parent_map = {}
    root_links = set()

    for joint in structure["joints"]:
        parent = joint["parent"]
        child = joint["child"]
        if parent not in parent_map:
            parent_map[parent] = []
        parent_map[parent].append((child, joint["name"], joint["type"]))

    # Find root links (not children of any joint)
    all_children = {j["child"] for j in structure["joints"]}
    root_links = set(structure["links"].keys()) - all_children

    def print_tree(link_name: str, prefix: str = "", is_last: bool = True):
        marker = "└── " if is_last else "├── "
        print(f"{prefix}{marker}{link_name}")

        children = parent_map.get(link_name, [])
        for i, (child, joint_name, joint_type) in enumerate(children):
            is_last_child = i == len(children) - 1
            extension = "    " if is_last else "│   "
            joint_marker = "└─[" if is_last_child else "├─["
            print(f"{prefix}{extension}{joint_marker}{joint_name} ({joint_type})]─→")
            print_tree(
                child,
                prefix + extension + ("    " if is_last_child else "│   "),
                is_last_child,
            )

    if root_links:
        for root_link in sorted(root_links):
            print_tree(root_link)
    else:
        print("  (No clear root link found)")

    # Meshes
    print(f"\n📦 Mesh Files ({len(structure['meshes'])}):")
    for mesh in structure["meshes"]:
        print(f"  {mesh}")

    # Summary
    print("\n📊 Summary:")
    print(f"  Total Links: {len(structure['links'])}")
    print(f"  Total Joints: {len(structure['joints'])}")
    print(f"  Total Meshes: {len(structure['meshes'])}")
    print(
        f"  Links with Meshes: {sum(1 for lnk in structure['links'].values() if lnk['has_mesh'])}"
    )
    print(
        f"  Links without Meshes: {sum(1 for lnk in structure['links'].values() if not lnk['has_mesh'])}"
    )

    print("\n" + "=" * 70)


def create_graph_visualization(structure: Dict, output_path: Path):
    """Create graph visualization using matplotlib."""
    if not HAS_MATPLOTLIB:
        print("Skipping graph visualization (matplotlib not available)")
        return

    fig, ax = plt.subplots(1, 1, figsize=(14, 10))
    ax.set_title("Robot Kinematic Structure", fontsize=16, fontweight="bold")
    ax.axis("off")

    # Build graph
    links = list(structure["links"].keys())
    joints = structure["joints"]

    # Position nodes (simple hierarchical layout)
    positions = {}
    y_levels = {}
    level = 0

    # Find root links
    all_children = {j["child"] for j in joints}
    root_links = [lnk for lnk in links if lnk not in all_children]

    if not root_links:
        root_links = [links[0]] if links else []

    def assign_level(link_name: str, current_level: int):
        if link_name in y_levels:
            y_levels[link_name] = max(y_levels[link_name], current_level)
        else:
            y_levels[link_name] = current_level

        # Find children
        for joint in joints:
            if joint["parent"] == link_name:
                assign_level(joint["child"], current_level + 1)

    for root in root_links:
        assign_level(root, 0)

    # Assign positions
    level_counts = {}
    for link, level in y_levels.items():
        level_counts[level] = level_counts.get(level, 0) + 1

    for link, level in y_levels.items():
        count_at_level = level_counts[level]
        index = sorted([lnk for lnk, lvl in y_levels.items() if lvl == level]).index(
            link
        )
        x = (index + 1) / (count_at_level + 1)
        y = 1.0 - (level * 0.25)
        positions[link] = (x, y)

    # Draw links (nodes)
    for link_name, (x, y) in positions.items():
        link_info = structure["links"][link_name]
        has_mesh = link_info["has_mesh"]

        # Draw node
        color = "#4CAF50" if has_mesh else "#FF9800"
        circle = plt.Circle((x, y), 0.03, color=color, zorder=3)
        ax.add_patch(circle)

        # Add label
        ax.text(
            x,
            y - 0.05,
            link_name,
            ha="center",
            va="top",
            fontsize=9,
            bbox=dict(
                boxstyle="round,pad=0.3", facecolor="white", alpha=0.8, edgecolor=color
            ),
        )

    # Draw joints (edges)
    for joint in joints:
        parent = joint["parent"]
        child = joint["child"]

        if parent in positions and child in positions:
            x1, y1 = positions[parent]
            x2, y2 = positions[child]

            # Draw arrow
            arrow = FancyArrowPatch(
                (x1, y1),
                (x2, y2),
                arrowstyle="->",
                mutation_scale=20,
                color="#2196F3",
                linewidth=2,
                zorder=1,
            )
            ax.add_patch(arrow)

            # Add joint label
            mid_x, mid_y = (x1 + x2) / 2, (y1 + y2) / 2
            ax.text(
                mid_x,
                mid_y + 0.02,
                joint["name"],
                ha="center",
                fontsize=8,
                bbox=dict(boxstyle="round,pad=0.2", facecolor="lightblue", alpha=0.7),
            )

    # Legend
    legend_elements = [
        mpatches.Patch(color="#4CAF50", label="Link (with mesh)"),
        mpatches.Patch(color="#FF9800", label="Link (no mesh)"),
        mpatches.Patch(color="#2196F3", label="Joint"),
    ]
    ax.legend(handles=legend_elements, loc="upper right")

    plt.tight_layout()
    plt.savefig(output_path, dpi=150, bbox_inches="tight")
    print(f"\n📊 Graph visualization saved to: {output_path}")


def main():
    """Main entry point."""
    parser = argparse.ArgumentParser(
        description="Visualize exported robot description to verify export correctness"
    )
    parser.add_argument(
        "output_dir", type=Path, help="Path to exported robot description directory"
    )
    parser.add_argument(
        "--graph",
        type=Path,
        help="Output path for graph visualization image (e.g., robot_graph.png)",
    )

    args = parser.parse_args()

    output_dir = args.output_dir.resolve()

    if not output_dir.exists():
        print(f"Error: Directory not found: {output_dir}")
        sys.exit(1)

    # Build structure
    print("Analyzing exported robot description...")
    structure = build_robot_structure(output_dir)

    # Print text visualization
    print_text_visualization(structure, output_dir)

    # Create graph visualization if requested
    if args.graph:
        create_graph_visualization(structure, args.graph)
    elif HAS_MATPLOTLIB:
        # Auto-generate graph if matplotlib is available
        graph_path = output_dir / "robot_structure.png"
        create_graph_visualization(structure, graph_path)


if __name__ == "__main__":
    main()
