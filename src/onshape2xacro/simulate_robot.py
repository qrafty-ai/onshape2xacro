#!/usr/bin/env python3
"""
Simulate robot using PyBullet to test joint movements.
Automatically tests all movable joints by rotating them through their limits.
"""
import sys
import argparse
import subprocess
from pathlib import Path
from typing import Optional, Dict
import xml.etree.ElementTree as ET

try:
    import pybullet as p
    import pybullet_data
    HAS_PYBULLET = True
except ImportError:
    HAS_PYBULLET = False
    print("Error: PyBullet not installed. Install with: uv pip install pybullet numpy")
    sys.exit(1)

try:
    import numpy as np
except ImportError:
    print("Error: NumPy not installed. Install with: uv pip install numpy")
    sys.exit(1)


def convert_xacro_to_urdf(xacro_path: Path, output_urdf: Path) -> bool:
    """Convert xacro to URDF using ROS xacro if available, otherwise simple processing."""
    # Try using ROS xacro command first (most reliable)
    try:
        result = subprocess.run(
            ["xacro", str(xacro_path)],
            capture_output=True,
            text=True,
            check=True,
            cwd=str(xacro_path.parent)
        )
        # Check if xacro actually expanded (should have <link> elements)
        if "<link" in result.stdout:
            output_urdf.write_text(result.stdout)
            print("✓ Converted xacro to URDF using ROS xacro")
            return True
        else:
            # xacro ran but didn't expand - fall through to fallback
            print("ROS xacro didn't expand macros, using fallback processing...")
    except (subprocess.CalledProcessError, FileNotFoundError):
        print("ROS xacro not found or failed, using basic xacro processing...")
    
    # Fallback: Simple xacro processing (expand includes and macros)
    # This handles the basic structure but may not work for complex xacro files
    try:
        ns = "{http://www.ros.org/wiki/xacro}"
        macro_defs = {}  # Store macro definitions: name -> (macro_element, base_dir)
        
        def collect_macros(elem, base_dir):
            """Recursively collect all macro definitions."""
            # Find includes
            includes = elem.findall(f".//{ns}include")
            for inc in includes:
                filename = inc.get("filename")
                if filename:
                    include_path = (base_dir / filename).resolve()
                    if include_path.exists():
                        include_tree = ET.parse(include_path)
                        include_root = include_tree.getroot()
                        # Recursively collect macros from included files
                        collect_macros(include_root, include_path.parent)
            
            # Find macro definitions in current element
            macros = elem.findall(f".//{ns}macro")
            for macro in macros:
                macro_name = macro.get("name")
                if macro_name:
                    macro_defs[macro_name] = (macro, base_dir)
        
        def expand_macros(elem, base_dir):
            """Recursively expand macro calls and includes."""
            # First, process includes to load macro definitions
            includes = list(elem.findall(f".//{ns}include"))
            for inc in includes:
                filename = inc.get("filename")
                if filename:
                    include_path = (base_dir / filename).resolve()
                    if include_path.exists():
                        include_tree = ET.parse(include_path)
                        include_root = include_tree.getroot()
                        collect_macros(include_root, include_path.parent)
            
            # Expand macro calls
            changed = True
            while changed:
                changed = False
                # Find all macro usages (tags with xacro namespace that match macro names)
                for macro_name, (macro_def, _) in macro_defs.items():
                    usages = list(elem.findall(f".//{ns}{macro_name}"))
                    for usage in usages:
                        # Get parent
                        parent = None
                        for p in elem.iter():
                            if usage in list(p):
                                parent = p
                                break
                        
                        if parent is not None:
                            idx = list(parent).index(usage)
                            # Copy macro children (deep copy)
                            for child in list(macro_def):
                                new_child = ET.fromstring(ET.tostring(child, encoding="unicode"))
                                # Recursively expand macros in the new child
                                expand_macros(new_child, base_dir)
                                parent.insert(idx, new_child)
                                idx += 1
                            parent.remove(usage)
                            changed = True
                
                # Remove include tags after processing
                includes = list(elem.findall(f".//{ns}include"))
                for inc in includes:
                    parent = None
                    for p in elem.iter():
                        if inc in list(p):
                            parent = p
                            break
                    if parent is not None:
                        parent.remove(inc)
                        changed = True
        
        # Read entry point
        tree = ET.parse(xacro_path)
        root = tree.getroot()
        
        # Collect all macros first
        collect_macros(root, xacro_path.parent)
        
        # Expand macros
        expand_macros(root, xacro_path.parent)
        
        # Remove remaining macro definitions
        macros = list(root.findall(f".//{ns}macro"))
        for macro in macros:
            parent = None
            for p in root.iter():
                if macro in list(p):
                    parent = p
                    break
            if parent is not None:
                parent.remove(macro)
        
        # Replace ${prefix} with empty string in all attributes
        for elem in root.iter():
            for attr in list(elem.attrib.keys()):
                elem.attrib[attr] = elem.attrib[attr].replace("${prefix}", "")
        
        # Clean up namespace declarations and prefixes
        if "xmlns:xacro" in root.attrib:
            del root.attrib["xmlns:xacro"]
        for elem in root.iter():
            # Remove xacro namespace prefix from tags
            if elem.tag.startswith(ns):
                elem.tag = elem.tag.replace(ns, "")
        
        # Write URDF
        urdf_content = ET.tostring(root, encoding="unicode", method="xml")
        output_urdf.write_text(urdf_content)
        print("✓ Converted xacro to URDF using basic processing")
        print("  Note: For full xacro support, install ROS: sudo apt install ros-<distro>-xacro")
        return True
    except Exception as e:
        print(f"✗ Error processing xacro: {e}")
        import traceback
        traceback.print_exc()
        return False


def load_robot_urdf(urdf_path: Path, output_dir: Path) -> Optional[int]:
    """Load robot URDF into PyBullet."""
    if not HAS_PYBULLET:
        return None
    
    # Convert mesh paths to absolute paths
    urdf_content = urdf_path.read_text()
    
    # Replace relative mesh paths with absolute paths
    mesh_dir = output_dir / "meshes"
    if mesh_dir.exists():
        import re
        
        # Find all mesh filename attributes
        def replace_mesh_path(match):
            mesh_rel = match.group(1)
            mesh_abs = None
            
            # Handle relative paths
            # The original xacro is in output_dir/urdf/, meshes are in output_dir/meshes/
            # So ../meshes/file.stl from urdf/ means output_dir/meshes/file.stl
            # And ../../meshes/file.stl from urdf/subdir/ means output_dir/meshes/file.stl
            if mesh_rel.startswith("../../meshes/"):
                # Relative from urdf/subdir/ directory: ../../meshes/file.stl -> output_dir/meshes/file.stl
                mesh_name = mesh_rel.replace("../../meshes/", "")
                mesh_abs = (output_dir / "meshes" / mesh_name).resolve()
            elif mesh_rel.startswith("../meshes/"):
                # Relative from urdf/ directory: ../meshes/file.stl -> output_dir/meshes/file.stl
                mesh_name = mesh_rel.replace("../meshes/", "")
                mesh_abs = (output_dir / "meshes" / mesh_name).resolve()
            elif mesh_rel.startswith("meshes/"):
                # Already relative to output_dir
                mesh_name = mesh_rel.replace("meshes/", "")
                mesh_abs = (output_dir / "meshes" / mesh_name).resolve()
            elif mesh_rel.startswith("../"):
                # Other ../ paths - resolve relative to output_dir
                mesh_abs = (output_dir / mesh_rel[3:]).resolve()
            else:
                # Try relative to output_dir first (most common)
                mesh_abs = (output_dir / mesh_rel).resolve()
                if not mesh_abs.exists():
                    # Fallback to relative to urdf directory
                    mesh_abs = (urdf_path.parent / mesh_rel).resolve()
            
            # Verify file exists
            if not mesh_abs.exists():
                print(f"Warning: Mesh file not found: {mesh_abs}")
                print(f"  Original path: {mesh_rel}")
                print(f"  Resolved to: {mesh_abs}")
            
            return f'filename="{mesh_abs}"'
        
        urdf_content = re.sub(r'filename="([^"]*\.stl)"', replace_mesh_path, urdf_content)
    
    # Write modified URDF to temp file
    temp_urdf = output_dir / "temp_robot.urdf"
    temp_urdf.write_text(urdf_content)
    
    # Initialize PyBullet
    print("Initializing PyBullet...")
    p.connect(p.GUI)
    p.setAdditionalSearchPath(pybullet_data.getDataPath())
    p.setAdditionalSearchPath(str(output_dir))
    
    # Load robot
    print("Loading robot...")
    try:
        robot_id = p.loadURDF(str(temp_urdf), useFixedBase=True, flags=p.URDF_USE_MATERIAL_COLORS_FROM_MTL)
        print(f"✓ Robot loaded successfully (ID: {robot_id})")
        return robot_id
    except Exception as e:
        print(f"✗ Failed to load robot: {e}")
        return None


def automated_joint_test(robot_id: int, joint_info: Dict):
    """Automatically test all movable joints."""
    print("\n" + "=" * 70)
    print("Automated Joint Testing")
    print("=" * 70)
    
    num_joints = p.getNumJoints(robot_id)
    print(f"\nFound {num_joints} joints")
    
    tested_count = 0
    success_count = 0
    
    for i in range(num_joints):
        joint_info_pb = p.getJointInfo(robot_id, i)
        joint_name = joint_info_pb[1].decode('utf-8')
        joint_type = joint_info_pb[2]
        
        # Only test revolute and prismatic joints
        if joint_type not in [p.JOINT_REVOLUTE, p.JOINT_PRISMATIC]:
            continue
        
        tested_count += 1
        print(f"\nTesting joint {i}: {joint_name}")
        
        # Get joint limits
        lower_limit = joint_info_pb[8]
        upper_limit = joint_info_pb[9]
        
        if lower_limit >= upper_limit:
            print(f"  ⚠ Warning: Invalid limits [{lower_limit}, {upper_limit}]")
            continue
        
        # Test joint by moving it through its range
        steps = 10
        success = True
        
        try:
            for step in range(steps + 1):
                # Interpolate between lower and upper limit
                target = lower_limit + (upper_limit - lower_limit) * (step / steps)
                p.setJointMotorControl2(
                    robot_id, i, p.POSITION_CONTROL, targetPosition=target, force=1000
                )
                
                # Step simulation
                for _ in range(10):
                    p.stepSimulation()
                
                # Check if joint moved (basic check)
                current_pos = p.getJointState(robot_id, i)[0]
                if abs(current_pos - target) > 0.1:  # Allow some tolerance
                    if step > 0:  # Don't fail on first step
                        success = False
            
            if success:
                print(f"  ✓ Joint {joint_name} tested successfully")
                success_count += 1
            else:
                print(f"  ✗ Joint {joint_name} failed to reach target positions")
        except Exception as e:
            print(f"  ✗ Error testing joint {joint_name}: {e}")
            success = False
    
    print("\n" + "=" * 70)
    print(f"Test Summary: {success_count}/{tested_count} joints passed")
    print("=" * 70)
    
    # Keep simulation running
    print("\nSimulation running. Close the PyBullet window to exit.")
    while True:
        p.stepSimulation()
        import time
        time.sleep(1.0 / 240.0)  # 240 Hz


def main():
    """Main entry point."""
    parser = argparse.ArgumentParser(
        description="Simulate robot using PyBullet and test joint movements"
    )
    parser.add_argument(
        "output_dir",
        type=Path,
        help="Path to exported robot description directory"
    )
    
    args = parser.parse_args()
    
    output_dir = args.output_dir.resolve()
    if not output_dir.exists():
        print(f"Error: Directory not found: {output_dir}")
        sys.exit(1)
    
    # Find entry point xacro file
    urdf_dir = output_dir / "urdf"
    if not urdf_dir.exists():
        print(f"Error: urdf/ directory not found in {output_dir}")
        sys.exit(1)
    
    # Look for .urdf.xacro files
    xacro_files = list(urdf_dir.glob("*.urdf.xacro"))
    if not xacro_files:
        print(f"Error: No .urdf.xacro files found in {urdf_dir}")
        sys.exit(1)
    
    entry_point = xacro_files[0]
    if len(xacro_files) > 1:
        print(f"Warning: Multiple .urdf.xacro files found, using: {entry_point.name}")
    
    print(f"Found entry point: {entry_point}")
    
    # Convert xacro to URDF
    print("\nConverting xacro to URDF...")
    temp_urdf = output_dir / "temp_robot.urdf"
    if not convert_xacro_to_urdf(entry_point, temp_urdf):
        print("✗ Failed to convert xacro to URDF")
        sys.exit(1)
    
    # Load robot
    robot_id = load_robot_urdf(temp_urdf, output_dir)
    if robot_id is None:
        print("✗ Failed to load robot")
        sys.exit(1)
    
    # Get joint information
    num_joints = p.getNumJoints(robot_id)
    joint_info = {}
    for i in range(num_joints):
        info = p.getJointInfo(robot_id, i)
        joint_info[i] = {
            "name": info[1].decode('utf-8'),
            "type": info[2],
            "lower_limit": info[8],
            "upper_limit": info[9],
        }
    
    # Run automated joint tests
    automated_joint_test(robot_id, joint_info)
    
    # Cleanup
    p.disconnect()


if __name__ == "__main__":
    main()

