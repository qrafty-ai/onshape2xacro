#!/usr/bin/env python3
"""
Convert xacro file to URDF file.
This tool handles xacro expansion when ROS xacro doesn't work properly.
"""
import sys
import argparse
from pathlib import Path
import xml.etree.ElementTree as ET
import subprocess


def fix_mesh_paths(urdf_path: Path, xacro_dir: Path) -> None:
    """Fix relative mesh paths in URDF to absolute paths."""
    import re
    
    urdf_content = urdf_path.read_text()
    
    # Find meshes directory (usually in parent of urdf directory)
    # xacro is in output_dir/urdf/, meshes are in output_dir/meshes/
    meshes_dir = xacro_dir.parent / "meshes"
    if not meshes_dir.exists():
        # Try alternative: meshes might be in same directory as xacro
        meshes_dir = xacro_dir / "meshes"
    
    if not meshes_dir.exists():
        print(f"  Warning: Meshes directory not found at {meshes_dir}")
        return
    
    def replace_mesh_path(match):
        mesh_rel = match.group(1)
        mesh_abs = None
        
        # Handle relative paths
        if mesh_rel.startswith("../../meshes/"):
            mesh_name = mesh_rel.replace("../../meshes/", "")
            mesh_abs = (meshes_dir / mesh_name).resolve()
        elif mesh_rel.startswith("../meshes/"):
            mesh_name = mesh_rel.replace("../meshes/", "")
            mesh_abs = (meshes_dir / mesh_name).resolve()
        elif mesh_rel.startswith("meshes/"):
            mesh_name = mesh_rel.replace("meshes/", "")
            mesh_abs = (meshes_dir / mesh_name).resolve()
        else:
            # Try relative to meshes_dir
            mesh_abs = (meshes_dir / mesh_rel).resolve()
        
        # Verify file exists, try case-insensitive and handle .stl.stl issue
        if not mesh_abs.exists():
            # Try case-insensitive search
            mesh_name_lower = mesh_abs.name.lower()
            if meshes_dir.exists():
                # Find file with case-insensitive match
                for actual_file in meshes_dir.iterdir():
                    if actual_file.name.lower() == mesh_name_lower:
                        mesh_abs = actual_file.resolve()
                        break
                    # Also try with .stl.stl (double extension)
                    if actual_file.name.lower() == mesh_name_lower + ".stl":
                        mesh_abs = actual_file.resolve()
                        break
        
        # Final check
        if not mesh_abs.exists():
            print(f"  Warning: Mesh file not found: {mesh_abs}")
            return match.group(0)  # Return original if not found
        
        return f'filename="{mesh_abs}"'
    
    urdf_content = re.sub(r'filename="([^"]*\.stl)"', replace_mesh_path, urdf_content)
    urdf_path.write_text(urdf_content)


def convert_xacro_to_urdf(xacro_path: Path, output_urdf: Path, fix_paths: bool = True) -> bool:
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
            
            # Fix mesh paths if requested
            if fix_paths:
                fix_mesh_paths(output_urdf, xacro_path.parent)
            
            print(f"✓ Converted xacro to URDF using ROS xacro")
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
        
        # Fix mesh paths if requested
        if fix_paths:
            fix_mesh_paths(output_urdf, xacro_path.parent)
        
        print(f"✓ Converted xacro to URDF using basic processing")
        print("  Note: For full xacro support, install ROS: sudo apt install ros-<distro>-xacro")
        return True
    except Exception as e:
        print(f"✗ Error processing xacro: {e}")
        import traceback
        traceback.print_exc()
        return False


def main():
    """Main entry point."""
    parser = argparse.ArgumentParser(
        description="Convert xacro file to URDF file"
    )
    parser.add_argument(
        "xacro_file",
        type=Path,
        help="Path to xacro file"
    )
    parser.add_argument(
        "-o", "--output",
        type=Path,
        help="Output URDF file path (default: same name with .urdf extension)"
    )
    parser.add_argument(
        "--no-fix-paths",
        action="store_true",
        help="Don't convert relative mesh paths to absolute paths"
    )
    
    args = parser.parse_args()
    
    xacro_path = args.xacro_file.resolve()
    if not xacro_path.exists():
        print(f"Error: File not found: {xacro_path}")
        sys.exit(1)
    
    if args.output:
        output_path = args.output.resolve()
    else:
        output_path = xacro_path.with_suffix(".urdf")
    
    fix_paths = not args.no_fix_paths
    if convert_xacro_to_urdf(xacro_path, output_path, fix_paths=fix_paths):
        print(f"\n✓ URDF file saved to: {output_path}")
        if fix_paths:
            print("  ✓ Mesh paths converted to absolute paths")
        print(f"\nYou can now use urdf-viz to view it:")
        print(f"  urdf-viz {output_path}")
    else:
        print("\n✗ Failed to convert xacro to URDF")
        sys.exit(1)


if __name__ == "__main__":
    main()

