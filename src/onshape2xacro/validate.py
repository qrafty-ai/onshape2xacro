#!/usr/bin/env python3
"""
Simple xacro validation script that checks XML structure without requiring ROS.
This is a basic validator - for full xacro processing, you need ROS installed.
"""
import sys
import xml.etree.ElementTree as ET
from pathlib import Path


def validate_xacro_file(file_path: Path):
    """Validate that a xacro file is well-formed XML."""
    try:
        # Parse the XML file
        tree = ET.parse(file_path)
        root = tree.getroot()
        
        print(f"✓ File is well-formed XML")
        print(f"✓ Root element: {root.tag}")
        
        # Check for common xacro elements
        if root.tag == "robot":
            print(f"✓ Root element is 'robot' (correct for URDF/xacro)")
            if "name" in root.attrib:
                print(f"✓ Robot name: {root.attrib['name']}")
        else:
            print(f"⚠ Warning: Root element is '{root.tag}', expected 'robot'")
        
        # Count elements
        links = list(root.iter("link"))
        joints = list(root.iter("joint"))
        includes = list(root.iter("{http://www.ros.org/wiki/xacro}include"))
        macros = list(root.iter("{http://www.ros.org/wiki/xacro}macro"))
        
        print(f"\nFile structure:")
        print(f"  - Links: {len(links)}")
        print(f"  - Joints: {len(joints)}")
        print(f"  - Xacro includes: {len(includes)}")
        print(f"  - Xacro macros: {len(macros)}")
        
        # Check for xacro namespace
        if "xacro" in root.attrib.get("xmlns:xacro", ""):
            print(f"✓ Xacro namespace declared")
        else:
            # Check if any xacro elements exist
            if includes or macros:
                print(f"⚠ Warning: Xacro elements found but namespace might not be properly declared")
        
        # List included files
        if includes:
            print(f"\nIncluded files:")
            for inc in includes:
                filename = inc.get("filename", "unknown")
                print(f"  - {filename}")
        
        # List macros
        if macros:
            print(f"\nDefined macros:")
            for macro in macros:
                name = macro.get("name", "unknown")
                params = macro.get("params", "")
                print(f"  - {name}({params})")
        
        print(f"\n✓ Basic validation passed!")
        print(f"\nNote: This is a basic XML structure check.")
        print(f"For full xacro processing (macro expansion, property substitution),")
        print(f"you need ROS installed: sudo apt install ros-<distro>-xacro")
        
        return True
        
    except ET.ParseError as e:
        print(f"✗ XML Parse Error: {e}")
        print(f"  Line {e.lineno}: {e.msg}")
        return False
    except FileNotFoundError:
        print(f"✗ File not found: {file_path}")
        return False
    except Exception as e:
        print(f"✗ Error: {e}")
        return False


def main():
    """Main entry point for the validation script."""
    if len(sys.argv) < 2:
        print("Usage: validate-xacro <path_to_xacro_file>")
        print("   or: uv run validate-xacro <path_to_xacro_file>")
        sys.exit(1)
    
    file_path = Path(sys.argv[1])
    success = validate_xacro_file(file_path)
    sys.exit(0 if success else 1)


if __name__ == "__main__":
    main()

