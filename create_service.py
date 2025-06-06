#!/usr/bin/env python3

import argparse
from pathlib import Path

def main():
    """
    Creates a sample ROS 2 service definition file (.srv) in a specified package for the user to edit.
    """
    parser = argparse.ArgumentParser(
        description="Create a sample ROS 2 .srv file in the specified package.",
        formatter_class=argparse.RawTextHelpFormatter
    )
    parser.add_argument("package_name",
                        help="Name of the existing ROS 2 package (e.g., my_interfaces).")
    parser.add_argument("service_name",
                        help="Name for the new service file in CamelCase (e.g., AddTwoInts).")
    parser.add_argument("--ws", default=".",
                        help="Path to the ROS 2 workspace root (default: current directory).")

    args = parser.parse_args()

    # --- Path Resolution ---
    workspace_path = Path(args.ws).resolve()
    package_path = workspace_path / "src" / args.package_name
    srv_dir = package_path / "srv"
    service_file_path = srv_dir / f"{args.service_name}.srv"

    print(f"Workspace path: {workspace_path}")
    print(f"Package path:   {package_path}")

    if not package_path.is_dir():
        print(f"\nError: Package directory '{package_path}' not found.")
        return

    # --- File Creation ---
    try:
        # Create the srv directory if it doesn't exist
        srv_dir.mkdir(exist_ok=True)
        print(f"Ensured 'srv' directory exists at: {srv_dir}")

        if service_file_path.exists():
            print(f"\nError: Service file '{service_file_path}' already exists.")
            return

        # Construct the content of the example .srv file
        file_content = """# This is an auto-generated example service file.
#
# Replace the content with your actual service definition.
#
# Request fields go above the '---' line.
# Each line is a type and a name, e.g., "string data" or "int64 value".
# ---
# Supported types include:
# bool, byte, char, float32, float64, int8, uint8, int16, uint16, int32, uint32, int64, uint64, string
# You can also use other messages from this package or other packages, e.g., "std_msgs/Header header"

# REQUEST
string item_name
int32 quantity
---
# RESPONSE
# Response fields go below the '---' line.
bool success
string message
"""

        with open(service_file_path, "w") as f:
            f.write(file_content)

        print(f"\nSuccessfully created sample service file: '{service_file_path}'")
        print("--- File Content ---")
        print(file_content)
        print("--------------------")

        print("\nNext steps:")
        print(f"1. **IMPORTANT**: Open and edit the new file with your custom service definition:\n   '{service_file_path}'")
        print(f"2. Build your package to generate the service code:\n   cd {workspace_path} && colcon build --packages-select {args.package_name}")
        print("3. Source the workspace:\n   source install/setup.bash")
        print(f"4. You can now import this service in your nodes with:\n   from {args.package_name}.srv import {args.service_name}")

    except IOError as e:
        print(f"\nError creating service file: {e}")
    except Exception as e:
        print(f"\nAn unexpected error occurred: {e}")

if __name__ == "__main__":
    main()