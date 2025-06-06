#!/usr/bin/env python3

import argparse
from pathlib import Path


def main():
    """
    Removes a ROS 2 service definition file (.srv) from a specified package.
    """
    parser = argparse.ArgumentParser(
        description="Remove a ROS 2 .srv file from the specified package."
    )
    parser.add_argument("package_name",
                        help="Name of the ROS 2 package (e.g., my_interfaces).")
    parser.add_argument("service_name",
                        help="Name of the service file to remove (e.g., AddTwoInts).")
    parser.add_argument("--ws", default=".",
                        help="Path to the ROS 2 workspace root (default: current directory).")
    parser.add_argument("-y", "--yes", action="store_true",
                        help="Automatically answer yes to the deletion confirmation.")

    args = parser.parse_args()

    # --- Path Resolution ---
    workspace_path = Path(args.ws).resolve()
    package_path = workspace_path / "src" / args.package_name
    service_file_path = package_path / "srv" / f"{args.service_name}.srv"

    print(f"Workspace path: {workspace_path}")
    print(f"Package path:   {package_path}")

    if not service_file_path.exists():
        print(f"\nInfo: Service file '{service_file_path}' does not exist. Nothing to do.")
        return

    # --- File Deletion ---
    try:
        print(f"\nFound service file to delete: {service_file_path}")

        do_delete = False
        if args.yes:
            do_delete = True
        else:
            response = input("Are you sure you want to permanently delete this file? [y/N]: ").lower().strip()
            if response == 'y':
                do_delete = True

        if do_delete:
            service_file_path.unlink()
            print(f"Successfully deleted '{service_file_path}'.")
            print("\nNext steps:")
            print(
                f"1. Re-build your package to remove the old generated files:\n   cd {workspace_path} && colcon build --packages-select {args.package_name}")
            print("2. Source the workspace:\n   source install/setup.bash")
        else:
            print("Deletion cancelled by user.")

    except OSError as e:
        print(f"\nError deleting service file: {e}")
    except Exception as e:
        print(f"\nAn unexpected error occurred: {e}")


if __name__ == "__main__":
    main()
