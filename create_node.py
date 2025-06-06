#!/usr/bin/env python3

import argparse
import os
import re
from pathlib import Path


def to_camel_case(snake_str):
    """Converts snake_case_string to CamelCaseString."""
    return "".join(x.capitalize() for x in snake_str.lower().split("_"))


def create_node_file(package_path: Path, package_name: str, node_file_name: str, ros_node_name: str):
    """Creates the Python node file with boilerplate code."""
    node_class_name = to_camel_case(node_file_name)
    node_content = f"""#!/usr/bin/env python3
import rclpy
from rclpy.node import Node

class {node_class_name}(Node):
    def __init__(self):
        super().__init__('{ros_node_name}')
        self.get_logger().info(f"Node '{ros_node_name}' (class: {node_class_name}) has been started.")
        # Add your publishers, subscribers, timers, etc. here
        # Example: self.timer = self.create_timer(1.0, self.timer_callback)

    # def timer_callback(self):
    #     self.get_logger().info('Timer callback triggered.')

def main(args=None):
    rclpy.init(args=args)
    node = {node_class_name}()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
"""
    # Path for the node file: src/<package_name>/<package_name>/<node_file_name>.py
    node_module_dir = package_path / package_name
    if not node_module_dir.is_dir():
        print(f"Error: Python module directory '{node_module_dir}' not found.")
        print(
            f"Ensure your package '{package_name}' has an inner directory also named '{package_name}' for its Python modules.")
        return False

    node_file_path = node_module_dir / f"{node_file_name}.py"

    if node_file_path.exists():
        print(f"Error: Node file '{node_file_path}' already exists.")
        return False

    try:
        with open(node_file_path, "w") as f:
            f.write(node_content)
        # Make the file executable
        os.chmod(node_file_path, 0o755)
        print(f"Successfully created node file: '{node_file_path}'")
        return True
    except IOError as e:
        print(f"Error creating node file: {e}")
        return False


def add_executable_to_cmake(package_path: Path, package_name: str, node_file_name: str, executable_name: str):
    """Adds the new node to CMakeLists.txt as a separate install(PROGRAMS ...) block."""
    cmake_path = package_path / "CMakeLists.txt"
    if not cmake_path.exists():
        print(f"Error: 'CMakeLists.txt' not found at '{cmake_path}'")
        return False

    try:
        with open(cmake_path, "r") as f:
            content = f.read()

        # Create a new install block for this executable
        new_install_block = f"""
install(PROGRAMS
  ${{PROJECT_NAME}}/{node_file_name}.py
  DESTINATION lib/${{PROJECT_NAME}}
  RENAME {executable_name}
)
"""

        # Find a good place to insert the new install block
        # Look for existing install(PROGRAMS blocks or other install blocks
        install_pattern = r'install\(PROGRAMS[^)]*\)'
        matches = list(re.finditer(install_pattern, content, re.DOTALL))

        if matches:
            # Insert after the last install(PROGRAMS block
            last_match = matches[-1]
            insertion_point = last_match.end()
            modified_content = content[:insertion_point] + new_install_block + content[insertion_point:]
        else:
            # Look for any install blocks
            install_pattern_general = r'install\([^)]*\)'
            matches_general = list(re.finditer(install_pattern_general, content, re.DOTALL))

            if matches_general:
                # Insert after the last install block
                last_match = matches_general[-1]
                insertion_point = last_match.end()
                modified_content = content[:insertion_point] + new_install_block + content[insertion_point:]
            else:
                # Look for a good place after find_package statements
                lines = content.split('\n')
                insert_after_index = -1

                for i, line in enumerate(lines):
                    if (line.strip().startswith("find_package(") or
                            line.strip().startswith("# Find dependencies") or
                            line.strip().startswith("rosidl_generate_interfaces")):
                        insert_after_index = i

                if insert_after_index == -1:
                    # Fallback: insert after project declaration
                    for i, line in enumerate(lines):
                        if line.strip().startswith("project("):
                            insert_after_index = i
                            break

                if insert_after_index != -1:
                    # Insert the new install block
                    lines.insert(insert_after_index + 1, new_install_block.strip())
                    modified_content = '\n'.join(lines)
                else:
                    print("Error: Could not find a suitable place to insert the install(PROGRAMS) block.")
                    return False

        # Write the modified content back
        with open(cmake_path, "w") as f:
            f.write(modified_content)

        print(f"Successfully updated 'CMakeLists.txt' with executable '{executable_name}'.")
        return True

    except IOError as e:
        print(f"Error processing 'CMakeLists.txt': {e}")
        return False
    except Exception as e:
        print(f"An unexpected error occurred while processing 'CMakeLists.txt': {e}")
        print("Please review 'CMakeLists.txt' manually.")
        return False


def add_executable_to_bash(workspace_path: Path, package_name: str, executable_name: str, bash_script_name: str):
    """Adds the new executable to the DEFAULT_NODES list in the specified bash script."""
    bash_script_path = workspace_path / bash_script_name
    if not bash_script_path.is_file():
        # This is not a fatal error, as the user may not have this script.
        print(f"\nInfo: Bash script '{bash_script_path}' not found. Skipping update.")
        return False

    try:
        with open(bash_script_path, "r") as f:
            content = f.read()

        # The new entry to be added to the DEFAULT_NODES array
        new_entry_str = f'"{package_name} {executable_name}"'

        # Check if the entry already exists to avoid duplicates
        if new_entry_str in content:
            print(f"Node '{package_name} {executable_name}' is already in '{bash_script_name}'. No changes made.")
            return True

        # Regex to find the DEFAULT_NODES array block. It handles single or multi-line definitions.
        # It captures the part before the entries, the entries themselves, and the closing parenthesis.
        pattern = re.compile(r'(DEFAULT_NODES=\()([\s\S]*?)(\))', re.MULTILINE)
        match = pattern.search(content)

        if not match:
            print(f"Error: Could not find a 'DEFAULT_NODES=(...)' array in '{bash_script_name}'.")
            print("Please add the node manually.")
            return False

        # Reconstruct the block with the new entry
        prefix = match.group(1)  # e.g., "DEFAULT_NODES=("
        entries = match.group(2)  # The existing content inside the parentheses
        suffix = match.group(3)  # e.g., ")"

        # Add the new entry with proper indentation and a preceding newline
        # .rstrip() handles cases where there might be extra whitespace
        new_entries = entries.rstrip() + f'\n    {new_entry_str}\n'

        new_block = prefix + new_entries + suffix

        # Replace the old block with the new one
        modified_content = content.replace(match.group(0), new_block)

        with open(bash_script_path, "w") as f:
            f.write(modified_content)

        print(f"Successfully added '{package_name} {executable_name}' to DEFAULT_NODES in '{bash_script_name}'.")
        return True

    except IOError as e:
        print(f"Error processing '{bash_script_name}': {e}")
        return False
    except Exception as e:
        print(f"An unexpected error occurred while processing '{bash_script_name}': {e}")
        return False


def main():
    parser = argparse.ArgumentParser(
        description="Create a new ROS 2 Python node and add it to CMakeLists.txt and an optional run script.",
        formatter_class=argparse.RawTextHelpFormatter
    )
    parser.add_argument("package_name", help="Name of the existing ROS 2 package (e.g., my_py_pkg)")
    parser.add_argument("node_file_name",
                        help="Name for the new Python node file (e.g., my_new_node for my_new_node.py)")
    parser.add_argument("executable_name", help="Desired executable name for ros2 run (e.g., new_node_runner)")
    parser.add_argument("--ws", default=".", help="Path to the ROS 2 workspace root (default: current directory)")
    parser.add_argument("--run-script", default="start_project.bash",
                        help="Name of the bash run script in the workspace root to update (default: run.sh)")

    args = parser.parse_args()

    workspace_path = Path(args.ws).resolve()
    package_path = workspace_path / "src" / args.package_name

    print(f"Workspace path: {workspace_path}")
    print(f"Package path: {package_path}")

    if not package_path.is_dir():
        print(f"Error: Package directory '{package_path}' not found.")
        return

    # Use executable_name for the ROS node name, as it's often a good choice
    ros_node_name_for_init = args.executable_name
    if not create_node_file(package_path, args.package_name, args.node_file_name, ros_node_name_for_init):
        return

    # Add executable to CMakeLists.txt
    # Add executable to CMakeLists.txt
    if not add_executable_to_cmake(package_path, args.package_name, args.node_file_name, args.executable_name):
        print("\nFailed to update 'CMakeLists.txt' automatically. Please add the executable manually.")
        return
    # Add executable to the default bash launcher script
    if not add_executable_to_bash(workspace_path, args.package_name, args.executable_name, args.run_script):
        print(
            f"\nFailed to update '{args.run_script}' automatically. Please add the node to the DEFAULT_NODES list manually.")
        return

    print("\n" + "=" * 30)
    print("Node creation process complete.")
    print("=" * 30)


if __name__ == "__main__":
    main()
