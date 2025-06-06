#!/usr/bin/env python3

import argparse
import os
import re
from pathlib import Path


def confirm_action(prompt):
    """Gets a yes/no confirmation from the user."""
    while True:
        response = input(f"{prompt} [y/n]: ").lower().strip()
        if response == 'y':
            return True
        if response == 'n':
            return False
        print("Please answer 'y' or 'n'.")


def remove_node_file(package_path: Path, package_name: str, node_module_name: str):
    """Removes the Python node file after confirmation."""
    node_file_path = package_path / package_name / f"{node_module_name}.py"

    if not node_file_path.exists():
        print(f"Info: Node file '{node_file_path}' does not exist. No file to delete.")
        return True  # Considered success as the goal is for it to be gone

    print(f"The following Python node file will be deleted: {node_file_path}")
    if confirm_action("Are you sure you want to delete this file?"):
        try:
            node_file_path.unlink()
            print(f"Successfully deleted node file: '{node_file_path}'")
            return True
        except OSError as e:
            print(f"Error deleting node file '{node_file_path}': {e}")
            return False
    else:
        print("File deletion cancelled by user.")
        return False  # User cancelled


def remove_executable_from_cmake(package_path: Path, package_name: str, node_module_name: str, executable_name: str):
    """Attempts to remove the node's install block from CMakeLists.txt."""
    cmake_path = package_path / "CMakeLists.txt"

    if not cmake_path.exists():
        print(f"Error: 'CMakeLists.txt' not found at '{cmake_path}'. Cannot remove executable.")
        return False

    try:
        with open(cmake_path, "r") as f:
            content = f.read()

        # Pattern to match install(PROGRAMS blocks that contain our executable
        # This matches the entire install block that includes our specific file
        install_pattern = re.compile(
            r'# Install ' + re.escape(node_module_name) + r' executable\s*\n'  # Comment line
                                                          r'install\(PROGRAMS\s*\n'  # install(PROGRAMS line
                                                          r'\s*' + re.escape(package_name) + r'/' + re.escape(
                node_module_name) + r'\.py\s*\n'  # File line
                                    r'\s*DESTINATION\s+lib/\$\{PROJECT_NAME\}\s*\n'  # DESTINATION line
                                    r'\s*RENAME\s+' + re.escape(executable_name) + r'\s*\n'  # RENAME line
                                                                                   r'\)\s*\n?',  # Closing parenthesis
            re.MULTILINE | re.DOTALL
        )

        # Alternative pattern for install blocks without the comment
        install_pattern_no_comment = re.compile(
            r'install\(PROGRAMS\s*\n'  # install(PROGRAMS line
            r'\s*' + re.escape(package_name) + r'/' + re.escape(node_module_name) + r'\.py\s*\n'  # File line
                                                                                    r'\s*DESTINATION\s+lib/\$\{PROJECT_NAME\}\s*\n'  # DESTINATION line
                                                                                    r'\s*RENAME\s+' + re.escape(
                executable_name) + r'\s*\n'  # RENAME line
                                   r'\)\s*\n?',  # Closing parenthesis
            re.MULTILINE | re.DOTALL
        )

        # More flexible pattern to catch variations in formatting
        flexible_pattern = re.compile(
            r'install\s*\(\s*PROGRAMS[^)]*?'  # Start of install(PROGRAMS
            + '\${PROJECT_NAME}/' + re.escape(node_module_name) + r'\.py'  # Our file
                                                                             r'[^)]*?RENAME\s+' + re.escape(
                executable_name) + r'[^)]*?\)',  # RENAME and closing
            re.MULTILINE | re.DOTALL
        )

        # Try each pattern in order of specificity
        patterns_to_try = [install_pattern, install_pattern_no_comment, flexible_pattern]
        match_found = None
        pattern_used = None

        for pattern in patterns_to_try:
            match = pattern.search(content)
            if match:
                match_found = match
                pattern_used = pattern
                break

        if not match_found:
            print(
                f"Info: Install block for executable '{executable_name}' (module '{node_module_name}') not found in '{cmake_path}'.")
            print("No changes made to CMakeLists.txt for this executable.")
            return False  # Considered success if it's already gone or never existed

        print(f"\nThe following install block will be removed from '{cmake_path}':")
        print("---")
        print(match_found.group(0).strip())
        print("---")

        if confirm_action("Are you sure you want to remove this install block?"):
            # Remove the matched block
            modified_content = pattern_used.sub("", content)

            # Clean up potential resulting multiple blank lines
            modified_content = re.sub(r'\n\s*\n\s*\n', '\n\n', modified_content)

            # Create a backup before overwriting
            backup_path = cmake_path.with_suffix(".txt.bak")
            try:
                with open(backup_path, "w") as bf:
                    bf.write(content)
                print(f"Backup of original CMakeLists.txt saved to '{backup_path}'")
            except IOError as e:
                print(f"Warning: Could not create backup of CMakeLists.txt: {e}")

            with open(cmake_path, "w") as f:
                f.write(modified_content)
            print(f"Successfully removed install block for '{executable_name}' from 'CMakeLists.txt'.")
            return True
        else:
            print("Install block removal cancelled by user.")
            return False  # User cancelled

    except IOError as e:
        print(f"Error processing 'CMakeLists.txt': {e}")
        return False
    except Exception as e:
        print(f"An unexpected error occurred while processing 'CMakeLists.txt': {e}")
        return False


def remove_node_file_auto(package_path: Path, package_name: str, node_module_name: str):
    """Removes the Python node file without confirmation (for -y flag)."""
    node_file_path = package_path / package_name / f"{node_module_name}.py"

    if not node_file_path.exists():
        print(f"Info: Node file '{node_file_path}' does not exist. No file to delete.")
        return True

    try:
        node_file_path.unlink()
        print(f"Successfully deleted node file (auto-confirm): '{node_file_path}'")
        return True
    except OSError as e:
        print(f"Error deleting node file '{node_file_path}': {e}")
        return False


def remove_executable_from_cmake_auto(package_path: Path, package_name: str, node_module_name: str,
                                      executable_name: str):
    """Removes the executable from CMakeLists.txt without confirmation (for -y flag)."""
    cmake_path = package_path / "CMakeLists.txt"

    if not cmake_path.exists():
        print(f"Error: 'CMakeLists.txt' not found at '{cmake_path}'. Cannot remove executable.")
        return False

    try:
        with open(cmake_path, "r") as f:
            content = f.read()

        # Use the same patterns as the interactive version
        install_pattern = re.compile(
            r'# Install ' + re.escape(node_module_name) + r' executable\s*\n'
                                                          r'install\(PROGRAMS\s*\n'
                                                          r'\s*' + re.escape(package_name) + r'/' + re.escape(
                node_module_name) + r'\.py\s*\n'
                                    r'\s*DESTINATION\s+lib/\$\{PROJECT_NAME\}\s*\n'
                                    r'\s*RENAME\s+' + re.escape(executable_name) + r'\s*\n'
                                                                                   r'\)\s*\n?',
            re.MULTILINE | re.DOTALL
        )

        install_pattern_no_comment = re.compile(
            r'install\(PROGRAMS\s*\n'
            r'\s*' + re.escape(package_name) + r'/' + re.escape(node_module_name) + r'\.py\s*\n'
                                                                                    r'\s*DESTINATION\s+lib/\$\{PROJECT_NAME\}\s*\n'
                                                                                    r'\s*RENAME\s+' + re.escape(
                executable_name) + r'\s*\n'
                                   r'\)\s*\n?',
            re.MULTILINE | re.DOTALL
        )

        flexible_pattern = re.compile(
            r'install\s*\(\s*PROGRAMS[^)]*?'
            + re.escape(package_name) + r'/' + re.escape(node_module_name) + r'\.py'
                                                                             r'[^)]*?RENAME\s+' + re.escape(
                executable_name) + r'[^)]*?\)',
            re.MULTILINE | re.DOTALL
        )

        patterns_to_try = [install_pattern, install_pattern_no_comment, flexible_pattern]
        match_found = None
        pattern_used = None

        for pattern in patterns_to_try:
            match = pattern.search(content)
            if match:
                match_found = match
                pattern_used = pattern
                break

        if not match_found:
            print(f"Info: Install block for executable '{executable_name}' not found in CMakeLists.txt.")
            return False

        print(f"Removing install block for '{executable_name}' (auto-confirm)...")

        # Remove the matched block
        modified_content = pattern_used.sub("", content)
        modified_content = re.sub(r'\n\s*\n\s*\n', '\n\n', modified_content)

        # Create backup
        backup_path = cmake_path.with_suffix(".txt.bak")
        try:
            with open(backup_path, "w") as bf:
                bf.write(content)
            print(f"Backup saved to '{backup_path}'")
        except IOError as e:
            print(f"Warning: Could not create backup: {e}")

        with open(cmake_path, "w") as f:
            f.write(modified_content)
        print(f"Successfully removed install block for '{executable_name}' from CMakeLists.txt.")
        return True

    except Exception as e:
        print(f"Error processing CMakeLists.txt: {e}")
        return False


def remove_executable_from_bash(workspace_path: Path, package_name: str, executable_name: str, bash_script_name: str):
    """Removes the node's entry from the DEFAULT_NODES array in a bash script, with confirmation."""
    bash_script_path = workspace_path / bash_script_name
    if not bash_script_path.is_file():
        print(f"\nInfo: Run script '{bash_script_path}' not found. Skipping.")
        return False  # Success, as the goal is for the entry to be gone

    try:
        with open(bash_script_path, "r") as f:
            lines = f.readlines()

        # The string we are looking to remove from the run script
        target_line_content = f'"{package_name} {executable_name}"'

        line_to_remove_index = -1
        for i, line in enumerate(lines):
            if target_line_content in line:
                line_to_remove_index = i
                break

        if line_to_remove_index == -1:
            print(
                f"Info: Entry for '{package_name} {executable_name}' not found in '{bash_script_name}'. No changes needed.")
            return False  # Success, it's already gone

        print(f"\nThe following line will be removed from '{bash_script_name}':")
        print("---")
        print(lines[line_to_remove_index].strip())
        print("---")

        if confirm_action("Are you sure you want to remove this default node entry?"):
            original_content = "".join(lines)
            del lines[line_to_remove_index]
            modified_content = "".join(lines)

            # Optional: Create a backup of the run script
            backup_path = bash_script_path.with_suffix(".bak")
            try:
                with open(backup_path, "w") as bf:
                    bf.write(original_content)
                print(f"Backup of original run script saved to '{backup_path}'")
            except IOError as e:
                print(f"Warning: Could not create backup of run script: {e}")

            with open(bash_script_path, "w") as f:
                f.write(modified_content)
            print(f"Successfully removed entry from '{bash_script_name}'.")
            return True
        else:
            print("Run script modification cancelled by user.")
            return False

    except IOError as e:
        print(f"Error processing '{bash_script_name}': {e}")
        return False


def remove_executable_from_bash_auto(workspace_path: Path, package_name: str, executable_name: str,
                                     bash_script_name: str):
    """Removes the node's entry from the DEFAULT_NODES array in a bash script, without confirmation."""
    bash_script_path = workspace_path / bash_script_name
    if not bash_script_path.is_file():
        return False  # Not an error, just means we don't need to do anything.

    try:
        with open(bash_script_path, "r") as f:
            lines = f.readlines()

        target_line_content = f'"{package_name} {executable_name}"'

        # Create a new list of lines, excluding the one we want to remove
        new_lines = [line for line in lines if target_line_content not in line]

        if len(lines) == len(new_lines):
            print(
                f"Info: Entry for '{package_name} {executable_name}' not found in '{bash_script_name}'. No changes needed.")
            return False

        # Write the modified content back
        with open(bash_script_path, "w") as f:
            f.writelines(new_lines)

        print(f"Successfully removed entry from '{bash_script_name}' (auto-confirm).")
        return True

    except IOError as e:
        print(f"Error processing '{bash_script_name}': {e}")
        return False


def main():
    parser = argparse.ArgumentParser(
        description="Remove a ROS 2 Python node file and its entries from build and run scripts.")
    parser.add_argument("package_name", help="Name of the ROS 2 package (e.g., my_py_pkg)")
    parser.add_argument("node_module_name",
                        help="Name of the Python node file (without .py extension, e.g., my_node_to_delete)")
    parser.add_argument("executable_name",
                        help="Executable name used in CMakeLists.txt and for ros2 run (e.g., node_runner)")
    parser.add_argument("--ws", default=".", help="Path to the ROS 2 workspace root (default: current directory)")
    parser.add_argument("--run-script", default="start_project.bash", help="Name of the bash run script to update (default: start_project.bash)")
    parser.add_argument("-y", "--yes", action="store_true",
                        help="Automatically answer yes to all confirmations (USE WITH CAUTION)")

    args = parser.parse_args()

    workspace_path = Path(args.ws).resolve()
    package_src_path = workspace_path / "src" / args.package_name

    print(
        f"--- Attempting to remove node '{args.node_module_name}' (executable '{args.executable_name}') from package '{args.package_name}' ---")
    print(f"Workspace path: {workspace_path}")
    print(f"Package source path: {package_src_path}")

    if not package_src_path.is_dir():
        print(f"Error: Package source directory '{package_src_path}' not found.")
        return

    python_module_dir = package_src_path / args.package_name
    if not python_module_dir.is_dir():
        print(f"Error: Python module directory '{python_module_dir}' for package '{args.package_name}' not found.")
        print("This script expects the Python modules to be in a subdirectory with the same name as the package.")
        return

    # Step 1: Remove the node file
    if args.yes:
        file_deleted_successfully = remove_node_file_auto(package_src_path, args.package_name, args.node_module_name)
    else:
        file_deleted_successfully = remove_node_file(package_src_path, args.package_name, args.node_module_name)

    if not file_deleted_successfully:
        print("Aborting further changes as file deletion was not successful or was cancelled.")
        return

    # Step 2: Remove the executable from CMakeLists.txt
    if args.yes:
        executable_removed_successfully = remove_executable_from_cmake_auto(
            package_src_path, args.package_name, args.node_module_name, args.executable_name)
    else:
        executable_removed_successfully = remove_executable_from_cmake(
            package_src_path, args.package_name, args.node_module_name, args.executable_name)

    # Step 3: Remove the executable from the bash run script
    if args.yes:
        bash_removed_successfully = remove_executable_from_bash_auto(
            workspace_path, args.package_name, args.executable_name, args.run_script)
    else:
        bash_removed_successfully = remove_executable_from_bash(
            workspace_path, args.package_name, args.executable_name, args.run_script)


    print("\n--- Node removal process complete ---")
    if file_deleted_successfully:
        print(f"- Node file '{args.package_name}/{args.node_module_name}.py' should be deleted.")
    if executable_removed_successfully:
        print(f"- Install block for '{args.executable_name}' should be removed from 'CMakeLists.txt'.")
    if bash_removed_successfully:
        print(f"- Entry for '{args.executable_name}' should be removed from '{args.run_script}'.")

    # Define colors for terminal output
    C_YELLOW = '\033[93m'
    C_END = '\033[0m'

    # Check if the operations were successful and print warnings if not
    if not executable_removed_successfully:
        print(f"{C_YELLOW}Warning: The executable was not removed from CMakeLists.txt. "
              f"This might be because you cancelled the operation or the entry was not found. "
              f"Please review the file manually.{C_END}")

    if not bash_removed_successfully:
        print(f"{C_YELLOW}Warning: The executable was not removed from '{args.run_script}'. "
              f"This might be because you cancelled the operation, the script was not found, or the entry was not found. "
              f"Please review the file manually.{C_END}")
    print("\n" + "=" * 30)
    print("Node removal process complete.")
    print("=" * 30)
if __name__ == "__main__":
    main()
