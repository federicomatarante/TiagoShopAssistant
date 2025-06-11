#!/bin/bash

# === Configuration: Define Your Default Nodes ===
# Edit this list to specify which nodes run by default when the script
# is called without arguments.
# Each entry should be a string in the format "package_name executable_name".

DEFAULT_NODES=(
    "tiago database"
    "tiago database_tester"
    "tiago map_provider"
)
# =================================================

# --- Script Logic ---

# Assuming this script is in the workspace root, or it's run from there.
WORKSPACE_ROOT=$(pwd)

# Basic check if we're in a recognizable ROS 2 workspace root
if [ ! -d "$WORKSPACE_ROOT/src" ] || [ ! -d "$WORKSPACE_ROOT/install" ] && [ ! -d "$WORKSPACE_ROOT/build" ]; then
    echo "Warning: This script should ideally be run from the root of your ROS 2 workspace."
    echo "Attempting to continue, but paths might be incorrect if not in workspace root."
    # For a stricter check, you could exit here:
    # echo "Error: Please run this script from the root of your ROS 2 workspace."
    # exit 1
fi

echo "=== 1. Building all packages in workspace: $WORKSPACE_ROOT ==="
cd "$WORKSPACE_ROOT" || exit 1 # Navigate to workspace root or exit if it fails

if colcon build --symlink-install --event-handlers console_direct+; then
    echo "=== Build successful ==="
else
    echo "XXX Build failed. Aborting. XXX"
    exit 1
fi

echo ""
echo "=== 2. Sourcing the workspace ==="
if [ -f "$WORKSPACE_ROOT/install/setup.bash" ]; then
    source "$WORKSPACE_ROOT/install/setup.bash"
else
    echo "XXX Error: install/setup.bash not found. Has the workspace been built correctly? XXX"
    exit 1
fi

# Array to hold PIDs of background processes
declare -a PIDS=()
declare -a NODE_DESCRIPTIONS=() # For better shutdown messages

# Function to kill all background processes
cleanup() {
    echo -e "\n\n=== Shutting down nodes... ==="
    if [ ${#PIDS[@]} -eq 0 ]; then
        echo "No nodes were recorded to shut down."
    else
        for i in "${!PIDS[@]}"; do
            PID=${PIDS[$i]}
            DESC=${NODE_DESCRIPTIONS[$i]}
            if ps -p "$PID" > /dev/null; then # Check if process exists
                echo "Stopping $DESC (PID $PID)..."
                kill "$PID" # Send SIGTERM
            else
                echo "$DESC (PID $PID) was already stopped."
            fi
        done

        # Wait for processes to terminate
        # You can add a timeout here if needed
        for PID in "${PIDS[@]}"; do
            if ps -p "$PID" > /dev/null; then
                wait "$PID" 2>/dev/null # Wait for specific PID, ignore "No such process" if already killed
            fi
        done
    fi
    echo "=== Shutdown complete. ==="
    exit 0 # Exit cleanly
}

# Trap SIGINT (Ctrl+C) and SIGTERM (system shutdown) and call cleanup
trap cleanup SIGINT SIGTERM

# Determine which nodes to run
NODES_TO_LAUNCH_SPECS=() # Will store "package_name executable_name" strings
if [ "$#" -eq 0 ]; then
    echo ""
    echo "--- No specific nodes provided on command line, running default set configured in script. ---"
    if [ ${#DEFAULT_NODES[@]} -eq 0 ]; then
        echo "--- No default nodes configured in the script. Nothing to run. Exiting. ---"
        exit 0
    fi
    NODES_TO_LAUNCH_SPECS=("${DEFAULT_NODES[@]}")
else
    echo ""
    echo "--- Running nodes specified on command line. ---"
    # Each argument is expected as "package_name/executable_name"
    for arg_node_spec in "$@"; do
        package_name=$(echo "$arg_node_spec" | cut -d'/' -f1)
        # Use sed to robustly get the part after the first '/'
        executable_name=$(echo "$arg_node_spec" | sed 's|^[^/]*/||')

        if [ "$package_name" == "$arg_node_spec" ] || [ -z "$executable_name" ] || [ "$package_name" == "$executable_name" ]; then
            echo "XXX Error: Invalid node specification '$arg_node_spec'. Expected format: 'package_name/executable_name'. XXX"
            echo "Skipping this entry."
            continue
        fi
        NODES_TO_LAUNCH_SPECS+=("$package_name $executable_name")
    done
    if [ ${#NODES_TO_LAUNCH_SPECS[@]} -eq 0 ]; then
        echo "--- No valid nodes were specified on the command line after parsing. Nothing to run. Exiting. ---"
        exit 0
    fi
fi

echo ""
echo "=== 3. Starting ROS 2 Nodes ==="
for node_info_str in "${NODES_TO_LAUNCH_SPECS[@]}"; do
    # Split the string "package_name executable_name" into two variables
    read -r current_package_name current_executable_name <<< "$node_info_str"

    if [ -z "$current_package_name" ] || [ -z "$current_executable_name" ]; then
        echo "Warning: Skipping invalid or empty node entry '$node_info_str'."
        continue
    fi

    node_description="'$current_executable_name' from package '$current_package_name'"
    echo "Starting $node_description..."

    # Run the node in the background
    # You can redirect output if needed:
    # ros2 run "$current_package_name" "$current_executable_name" > "${current_executable_name}_${current_package_name}.log" 2>&1 &
    ros2 run "$current_package_name" "$current_executable_name" &

    PIDS+=($!) # Store PID of the last backgrounded process
    NODE_DESCRIPTIONS+=("$node_description")
    sleep 0.5 # Small delay to allow node to potentially print its startup messages
done

echo ""
if [ ${#PIDS[@]} -gt 0 ]; then
    echo "=== All specified nodes launched. Monitoring PIDs: ${PIDS[*]}. ==="
    echo "Press Ctrl+C to stop all nodes and exit."
    # Wait for all background processes to complete.
    # Script will remain here until all background PIDs exit or Ctrl+C is pressed.
    wait
else
    echo "--- No nodes were actually launched. ---"
fi

# If 'wait' returns because all nodes exited by themselves (not via Ctrl+C)
# call cleanup to ensure a consistent exit message.
cleanup