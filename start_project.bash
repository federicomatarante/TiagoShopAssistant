#!/bin/bash

# === Configuration: Define Your Default Nodes ===
# Edit this list to specify which nodes run by default when the script
# is called without arguments.
# Each entry should be a string in the format "package_name executable_name".

DEFAULT_NODES=(
    "tiago map_provider"
    "tiago map_tester"
    "tiago database_interface"
    "tiago test_database_node"
    "tiago path_planner"
    "tiago vision_controller"
    "tiago controller"
    "tiago tiago_navigation.launch"
)

# === Tmux Configuration ===
SESSION_NAME="ros2_nodes"
USE_TMUX=true  # Set to false to use the old background process method

# =================================================

# --- Script Logic ---

# Function to display help
show_help() {
    echo "Usage: $0 [OPTIONS] [package1/executable1] [package2/executable2] ..."
    echo ""
    echo "Options:"
    echo "  -h, --help          Show this help message"
    echo "  --no-tmux          Run nodes as background processes instead of tmux"
    echo "  --tmux             Force tmux mode (default)"
    echo "  --session NAME     Use custom tmux session name (default: $SESSION_NAME)"
    echo ""
    echo "Examples:"
    echo "  $0                           # Run default nodes in tmux"
    echo "  $0 --no-tmux                # Run default nodes as background processes"
    echo "  $0 tiago/database            # Run specific node in tmux"
    echo "  $0 --session my_session tiago/database tiago/map_provider"
    echo ""
    echo "Default nodes configured:"
    for node in "${DEFAULT_NODES[@]}"; do
        echo "  - $node"
    done
}

# Parse command line arguments
NODES_FROM_ARGS=()
while [[ $# -gt 0 ]]; do
    case $1 in
        -h|--help)
            show_help
            exit 0
            ;;
        --no-tmux)
            USE_TMUX=false
            shift
            ;;
        --tmux)
            USE_TMUX=true
            shift
            ;;
        --session)
            SESSION_NAME="$2"
            shift 2
            ;;
        *)
            NODES_FROM_ARGS+=("$1")
            shift
            ;;
    esac
done

# Check if tmux is available when needed
if [ "$USE_TMUX" = true ] && ! command -v tmux &> /dev/null; then
    echo "Warning: tmux is not installed. Falling back to background process mode."
    echo "To install tmux: sudo apt-get install tmux"
    USE_TMUX=false
fi

# Assuming this script is in the workspace root, or it's run from there.
WORKSPACE_ROOT=$(pwd)

# Basic check if we're in a recognizable ROS 2 workspace root
if [ ! -d "$WORKSPACE_ROOT/src" ] || [ ! -d "$WORKSPACE_ROOT/install" ] && [ ! -d "$WORKSPACE_ROOT/build" ]; then
    echo "Warning: This script should ideally be run from the root of your ROS 2 workspace."
    echo "Attempting to continue, but paths might be incorrect if not in workspace root."
fi

echo "=== 1. Building all packages in workspace: $WORKSPACE_ROOT ==="
cd "$WORKSPACE_ROOT" || exit 1

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

# Determine which nodes to run
NODES_TO_LAUNCH_SPECS=()
if [ ${#NODES_FROM_ARGS[@]} -eq 0 ]; then
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
    for arg_node_spec in "${NODES_FROM_ARGS[@]}"; do
        package_name=$(echo "$arg_node_spec" | cut -d'/' -f1)
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

# === TMUX MODE ===
if [ "$USE_TMUX" = true ]; then
    echo ""
    echo "=== 3. Starting ROS 2 Nodes in Tmux Session: $SESSION_NAME ==="

    # Kill existing session if it exists
    if tmux has-session -t "$SESSION_NAME" 2>/dev/null; then
        echo "Killing existing tmux session '$SESSION_NAME'..."
        tmux kill-session -t "$SESSION_NAME"
    fi

    # <<<<<<<<<<<<<<<< START OF MODIFIED TMUX LOGIC <<<<<<<<<<<<<<<<

    # 1. Start the Map Server as the FIRST window in a new session.
    echo "--> Creating tmux session '$SESSION_NAME' and starting map_server..."
    tmux new-session -d -s "$SESSION_NAME" -n "map_server"
    tmux send-keys -t "$SESSION_NAME:map_server" "source install/setup.bash" Enter
    tmux send-keys -t "$SESSION_NAME:map_server" "ros2 run nav2_map_server map_server --ros-args -p yaml_filename:=$WORKSPACE_ROOT/install/tiago/share/tiago/maps/my_map.yaml" Enter

    # 2. Wait, then activate the map server from the main script.
    echo "--> Waiting for Map Server to initialize..."
    sleep 3 # Use 3 seconds for more stability
    echo "--> Activating the Map Server..."
    ros2 lifecycle set /map_server configure
    ros2 lifecycle set /map_server activate
    echo "--> Map Server is active."

    # 3. Start RViz in its own dedicated window.
    # echo "--> Starting RViz..."
    # tmux new-window -t "$SESSION_NAME" -n "rviz"
    # tmux send-keys -t "$SESSION_NAME:rviz" "source install/setup.bash" Enter
    # tmux send-keys -t "$SESSION_NAME:rviz" "ros2 run rviz2 rviz2 -d \$(ros2 pkg prefix tiago)/share/tiago/rviz/path_planning.rviz" Enter

    # 4. Now, loop through the user's default nodes and add them as NEW windows.
    for node_info_str in "${NODES_TO_LAUNCH_SPECS[@]}"; do
        read -r current_package_name current_executable_name <<< "$node_info_str"
        if [ -z "$current_package_name" ] || [ -z "$current_executable_name" ]; then
            echo "Warning: Skipping invalid or empty node entry '$node_info_str'."
            continue
        fi

        window_name="${current_package_name}_${current_executable_name}"
        echo "Creating window '$window_name'..."
        tmux new-window -t "$SESSION_NAME" -n "$window_name"

        tmux send-keys -t "$SESSION_NAME:$window_name" "cd '$WORKSPACE_ROOT'" Enter
        tmux send-keys -t "$SESSION_NAME:$window_name" "source install/setup.bash" Enter
        tmux send-keys -t "$SESSION_NAME:$window_name" "echo 'Starting $current_executable_name from package $current_package_name...'" Enter
        tmux send-keys -t "$SESSION_NAME:$window_name" "ros2 run $current_package_name $current_executable_name" Enter
        sleep 0.5
    done
    
    # <<<<<<<<<<<<<<<< END OF MODIFIED TMUX LOGIC <<<<<<<<<<<<<<<<

    # Create a control window for monitoring
    echo "Creating control window..."
    tmux new-window -t "$SESSION_NAME" -n "control"
    tmux send-keys -t "$SESSION_NAME:control" "cd '$WORKSPACE_ROOT'" Enter
    tmux send-keys -t "$SESSION_NAME:control" "source install/setup.bash" Enter
    tmux send-keys -t "$SESSION_NAME:control" "echo 'ROS2 Nodes Control Panel'" Enter
    tmux send-keys -t "$SESSION_NAME:control" "echo 'Available commands:'" Enter
    tmux send-keys -t "$SESSION_NAME:control" "echo '  ros2 node list    - List all running nodes'" Enter
    tmux send-keys -t "$SESSION_NAME:control" "echo '  ros2 topic list   - List all topics'" Enter
    tmux send-keys -t "$SESSION_NAME:control" "echo '  htop              - Monitor system resources'" Enter
    tmux send-keys -t "$SESSION_NAME:control" "echo ''" Enter
    tmux send-keys -t "$SESSION_NAME:control" "echo 'Use Ctrl+B then:'" Enter
    tmux send-keys -t "$SESSION_NAME:control" "echo '  n - Next window'" Enter
    tmux send-keys -t "$SESSION_NAME:control" "echo '  p - Previous window'" Enter
    tmux send-keys -t "$SESSION_NAME:control" "echo '  [0-9] - Go to window number'" Enter
    tmux send-keys -t "$SESSION_NAME:control" "echo '  d - Detach session'" Enter
    tmux send-keys -t "$SESSION_NAME:control" "echo ''" Enter
    tmux send-keys -t "$SESSION_NAME:control" "echo 'To kill all nodes and exit: tmux kill-session -t $SESSION_NAME'" Enter

    # Go to the first node window
    tmux select-window -t "$SESSION_NAME:0"

    echo ""
    echo "=== Tmux session '$SESSION_NAME' created successfully! ==="
    echo ""
    echo "Tmux Quick Reference:"
    echo "  - Switch windows: Ctrl+B then n/p (next/previous) or number key"
    echo "  - Detach session: Ctrl+B then d"
    echo "  - Reattach later: tmux attach -t $SESSION_NAME"
    echo "  - Kill session: tmux kill-session -t $SESSION_NAME"
    echo ""
    echo "Attaching to session now..."
    sleep 2

    # Attach to the session
    tmux attach-session -t "$SESSION_NAME"

    echo "Tmux session detached or ended."

# === BACKGROUND PROCESS MODE ===
else
    echo ""
    echo "=== 3. Starting ROS 2 Nodes as Background Processes ==="

    # Array to hold PIDs of background processes
    declare -a PIDS=()
    declare -a NODE_DESCRIPTIONS=()

    # Function to kill all background processes
    cleanup() {
        echo -e "\n\n=== Shutting down nodes... ==="
        if [ ${#PIDS[@]} -eq 0 ]; then
            echo "No nodes were recorded to shut down."
        else
            for i in "${!PIDS[@]}"; do
                PID=${PIDS[$i]}
                DESC=${NODE_DESCRIPTIONS[$i]}
                if ps -p "$PID" > /dev/null; then
                    echo "Stopping $DESC (PID $PID)..."
                    kill "$PID"
                else
                    echo "$DESC (PID $PID) was already stopped."
                fi
            done

            for PID in "${PIDS[@]}"; do
                if ps -p "$PID" > /dev/null; then
                    wait "$PID" 2>/dev/null
                fi
            done
        fi
        echo "=== Shutdown complete. ==="
        exit 0
    }

    # Trap SIGINT (Ctrl+C) and SIGTERM (system shutdown) and call cleanup
    trap cleanup SIGINT SIGTERM

    # <<<<<<<<<<<<<<<< START OF MODIFIED BACKGROUND LOGIC <<<<<<<<<<<<<<<<

    # 1. Start and activate the Map Server.
    echo "--> Starting and activating the Map Server..."
    ros2 run nav2_map_server map_server --ros-args -p yaml_filename:=$WORKSPACE_ROOT/install/tiago/share/tiago/maps/my_map.yaml &
    PIDS+=($!)
    NODE_DESCRIPTIONS+=("Map Server")
    echo "--> Waiting for Map Server to initialize..."
    sleep 3 # Use 3 seconds for more stability
    echo "--> Activating the Map Server..."
    ros2 lifecycle set /map_server configure
    ros2 lifecycle set /map_server activate
    echo "--> Map Server is active."

    # 2. Start RViz.
    # echo "--> Starting RViz..."
    # ros2 run rviz2 rviz2 -d $(ros2 pkg prefix tiago)/share/tiago/rviz/path_planning.rviz &
    # PIDS+=($!)
    # NODE_DESCRIPTIONS+=("RViz")

    # <<<<<<<<<<<<<<<< END OF MODIFIED BACKGROUND LOGIC <<<<<<<<<<<<<<<<

    for node_info_str in "${NODES_TO_LAUNCH_SPECS[@]}"; do
        read -r current_package_name current_executable_name <<< "$node_info_str"

        if [ -z "$current_package_name" ] || [ -z "$current_executable_name" ]; then
            echo "Warning: Skipping invalid or empty node entry '$node_info_str'."
            continue
        fi

        node_description="'$current_executable_name' from package '$current_package_name'"
        echo "Starting $node_description..."

        ros2 run "$current_package_name" "$current_executable_name" &

        PIDS+=($!)
        NODE_DESCRIPTIONS+=("$node_description")
        sleep 0.5
    done

    echo ""
    if [ ${#PIDS[@]} -gt 0 ]; then
        echo "=== All specified nodes launched. Monitoring PIDs: ${PIDS[*]}. ==="
        echo "Press Ctrl+C to stop all nodes and exit."
        wait
    else
        echo "--- No nodes were actually launched. ---"
    fi

    cleanup
fi