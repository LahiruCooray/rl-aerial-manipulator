#!/bin/bash

# Launch script for manipulator simulation and control
# Usage: ./launch_all.sh start|stop

WORKSPACE_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PID_FILE="$WORKSPACE_DIR/.launch_pids"

start_system() {
    echo "Starting manipulator launch sequence..."
    echo "Workspace: $WORKSPACE_DIR"
    
    # Clear old PID file
    > "$PID_FILE"
    
    # Terminal 1: Build the workspace
    xterm -hold -title "Build" -e "
    cd '$WORKSPACE_DIR'
    echo '=== Building workspace ==='
    colcon build
    echo ''
    echo 'Build complete. Press Enter to close...'
    read
    " &
    
    # Wait for build to complete before launching other terminals
    sleep 5
    
    # Terminal 2: Launch Gazebo
    xterm -hold -title "Gazebo" -e "
    cd '$WORKSPACE_DIR'
    . install/setup.bash
    echo '=== Launching Gazebo ==='
    ros2 launch manipulator_description gazebo.launch.py
    " &
    echo $! >> "$PID_FILE"
    
    # Wait for Gazebo to initialize
    sleep 3
    
    # Terminal 3: Launch Controller
    xterm -hold -title "Controller" -e "
    cd '$WORKSPACE_DIR'
    . install/setup.bash
    echo '=== Launching Controller ==='
    ros2 launch manipulator_controller controller.launch.py
    " &
    echo $! >> "$PID_FILE"
    
    # Wait for controller to initialize
    sleep 2
    
    # Terminal 4: Launch MoveIt
    xterm -hold -title "MoveIt" -e "
    cd '$WORKSPACE_DIR'
    . install/setup.bash
    echo '=== Launching MoveIt ==='
    ros2 launch manipulator_moveit moveit.launch.py
    " &
    echo $! >> "$PID_FILE"
    
    # Wait for MoveIt to initialize
    sleep 3
    
    # Terminal 5: Launch Remote Interface
    xterm -hold -title "Remote Interface" -e "
    cd '$WORKSPACE_DIR'
    . install/setup.bash
    echo '=== Launching Remote Interface ==='
    ros2 launch manipulator_remote remote_interface.launch.py
    " &
    echo $! >> "$PID_FILE"
    
    echo "All terminals launched!"
    echo "PID file saved to: $PID_FILE"
    echo "To stop all processes, run: ./launch_all.sh stop"
}

stop_system() {
    echo "Stopping manipulator system..."
    
    if [ ! -f "$PID_FILE" ]; then
        echo "No PID file found. Killing all ROS2 and Gazebo processes..."
        pkill -f "ros2 launch"
        pkill -f "gz sim"
        pkill -f "gzserver"
        pkill -f "gzclient"
        pkill -f "ruby.*gz"
        echo "Processes killed."
        return
    fi
    
    # Kill processes from PID file
    while read -r pid; do
        if [ -n "$pid" ]; then
            echo "Killing process $pid..."
            kill -TERM "$pid" 2>/dev/null || true
            # Also kill child processes
            pkill -P "$pid" 2>/dev/null || true
        fi
    done < "$PID_FILE"
    
    # Additional cleanup for ROS2 and Gazebo
    sleep 1
    pkill -f "ros2 launch"
    pkill -f "gz sim"
    pkill -f "gzserver"
    pkill -f "gzclient"
    pkill -f "ruby.*gz"
    
    # Remove PID file
    rm -f "$PID_FILE"
    
    echo "System stopped."
}

# Main script logic
case "$1" in
    start)
        start_system
        ;;
    stop)
        stop_system
        ;;
    *)
        echo "Usage: $0 {start|stop}"
        echo "  start - Launch all manipulator components"
        echo "  stop  - Stop all running processes"
        exit 1
        ;;
esac
