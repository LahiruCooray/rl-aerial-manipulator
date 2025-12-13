#!/bin/bash
# Master control script for manipulator simulation
# Usage: ./manipulator_control.sh [start|stop|restart|status]

WORKSPACE_DIR=~/newlocalrepo/rl-aerial-manipulator/Manipulator

show_usage() {
    echo "================================================"
    echo "  Manipulator Simulation Control Script"
    echo "================================================"
    echo ""
    echo "Usage: ./manipulator_control.sh [command]"
    echo ""
    echo "Commands:"
    echo "  start     - Launch all components (interactive mode)"
    echo "  stop      - Stop all running components"
    echo "  restart   - Stop and restart all components"
    echo "  status    - Show status of all components"
    echo "  help      - Show this help message"
    echo ""
    echo "Manual launch (in separate terminals):"
    echo "  Terminal 1: ./launch_gazebo.sh"
    echo "  Terminal 2: ./launch_controllers.sh (wait 5s after Gazebo)"
    echo "  Terminal 3: ./launch_moveit.sh (wait for controllers)"
    echo ""
    echo "================================================"
}

stop_all() {
    echo "================================================"
    echo "  Stopping All Manipulator Components"
    echo "================================================"
    
    echo "[1/4] Stopping MoveIt and RViz..."
    pkill -9 -f "move_group" 2>/dev/null
    pkill -9 -f "rviz" 2>/dev/null
    
    echo "[2/4] Stopping controllers..."
    pkill -9 -f "controller_manager" 2>/dev/null
    pkill -9 -f "spawner" 2>/dev/null
    
    echo "[3/4] Stopping Gazebo..."
    pkill -9 -f "gz sim" 2>/dev/null
    pkill -9 -f "gz_ros_control" 2>/dev/null
    pkill -9 -f "robot_state_publisher" 2>/dev/null
    
    echo "[4/4] Restarting ROS 2 daemon..."
    ros2 daemon stop 2>/dev/null
    sleep 1
    ros2 daemon start
    sleep 1
    
    echo ""
    echo "✓ All components stopped successfully"
    echo "================================================"
}

show_status() {
    echo "================================================"
    echo "  Manipulator Simulation Status"
    echo "================================================"
    echo ""
    
    # Check Gazebo
    echo "Gazebo (DART Physics):"
    if pgrep -f "gz sim" > /dev/null; then
        echo "  ✓ Running"
        if ros2 node list 2>/dev/null | grep -q "gz_ros_control"; then
            echo "  ✓ gz_ros_control node active"
        else
            echo "  ✗ gz_ros_control node NOT found"
        fi
    else
        echo "  ✗ Not running"
    fi
    echo ""
    
    # Check Controllers
    echo "Controllers:"
    if pgrep -f "controller_manager" > /dev/null; then
        echo "  ✓ controller_manager running"
        
        # Check individual controllers
        if ros2 control list_controllers 2>/dev/null | grep -q "joint_state_broadcaster"; then
            STATUS=$(ros2 control list_controllers 2>/dev/null | grep "joint_state_broadcaster" | awk '{print $2}')
            echo "  • joint_state_broadcaster: $STATUS"
        fi
        
        if ros2 control list_controllers 2>/dev/null | grep -q "arm_controller"; then
            STATUS=$(ros2 control list_controllers 2>/dev/null | grep "arm_controller" | awk '{print $2}')
            echo "  • arm_controller: $STATUS"
        fi
        
        if ros2 control list_controllers 2>/dev/null | grep -q "gripper_controller"; then
            STATUS=$(ros2 control list_controllers 2>/dev/null | grep "gripper_controller" | awk '{print $2}')
            echo "  • gripper_controller: $STATUS"
        fi
    else
        echo "  ✗ Not running"
    fi
    echo ""
    
    # Check MoveIt
    echo "MoveIt 2:"
    if pgrep -f "move_group" > /dev/null; then
        echo "  ✓ move_group running"
    else
        echo "  ✗ Not running"
    fi
    
    if pgrep -f "rviz" > /dev/null; then
        echo "  ✓ RViz running"
    else
        echo "  ✗ Not running"
    fi
    echo ""
    
    # Show active ROS nodes count
    NODE_COUNT=$(ros2 node list 2>/dev/null | wc -l)
    echo "Active ROS 2 nodes: $NODE_COUNT"
    echo ""
    echo "================================================"
}

start_interactive() {
    echo "================================================"
    echo "  Starting Manipulator Simulation"
    echo "================================================"
    echo ""
    echo "This will launch components in 3 separate terminal windows."
    echo "You need to have a terminal emulator installed."
    echo ""
    echo "Supported terminal emulators:"
    echo "  - gnome-terminal"
    echo "  - konsole"
    echo "  - xfce4-terminal"
    echo "  - xterm"
    echo ""
    read -p "Continue? (y/n) " -n 1 -r
    echo
    if [[ ! $REPLY =~ ^[Yy]$ ]]; then
        exit 1
    fi
    
    # Detect terminal emulator
    if command -v gnome-terminal &> /dev/null; then
        TERM_CMD="gnome-terminal --"
    elif command -v konsole &> /dev/null; then
        TERM_CMD="konsole -e"
    elif command -v xfce4-terminal &> /dev/null; then
        TERM_CMD="xfce4-terminal -e"
    elif command -v xterm &> /dev/null; then
        TERM_CMD="xterm -e"
    else
        echo "❌ No supported terminal emulator found!"
        echo ""
        echo "Please launch manually in separate terminals:"
        echo "  Terminal 1: ./launch_gazebo.sh"
        echo "  Terminal 2: ./launch_controllers.sh"
        echo "  Terminal 3: ./launch_moveit.sh"
        exit 1
    fi
    
    cd "$WORKSPACE_DIR"
    
    echo ""
    echo "Launching Gazebo..."
    $TERM_CMD bash -c "./launch_gazebo.sh; exec bash" &
    sleep 8
    
    echo "Launching Controllers..."
    $TERM_CMD bash -c "./launch_controllers.sh; exec bash" &
    sleep 5
    
    echo "Launching MoveIt..."
    $TERM_CMD bash -c "./launch_moveit.sh; exec bash" &
    
    echo ""
    echo "✓ All components launched in separate terminals"
    echo "================================================"
}

# Main script logic
case "$1" in
    start)
        stop_all
        sleep 2
        start_interactive
        ;;
    stop)
        stop_all
        ;;
    restart)
        stop_all
        sleep 2
        start_interactive
        ;;
    status)
        show_status
        ;;
    help|--help|-h)
        show_usage
        ;;
    *)
        show_usage
        exit 1
        ;;
esac
