#!/bin/bash
# Launch script to visualize hexacopter URDF in RViz2
# This script starts robot_state_publisher, joint_state_publisher_gui, and rviz2

set -e

echo "=========================================="
echo "Hexacopter URDF Visualization in RViz2"
echo "=========================================="

# Get the directory where this script is located
SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"
URDF_FILE="${SCRIPT_DIR}/custom_hexa/model.urdf"

# Check if URDF file exists
if [ ! -f "$URDF_FILE" ]; then
    echo "❌ ERROR: URDF file not found at: $URDF_FILE"
    exit 1
fi

echo "✅ Found URDF: $URDF_FILE"
echo ""

# Source ROS 2
if [ -f "/opt/ros/jazzy/setup.bash" ]; then
    echo "🔄 Sourcing ROS 2 Jazzy..."
    source /opt/ros/jazzy/setup.bash
else
    echo "❌ ERROR: ROS 2 Jazzy not found at /opt/ros/jazzy"
    exit 1
fi

# Check if workspace install exists and source it
WORKSPACE_SETUP="${SCRIPT_DIR}/../install/local_setup.bash"
if [ -f "$WORKSPACE_SETUP" ]; then
    echo "🔄 Sourcing workspace..."
    source "$WORKSPACE_SETUP"
fi

echo ""
echo "🚀 Starting components..."
echo ""

# Cleanup function to kill background processes on exit
cleanup() {
    echo ""
    echo "🛑 Shutting down..."
    kill $ROBOT_STATE_PID $JOINT_STATE_PID 2>/dev/null || true
    exit 0
}

trap cleanup SIGINT SIGTERM

# Start robot_state_publisher
echo "1️⃣  Starting robot_state_publisher..."
ros2 run robot_state_publisher robot_state_publisher \
    --ros-args -p robot_description:="$(cat $URDF_FILE)" \
    > /tmp/robot_state_publisher.log 2>&1 &
ROBOT_STATE_PID=$!

# Wait a moment for it to start
sleep 1

# Check if robot_state_publisher is still running
if ! ps -p $ROBOT_STATE_PID > /dev/null; then
    echo "❌ ERROR: robot_state_publisher failed to start"
    echo "Check log: /tmp/robot_state_publisher.log"
    cat /tmp/robot_state_publisher.log
    exit 1
fi
echo "   ✅ PID: $ROBOT_STATE_PID"

# Start joint_state_publisher_gui
echo "2️⃣  Starting joint_state_publisher_gui..."
ros2 run joint_state_publisher_gui joint_state_publisher_gui \
    > /tmp/joint_state_publisher.log 2>&1 &
JOINT_STATE_PID=$!

# Wait a moment for it to start
sleep 1

# Check if joint_state_publisher_gui is still running
if ! ps -p $JOINT_STATE_PID > /dev/null; then
    echo "❌ ERROR: joint_state_publisher_gui failed to start"
    echo "Check log: /tmp/joint_state_publisher.log"
    cat /tmp/joint_state_publisher.log
    cleanup
    exit 1
fi
echo "   ✅ PID: $JOINT_STATE_PID"

echo ""
echo "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"
echo "✨ Components running successfully!"
echo "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"
echo ""
echo "📋 RViz2 Configuration:"
echo "   1. Set Fixed Frame to: world"
echo "   2. Add RobotModel display if not present"
echo "   3. Set Description Topic to: /robot_description"
echo "   4. Zoom in (hexacopter is ~30cm wide)"
echo ""
echo "Press Ctrl+C to stop all processes"
echo "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"
echo ""

# Start RViz2 (foreground - will block until closed)
echo "3️⃣  Launching RViz2..."
sleep 1

# Try to use urdf_tutorial config if available, otherwise plain rviz2
if [ -f "/opt/ros/jazzy/share/urdf_tutorial/rviz/urdf.rviz" ]; then
    rviz2 -d /opt/ros/jazzy/share/urdf_tutorial/rviz/urdf.rviz
else
    rviz2
fi

# When RViz2 closes, cleanup will be called automatically
cleanup
