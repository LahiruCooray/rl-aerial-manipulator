#!/bin/bash
# Script to launch MoveIt for manipulator
# This script ensures clean restart by killing previous instances

echo "================================================"
echo "  Launching MoveIt 2 for Manipulator"
echo "================================================"

# Kill previous MoveIt/RViz instances
echo "[1/4] Cleaning up previous MoveIt/RViz instances..."
pkill -9 -f "move_group" 2>/dev/null
pkill -9 -f "rviz" 2>/dev/null
sleep 1

# Restart ROS 2 daemon to clear cached nodes
echo "[2/4] Restarting ROS 2 daemon..."
ros2 daemon stop 2>/dev/null
sleep 1
ros2 daemon start
sleep 1



# Source the workspace
echo "[3/4] Sourcing workspace..."
source /opt/ros/jazzy/setup.bash
source install/setup.bash

# Check if controllers are active
echo "[4/4] Checking if controllers are active..."
if ros2 control list_controllers 2>/dev/null | grep -q "active"; then
    echo "✓ Controllers are active"
else
    echo ""
    echo "⚠️  WARNING: Controllers don't appear to be active!"
    echo "   Expected active controllers:"
    echo "     - joint_state_broadcaster"
    echo "     - arm_controller"
    echo "     - gripper_controller"
    echo ""
    echo "   Please ensure:"
    echo "     1. Gazebo is running (./launch_gazebo.sh)"
    echo "     2. Controllers are launched (./launch_controllers.sh)"
    echo ""
    read -p "Continue anyway? (y/n) " -n 1 -r
    echo
    if [[ ! $REPLY =~ ^[Yy]$ ]]; then
        exit 1
    fi
fi

echo ""
echo "Launching MoveIt 2 with RViz..."
echo ""
echo "Expected output:"
echo "  ✓ RViz window opens with manipulator model"
echo "  ✓ Planning Scene Monitor connected"
echo "  ✓ Motion planning interface ready"
echo ""
echo "In RViz, you can:"
echo "  - Use MotionPlanning plugin to plan trajectories"
echo "  - Drag interactive markers to set goal poses"
echo "  - Click 'Plan & Execute' to move the arm"
echo ""
echo "================================================"

ros2 launch manipulator_moveit moveit.launch.py
