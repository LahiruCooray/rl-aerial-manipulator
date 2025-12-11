#!/bin/bash
# Script to launch ros2_control controllers for manipulator
# This script ensures clean restart by killing previous instances

echo "================================================"
echo "  Launching Manipulator Controllers"
echo "================================================"

# Kill previous controller instances
echo "[1/4] Cleaning up previous controller instances..."
pkill -9 -f "controller_manager" 2>/dev/null
pkill -9 -f "spawner" 2>/dev/null
sleep 1

# Restart ROS 2 daemon to clear cached nodes
echo "[2/4] Restarting ROS 2 daemon..."
ros2 daemon stop 2>/dev/null
sleep 1
ros2 daemon start
sleep 1

# Navigate to workspace
cd ~/newlocalrepo/rl-aerial-manipulator/Manipulator

# Source the workspace
echo "[3/4] Sourcing workspace..."
source /opt/ros/jazzy/setup.bash
source install/setup.bash

# Wait for Gazebo to be ready
echo "[4/4] Checking if Gazebo is running..."
if ! ros2 node list 2>/dev/null | grep -q "gz_ros_control"; then
    echo ""
    echo "⚠️  WARNING: Gazebo doesn't appear to be running!"
    echo "   Please launch Gazebo first using: ./launch_gazebo.sh"
    echo ""
    read -p "Continue anyway? (y/n) " -n 1 -r
    echo
    if [[ ! $REPLY =~ ^[Yy]$ ]]; then
        exit 1
    fi
fi

echo ""
echo "Launching controllers..."
echo ""
echo "Expected output:"
echo "  ✓ [INFO] [spawner_joint_state_broadcaster]: Configured and activated joint_state_broadcaster"
echo "  ✓ [INFO] [spawner_arm_controller]: Configured and activated arm_controller"
echo "  ✓ [INFO] [spawner_gripper_controller]: Configured and activated gripper_controller"
echo ""
echo "================================================"

ros2 launch manipulator_controller controller.launch.py
