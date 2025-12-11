#!/bin/bash
# Script to launch Gazebo with DART physics for manipulator
# This script ensures clean restart by killing previous instances

echo "================================================"
echo "  Launching Manipulator in Gazebo (DART)"
echo "================================================"

# Kill previous Gazebo instances
echo "[1/3] Cleaning up previous Gazebo instances..."
pkill -9 -f "gz sim" 2>/dev/null
pkill -9 -f "gz_ros_control" 2>/dev/null
sleep 1

# Navigate to workspace
cd ~/newlocalrepo/rl-aerial-manipulator/Manipulator

# Source the workspace
echo "[2/3] Sourcing workspace..."
source /opt/ros/jazzy/setup.bash
source install/setup.bash

# Launch Gazebo
echo "[3/3] Launching Gazebo..."
echo ""
echo "Wait for these confirmations:"
echo "  ✓ [Dbg] [Physics.cc:883] Loaded [gz::physics::dartsim::Plugin]"
echo "  ✓ Manipulator model spawned in simulation"
echo ""
echo "================================================"

ros2 launch manipulator_description gazebo.launch.py
