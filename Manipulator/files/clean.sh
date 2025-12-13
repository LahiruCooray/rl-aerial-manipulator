#!/bin/bash

echo "🔹 Killing all Gazebo / gz-sim processes..."
pkill -9 gz 2>/dev/null
pkill -9 gzserver 2>/dev/null
pkill -9 gz-sim 2>/dev/null
pkill -9 gz-gui 2>/dev/null

echo "🔹 Removing Gazebo cache (~/.gz)..."
rm -rf ~/.gz

echo "🔹 Cleaning ROS 2 workspace build/install/log..."
rm -rf build install log

echo "🔹 Clean complete! Run: colcon build && source install/setup.bash"
