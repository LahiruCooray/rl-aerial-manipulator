#!/bin/bash
# Verify PX4-Gazebo bridge is working
# Run this AFTER you've started the simulation

echo "=== Checking Gazebo Topics ==="
gz topic -l | grep -E "f550|motor|imu"

echo ""
echo "=== Checking motor command topic ==="
gz topic -e -t /f550/command/motor_speed -n 1

echo ""
echo "=== Checking IMU topic ==="
gz topic -e -t /f550/imu -n 1
