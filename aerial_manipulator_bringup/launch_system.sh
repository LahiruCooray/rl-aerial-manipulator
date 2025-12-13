#!/bin/bash
# Launch Aerial Manipulator (Hexacopter + Robot Arm) with PX4 SITL

set -e

# Colors for output
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
NC='\033[0m' # No Color

echo -e "${YELLOW}Launching Aerial Manipulator with PX4 SITL...${NC}"

# Check if we're in PX4-Autopilot directory, if not change to it
if [ ! -f "ROMFS/px4fmu_common/init.d-posix/airframes/4023_gz_custom_hexa_arm" ]; then
    echo -e "${YELLOW}Changing to PX4-Autopilot directory...${NC}"
    cd ~/PX4-Autopilot
fi

# Kill any existing PX4 instances
echo -e "${YELLOW}Killing any existing PX4/Gazebo instances...${NC}"
pkill -9 -f px4 2>/dev/null || true
pkill -9 -f gz 2>/dev/null || true
pkill -9 -f ruby 2>/dev/null || true
sleep 2

echo -e "${GREEN}Starting Aerial Manipulator SITL...${NC}"
echo -e "${GREEN}Model: custom_hexa_arm (Hexacopter + Manipulator)${NC}"
echo -e "${GREEN}Airframe: 4023_gz_custom_hexa_arm (Hexarotor X + Arm)${NC}"
echo -e "${GREEN}World: custom_hexa_world${NC}"
echo ""

# Set custom world file
export PX4_GZ_WORLD=custom_hexa_world

# Add ROS 2 Gazebo plugins to path for gz_ros2_control
export GZ_SIM_SYSTEM_PLUGIN_PATH=/opt/ros/jazzy/lib:$GZ_SIM_SYSTEM_PLUGIN_PATH
export LD_LIBRARY_PATH=/opt/ros/jazzy/lib:$LD_LIBRARY_PATH

make px4_sitl gz_custom_hexa_arm
