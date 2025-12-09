#!/bin/bash
# Launch Gazebo only (without PX4) for motor testing
# This allows you to test the hexacopter model and motors directly

set -e

# Colors for output
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
NC='\033[0m' # No Color

echo -e "${YELLOW}Launching Gazebo Only - Custom Hexacopter${NC}"

# Set Gazebo model path to include custom_hexa model
export GZ_SIM_RESOURCE_PATH=$GZ_SIM_RESOURCE_PATH:$(pwd)

# Kill any existing Gazebo instances
echo -e "${YELLOW}Killing any existing Gazebo instances...${NC}"
pkill -9 -f gz 2>/dev/null || true
pkill -9 -f ruby 2>/dev/null || true
sleep 2

echo -e "${GREEN}Starting Gazebo with custom_hexa_world${NC}"
echo -e "${GREEN}World file: $(pwd)/worlds/custom_hexa_world.sdf${NC}"
echo ""
echo -e "${YELLOW}To test motors, run in another terminal:${NC}"
echo -e "${GREEN}  cd $(pwd)${NC}"
echo -e "${GREEN}  ./test_motors.py${NC}"
echo ""

# Launch Gazebo with the custom world
gz sim worlds/custom_hexa_world.sdf
