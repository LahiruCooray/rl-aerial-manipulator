#!/bin/bash
# Launch Custom Hexacopter with PX4 SITL
# Make sure you're in the PX4-Autopilot directory before running this

set -e

# Colors for output
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
NC='\033[0m' # No Color

echo -e "${YELLOW}Launching Custom Hexacopter with PX4 SITL...${NC}"

# Check if we're in PX4-Autopilot directory
if [ ! -f "ROMFS/px4fmu_common/init.d-posix/airframes/4022_gz_custom_hexa" ]; then
    echo -e "${YELLOW}Not in PX4-Autopilot directory, changing directory...${NC}"
    cd ~/PX4-Autopilot
fi

# Kill any existing PX4 instances
echo -e "${YELLOW}Killing any existing PX4/Gazebo instances...${NC}"
pkill -9 -f px4 2>/dev/null || true
pkill -9 -f gz 2>/dev/null || true
pkill -9 -f ruby 2>/dev/null || true
sleep 2

echo -e "${GREEN}Starting Custom Hexacopter SITL...${NC}"
echo -e "${GREEN}Model: custom_hexa (Hexacopter)${NC}"
echo -e "${GREEN}Airframe: 4022 (Hexarotor X)${NC}"
echo -e "${GREEN}World: custom_hexa_world${NC}"
echo ""

# Set custom world file
export PX4_GZ_WORLD=custom_hexa_world

make px4_sitl gz_custom_hexa
