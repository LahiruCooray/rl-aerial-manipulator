#!/bin/bash

# Script to convert manipulator URDF to SDF for Gazebo inclusion
# This must be done before launching the integrated system

echo "Converting manipulator URDF to SDF..."

WORKSPACE_ROOT="/home/lahiru/newlocalrepo/rl-aerial-manipulator"
MANIPULATOR_DIR="$WORKSPACE_ROOT/Manipulator/src/manipulator_description"
URDF_FILE="$MANIPULATOR_DIR/urdf/manipulator.urdf.xacro"
SDF_DIR="$MANIPULATOR_DIR/sdf"
SDF_FILE="$SDF_DIR/manipulator.sdf"

# Create SDF directory if it doesn't exist
mkdir -p "$SDF_DIR"

# Check if URDF file exists
if [ ! -f "$URDF_FILE" ]; then
    echo "ERROR: URDF file not found at $URDF_FILE"
    exit 1
fi

# This requires ROS 2 to be sourced
if ! command -v ros2 &> /dev/null; then
    echo "ERROR: ROS 2 not found. Please source your ROS 2 installation first:"
    echo "  source /opt/ros/jazzy/setup.bash"
    exit 1
fi

# Check if manipulator workspace is sourced
MANIPULATOR_INSTALL="$WORKSPACE_ROOT/Manipulator/install"
if [ -d "$MANIPULATOR_INSTALL" ]; then
    echo "Found manipulator workspace, checking if sourced..."
    if ! ros2 pkg list | grep -q "manipulator_description"; then
        echo ""
        echo "⚠️  ERROR: manipulator_description package not found in ROS package path."
        echo ""
        echo "You need to source the manipulator workspace first:"
        echo "  cd $WORKSPACE_ROOT/Manipulator"
        echo "  source install/setup.bash"
        echo ""
        echo "Then run this script again:"
        echo "  cd $WORKSPACE_ROOT/aerial_manipulator_bringup"
        echo "  ./convert_urdf_to_sdf.sh"
        echo ""
        exit 1
    fi
else
    echo ""
    echo "⚠️  WARNING: Manipulator workspace not built yet."
    echo ""
    echo "You need to build the manipulator workspace first:"
    echo "  cd $WORKSPACE_ROOT/Manipulator"
    echo "  colcon build"
    echo "  source install/setup.bash"
    echo ""
    echo "Then run this script again:"
    echo "  cd $WORKSPACE_ROOT/aerial_manipulator_bringup"
    echo "  ./convert_urdf_to_sdf.sh"
    echo ""
    exit 1
fi

# Process xacro to URDF first (if using xacro)
echo "Processing xacro to URDF..."
TEMP_URDF="/tmp/manipulator_processed.urdf"

# Process xacro file
xacro "$URDF_FILE" > "$TEMP_URDF"

if [ $? -ne 0 ]; then
    echo "ERROR: Failed to process xacro file"
    exit 1
fi

# Convert URDF to SDF
echo "Converting URDF to SDF..."

if ! command -v gz &> /dev/null; then
    echo "ERROR: Gazebo (gz) command not found. Please install Gazebo first."
    exit 1
fi

gz sdf -p "$TEMP_URDF" > "$SDF_FILE"

if [ $? -ne 0 ]; then
    echo "ERROR: Failed to convert URDF to SDF"
    rm -f "$TEMP_URDF"
    exit 1
fi

# Clean up
rm -f "$TEMP_URDF"

echo "SUCCESS! SDF file created at: $SDF_FILE"
echo ""
echo "Next steps:"
echo "1. Review the generated SDF file"
echo "2. Update world file path if needed: aerial_manipulator_bringup/worlds/hexa_with_arm.sdf"
echo "3. Launch the integrated system:"
echo "   ros2 launch aerial_manipulator_bringup/launch/sim_bringup.launch.py"
