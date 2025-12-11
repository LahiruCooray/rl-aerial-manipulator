#!/bin/bash

# Complete setup script for aerial manipulator integration
# This script will:
#   1. Check if Manipulator workspace is built
#   2. Build it if needed
#   3. Source the workspace
#   4. Convert URDF to SDF
#   5. Verify everything is ready

set -e  # Exit on any error

WORKSPACE_ROOT="/home/lahiru/newlocalrepo/rl-aerial-manipulator"
MANIPULATOR_DIR="$WORKSPACE_ROOT/Manipulator"
INTEGRATION_DIR="$WORKSPACE_ROOT/aerial_manipulator_bringup"

echo "================================================"
echo "  Aerial Manipulator Integration Setup"
echo "================================================"
echo ""

# Check ROS 2 installation
echo "[1/5] Checking ROS 2 installation..."
if ! command -v ros2 &> /dev/null; then
    echo "❌ ERROR: ROS 2 not found!"
    echo ""
    echo "Please source ROS 2 first:"
    echo "  source /opt/ros/jazzy/setup.bash"
    echo ""
    exit 1
fi
echo "✅ ROS 2 found: $(ros2 --version)"
echo ""

# Check if Manipulator workspace exists
echo "[2/5] Checking Manipulator workspace..."
if [ ! -d "$MANIPULATOR_DIR" ]; then
    echo "❌ ERROR: Manipulator directory not found at $MANIPULATOR_DIR"
    exit 1
fi

# Check if workspace is built
if [ ! -d "$MANIPULATOR_DIR/install" ]; then
    echo "⚠️  Manipulator workspace not built yet. Building now..."
    echo ""
    cd "$MANIPULATOR_DIR"
    colcon build --symlink-install
    echo ""
    echo "✅ Build complete!"
else
    echo "✅ Manipulator workspace already built"
fi
echo ""

# Source the workspace
echo "[3/5] Sourcing Manipulator workspace..."
cd "$MANIPULATOR_DIR"
# 1) Source ROS
source /opt/ros/${ROS_DISTRO}/setup.bash



source install/setup.bash


if ros2 pkg list | grep -q "manipulator_description"; then
    echo "✅ manipulator_description package found in ROS path"
else
    echo "❌ ERROR: Failed to source manipulator_description package"
    exit 1
fi
echo ""

# Convert URDF to SDF
echo "[4/5] Converting URDF to SDF..."
cd "$INTEGRATION_DIR"

URDF_FILE="$MANIPULATOR_DIR/src/manipulator_description/urdf/manipulator.urdf.xacro"
SDF_DIR="$MANIPULATOR_DIR/src/manipulator_description/sdf"
SDF_FILE="$SDF_DIR/manipulator.sdf"

# Create SDF directory
mkdir -p "$SDF_DIR"

# Process xacro to URDF
TEMP_URDF="/tmp/manipulator_processed.urdf"
echo "   Processing xacro..."
xacro "$URDF_FILE" > "$TEMP_URDF"

if [ $? -ne 0 ]; then
    echo "❌ ERROR: Failed to process xacro file"
    rm -f "$TEMP_URDF"
    exit 1
fi

# Convert URDF to SDF
echo "   Converting to SDF..."
if ! command -v gz &> /dev/null; then
    echo "❌ ERROR: Gazebo (gz) command not found!"
    echo "Please install Gazebo Harmonic first."
    rm -f "$TEMP_URDF"
    exit 1
fi

gz sdf -p "$TEMP_URDF" > "$SDF_FILE"

if [ $? -ne 0 ]; then
    echo "❌ ERROR: Failed to convert URDF to SDF"
    rm -f "$TEMP_URDF"
    exit 1
fi

rm -f "$TEMP_URDF"
echo "✅ SDF file created: $SDF_FILE"
echo ""

# Verify files
echo "[5/5] Verifying setup..."

# Check hexacopter model
HEXA_MODEL="$WORKSPACE_ROOT/hexacopter_description/custom_hexa/model.sdf"
if [ -f "$HEXA_MODEL" ]; then
    echo "✅ Hexacopter model found"
else
    echo "⚠️  WARNING: Hexacopter model not found at $HEXA_MODEL"
    echo "   You may need to update the path in worlds/hexa_with_arm.sdf"
fi

# Check manipulator SDF
if [ -f "$SDF_FILE" ]; then
    echo "✅ Manipulator SDF found"
else
    echo "❌ ERROR: Manipulator SDF not created!"
    exit 1
fi

# Check world file
WORLD_FILE="$INTEGRATION_DIR/worlds/hexa_with_arm.sdf"
if [ -f "$WORLD_FILE" ]; then
    echo "✅ World file found"
else
    echo "❌ ERROR: World file not found!"
    exit 1
fi

# Check launch file
LAUNCH_FILE="$INTEGRATION_DIR/launch/sim_bringup.launch.py"
if [ -f "$LAUNCH_FILE" ]; then
    echo "✅ Launch file found"
else
    echo "❌ ERROR: Launch file not found!"
    exit 1
fi

echo ""
echo "================================================"
echo "  ✅ Setup Complete!"
echo "================================================"
echo ""
echo "Next steps:"
echo ""
echo "1. Review the paths in the world file:"
echo "   nano $WORLD_FILE"
echo ""
echo "2. Launch the integrated system:"
echo "   # In a new terminal, source both ROS and the workspace:"
echo "   source /opt/ros/jazzy/setup.bash"
echo "   source $MANIPULATOR_DIR/install/setup.bash"
echo "   "
echo "   # Then launch:"
echo "   ros2 launch $LAUNCH_FILE"
echo ""
echo "3. Verify TF tree:"
echo "   ros2 run tf2_tools view_frames"
echo ""
echo "See QUICKSTART.md for detailed instructions."
echo ""
