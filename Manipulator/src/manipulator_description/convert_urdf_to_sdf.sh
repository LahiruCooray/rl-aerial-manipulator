#!/bin/bash

# Convert URDF/xacro to SDF and fix mesh URIs for Gazebo
# Usage: ./convert_urdf_to_sdf.sh
# Note: Run this after building the workspace (colcon build)

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WORKSPACE_DIR="$SCRIPT_DIR/../../.."
XACRO_FILE="$SCRIPT_DIR/urdf/manipulator.urdf.xacro"
SDF_DIR="$SCRIPT_DIR/sdf"
SDF_FILE="$SDF_DIR/manipulator.sdf"
TEMP_URDF="/tmp/manipulator_temp.urdf"

# Source ROS and workspace
if [ -f "$WORKSPACE_DIR/install/setup.bash" ]; then
    echo "Sourcing workspace..."
    source "$WORKSPACE_DIR/install/setup.bash"
else
    echo "Warning: Workspace not built. Run 'colcon build' first."
    echo "Attempting conversion anyway..."
fi

echo "Converting xacro to URDF..."

# Process xacro to URDF first
xacro "$XACRO_FILE" > "$TEMP_URDF"

if [ $? -ne 0 ]; then
    echo "Error: Failed to process xacro file"
    rm -f "$TEMP_URDF"
    exit 1
fi

echo "Converting URDF to SDF..."

# Create sdf directory if it doesn't exist
mkdir -p "$SDF_DIR"

# Convert URDF to SDF using Gazebo
gz sdf -p "$TEMP_URDF" > "$SDF_FILE.tmp"

if [ $? -ne 0 ]; then
    echo "Error: Failed to convert URDF to SDF"
    rm -f "$SDF_FILE.tmp" "$TEMP_URDF"
    exit 1
fi

# Fix mesh URIs: Replace package:// with absolute file:// paths
sed -i "s|package://manipulator_description/meshes|file://$SCRIPT_DIR/meshes|g" "$SDF_FILE.tmp"

# Move to final location
mv "$SDF_FILE.tmp" "$SDF_FILE"

# Clean up temporary URDF
rm -f "$TEMP_URDF"

echo "✓ Conversion complete: $SDF_FILE"
echo "  Mesh URIs updated to use absolute file:// paths"
