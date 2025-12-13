#!/bin/bash
# Calls the existing Manipulator/launch_moveit.sh
# Run this AFTER launch_controllers.sh

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
cd "$SCRIPT_DIR/../Manipulator"
./launch_moveit.sh
