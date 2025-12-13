#!/bin/bash
# Calls the existing Manipulator/launch_controllers.sh
# Run this AFTER launch_system.sh

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
cd "$SCRIPT_DIR/../Manipulator"
./launch_controllers.sh
