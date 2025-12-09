#!/bin/bash
# Wrapper to run test_motors.py with correct environment

# Source ROS if needed
if [ -f "/opt/ros/jazzy/setup.bash" ]; then
    source /opt/ros/jazzy/setup.bash
fi

# Add gz to PATH if not already there
if ! command -v gz &> /dev/null; then
    export PATH="/opt/ros/jazzy/opt/gz_tools_vendor/bin:$PATH"
fi

# Run the Python script
cd "$(dirname "$0")"
python3 ./test_motors.py "$@"
