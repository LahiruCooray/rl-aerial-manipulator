#!/bin/bash
# Quick test script to verify manipulator system is working
# This script checks each component and reports any issues

echo "================================================"
echo "  Manipulator System Test"
echo "================================================"
echo ""

# Color codes
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
NC='\033[0m' # No Color

PASS=0
FAIL=0
WARN=0

test_check() {
    local test_name="$1"
    local test_result="$2"
    local test_message="$3"
    
    if [ "$test_result" -eq 0 ]; then
        echo -e "${GREEN}✓${NC} $test_name"
        ((PASS++))
    elif [ "$test_result" -eq 2 ]; then
        echo -e "${YELLOW}⚠${NC} $test_name - $test_message"
        ((WARN++))
    else
        echo -e "${RED}✗${NC} $test_name - $test_message"
        ((FAIL++))
    fi
}

echo "[1/10] Checking Gazebo..."
if pgrep -f "gz sim" > /dev/null; then
    test_check "Gazebo process" 0
else
    test_check "Gazebo process" 1 "Not running"
fi

echo ""
echo "[2/10] Checking gz_ros_control node..."
if ros2 node list 2>/dev/null | grep -q "gz_ros_control"; then
    test_check "gz_ros_control node" 0
else
    test_check "gz_ros_control node" 1 "Not found"
fi

echo ""
echo "[3/10] Checking DART physics plugin..."
if ros2 topic list 2>/dev/null | grep -q "/world/empty"; then
    test_check "Gazebo world topics" 0
else
    test_check "Gazebo world topics" 1 "World topics not found"
fi

echo ""
echo "[4/10] Checking robot_state_publisher..."
if ros2 node list 2>/dev/null | grep -q "robot_state_publisher"; then
    test_check "robot_state_publisher" 0
else
    test_check "robot_state_publisher" 1 "Not running"
fi

echo ""
echo "[5/10] Checking controller_manager..."
if ros2 node list 2>/dev/null | grep -q "controller_manager"; then
    test_check "controller_manager node" 0
    
    # Check if it's responding
    if timeout 2 ros2 control list_controllers > /dev/null 2>&1; then
        test_check "controller_manager service" 0
    else
        test_check "controller_manager service" 1 "Not responding"
    fi
else
    test_check "controller_manager node" 1 "Not running"
fi

echo ""
echo "[6/10] Checking controllers..."

# Check joint_state_broadcaster
if ros2 control list_controllers 2>/dev/null | grep -q "joint_state_broadcaster"; then
    STATUS=$(ros2 control list_controllers 2>/dev/null | grep "joint_state_broadcaster" | awk '{print $2}')
    if [ "$STATUS" = "active" ]; then
        test_check "joint_state_broadcaster" 0
    else
        test_check "joint_state_broadcaster" 2 "Status: $STATUS"
    fi
else
    test_check "joint_state_broadcaster" 1 "Not loaded"
fi

# Check arm_controller
if ros2 control list_controllers 2>/dev/null | grep -q "arm_controller"; then
    STATUS=$(ros2 control list_controllers 2>/dev/null | grep "arm_controller" | awk '{print $2}')
    if [ "$STATUS" = "active" ]; then
        test_check "arm_controller" 0
    else
        test_check "arm_controller" 2 "Status: $STATUS"
    fi
else
    test_check "arm_controller" 1 "Not loaded"
fi

# Check gripper_controller
if ros2 control list_controllers 2>/dev/null | grep -q "gripper_controller"; then
    STATUS=$(ros2 control list_controllers 2>/dev/null | grep "gripper_controller" | awk '{print $2}')
    if [ "$STATUS" = "active" ]; then
        test_check "gripper_controller" 0
    else
        test_check "gripper_controller" 2 "Status: $STATUS"
    fi
else
    test_check "gripper_controller" 1 "Not loaded"
fi

echo ""
echo "[7/10] Checking hardware interfaces..."
HW_INTERFACES=$(ros2 control list_hardware_interfaces 2>/dev/null | grep -c "available")
if [ "$HW_INTERFACES" -ge 4 ]; then
    test_check "Hardware interfaces" 0
    echo "     Found $HW_INTERFACES available interfaces"
else
    test_check "Hardware interfaces" 2 "Only $HW_INTERFACES found (expected 4+)"
fi

echo ""
echo "[8/10] Checking joint_states topic..."
if timeout 2 ros2 topic echo /joint_states --once > /dev/null 2>&1; then
    test_check "Joint states publishing" 0
else
    test_check "Joint states publishing" 1 "No data on /joint_states"
fi

echo ""
echo "[9/10] Checking MoveIt..."
if ros2 node list 2>/dev/null | grep -q "move_group"; then
    test_check "move_group node" 0
else
    test_check "move_group node" 1 "Not running"
fi

echo ""
echo "[10/10] Checking RViz..."
if pgrep -f "rviz" > /dev/null; then
    test_check "RViz process" 0
else
    test_check "RViz process" 1 "Not running"
fi

echo ""
echo "================================================"
echo "  Test Summary"
echo "================================================"
echo -e "${GREEN}Passed:${NC}  $PASS"
echo -e "${YELLOW}Warnings:${NC} $WARN"
echo -e "${RED}Failed:${NC}  $FAIL"
echo ""

TOTAL=$((PASS + WARN + FAIL))
if [ $TOTAL -gt 0 ]; then
    SUCCESS_RATE=$((PASS * 100 / TOTAL))
    echo "Success Rate: ${SUCCESS_RATE}%"
fi

echo "================================================"
echo ""

# Overall result
if [ $FAIL -eq 0 ] && [ $WARN -eq 0 ]; then
    echo -e "${GREEN}✓ All systems operational!${NC}"
    echo ""
    echo "You can now:"
    echo "  • Plan and execute trajectories in RViz"
    echo "  • Control the arm via topics/services"
    echo "  • Test motion planning with different planners"
    exit 0
elif [ $FAIL -eq 0 ]; then
    echo -e "${YELLOW}⚠ System running with warnings${NC}"
    echo ""
    echo "Some components may not be fully operational."
    echo "Check the warnings above for details."
    exit 0
else
    echo -e "${RED}✗ System has failures${NC}"
    echo ""
    echo "Troubleshooting steps:"
    echo "  1. Stop all components: ./manipulator_control.sh stop"
    echo "  2. Restart: ./manipulator_control.sh start"
    echo "  3. Check logs in each terminal window"
    echo "  4. Run this test again"
    exit 1
fi
