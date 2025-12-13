# Manipulator Launch Scripts

Convenient scripts for launching the manipulator simulation with automatic cleanup and restart functionality.

## 🚀 Quick Start

### Option 1: Automated Launch (Recommended)
```bash
./manipulator_control.sh start
```
This will automatically open 3 terminal windows with Gazebo, Controllers, and MoveIt.

### Option 2: Manual Launch (3 separate terminals)

**Terminal 1: Gazebo**
```bash
./launch_gazebo.sh
```
Wait for: `[Dbg] [Physics.cc:883] Loaded [gz::physics::dartsim::Plugin]`

**Terminal 2: Controllers** (wait ~5 seconds after Gazebo)
```bash
./launch_controllers.sh
```
Wait for: All three controllers show "Configured and activated"

**Terminal 3: MoveIt** (wait for controllers to activate)
```bash
./launch_moveit.sh
```
RViz should open with the manipulator model

## 📋 Master Control Script

```bash
./manipulator_control.sh [command]
```

### Commands:
- **start** - Stop any running instances and launch all components
- **stop** - Stop all running components (Gazebo, controllers, MoveIt)
- **restart** - Stop and restart everything
- **status** - Show status of all components
- **help** - Show usage information

### Examples:
```bash
# Check what's running
./manipulator_control.sh status

# Stop everything
./manipulator_control.sh stop

# Full restart
./manipulator_control.sh restart
```

## 🔧 Individual Scripts

### launch_gazebo.sh
- Kills previous Gazebo instances
- Sources workspace
- Launches Gazebo with DART physics engine
- Spawns manipulator model

### launch_controllers.sh
- Kills previous controller instances
- Restarts ROS 2 daemon
- Checks if Gazebo is running
- Launches ros2_control controllers:
  - `joint_state_broadcaster`
  - `arm_controller` (joints 1-3)
  - `gripper_controller` (joint 4)

### launch_moveit.sh
- Kills previous MoveIt/RViz instances
- Restarts ROS 2 daemon
- Checks if controllers are active
- Launches MoveIt 2 with RViz visualization

## ✅ Expected Behavior

### Gazebo Launch Success:
```
[Dbg] [Physics.cc:883] Loaded [gz::physics::dartsim::Plugin]
```
- Manipulator spawns at origin with joints at safe positions (not at limits)

### Controller Launch Success:
```
[INFO] [spawner_joint_state_broadcaster]: Configured and activated joint_state_broadcaster
[INFO] [spawner_arm_controller]: Configured and activated arm_controller
[INFO] [spawner_gripper_controller]: Configured and activated gripper_controller
```
- All three controllers activate successfully
- No "Failed to activate controller" errors

### MoveIt Launch Success:
- RViz window opens
- Manipulator model visible in planning scene
- MotionPlanning panel shows available planning groups:
  - `arm` (joints 1-3)
  - `gripper` (joint 4, with joint 5 mimicking)

## 🐛 Troubleshooting

### "Controller already loaded" Warning
This is **normal** if gz_ros2_control loads controllers automatically. The important part is activation success.

### Controllers Fail to Activate
```bash
# Check controller status
ros2 control list_controllers

# Check hardware interfaces
ros2 control list_hardware_interfaces

# Check parameters loaded
ros2 param list /controller_manager
```

### Multiple Instances Running
```bash
# Stop everything cleanly
./manipulator_control.sh stop

# Verify nothing is running
./manipulator_control.sh status

# Restart fresh
./manipulator_control.sh start
```

### ROS 2 Daemon Issues
The scripts automatically restart the daemon, but if needed:
```bash
ros2 daemon stop
ros2 daemon start
```

## 🔍 Verification Commands

After launching, verify everything works:

```bash
# Check all controllers are active
ros2 control list_controllers

# Check joint states are publishing
ros2 topic echo /joint_states --once

# Check MoveIt is ready
ros2 topic list | grep move_group

# Test simple motion
ros2 topic pub --once /arm_controller/commands std_msgs/msg/Float64MultiArray \
  "data: [0.5, 0.5, 0.5]"
```

## 📊 Robot Specifications

- **DOF**: 5 (3 arm joints + 2 gripper joints)
- **Physics**: DART engine
- **Servos**: TowerPro MG996R
  - Max velocity: 3.0 rad/s (arm), 2.0 rad/s (gripper)
  - Max acceleration: 8.0 rad/s² (arm), 6.0 rad/s² (gripper)
  - Scaling factor: 0.7 (70% of max for smooth operation)
- **Joint Limits**:
  - joint_1: [-3.14, 3.14] rad
  - joint_2, joint_3: [-1.55, 1.55] rad (0.02 rad safety margin)
  - joint_4: [0.01, 1.57] rad (gripper)
  - joint_5: [-1.57, -0.01] rad (mimic of joint_4 with multiplier=-1)

## 🎯 Motion Planning in RViz

1. In the MotionPlanning panel, select planning group: **arm**
2. Drag the interactive marker to set a goal pose
3. Click **Plan** to generate trajectory (uses STOMP planner)
4. Click **Execute** to move the arm in simulation
5. For gripper control, select planning group: **gripper**

## 📝 Notes

- Scripts automatically handle cleanup of previous instances
- Wait for each component to fully start before launching the next
- The master control script can launch in separate terminal windows (requires gnome-terminal, konsole, xfce4-terminal, or xterm)
- All scripts assume workspace is at: `~/newlocalrepo/rl-aerial-manipulator/Manipulator`

## 🔗 Related Files

- URDF: `src/manipulator_description/urdf/manipulator.urdf.xacro`
- Controllers: `src/manipulator_controller/config/manipulator_controllers.yaml`
- MoveIt Config: `src/manipulator_moveit/config/`
- Joint Limits: `src/manipulator_moveit/config/joint_limits.yaml`
