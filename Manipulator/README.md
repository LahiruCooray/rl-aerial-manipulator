# Manipulator Package

ROS2 workspace for robotic manipulator simulation, control, and motion planning with Gazebo and MoveIt integration.

### Simulation Video
[Watch the simulation in action](files/gazebo_arm_simulation.webm)

## Overview

This workspace contains the complete ROS2-based system for simulating and controlling the robotic manipulator. It includes robot description files, controller configurations, MoveIt motion planning setup, custom message definitions, and remote interface capabilities.

## Package Structure

```
Manipulator/
├── src/
│   ├── manipulator_description/    # Robot URDF/Xacro models and visualization
│   ├── manipulator_controller/     # Controller configuration and launch files
│   ├── manipulator_moveit/         # MoveIt motion planning configuration
│   ├── manipulator_msgs/           # Custom ROS2 message/action definitions
│   ├── manipulator_remote/         # Remote control interface
│   └── python_scripts/             # Python utility scripts
├── build/                          # Build artifacts (auto-generated)
├── install/                        # Installed packages (auto-generated)
├── log/                            # Build and runtime logs (auto-generated)
├── files/                          # Additional resources and utilities
└── launch_all.sh                   # Automated launch script
```

## Packages Description

### manipulator_description
- Contains URDF/Xacro files for robot model definition
- Includes mesh files for visualization
- RViz configuration files
- Gazebo simulation launch files
- Dependencies: `urdf`, `xacro`, `rviz2`, `robot_state_publisher`, `ros_gz_sim`, `ros_gz_bridge`

### manipulator_controller
- Controller configurations for joint control
- Launch files for controller manager
- Integration with ros2_control framework
- Dependencies: `controller_manager`, `joint_trajectory_controller`, `joint_state_broadcaster`, `ros2_controllers`

### manipulator_moveit
- MoveIt configuration for motion planning
- Planning scene setup
- Kinematics solver configuration
- Dependencies: MoveIt2 packages

### manipulator_msgs
- Custom message definitions
- Custom action definitions
- Interface definitions for manipulator-specific communication

### manipulator_remote
- Remote control interface
- Teleoperation capabilities
- High-level command interface

### python_scripts
- Python utility scripts
- Helper functions and tools

## Prerequisites

- **OS**: Ubuntu 24.04 (recommended for Jazzy)
- **ROS2**: Jazzy Jalisco
- **Gazebo**: Gazebo Harmonic
- **Python**: 3.12+

### Installation Guides

Follow the official installation guides for the required components:

1. **ROS2 Jazzy Installation**
   - Follow: [ROS2 Jazzy Installation Guide](https://docs.ros.org/en/jazzy/Installation.html)
   - Install the `ros-jazzy-desktop` package

2. **Gazebo Harmonic Installation**
   - Follow: [Gazebo Harmonic Installation Guide](https://gazebosim.org/docs/latest/getstarted/)

3. **ROS2 Control**
   - Follow: [ros2_control Installation](https://control.ros.org/rolling/index.html)

4. **MoveIt2 for Jazzy**
   - Follow: [MoveIt2 Installation Guide](https://moveit.ai/install-moveit2/binary/)


## Installation

### Step 1: Clone the Repository
```bash
git clone <repository-url>
cd rl-aerial-manipulator/Manipulator
```

### Step 2: Build the Workspace

Build all packages:
```bash
colcon build
```

### Step 3: Source the Workspace

After building, source the setup file:
```bash
source install/setup.bash
```

## Usage

### Quick Start (Automated Launch)

The easiest way to launch the entire system is using the provided launch script:

```bash
# Start all components
./launch_all.sh start

# Stop all components
./launch_all.sh stop
```

This script will automatically launch:
1. Build the workspace
2. Gazebo simulation environment
3. Robot controllers
4. MoveIt motion planning
5. Remote control interface

Each component runs in a separate terminal for easy monitoring.

### Manual Launch (Individual Components)

#### 1. Launch Gazebo Simulation
```bash
source install/setup.bash
ros2 launch manipulator_description gazebo.launch.py
```

#### 2. Launch Controllers
```bash
source install/setup.bash
ros2 launch manipulator_controller controller.launch.py
```

#### 3. Launch MoveIt
```bash
source install/setup.bash
ros2 launch manipulator_moveit moveit.launch.py
```

#### 4. Launch Remote Interface
```bash
source install/setup.bash
ros2 launch manipulator_remote remote_interface.launch.py
```

### Visualization Only (RViz)

To visualize the robot model without Gazebo:
```bash
source install/setup.bash
ros2 launch manipulator_description display.launch.py
```

## License

See individual package.xml files for license information.

## Maintainer

- **Name**: Rajitha Niroshan
- **Email**: nrajitha78@gmail.com


