# Custom Hexacopter Model

This directory contains the hexacopter model for Gazebo simulation and RViz visualization.

## Files

- `model.sdf` - Gazebo SDF model
- `model.urdf` - URDF model for RViz visualization and ROS 2 tools
- `model.config` - Gazebo model configuration
- `meshes/` - STL mesh files for visual geometry

## Visualizing the URDF in RViz2

### Prerequisites

Make sure you have ROS 2 installed with the following packages:

```bash
sudo apt install ros-jazzy-robot-state-publisher ros-jazzy-joint-state-publisher-gui ros-jazzy-rviz2
```

### Step 1: Start Robot State Publisher

Open a terminal and run:

```bash
cd /<path_to_repo>/rl-aerial-manipulator/hexacopter_description/custom_hexa

ros2 run robot_state_publisher robot_state_publisher --ros-args -p robot_description:="$(cat model.urdf)"
```

### Step 2: Start Joint State Publisher GUI

In a new terminal:

```bash
ros2 run joint_state_publisher_gui joint_state_publisher_gui
```

### Step 3: Launch RViz2

In another terminal:

```bash
rviz2 -d /opt/ros/jazzy/share/urdf_tutorial/rviz/urdf.rviz
```

Or simply:

```bash
rviz2
```

### Step 4: Configure RViz2

1. **Set Fixed Frame**: In the left panel under "Global Options", change **Fixed Frame** to `world`

2. **Add RobotModel Display** (if not already present):
   - Click **Add** button (bottom left)
   - Select **By display type** → **RobotModel**
   - Click **OK**

3. **Configure RobotModel**:
   - Expand the RobotModel in the Displays panel
   - Set **Description Topic** to `/robot_description`
   - Ensure **Visual Enabled** is checked ✅

4. **Add TF Display** (optional, to see coordinate frames):
   - Click **Add** → **TF** → **OK**


### Quick Launch (All-in-One)

Run all three commands in separate terminals, or use this one-liner with background processes:

```bash
cd /home/lahiru/newlocalrepo/rl-aerial-manipulator/hexacopter_description/custom_hexa

# Terminal 1
ros2 run robot_state_publisher robot_state_publisher --ros-args -p robot_description:="$(cat model.urdf)" &

# Terminal 2  
ros2 run joint_state_publisher_gui joint_state_publisher_gui &

# Terminal 3
rviz2 -d /opt/ros/jazzy/share/urdf_tutorial/rviz/urdf.rviz
```

### Troubleshooting

**Robot not visible in RViz2:**
- Ensure Fixed Frame is set to `world`
- Check that RobotModel status shows "OK"
- Zoom in significantly (drone is ~30cm wide)
- Verify `/robot_description` topic exists: `ros2 topic list | grep robot_description`

**TF errors:**
- Make sure `joint_state_publisher_gui` is running
- Check TF tree: `ros2 run tf2_tools view_frames`

**Mesh loading errors:**
- Verify mesh files exist in the `meshes/` folder
- Check file permissions: `ls -la meshes/`

