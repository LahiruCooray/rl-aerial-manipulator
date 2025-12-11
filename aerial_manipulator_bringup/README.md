# Aerial Manipulator Bringup

This package integrates the hexacopter and manipulator into a complete aerial manipulator system.

## Structure

```
aerial_manipulator_bringup/
├── launch/
│   └── sim_bringup.launch.py    # Main launch file for the complete system
├── worlds/
│   └── hexa_with_arm.sdf        # Gazebo world with hexacopter + manipulator
├── config/                       # Configuration files (if needed)
└── README.md
```

## Key Integration Points

### 1. Gazebo Physical Connection

In `worlds/hexa_with_arm.sdf`, the hexacopter and manipulator are connected via a fixed joint:

```xml
<joint name="hexa_arm_fixed" type="fixed">
  <parent>hexacopter::base_link</parent>
  <child>manipulator::base_plate</child>
</joint>
```

**Frame names:**
- Hexacopter main body: `base_link`
- Manipulator base: `base_plate`

### 2. ROS TF Connection

The static transform publisher in the launch file matches the Gazebo mounting:

```bash
ros2 run tf2_ros static_transform_publisher \
  0 0 -0.15 0 0 0 \
  base_link base_plate
```

This ensures TF tree consistency:
```
world → base_link (from PX4/Gazebo)
      → base_plate (static TF)
                  → motor_1, motor_2, ... (from robot_state_publisher)
```

## Setup Instructions

### Prerequisites

1. **Convert manipulator URDF to SDF** (required for Gazebo inclusion):
   ```bash
   cd Manipulator/src/manipulator_description
   mkdir -p sdf
   gz sdf -p urdf/manipulator.urdf.xacro > sdf/manipulator.sdf
   ```

2. **Verify hexacopter SDF location**:
   - Check that `hexacopter_description/custom_hexa/model.sdf` exists
   - Update path in `worlds/hexa_with_arm.sdf` if different

3. **Update paths in launch file**:
   - Edit `launch/sim_bringup.launch.py`
   - Set correct paths for:
     - Manipulator URDF
     - Controller launch files
     - MoveIt launch files
     - PX4 directory (if using)

### Adjust Mounting Position

To change where the arm mounts under the drone:

1. **In Gazebo** (`worlds/hexa_with_arm.sdf`):
   ```xml
   <include>
     <name>manipulator</name>
     <pose>0 0 0.85 0 0 0</pose>  <!-- Adjust this -->
   </include>
   ```

2. **In TF** (`launch/sim_bringup.launch.py`):
   ```python
   arguments=[
       '0', '0', '-0.15',  # x, y, z - must match Gazebo offset
       '0', '0', '0',      # roll, pitch, yaw
       'base_link', 'base_plate'
   ]
   ```

**Important:** The TF offset must match the relative position in Gazebo!

## Usage

### Basic Launch

```bash
# Make the launch file executable
chmod +x aerial_manipulator_bringup/launch/sim_bringup.launch.py

# Launch the complete system
ros2 launch aerial_manipulator_bringup/launch/sim_bringup.launch.py
```

### Separate Components (for testing)

If you need to test components individually:

**Hexacopter only:**
```bash
cd hexacopter_description
./launch_sitl.sh
```

**Manipulator only:**
```bash
cd Manipulator
./launch_gazebo.sh
./launch_moveit.sh
```

**Combined system:**
```bash
ros2 launch aerial_manipulator_bringup/launch/sim_bringup.launch.py
```

## Verification

### Check TF Tree

```bash
# View TF frames
ros2 run tf2_tools view_frames

# Check specific transform
ros2 run tf2_ros tf2_echo base_link base_plate
```

### Check Gazebo Joint

```bash
# List all joints in simulation
gz topic -e -t /world/aerial_manipulator_world/model/hexacopter/joint_state
```

### Visualize in RViz

```bash
ros2 run rviz2 rviz2

# Add:
# - RobotModel (to see manipulator)
# - TF (to see frame tree)
# - Marker (for end-effector pose)
```

## Troubleshooting

### "Model not found" error
- Check paths in `worlds/hexa_with_arm.sdf` are absolute and correct
- Verify both models exist at specified locations
- Try setting `GZ_SIM_RESOURCE_PATH` environment variable

### TF tree broken
- Verify `base_link` and `base_plate` frame names match your models
- Check static_transform_publisher is running: `ros2 node list`
- Use `ros2 run tf2_tools view_frames` to visualize tree

### Manipulator not moving with hexacopter
- Verify fixed joint in Gazebo world file
- Check joint parent/child names match model link names
- Confirm models are actually connected (not just positioned near each other)

### Controllers not working
- Source the workspace: `source Manipulator/install/setup.bash`
- Check controller manager is running: `ros2 control list_controllers`
- Verify ros_gz_bridge is bridging necessary topics

## Next Steps

Once basic integration works:

1. **Tune PID controllers** for arm joints under flight dynamics
2. **Add safety limits** to prevent arm motion from destabilizing flight
3. **Implement coordinated control** between flight controller and arm
4. **Test grasp operations** while hovering/moving
5. **Add sensors** (cameras, force sensors) for manipulation tasks

## References

- Hexacopter setup: `hexacopter_description/README.md`
- Manipulator setup: `Manipulator/src/manipulator_description/README.md`
- Launch scripts: `Manipulator/LAUNCH_SCRIPTS_README.md`
