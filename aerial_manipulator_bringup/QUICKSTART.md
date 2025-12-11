# Quick Start Guide - Aerial Manipulator Integration

## 🎯 What This Package Does

This `aerial_manipulator_bringup` package is your **integration layer** that:
- ✅ Keeps your hexacopter code clean and separate
- ✅ Keeps your manipulator code clean and separate  
- ✅ Combines them together for the full aerial manipulator system

**Think of it as the "glue" between your drone and arm.**

## 📁 What You Have Now

```
rl-aerial-manipulator/
├── hexacopter_description/        ← drone 
├── Manipulator/                   ← arm 
└── aerial_manipulator_bringup/    ← NEW! Integration package
    ├── launch/
    │   └── sim_bringup.launch.py  ← Launch everything together
    ├── worlds/
    │   └── hexa_with_arm.sdf      ← Gazebo world with fixed joint
    ├── config/                     ← (for future configs)
    ├── convert_urdf_to_sdf.sh     ← Helper script
    └── README.md                   ← Full documentation
```

## 🚀 Setup (Do Once)

### Step 1: Convert URDF to SDF

Your manipulator is described in URDF, but Gazebo needs SDF. Convert it:

```bash
cd /home/lahiru/newlocalrepo/rl-aerial-manipulator/aerial_manipulator_bringup
./convert_urdf_to_sdf.sh
```

This creates: `Manipulator/src/manipulator_description/sdf/manipulator.sdf`

### Step 2: Verify Frame Names

**Critical:** Make sure frame names match!

**Hexacopter:** `base_link` (main body)  
**Manipulator:** `base_plate` (first link of arm)

These are already set up correctly in your files ✅

### Step 3: Adjust Mounting Position (Optional)

The arm is currently set to mount **15 cm below** the hexacopter center.

**To change this:**

1. Edit `worlds/hexa_with_arm.sdf`, line with manipulator pose:
   ```xml
   <pose>0 0 0.85 0 0 0</pose>  ← Change Z value
   ```

2. Edit `launch/sim_bringup.launch.py`, static TF arguments:
   ```python
   arguments=['0', '0', '-0.15',  ← Change Z to match
   ```

**Rule:** If manipulator pose Z = 0.85 and hexacopter pose Z = 1.0,  
then TF offset Z = 0.85 - 1.0 = -0.15 ✅

## 🎮 Usage

### Launch the Complete System

```bash
cd /home/lahiru/newlocalrepo/rl-aerial-manipulator

# Make sure ROS 2 is sourced
source /opt/ros/humble/setup.bash

# Source your workspace (if built)
source Manipulator/install/setup.bash

# Launch everything!
ros2 launch aerial_manipulator_bringup/launch/sim_bringup.launch.py
```

**This starts:**
- ✅ Gazebo with hexacopter + manipulator
- ✅ Fixed joint connecting them
- ✅ Static TF (base_link → base_plate)
- ✅ Robot state publisher for the arm
- ✅ ROS-Gazebo bridge

### Verify It's Working

**Check TF tree:**
```bash
ros2 run tf2_tools view_frames
# Opens a PDF showing: world → base_link → base_plate → joints...
```

**Check the transform:**
```bash
ros2 run tf2_ros tf2_echo base_link base_plate
# Should show the -0.15m Z offset
```

**View in RViz:**
```bash
ros2 run rviz2 rviz2
# Add → RobotModel
# Add → TF
# Fixed Frame = base_link
```

## 🔧 Next TODOs in Launch File

The launch file has some commented sections you need to uncomment when ready:

### 1. Add PX4 SITL (if using PX4)
```python
# Around line 95 in sim_bringup.launch.py
px4_sitl = ExecuteProcess(
    cmd=['make', 'px4_sitl', 'gz_custom_hexa'],
    cwd='/path/to/PX4-Autopilot',  # ← Update this path
    output='screen'
)
```

### 2. Add Controller Manager
```python
# Around line 90
controller_manager = IncludeLaunchDescription(
    PythonLaunchDescriptionSource([
        os.path.join(workspace_root, 'Manipulator/launch/controllers.launch.py')
    ])
)
```

### 3. Add MoveIt2
```python
# Around line 99
moveit = IncludeLaunchDescription(
    PythonLaunchDescriptionSource([
        os.path.join(workspace_root, 'Manipulator/launch/moveit.launch.py')
    ])
)
```

## 🧪 Testing Individual Components

Before launching the full system, you can test parts separately:

**Just the hexacopter:**
```bash
cd hexacopter_description
./launch_sitl.sh
```

**Just the manipulator:**
```bash
cd Manipulator
./launch_gazebo.sh
./launch_moveit.sh
```

**Combined (this package):**
```bash
ros2 launch aerial_manipulator_bringup/launch/sim_bringup.launch.py
```

## 🔍 Key Concepts

### Why the Fixed Joint?

In Gazebo world file:
```xml
<joint name="hexa_arm_fixed" type="fixed">
  <parent>hexacopter::base_link</parent>
  <child>manipulator::base_plate</child>
</joint>
```

This **physically welds** the arm to the drone:
- When drone moves → arm moves with it
- Arm's mass affects drone's dynamics
- They act as one rigid system in physics simulation

### Why the Static TF?

In ROS:
```bash
ros2 run tf2_ros static_transform_publisher 0 0 -0.15 0 0 0 base_link base_plate
```

This tells ROS's TF system:
- "base_plate is always 15 cm below base_link"
- MoveIt can plan arm motions in the correct frame
- All ROS nodes see consistent transforms

**These two must match!** Gazebo = physics, TF = ROS knowledge.

## 📚 Full Documentation

See `README.md` in this directory for:
- Detailed troubleshooting
- Advanced configuration
- Safety considerations
- Next development steps

## 💡 Why This Structure?

**Before:** You might have been tempted to merge everything into `hexacopter_description/` or `Manipulator/`

**Problem:** Mixed concerns, hard to maintain, can't test components separately

**Now:** Clean separation:
- `hexacopter_description/` → only cares about drone ✈️
- `Manipulator/` → only cares about arm 🦾
- `aerial_manipulator_bringup/` → combines them 🤝

This is standard ROS 2 project structure used by professionals!

---

**Questions? Issues?**  
Check the troubleshooting section in `README.md` or the Gazebo/ROS 2 documentation.
