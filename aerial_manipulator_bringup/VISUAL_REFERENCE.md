# Visual Quick Reference

## 🔗 Connection Diagram

```
                    🚁 HEXACOPTER
                    ┌─────────────┐
                    │  base_link  │ ← Main body frame
                    └──────┬──────┘
                           │
                    FIXED JOINT (in Gazebo world)
                    STATIC TF (in ROS)
                    offset: [0, 0, -0.15, 0, 0, 0]
                           │
                           ▼
                    🦾 MANIPULATOR  
                    ┌─────────────┐
                    │ base_plate  │ ← Arm base frame
                    └──────┬──────┘
                           │
                    (Robot joints follow)
```

## 📂 File Locations

```
YOUR PROJECT
/home/lahiru/newlocalrepo/rl-aerial-manipulator/

├── hexacopter_description/custom_hexa/
│   └── model.sdf ...................... Hexacopter model (has base_link)
│
├── Manipulator/src/manipulator_description/
│   ├── urdf/
│   │   └── manipulator.urdf.xacro ..... Source URDF (has base_plate)
│   └── sdf/
│       └── manipulator.sdf ............ Generated SDF (for Gazebo)
│                                        ⚠️ Create with convert script!
│
└── aerial_manipulator_bringup/
    ├── worlds/
    │   └── hexa_with_arm.sdf .......... Gazebo world + FIXED JOINT
    │
    ├── launch/
    │   └── sim_bringup.launch.py ...... Starts everything + STATIC TF
    │
    └── convert_urdf_to_sdf.sh ......... Helper script (run once)
```

## 🎬 Startup Flow

```
1. YOU RUN:
   ros2 launch aerial_manipulator_bringup/launch/sim_bringup.launch.py
   
2. LAUNCHES:
   ┌─────────────────────────────────────────────────┐
   │  Gazebo (gz sim)                                │
   │  └─ Loads: worlds/hexa_with_arm.sdf             │
   │      ├─ Spawns hexacopter model                 │
   │      ├─ Spawns manipulator model                │
   │      └─ Creates fixed joint between them        │
   └─────────────────────────────────────────────────┘
   
   ┌─────────────────────────────────────────────────┐
   │  static_transform_publisher                     │
   │  └─ Publishes: base_link → base_plate           │
   │     (TF matches Gazebo physical connection)     │
   └─────────────────────────────────────────────────┘
   
   ┌─────────────────────────────────────────────────┐
   │  robot_state_publisher                          │
   │  └─ Publishes: base_plate → all arm joints      │
   │     (From manipulator.urdf.xacro)               │
   └─────────────────────────────────────────────────┘
   
   ┌─────────────────────────────────────────────────┐
   │  ros_gz_bridge                                  │
   │  └─ Bridges topics between Gazebo ↔ ROS        │
   └─────────────────────────────────────────────────┘

3. RESULT:
   ✅ Gazebo window shows connected hexa + arm
   ✅ TF tree: world → base_link → base_plate → joints
   ✅ Joint states published
   ✅ Ready for controller/MoveIt
```

## 🔍 Verification Commands

```bash
# 1. Check TF transform
ros2 run tf2_ros tf2_echo base_link base_plate
# Should show: Translation [0, 0, -0.15]

# 2. View TF tree
ros2 run tf2_tools view_frames
# Creates frames.pdf with visual tree

# 3. Check joint states
ros2 topic echo /joint_states --once
# Should show: joint_1, joint_2, joint_3, joint_4, joint_5

# 4. Check nodes
ros2 node list
# Should show:
#   /robot_state_publisher
#   /hexa_to_arm_tf (static transform)
#   /ros_gz_bridge
```

## ⚙️ Configuration Match Table

| Location | What | Value |
|----------|------|-------|
| `worlds/hexa_with_arm.sdf` line ~69 | Hexacopter spawn | `<pose>0 0 1 0 0 0</pose>` |
| `worlds/hexa_with_arm.sdf` line ~76 | Manipulator spawn | `<pose>0 0 0.85 0 0 0</pose>` |
| `worlds/hexa_with_arm.sdf` line ~84 | Fixed joint parent | `hexacopter::base_link` |
| `worlds/hexa_with_arm.sdf` line ~87 | Fixed joint child | `manipulator::base_plate` |
| `launch/sim_bringup.launch.py` line ~47 | Static TF offset | `'0', '0', '-0.15'` |
| `launch/sim_bringup.launch.py` line ~50 | Static TF parent | `'base_link'` |
| `launch/sim_bringup.launch.py` line ~51 | Static TF child | `'base_plate'` |

**Math:** `0.85 - 1.0 = -0.15` ✅ (manipulator - hexacopter = TF offset)

## 🎨 Color Code

Throughout the docs:
- 🚁 = Hexacopter related
- 🦾 = Manipulator related  
- 🤝 = Integration related
- ✅ = Completed/verified
- ⚠️ = Action required
- 🔧 = Configuration point
- 📚 = Documentation

## 📏 Coordinate Frame Convention

```
Z-axis points UP
Y-axis points LEFT (when facing forward)
X-axis points FORWARD

Example positions:
┌──────────────────┬─────────────────────────┐
│ Object           │ Position [x, y, z]      │
├──────────────────┼─────────────────────────┤
│ Ground           │ [*, *, 0]               │
│ Hexacopter       │ [0, 0, 1.0]             │
│ Manipulator base │ [0, 0, 0.85]            │
│ Under drone      │ [0, 0, -0.15 from drone]│
└──────────────────┴─────────────────────────┘

Rotations:
- Roll  = rotation around X-axis
- Pitch = rotation around Y-axis  
- Yaw   = rotation around Z-axis
```

## 🎯 Frame Name Reference

```
Gazebo World File (.sdf)
├─ Model: hexacopter
│  └─ Link: base_link ........... hexacopter::base_link
│
└─ Model: manipulator
   ├─ Link: base_plate .......... manipulator::base_plate
   ├─ Link: motor_1 ............. manipulator::motor_1
   ├─ Link: motor_2 ............. manipulator::motor_2
   └─ Link: ... (more links)

ROS TF Tree
├─ world
│  └─ base_link ................. (from Gazebo/PX4)
│     └─ base_plate ............. (static TF)
│        ├─ motor_1 ............. (robot_state_publisher)
│        ├─ motor_2
│        └─ ... (more frames)
```

## 🔧 Adjustment Cheat Sheet

### To mount arm 20cm below drone (instead of 15cm):

1. **In worlds/hexa_with_arm.sdf:**
   ```xml
   <pose>0 0 0.80 0 0 0</pose>  <!-- was 0.85, now 0.80 -->
   ```

2. **In launch/sim_bringup.launch.py:**
   ```python
   arguments=['0', '0', '-0.20',  # was -0.15, now -0.20
   ```

### To mount arm offset to the right by 10cm:

1. **In worlds/hexa_with_arm.sdf:**
   ```xml
   <pose>0 0.10 0.85 0 0 0</pose>  <!-- Y changed from 0 to 0.10 -->
   ```

2. **In launch/sim_bringup.launch.py:**
   ```python
   arguments=['0', '0.10', '-0.15',  # Y changed from 0 to 0.10
   ```

### To rotate arm 90° around Z-axis:

1. **In worlds/hexa_with_arm.sdf:**
   ```xml
   <pose>0 0 0.85 0 0 1.5708</pose>  <!-- Yaw = π/2 radians -->
   ```

2. **In launch/sim_bringup.launch.py:**
   ```python
   arguments=['0', '0', '-0.15', '0', '0', '1.5708',  # Yaw added
   ```

## 🚨 Common Mistakes to Avoid

❌ **DON'T:** Copy models into `aerial_manipulator_bringup/`
✅ **DO:** Reference them from their original locations

❌ **DON'T:** Edit `hexacopter_description/` or `Manipulator/`
✅ **DO:** Keep integration changes in `aerial_manipulator_bringup/`

❌ **DON'T:** Use relative paths in world file
✅ **DO:** Use absolute paths like `/home/lahiru/.../model.sdf`

❌ **DON'T:** Mismatch TF and Gazebo offsets
✅ **DO:** Keep them mathematically consistent

❌ **DON'T:** Forget to convert URDF to SDF
✅ **DO:** Run `convert_urdf_to_sdf.sh` before first launch

## 📞 Quick Help

| Issue | File to Check |
|-------|---------------|
| Models not found | `worlds/hexa_with_arm.sdf` (paths) |
| TF errors | `launch/sim_bringup.launch.py` (static TF) |
| Frame names wrong | Check both `.sdf` model files |
| Setup instructions | `QUICKSTART.md` |
| How it works | `ARCHITECTURE.md` |
| Step-by-step checks | `CHECKLIST.md` |

---

**Need help?** Start with `QUICKSTART.md` → Then check `CHECKLIST.md`
