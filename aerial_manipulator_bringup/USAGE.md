# Aerial Manipulator Launch

## 3 Commands

```bash
# Terminal 1: PX4 + Gazebo
cd aerial_manipulator_bringup
./launch_system.sh

# Terminal 2: Controllers (wait for "Ready for takeoff")
cd aerial_manipulator_bringup
./launch_controllers.sh

# Terminal 3: MoveIt2 (wait for controllers active)
cd aerial_manipulator_bringup
./launch_moveit.sh
```

## What Happens

1. `launch_system.sh` → Launches PX4 SITL + Gazebo with combined model
2. `launch_controllers.sh` → Calls `Manipulator/launch_controllers.sh`
3. `launch_moveit.sh` → Calls `Manipulator/launch_moveit.sh`

**All existing Manipulator scripts are reused without modification.**
