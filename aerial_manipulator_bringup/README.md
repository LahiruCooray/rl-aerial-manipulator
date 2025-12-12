# Aerial Manipulator Bringup

Simple launch script for the integrated aerial manipulator system with PX4 SITL and Gazebo Harmonic.

## Quick Start

```bash
./launch_system.sh
```

This launches PX4 SITL with the combined `custom_hexa_arm` model (hexacopter + manipulator integrated).

## What It Does

1. Kills any existing PX4/Gazebo instances
2. Changes to `~/PX4-Autopilot` directory
3. Sets world to `custom_hexa_world`
4. Runs `make px4_sitl gz_custom_hexa_arm`
5. Spawns the fully integrated aerial manipulator in Gazebo

## System Architecture

**Combined Model Approach:**
- **Model**: `custom_hexa_arm` (hexacopter + manipulator in single SDF)
- **PX4 Airframe**: `4023_gz_custom_hexa_arm`
- **Physical Connection**: Fixed joint in SDF
- **World**: `custom_hexa_world`

**In Gazebo:**
- Single model spawned as `custom_hexa_arm_0`
- Manipulator physically attached via fixed joint
- No runtime attachment needed

## Verification

Check in Gazebo entity tree for `custom_hexa_arm_0` containing both hexacopter and manipulator links.

## Troubleshooting

**Model not found:**
```bash
# Verify symlinks:
ls -la ~/PX4-Autopilot/Tools/simulation/gz/models/custom_hexa_arm
ls -la ~/PX4-Autopilot/ROMFS/px4fmu_common/init.d-posix/airframes/4023_gz_custom_hexa_arm
```
