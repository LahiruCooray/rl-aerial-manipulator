# Hexacopter Description

## Overview

This directory contains a custom hexacopter model for Gazebo simulation with PX4 autopilot integration. The model is designed for reinforcement learning (RL) research with a focus on simulation-to-real transfer.

**Key Features:**
- Accurate physics model matching real hardware specifications
- PX4 SITL (Software-In-The-Loop) integration
- Sensor noise modeling for realistic simulation
- Optimized for RL training and testing

**Hardware Specifications:**
- Battery: 4S LiPo (14.8V nominal)
- Propellers: 11×5.7 inches
- Motors: 1250 KV brushless
- Configuration: Hexarotor X-layout
- Total mass: ~2.0 kg

## Directory Structure

```
hexacopter_description/
├── custom_hexa/              # Gazebo model definition
│   ├── model.config          # Model metadata
│   ├── model.sdf             # Complete SDF vehicle description
│   └── meshes/               # 3D mesh files for visualization
├── airframe/
│   └── 4022_gz_custom_hexa   # PX4 airframe configuration
├── custom_hexa_world.sdf     # World file for Gazebo simulation
├── launch_sitl.sh            # Quick launch script for SITL
├── verify_bridge.sh          # Diagnostic script for topic verification
└── README.md                 # This file
```

### File Descriptions

- **`custom_hexa/model.sdf`**: Main Gazebo model file containing vehicle geometry, inertial properties, sensors (IMU), and motor plugin configurations
- **`airframe/4022_gz_custom_hexa`**: PX4 airframe configuration with control allocation parameters, motor mapping, and flight controller gains
- **`custom_hexa_world.sdf`**: Optional Gazebo world environment file
- **`launch_sitl.sh`**: Convenience script to start PX4 SITL with Gazebo and load this model
- **`verify_bridge.sh`**: Diagnostic tool to check Gazebo-PX4 communication bridge

## Coordinate Frames

Understanding coordinate frame conventions is critical when modifying rotor positions or debugging control issues.

**Gazebo SDF (FLU Convention):**
- X-axis: Forward
- Y-axis: Left
- Z-axis: Up

**PX4 Control Allocation (FRD Convention):**
- X-axis: Forward
- Y-axis: Right
- Z-axis: Down

**Conversion Rules:**
When transferring rotor positions from SDF to PX4:
```
FRD.x =  SDF.x
FRD.y = -SDF.y
FRD.z = -SDF.z
```

## Motor Configuration

### Physical Layout

The hexacopter uses an X-configuration with 6 rotors:

| Motor | Position (X, Y) [m] | Rotation | SDF Link | PX4 Output |
|-------|---------------------|----------|----------|------------|
| 0     | (-0.256, -0.148)    | CW       | rotor_0  | Motor1     |
| 1     | (0.256, 0.148)      | CCW      | rotor_1  | Motor2     |
| 2     | (-0.256, 0.148)     | CCW      | rotor_2  | Motor3     |
| 3     | (0.256, -0.148)     | CW       | rotor_3  | Motor4     |
| 4     | (0.0, -0.295)       | CCW      | rotor_4  | Motor5     |
| 5     | (0.0, 0.295)        | CW       | rotor_5  | Motor6     |

### Motor Parameters (SDF)

All six motors share identical dynamic parameters:

| Parameter           | Value           | Unit          | Description                        |
|---------------------|-----------------|---------------|------------------------------------|
| `maxRotVelocity`    | 820.0           | rad/s         | Maximum motor speed (current-limited) |
| `motorConstant`     | 2.11×10⁻⁵       | N/(rad/s)²    | Thrust coefficient (kF)            |
| `momentConstant`    | 0.0168          | m             | Torque-to-thrust ratio (Q/T)       |
| `timeConstantUp`    | 0.015           | s             | Motor spin-up time constant        |
| `timeConstantDown`  | 0.015           | s             | Motor spin-down time constant      |

### PX4 Airframe Configuration

**Control Allocation:**
- `CA_ROTOR_COUNT`: 6
- `CA_ROTORx_KM`: ±0.0168 (positive for CCW, negative for CW)
- Motor positions configured in FRD frame

**Gazebo Bridge Mapping:**
- `SIM_GZ_EC_FUNC1` to `SIM_GZ_EC_FUNC6`: 101 to 106 (Motor1 to Motor6)
- Maps 1:1 to SDF `actuator_number` 0 to 5

**Motor Speed Limits:**
- Minimum: 150 rad/s
- Maximum: 820 rad/s

**Flight Controller Gains:**

The airframe includes conservative initial tuning values suitable for SITL testing:

- **Rate Control (P/I/D):**
  - Roll: 0.15 / 0.2 / 0.003
  - Pitch: 0.15 / 0.2 / 0.003
  - Yaw: 0.2 / 0.1 / 0.0

- **Attitude Control (P):**
  - Roll: 6.5
  - Pitch: 6.5
  - Yaw: 2.8

- **Thrust:**
  - Hover throttle: 0.5 (50%)
  - Minimum manual throttle: 8%

**Note:** These gains are conservative starting points. You will need to retune them when transferring to real hardware.

## Sensors

### IMU (Inertial Measurement Unit)

The model includes a simulated IMU based on the ICM-42688-P sensor:

- **Update rate:** 250 Hz
- **Gyroscope noise:** 0.00034 rad/s (standard deviation)
- **Accelerometer noise:** 0.004 m/s² (standard deviation)
- **Bias modeling:** Dynamic bias drift for both gyro and accelerometer
- **Location:** Positioned at (0, 0, 0.02) relative to base_link

## Quick Start Guide

### Prerequisites

- PX4 Autopilot installed and built
- Gazebo (gz-sim) installed
- ROS 2 (if using bridge features)

### Setup Steps

**1. Make the model discoverable to Gazebo:**

From the `hexacopter_description` directory:
```bash
export GZ_SIM_RESOURCE_PATH=$GZ_SIM_RESOURCE_PATH:$(pwd)
```

Add this to your `~/.bashrc` for persistence:
```bash
echo 'export GZ_SIM_RESOURCE_PATH=$GZ_SIM_RESOURCE_PATH:/path/to/hexacopter_description' >> ~/.bashrc
```

**2. Install the PX4 airframe (one-time setup):**

Copy or symlink the airframe file to your PX4 installation:
```bash
cd PX4-Autopilot
ln -s /absolute/path/to/hexacopter_description/airframe/4022_gz_custom_hexa \
      ROMFS/px4fmu_common/init.d-posix/airframes/4022_gz_custom_hexa
```

**3. Launch the simulation:**

Option A - Using the launch script:
```bash
cd PX4-Autopilot
/path/to/hexacopter_description/launch_sitl.sh
```

Option B - Using PX4 make targets:
```bash
cd PX4-Autopilot
make px4_sitl gz_custom_hexa
```

**4. Verify the Gazebo-PX4 bridge:**

After the simulation starts, run the verification script:
```bash
/path/to/hexacopter_description/verify_bridge.sh
```

Expected output:
- Lists active topics including `/custom_hexa/command/motor_speed` and `/custom_hexa/imu`
- Displays sample messages from each topic
- Confirms bidirectional communication

## Simulation Configuration

### SITL-Specific Parameters

The airframe automatically sets these parameters for SITL convenience:

- `SIM_GZ_EN = 1`: Enable Gazebo simulation
- `EKF2_HGT_REF = 1`: Use barometer for altitude reference
- `COM_ARM_WO_GPS = 1`: Allow arming without GPS (for indoor testing)

### Inertial Properties

The base_link inertial properties represent the combined mass of:
- Frame structure (carbon fiber plates)
- Battery (4S LiPo)
- Flight controller and electronics
- Motor mounts

**Total mass:** 1.9949 kg  
**Center of mass:** 0.0425 m above base  
**Inertia tensor (kg⋅m²):**
```
Ixx = Iyy = 0.0106
Izz = 0.0187
```

These values are calculated for a regular hexagonal prism approximation matching the real platform dimensions.

## Troubleshooting

### Common Issues

**Problem:** Gazebo cannot find the model  
**Solution:** Verify `GZ_SIM_RESOURCE_PATH` includes the hexacopter_description directory

**Problem:** PX4 cannot find airframe 4022  
**Solution:** Check that the airframe file is properly linked in PX4's ROMFS directory

**Problem:** Motors not responding  
**Solution:** Run `verify_bridge.sh` to check topic communication

**Problem:** Vehicle unstable or oscillating  
**Solution:** Reduce rate controller gains (MC_ROLLRATE_P, MC_PITCHRATE_P) by 20-30%

**Problem:** Vehicle drifts or tilts on takeoff  
**Solution:** Check IMU calibration and verify rotor positions match the SDF exactly

## Design Philosophy

This model prioritizes **simulation-to-real transfer** for RL research:

1. **Accurate Physics:** Motor dynamics, inertial properties, and sensor noise match real hardware
2. **Consistent Control:** Identical control allocation between simulation and reality
3. **Realistic Sensors:** IMU noise characteristics based on actual sensor datasheets
4. **Tunable Parameters:** All critical parameters exposed for easy modification

The goal is to minimize the reality gap, allowing RL policies trained in simulation to transfer effectively to physical hardware.

## Related Documentation

- PX4 Autopilot: https://docs.px4.io/
- Gazebo Sim: https://gazebosim.org/
- Motor parameter calculations: See `Hexacopter Motor Parameter Selection.pdf` in this directory

## License

This model configuration follows the same license as the parent repository.

