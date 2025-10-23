hexacopter_description

Overview
- Custom Gazebo (gz) hexacopter model and matching PX4 airframe tuned for RL research and sim→real consistency.
- Hardware intent: 4S (14.8 V), 11×5.7 props, 1250 KV motors; model inertias and link poses mirror the real platform.

Directory
- `custom_hexa/model.sdf` – Vehicle SDF: links/joints, sensors, and motor model plugins (MulticopterMotorModel).
- `custom_hexa_world.sdf` – Optional world file for gz.
- `airframe/4022_gz_custom_hexa` – PX4 airframe (Control Allocation hex) matching the SDF geometry and motor model.
- `launch_sitl.sh` – Convenience launcher for PX4 SITL with gz and this model.
- `verify_bridge.sh` – Quick checks that gz topics and motor command bridge are alive.
- `MOTOR_PARAMETERS_SUMMARY.md` – Notes and calculations for motor/prop parameters.

Coordinate Frames (important)
- SDF uses FLU: +x forward, +y left, +z up.
- PX4 Control Allocation uses FRD: +x forward, +y right, +z down.
- Conversions when setting rotor geometry: `FRD.x = SDF.x`, `FRD.y = −SDF.y`, `FRD.z = −SDF.z`.

Model Parameters (SDF)
- Motor dynamics (all six motors share the same values):
  - `maxRotVelocity = 820.0` rad/s (current‑limited)
  - `motorConstant (kF) = 2.11e-05` N/(rad/s)^2
  - `momentConstant (Q/T) = 0.0168` m
  - `timeConstantUp = timeConstantDown = 0.015`
  - `turningDirection`: cw on rotors 0,3,5; ccw on 1,2,4
  - `actuator_number`: 0..5 in rotor_0..5 order
- Rotor positions: set in `custom_hexa/model.sdf` under each `link name='rotor_i'` `<pose>`; radius and angles match the real frame.

Airframe Parameters (PX4)
- Control Allocation airframe with explicit rotor geometry:
  - `CA_ROTOR_COUNT = 6`
  - Rotor positions in FRD match the SDF poses (with y sign flipped).
  - Yaw moment coefficients: `CA_ROTORx_KM = ±0.0168` (CCW +, CW −) to match SDF `momentConstant`.
- Gazebo bridge mapping and limits:
  - `SIM_GZ_EC_FUNC1..6 = 101..106` (Motor1..Motor6) → 1:1 with SDF `actuator_number` 0..5.
  - `SIM_GZ_EC_MIN1..6 = 150` rad/s, `SIM_GZ_EC_MAX1..6 = 820` rad/s.
- Starter gains (safe, conservative) for SITL; expect to retune on hardware: `MC_*` rate P/I/D, `MC_*` attitude P, `MPC_THR_HOVER`.
- SITL conveniences set in this airframe: `SIM_GZ_EN=1`, `EKF2_HGT_REF=1`, `COM_ARM_WO_GPS=1`.

Quick Start (SITL)
1) Make the model discoverable by gz. From this directory:
   - `export GZ_SIM_RESOURCE_PATH=$GZ_SIM_RESOURCE_PATH:$(pwd)`
2) Ensure the airframe file is available to PX4 ROMFS (once per workspace):
   - Copy or symlink `airframe/4022_gz_custom_hexa` to `PX4-Autopilot/ROMFS/px4fmu_common/init.d-posix/airframes/4022_gz_custom_hexa`.
3) Launch PX4 SITL with gz:
   - In `PX4-Autopilot`: `./path/to/hexacopter_description/launch_sitl.sh`
   - Or directly: `make px4_sitl gz_custom_hexa`
4) Verify the bridge after the sim is running:
   - `./path/to/hexacopter_description/verify_bridge.sh`
   - Should list `/custom_hexa/command/motor_speed` and `/custom_hexa/imu` topics and print one sample message from each.

Notes
- This setup aims for consistent physics/allocation between gz and PX4, minimizing sim→real gaps for RL experiments.

