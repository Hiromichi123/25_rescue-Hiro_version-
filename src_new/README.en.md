# src_new (Quadcopter snapshot, reduced to mecanum base)

## Origin
This directory is copied from `Hiromichi123/Quadcopter` `main` branch and then reduced for a ground mecanum chassis workflow.

## What is kept
- ROS2 packages and resources useful for ground robotics workflows (`core_2026`, `messages`, `ros2_tools`, `cv_tools`, `vision_py`, `vision_rs`, `yolip`).
- Communication/message infrastructure for smart-car style control.

## What was removed/disabled
- Quadcopter flight-control and PX4 entrypoints.
- Drone-only actions (`Takeoff`, `Land`, `GoToTarget`, `ExecuteMission`).
- `core_rs` and PX4 bridge node (`lidar_to_px4_bridge`).

## Mecanum interface
`core_2026` adds `mecanum_controller_node`:
- Subscribes: `/cmd_vel` (`geometry_msgs/msg/Twist`)
- Publishes: `/mecanum/wheel_rpm` (`std_msgs/msg/Float32MultiArray`, order `[FL, FR, RL, RR]`)
- Publishes: `/smart_car/control_setpoint` (`messages/msg/SmartCarControlSetpoint`)

Kinematics mapping:
- `FL = (vx - vy - (L+W)*wz) / r`
- `FR = (vx + vy + (L+W)*wz) / r`
- `RL = (vx + vy - (L+W)*wz) / r`
- `RR = (vx - vy + (L+W)*wz) / r`

## Build and run
```bash
colcon build --packages-select messages ros2_tools core_2026
source install/setup.bash
ros2 launch core_2026 mecanum_base.launch.py
```

## Remaining calibration work
- Calibrate `wheel_radius_m`, `wheelbase_m`, `track_width_m` on real hardware.
- Tune `max_wheel_rpm` per motor/driver limits.
