# src_new (mecanum base)

## Purpose
This directory provides an independent ROS2 mecanum chassis workflow without affecting the existing `src/` tree.

## What is kept
- ROS2 packages and resources useful for ground robotics workflows (`core_2026`, `messages`, `ros2_tools`, `cv_tools`, `vision_py`).
- Communication/message infrastructure for smart-car style control.

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
