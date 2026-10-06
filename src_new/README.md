# src_new（Quadcopter 裁剪版：麦轮全向移动）

## 来源与目的
- 本目录代码来源：`Hiromichi123/Quadcopter` 仓库 `main` 分支快照。
- 本目录用途：在不影响目标仓库原有 `src/` 内容的前提下，提供一套独立的 ROS2 麦轮底盘版本。
- 已完成裁剪：移除/禁用四旋翼飞控、起降任务、PX4/姿态控制相关入口，保留对地移动有用的通信、消息与视觉基础设施。

## 主要保留目录
- `core_2026/`：主控包（保留小车任务节点，新增麦轮混控节点）
- `messages/`：平台/底盘与视觉相关消息、服务（仅保留 `TrackVelocity` action）
- `ros2_tools/`：里程计/相机相关基础节点（移除 `lidar_to_px4_bridge`）
- `cv_tools/`、`vision_py/`、`yolip/`、`vision_rs/`：视觉与工具链（未引入无人机控制语义）

## 已删减的无人机相关部分
- 删除 `core_rs/`（Rust 四旋翼控制）
- 删除 `core_2026` 中无人机主入口 launch（如 `core_launch.py`、`gazebo_launch.py`、`hover_*`）
- 删除 `ros2_tools/src/lidar_to_px4_bridge.cpp`
- 删除 `messages/action` 中 `Takeoff/Land/GoToTarget/ExecuteMission`

## 麦轮控制接口
新增节点：`core_2026::mecanum_controller_node`
- 订阅：`/cmd_vel` (`geometry_msgs/msg/Twist`)
  - `linear.x`：前后
  - `linear.y`：左右平移
  - `angular.z`：原地旋转
- 发布：
  - `/mecanum/wheel_rpm` (`std_msgs/msg/Float32MultiArray`)，顺序 `[FL, FR, RL, RR]`
  - `/smart_car/control_setpoint` (`messages/msg/SmartCarControlSetpoint`)

### 四轮混控关系（逆运动学）
设轮半径 `r`、半轴距 `L`、半轮距 `W`（本实现中通过 `wheelbase_m` 与 `track_width_m` 参数求和）：
- `FL = (vx - vy - (L+W)*wz) / r`
- `FR = (vx + vy + (L+W)*wz) / r`
- `RL = (vx + vy - (L+W)*wz) / r`
- `RR = (vx - vy + (L+W)*wz) / r`

节点参数：
- `wheel_radius_m`（默认 0.05）
- `wheelbase_m`（默认 0.18）
- `track_width_m`（默认 0.18）
- `max_wheel_rpm`（默认 300.0，输出限幅）

## 构建与运行（示例）
在工作区根目录执行：
```bash
colcon build --packages-select messages ros2_tools core_2026
source install/setup.bash
ros2 launch core_2026 mecanum_base.launch.py
```

如需带任务节点：
```bash
ros2 launch core_2026 car_mission.launch.py
```

## 仍需实机标定/未完成项
- `wheel_radius_m / wheelbase_m / track_width_m` 需按实车标定
- `max_wheel_rpm` 需根据电机与驱动器能力调整
- `SmartCarControlSetpoint` 到底层四轮驱动的映射依赖中位机/驱动实现，本目录仅提供 ROS2 侧混控与接口
