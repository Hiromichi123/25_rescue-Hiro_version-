# src_new（麦轮全向移动）

## 目录用途
- 本目录用于在不影响仓库原有 `src/` 内容的前提下，提供一套独立的 ROS2 麦轮底盘版本。
- 目录内容聚焦地面移动场景，保留通信、消息与视觉基础设施。

## 主要保留目录
- `core_2026/`：主控包（保留小车任务节点，新增麦轮混控节点）
- `messages/`：平台/底盘与视觉相关消息、服务（仅保留 `TrackVelocity` action）
- `ros2_tools/`：里程计/相机相关基础节点（移除 `lidar_to_px4_bridge`）
- `cv_tools/`、`vision_py/`：视觉与工具链

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
