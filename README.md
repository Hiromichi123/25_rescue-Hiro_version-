# 工训拾遗  

## src
core - 基于global坐标系的移动寻球版ros2主控代码
core_new - 基于局部坐标系的捞球ros2主控代码  
elrs_receiver - elrs接收机控制包  
point_detect - 基于mid360雷达的点云转换与聚类识别（获取全局坐标），未完成  
ros_tools - 包括雷达fastlio2定位消息的转换节点，对地摄像头，d435深度相机的图像流输出节点，以及对应自定义消息的接口功能包  
vel_control - 中位机控制包，对定点移动、局部速度移动转换成微速度并最终传达给底层的系统  
vision - opencv视觉节点，包括d435，摄像头的cv处理  
yolov8 - 基于yolo的视觉追踪  
