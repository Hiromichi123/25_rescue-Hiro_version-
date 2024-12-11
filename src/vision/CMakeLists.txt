cmake_minimum_required(VERSION 3.8)
project(vision)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(core REQUIRED)
find_package(ros_tools REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(livox_ros_driver2 REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(pcl_ros REQUIRED)
find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME} "msg/LidarBallPosition.msg")
ament_export_dependencies(rosidl_default_runtime)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

add_executable(livox_to_pointcloud2 src/livox_to_pointcloud2.cpp)
add_executable(point_detect_node src/point_detect_node.cpp)
add_executable(calibrate_filter src/calibrate_filter.cpp)

ament_target_dependencies(livox_to_pointcloud2 rclcpp sensor_msgs livox_ros_driver2)
ament_target_dependencies(point_detect_node rclcpp std_msgs sensor_msgs geometry_msgs pcl_conversions pcl_ros ros_tools)
ament_target_dependencies(calibrate_filter rclcpp std_msgs core)

rosidl_get_typesupport_target(cpp_typesupport_target ${PROJECT_NAME} "rosidl_typesupport_cpp")
target_link_libraries(point_detect_node ${cpp_typesupport_target})

install(DIRECTORY msg DESTINATION share/${PROJECT_NAME})
install(PROGRAMS
    scripts/ball_calibration.py
    scripts/ball_depth.py
  DESTINATION lib/${PROJECT_NAME}
)
install(TARGETS 
    livox_to_pointcloud2 
    point_detect_node
    calibrate_filter 
    DESTINATION 
  lib/${PROJECT_NAME})

ament_package()