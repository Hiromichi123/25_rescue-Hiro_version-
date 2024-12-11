cmake_minimum_required(VERSION 3.8)
project(point_detect)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(livox_ros_driver2 REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(pcl_ros REQUIRED)
find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/LidarBallPosition.msg"
  DEPENDENCIES std_msgs
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

add_executable(livox_to_pointcloud2 src/livox_to_pointcloud2.cpp)
add_executable(point_detect_node src/point_detect_node.cpp)

ament_target_dependencies(livox_to_pointcloud2 
    rclcpp
    sensor_msgs
    pcl_conversions
    pcl_ros
    livox_ros_driver2
)

ament_target_dependencies(point_detect_node 
    rclcpp
    std_msgs
    sensor_msgs
    geometry_msgs
    pcl_conversions
    pcl_ros)

rosidl_target_interfaces(point_detect_node
  ${PROJECT_NAME} "rosidl_typesupport_cpp"
)

install(DIRECTORY msg
  DESTINATION share/${PROJECT_NAME}/
)

install(TARGETS
  livox_to_pointcloud2
  point_detect_node
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
