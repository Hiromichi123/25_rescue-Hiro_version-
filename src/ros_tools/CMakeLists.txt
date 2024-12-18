cmake_minimum_required(VERSION 3.8)
project(ros_tools)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(OpenCV REQUIRED)
find_package(cv_bridge REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME} "msg/LidarPose.msg" "msg/BallPos.msg" "msg/BallPoss.msg")
ament_export_dependencies(rosidl_default_runtime)

add_executable(lidar_data_node src/lidar_data_node.cpp)
add_executable(ground_camera_node src/ground_camera_node.cpp)
add_executable(front_camera_node src/front_camera_node.cpp)
# add_executable(d435_node src/d435_node.cpp)

target_include_directories(lidar_data_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

# target_include_directories(d435_node PRIVATE ${realsense2_INCLUDE_DIRS})

rosidl_get_typesupport_target(cpp_typesupport_target ${PROJECT_NAME} "rosidl_typesupport_cpp")
target_link_libraries(lidar_data_node ${cpp_typesupport_target})

ament_target_dependencies(lidar_data_node rclcpp geometry_msgs tf2 tf2_geometry_msgs nav_msgs)
ament_target_dependencies(ground_camera_node rclcpp sensor_msgs OpenCV cv_bridge)
ament_target_dependencies(front_camera_node rclcpp sensor_msgs OpenCV cv_bridge)
# ament_target_dependencies(d435_node rclcpp sensor_msgs OpenCV cv_bridge realsense2)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

install(TARGETS lidar_data_node DESTINATION lib/${PROJECT_NAME})
install(TARGETS ground_camera_node DESTINATION lib/${PROJECT_NAME})
install(TARGETS front_camera_node DESTINATION lib/${PROJECT_NAME})
install(
  DIRECTORY msg
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
