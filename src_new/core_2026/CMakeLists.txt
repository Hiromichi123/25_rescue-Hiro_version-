cmake_minimum_required(VERSION 3.8)
project(core_2026)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(messages REQUIRED)

add_executable(car_mission_node src/layer3_mission/car_mission_executor.cpp)
add_executable(mecanum_controller_node src/layer3_mission/mecanum_controller.cpp)
add_executable(test_max_curvature src/layer3_mission/test_max_curvature.cpp)

foreach(target car_mission_node mecanum_controller_node test_max_curvature)
  target_include_directories(${target} PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/src>
    $<INSTALL_INTERFACE:include>
  )
endforeach()

ament_target_dependencies(car_mission_node
  rclcpp
  geometry_msgs
  nav_msgs
  std_msgs
  messages
)

ament_target_dependencies(mecanum_controller_node
  rclcpp
  geometry_msgs
  std_msgs
  messages
)

ament_target_dependencies(test_max_curvature
  rclcpp
  geometry_msgs
  nav_msgs
  std_msgs
  messages
)

set(CMAKE_INSTALL_RPATH /usr/local/lib)

install(TARGETS
  car_mission_node
  mecanum_controller_node
  test_max_curvature
  DESTINATION lib/${PROJECT_NAME}
)
install(DIRECTORY launch/ DESTINATION share/${PROJECT_NAME}/)

ament_package()
