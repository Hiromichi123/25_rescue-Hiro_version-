cmake_minimum_required(VERSION 3.8)
project(core_new)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(ros_tools REQUIRED)
find_package(std_msgs REQUIRED)
find_package(core REQUIRED)

add_executable(main_node src/main.cpp src/classes/car.cpp src/classes/ball.cpp)

target_include_directories(main_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${CMAKE_CURRENT_SOURCE_DIR}/src/classes
)

ament_target_dependencies(main_node rclcpp geometry_msgs nav_msgs ros_tools std_msgs core)
set(CMAKE_INSTALL_RPATH /usr/local/lib)


if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

install(TARGETS main_node DESTINATION lib/${PROJECT_NAME})

ament_package()
