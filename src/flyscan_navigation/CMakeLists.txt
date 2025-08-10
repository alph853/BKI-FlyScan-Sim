cmake_minimum_required(VERSION 3.8)
project(flyscan_navigation)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(flyscan_interfaces REQUIRED)
find_package(flyscan_common REQUIRED)
find_package(flyscan_core REQUIRED)

set(dependencies
  rclcpp
  rclcpp_action
  rclcpp_lifecycle
  std_msgs
  geometry_msgs
  nav_msgs
  sensor_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
  flyscan_interfaces
  flyscan_common
  flyscan_core
)

include_directories(include)

# NavigatorNode
add_executable(navigator_node src/navigator_node.cpp src/navigator.cpp src/path_planner.cpp src/collision_checker.cpp)
ament_target_dependencies(navigator_node ${dependencies})

# Install targets
install(TARGETS
  navigator_node
  DESTINATION lib/${PROJECT_NAME}
)

# Install directories
install(DIRECTORY
  include/
  DESTINATION include/
)

install(DIRECTORY
  launch/
  DESTINATION share/${PROJECT_NAME}/launch
)

install(DIRECTORY
  config/
  DESTINATION share/${PROJECT_NAME}/config
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()