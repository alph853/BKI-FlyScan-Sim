cmake_minimum_required(VERSION 3.12)
project(flyscan_drone_controller)

# ───────────────────────────
# Compiler Settings
# ───────────────────────────

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_EXTENSIONS OFF)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

if (CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
  add_compile_options(
    -Wall -Wextra -Wpedantic
    -fmacro-prefix-map=${CMAKE_CURRENT_SOURCE_DIR}=${PROJECT_NAME}
    -fdebug-prefix-map=${CMAKE_CURRENT_SOURCE_DIR}=${PROJECT_NAME}
  )
endif()

# ───────────────────────────
# Find Dependencies
# ───────────────────────────

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(px4_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(eigen3_cmake_module REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(pcl_ros REQUIRED)

find_package(flyscan_common REQUIRED)
find_package(flyscan_core REQUIRED)
find_package(flyscan_interfaces REQUIRED)

# ───────────────────────────
# Build Nodes
# ───────────────────────────

set(NODE_TARGETS)

set(common_packages
  rclcpp
  std_msgs
  std_srvs
  geometry_msgs
  sensor_msgs
  px4_msgs
  tf2

  flyscan_common
  flyscan_interfaces
)

# Build px4_controller executable
add_executable(px4_controller src/px4_controller.cpp)
target_include_directories(px4_controller
  PRIVATE
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
)
ament_target_dependencies(px4_controller 
  ${common_packages} 
  tf2_geometry_msgs 
  pcl_conversions 
  pcl_ros
)
target_link_libraries(px4_controller flyscan_core::base_node)
list(APPEND NODE_TARGETS px4_controller)

# Build px4_controller_component library
add_library(px4_controller_component SHARED src/px4_controller.cpp)
target_include_directories(px4_controller_component
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(px4_controller_component 
  ${common_packages} 
  tf2_geometry_msgs 
  pcl_conversions 
  pcl_ros
)
target_link_libraries(px4_controller_component flyscan_core::base_node)
list(APPEND NODE_TARGETS px4_controller_component)

# Build px4_ros_bridge executable
add_executable(px4_ros_bridge src/px4_ros_bridge.cpp)
target_include_directories(px4_ros_bridge
  PRIVATE
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
)
ament_target_dependencies(px4_ros_bridge ${common_packages} Eigen3 eigen3_cmake_module tf2_ros)
target_link_libraries(px4_ros_bridge flyscan_core::base_node)
list(APPEND NODE_TARGETS px4_ros_bridge)

rclcpp_components_register_nodes(px4_controller_component "flyscan::drone_controller::PX4Controller")

# Build teleop_node executable
add_executable(teleop_node src/teleop_node.cpp)
target_include_directories(teleop_node
  PRIVATE
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
)
ament_target_dependencies(teleop_node ${common_packages})
list(APPEND NODE_TARGETS teleop_node)

# ───────────────────────────
# Install Rules
# ───────────────────────────

install(TARGETS ${NODE_TARGETS}
  EXPORT ${PROJECT_NAME}-targets
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)
install(DIRECTORY include/ DESTINATION include/)

install(DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch
)
# ───────────────────────────
# Testing & Linting
# ───────────────────────────

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

# ───────────────────────────
# Finalize
# ───────────────────────────

ament_package()