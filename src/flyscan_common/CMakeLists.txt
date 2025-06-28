cmake_minimum_required(VERSION 3.12)
project(flyscan_common)

# ────────────────────────────
# Compiler settings
# ────────────────────────────

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

# ────────────────────────────
# Find dependencies
# ────────────────────────────

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
# find_package(rclcpp_components REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(lifecycle_msgs REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(pluginlib REQUIRED)

# Custom interface packages
find_package(flyscan_interfaces REQUIRED)

# ────────────────────────────
# Common interface libraries
# ────────────────────────────

add_library(${PROJECT_NAME} INTERFACE)
target_include_directories(${PROJECT_NAME}
  INTERFACE
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)

# Propagate all ROS deps
target_link_libraries(${PROJECT_NAME} INTERFACE
  rclcpp
  rclcpp_components
  rclcpp_lifecycle
  rclcpp_action
  pluginlib
  lifecycle_msgs

  flyscan_interfaces
)

# ───────────────────────────
# Install public headers
# ───────────────────────────

install(DIRECTORY include/ DESTINATION include/)

# ───────────────────────────
# Finalize
# ───────────────────────────

ament_export_include_directories(include)
ament_export_dependencies(
  ament_cmake
  rclcpp 
  rclcpp_components
  rclcpp_lifecycle
  rclcpp_action
  lifecycle_msgs
  pluginlib

  flyscan_interfaces
  )

ament_package()