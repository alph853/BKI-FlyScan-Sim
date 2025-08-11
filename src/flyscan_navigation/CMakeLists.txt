cmake_minimum_required(VERSION 3.12)
project(flyscan_navigation)

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

set(NODE_TARGETS)

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
)

set(libraries
  flyscan_core::base_node
)

# NavigatorNode
add_executable(navigator_node src/navigator_node.cpp src/navigator.cpp src/path_planner.cpp src/collision_checker.cpp)
target_include_directories(navigator_node
  PRIVATE
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
)
target_link_libraries(navigator_node ${libraries})
ament_target_dependencies(navigator_node ${dependencies})
list(APPEND NODE_TARGETS navigator_node)

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
install(DIRECTORY config/
  DESTINATION share/${PROJECT_NAME}/config
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