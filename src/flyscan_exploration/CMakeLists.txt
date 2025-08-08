cmake_minimum_required(VERSION 3.12)
project(flyscan_exploration)

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

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(OpenCV REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(grid_map_ros REQUIRED)
find_package(grid_map_core REQUIRED)
find_package(grid_map_cv REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(yaml-cpp REQUIRED)
find_package(ament_index_cpp REQUIRED)

find_package(flyscan_common REQUIRED)
find_package(flyscan_core REQUIRED)
find_package(flyscan_interfaces REQUIRED)

# ───────────────────────────
# Build Nodes
# ───────────────────────────

set(NODE_TARGETS)

set(dependencies
  rclcpp
  rclcpp_lifecycle
  sensor_msgs
  geometry_msgs
  nav_msgs
  std_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
  OpenCV
  cv_bridge
  grid_map_ros
  grid_map_core
  grid_map_cv
  visualization_msgs
  yaml-cpp
  ament_index_cpp
)

set(libraries
  flyscan_core::base_node
  ${OpenCV_LIBRARIES}
  yaml-cpp
)

# Build frontier_explorer executable
add_executable(frontier_explorer src/frontier_explorer.cpp)
target_include_directories(frontier_explorer
  PRIVATE
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
)
target_link_libraries(frontier_explorer ${libraries})
ament_target_dependencies(frontier_explorer ${dependencies})
list(APPEND NODE_TARGETS frontier_explorer)

# Build waypoint_mission executable
add_executable(waypoint_mission src/waypoint_mission.cpp)
target_include_directories(waypoint_mission
  PRIVATE
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
)
target_link_libraries(waypoint_mission ${libraries})
ament_target_dependencies(waypoint_mission ${dependencies})
list(APPEND NODE_TARGETS waypoint_mission)

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