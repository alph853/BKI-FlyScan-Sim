cmake_minimum_required(VERSION 3.8)
project(flyscan_interfaces)

# ───────────────────────────
# Compiler settings
# ───────────────────────────

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# ───────────────────────────
# Find dependencies
# ───────────────────────────

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)

find_package(std_msgs        REQUIRED)
find_package(geometry_msgs   REQUIRED)
find_package(nav_msgs        REQUIRED)
find_package(sensor_msgs     REQUIRED)
find_package(builtin_interfaces REQUIRED)

#
# rosidl_generate_interfaces already handles dependencies in the DEPENDENCIES section
#

# ───────────────────────────
# Generate interfaces
# ───────────────────────────

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/NodeInfo.msg"
  "msg/NodeHeartbeat.msg"

  "msg/OccupancyMap.msg"
  "msg/SemanticMap.msg"
  "msg/ExplorationMap.msg"
  "msg/Hypermap.msg"
  "msg/Odometry.msg"
  "msg/TeleopCommand.msg"

  "srv/GetRegisteredNodes.srv"
  "srv/RegisterNode.srv"
  "srv/UnregisterNode.srv"
  "srv/RequestRecovery.srv"

  "srv/QueryMap.srv"
  "srv/SetControlMode.srv"

  DEPENDENCIES std_msgs geometry_msgs nav_msgs sensor_msgs builtin_interfaces
)


# ───────────────────────────
# Finalize
# ───────────────────────────

ament_export_dependencies(rosidl_default_runtime)
ament_package()
