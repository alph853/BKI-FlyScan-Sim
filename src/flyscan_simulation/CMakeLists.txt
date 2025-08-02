cmake_minimum_required(VERSION 3.8)
project(flyscan_simulation)

# ───────────────────────────
# Compiler Settings
# ───────────────────────────

if (CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
  add_compile_options(
    -Wall -Wextra -Wpedantic
  )
endif()

# ───────────────────────────
# Find Dependencies
# ───────────────────────────

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
# find_package(rclcpp_components REQUIRED)
# find_package(flyscan_drone_controller REQUIRED)
# find_package(ros_gz REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)

find_package(cv_bridge REQUIRED)
find_package(OpenCV REQUIRED)

find_package(flyscan_common REQUIRED)

# ───────────────────────────
# Build Nodes
# ───────────────────────────

set(NODE_TARGETS)

macro(add_target name type source deps libs)
  if(${type} STREQUAL "executable")
    add_executable(${name} ${source})
    target_include_directories(${name}
      PRIVATE
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    )
  elseif(${type} STREQUAL "library")
    add_library(${name} SHARED ${source})
    target_include_directories(${name}
      PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:include>
    )
  endif()

  target_link_libraries(${name} ${libs})

  ament_target_dependencies(${name} ${deps})

  list(APPEND NODE_TARGETS ${name})
endmacro()

set(visualization_node_packages
  rclcpp
  visualization_msgs
  geometry_msgs
  nav_msgs
  std_msgs
  tf2
)

add_target(visualization_node executable
  "src/visualization_node.cpp"
  "${visualization_node_packages}"
  ""
)

set(video_streamer_packages
  rclcpp
  sensor_msgs
  cv_bridge
  OpenCV
  flyscan_common
)

add_target(video_streamer executable
  "src/video_streamer.cpp"
  "${video_streamer_packages}"
  "${cv_bridge_LIBRARIES}"
)

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
install(DIRECTORY worlds/
  DESTINATION share/${PROJECT_NAME}/worlds
)
install(DIRECTORY scripts/
  DESTINATION share/${PROJECT_NAME}/${PROJECT_NAME}
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