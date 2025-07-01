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
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(MAVSDK REQUIRED)

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

set(common_packages
  rclcpp
  std_msgs
  std_srvs
  geometry_msgs
  sensor_msgs
)

add_target(px4_controller_lib   library     src/px4_controller.cpp 
  "${common_packages}" MAVSDK::mavsdk
)

add_target(px4_control executable src/main.cpp
    ""  px4_controller_lib
)

add_target(controller_node executable src/controller_node.cpp
  "${common_packages}"  px4_controller_lib
)

# ───────────────────────────
# Install Rules
# ───────────────────────────

install(TARGETS ${NODE_TARGETS}
  EXPORT ${PROJECT_NAME}-targets
  ARCHIVE DESTINATION lib/${PROJECT_NAME}
  LIBRARY DESTINATION lib/${PROJECT_NAME}
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)
install(DIRECTORY include/ DESTINATION include/)

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