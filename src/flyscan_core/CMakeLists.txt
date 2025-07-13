cmake_minimum_required(VERSION 3.12)
project(flyscan_core)

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
find_package(rclcpp_components REQUIRED)
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

add_target(life_monitor   executable  src/life_monitor.cpp  flyscan_common  "")
add_target(base_node      library     src/base_node.cpp     flyscan_common  "")

add_target(life_monitor_component library
  src/life_monitor.cpp
  flyscan_common
  ""
)
rclcpp_components_register_nodes(life_monitor_component "flyscan::core::LifeMonitor")

# ───────────────────────────
# Install Rules
# ───────────────────────────

install(TARGETS ${NODE_TARGETS}
  EXPORT ${PROJECT_NAME}-targets
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)
install(EXPORT ${PROJECT_NAME}-targets
  DESTINATION share/${PROJECT_NAME}/cmake
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

ament_export_include_directories(include)
ament_export_targets(${PROJECT_NAME}-targets HAS_LIBRARY_TARGET)
ament_package()
