cmake_minimum_required(VERSION 3.8)
project(flyscan_perception)

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
find_package(rclcpp_lifecycle REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(OpenCV REQUIRED)

find_package(flyscan_core REQUIRED)
find_package(flyscan_common REQUIRED)

# Find ONNX Runtime
find_path(ONNXRUNTIME_INCLUDE_DIR 
  NAMES onnxruntime_cxx_api.h
  PATHS /usr/include/onnxruntime /usr/local/include/onnxruntime
)

find_library(ONNXRUNTIME_LIB
  NAMES onnxruntime
  PATHS /usr/lib /usr/local/lib /usr/lib/x86_64-linux-gnu
)

if(ONNXRUNTIME_INCLUDE_DIR AND ONNXRUNTIME_LIB)
  message(STATUS "Found ONNX Runtime: ${ONNXRUNTIME_LIB}")
  message(STATUS "ONNX Runtime include: ${ONNXRUNTIME_INCLUDE_DIR}")
else()
  message(WARNING "ONNX Runtime not found. Please install ONNX Runtime.")
  message(WARNING "You can install it with: sudo apt-get install libonnxruntime-dev")
  message(WARNING "Or download from: https://github.com/microsoft/onnxruntime/releases")
endif()

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

set(semantic_perception_packages
  rclcpp
  rclcpp_lifecycle
  sensor_msgs
  cv_bridge
  OpenCV

  flyscan_common
)

add_target(semantic_perception executable
  "src/semantic_perception.cpp"
  "${semantic_perception_packages}"
  "flyscan_core::base_node;${cv_bridge_LIBRARIES};${ONNXRUNTIME_LIB}"
)

# Include directories for semantic_perception
# target_include_directories(semantic_perception PUBLIC
#   $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
#   $<INSTALL_INTERFACE:include>
#   ${ONNXRUNTIME_INCLUDE_DIR}
# )

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

# ───────────────────────────
# Testing & Linting
# ───────────────────────────

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TR
UE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

# ───────────────────────────
# Finalize
# ───────────────────────────

ament_package()