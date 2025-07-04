cmake_minimum_required(VERSION 3.8)
project(flyscan_perception)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(OpenCV REQUIRED)

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

# Add executable
add_executable(video_streamer src/video_streamer.cpp)

# Include directories
target_include_directories(video_streamer PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${ONNXRUNTIME_INCLUDE_DIR}
)

target_link_libraries(video_streamer 
  ${cv_bridge_LIBRARIES}
  ${ONNXRUNTIME_LIB}
)

# Link libraries
ament_target_dependencies(video_streamer
  rclcpp
  sensor_msgs
  cv_bridge
  OpenCV
)

# Install targets
install(TARGETS video_streamer
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
