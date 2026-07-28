cmake_minimum_required(VERSION 3.14)
project(metis_ros2_inference)

# C++17 — minimum for ROS2 Jazzy
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# ROS2 dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(vision_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(image_transport REQUIRED)

# OpenCV (system package: libopencv-dev)
find_package(OpenCV REQUIRED COMPONENTS core imgproc dnn)

# AxRuntime — discovered via pkg-config (installed by voyager-sdk install.sh)
# If the build fails here, ensure 'axruntime' is on PKG_CONFIG_PATH.
find_package(PkgConfig REQUIRED)
pkg_check_modules(axruntime REQUIRED axruntime)

find_library(AXRUNTIME_LIB
  NAMES axruntime
  HINTS "${axruntime_LIBRARY_DIRS}"
  REQUIRED)

message(STATUS "AxRuntime  lib     : ${AXRUNTIME_LIB}")

# ONNX Runtime — bundled with voyager-sdk.
#
# Resolution priority:
#   1. -DONNXRUNTIME_ROOT=<path> on the colcon command line (explicit override)
#   2. $ENV{ONNX_DIR}            (set by ../workspace_setup.sh after arch detection)
#   3. Glob $HOME/voyager-sdk/operators/onnxruntime/onnxruntime-linux-<arch>-*
#      and the sibling-of-this-repo layout used by workspace_setup.sh,
#      where <arch> is mapped from CMAKE_HOST_SYSTEM_PROCESSOR.
if(NOT ONNXRUNTIME_ROOT)
  if(CMAKE_HOST_SYSTEM_PROCESSOR MATCHES "^(x86_64|amd64)$")
    set(_onnx_arch_tag "x64")
  elseif(CMAKE_HOST_SYSTEM_PROCESSOR MATCHES "^(aarch64|arm64)$")
    set(_onnx_arch_tag "aarch64")
  else()
    message(WARNING
      "Unrecognized host architecture '${CMAKE_HOST_SYSTEM_PROCESSOR}'; "
      "defaulting ONNX arch tag to 'x64'.")
    set(_onnx_arch_tag "x64")
  endif()

  if(DEFINED ENV{ONNX_DIR} AND IS_DIRECTORY "$ENV{ONNX_DIR}")
    set(ONNXRUNTIME_ROOT "$ENV{ONNX_DIR}"
      CACHE PATH "Path to ONNX Runtime (headers + lib)")
    message(STATUS "ONNX Runtime: using ONNX_DIR from environment: ${ONNXRUNTIME_ROOT}")
  else()
    file(GLOB _onnx_candidates
      "$ENV{HOME}/voyager-sdk/operators/onnxruntime/onnxruntime-linux-${_onnx_arch_tag}-*"
      "${CMAKE_CURRENT_SOURCE_DIR}/../../../../voyager-sdk/operators/onnxruntime/onnxruntime-linux-${_onnx_arch_tag}-*"
    )
    list(SORT _onnx_candidates ORDER DESCENDING)
    list(LENGTH _onnx_candidates _n_onnx)
    if(_n_onnx GREATER 0)
      list(GET _onnx_candidates 0 _onnx_root)
      set(ONNXRUNTIME_ROOT "${_onnx_root}"
        CACHE PATH "Path to ONNX Runtime (headers + lib)")
      message(STATUS "ONNX Runtime: auto-detected for arch '${_onnx_arch_tag}': ${ONNXRUNTIME_ROOT}")
      unset(_onnx_root)
    else()
      message(FATAL_ERROR
        "ONNX Runtime not found for arch '${_onnx_arch_tag}'.\n"
        "Searched:\n"
        "  $ENV{HOME}/voyager-sdk/operators/onnxruntime/onnxruntime-linux-${_onnx_arch_tag}-*\n"
        "  ${CMAKE_CURRENT_SOURCE_DIR}/../../../../voyager-sdk/operators/onnxruntime/onnxruntime-linux-${_onnx_arch_tag}-*\n"
        "Resolutions:\n"
        "  - Source experimental.axros/workspace_setup.sh (exports ONNX_DIR), then re-run colcon build, or\n"
        "  - colcon build --cmake-args -DONNXRUNTIME_ROOT=/absolute/path/to/onnxruntime")
    endif()
    unset(_onnx_candidates)
    unset(_n_onnx)
  endif()
  unset(_onnx_arch_tag)
endif()

find_path(ONNXRUNTIME_INCLUDE_DIR
  NAMES onnxruntime_cxx_api.h
  HINTS "${ONNXRUNTIME_ROOT}/include"
  REQUIRED)

find_library(ONNXRUNTIME_LIB
  NAMES onnxruntime
  HINTS "${ONNXRUNTIME_ROOT}/lib"
  REQUIRED)

message(STATUS "AxRuntime  include : ${axruntime_INCLUDE_DIRS}")
message(STATUS "AxRuntime  lib     : ${axruntime_LIBRARIES}")
message(STATUS "ONNXRuntime include: ${ONNXRUNTIME_INCLUDE_DIR}")
message(STATUS "ONNXRuntime lib    : ${ONNXRUNTIME_LIB}")

# Composable node shared library
add_library(${PROJECT_NAME} SHARED
  src/metis_inference_node.cpp
)

target_include_directories(${PROJECT_NAME}
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<BUILD_INTERFACE:${axruntime_INCLUDE_DIRS}/include>
    $<INSTALL_INTERFACE:include>
  PRIVATE
    ${ONNXRUNTIME_INCLUDE_DIR}
)

target_link_directories(${PROJECT_NAME}
  PRIVATE
    ${axruntime_LIBRARY_DIRS}
    "${ONNXRUNTIME_ROOT}/lib"
)

target_link_libraries(${PROJECT_NAME}
  ${AXRUNTIME_LIB}
  ${ONNXRUNTIME_LIB}
  ${OpenCV_LIBS}
)

ament_target_dependencies(${PROJECT_NAME}
  rclcpp
  rclcpp_components
  sensor_msgs
  vision_msgs
  cv_bridge
  image_transport
)

# Register as a composable node component
rclcpp_components_register_node(${PROJECT_NAME}
  PLUGIN "metis_ros2_inference::MetisInferenceNode"
  EXECUTABLE metis_inference_node)

# Standalone executable (wraps the same node without a container)
add_executable(metis_inference_standalone
  src/metis_inference_node_main.cpp
)

target_link_libraries(metis_inference_standalone
  ${PROJECT_NAME}
)

ament_target_dependencies(metis_inference_standalone
  rclcpp
)

# ────────────────────────────────────────────────────────────────
# Install
# ────────────────────────────────────────────────────────────────
install(TARGETS
  ${PROJECT_NAME}
  metis_inference_standalone
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include
)

install(DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch
)

# ────────────────────────────────────────────────────────────────
# Tests
# ────────────────────────────────────────────────────────────────
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
