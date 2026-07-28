cmake_minimum_required(VERSION 3.14)
project(axros_helper_nodes)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(image_transport REQUIRED)
find_package(message_filters REQUIRED)
find_package(vision_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(OpenCV REQUIRED COMPONENTS core imgproc imgcodecs videoio highgui)

# ── VideoPublisherNode shared library (composable node) ──────────────────────
add_library(video_publisher_node SHARED
  src/video_publisher_node.cpp
)

target_include_directories(video_publisher_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

ament_target_dependencies(video_publisher_node
  rclcpp
  rclcpp_components
  sensor_msgs
  cv_bridge
  image_transport
)

target_link_libraries(video_publisher_node
  ${OpenCV_LIBS}
)

rclcpp_components_register_node(video_publisher_node
  PLUGIN "axros_helper_nodes::VideoPublisherNode"
  EXECUTABLE video_publisher_node_exe
)

# ── SaveAnnotatedNode shared library (composable node) ──────────────────────
add_library(save_annotated_node SHARED
  src/save_annotated_node.cpp
)

target_include_directories(save_annotated_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

ament_target_dependencies(save_annotated_node
  rclcpp
  rclcpp_components
  sensor_msgs
  vision_msgs
  cv_bridge
  image_transport
  message_filters
)

target_link_libraries(save_annotated_node
  ${OpenCV_LIBS}
)

rclcpp_components_register_node(save_annotated_node
  PLUGIN "axros_helper_nodes::SaveAnnotatedNode"
  EXECUTABLE save_annotated_node_exe
)

# ── LiveInferenceNode shared library (composable node) ──────────────────────
add_library(live_inference_node SHARED
  src/live_inference_node.cpp
)

target_include_directories(live_inference_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

ament_target_dependencies(live_inference_node
  rclcpp
  rclcpp_components
  sensor_msgs
  vision_msgs
  geometry_msgs
  cv_bridge
  image_transport
  message_filters
)

target_link_libraries(live_inference_node
  ${OpenCV_LIBS}
)

rclcpp_components_register_node(live_inference_node
  PLUGIN "axros_helper_nodes::LiveInferenceNode"
  EXECUTABLE live_inference_node_exe
)

# ── Install ──────────────────────────────────────────────────────────────────
install(TARGETS video_publisher_node
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(TARGETS video_publisher_node_exe
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(TARGETS save_annotated_node
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(TARGETS save_annotated_node_exe
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(TARGETS live_inference_node
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(TARGETS live_inference_node_exe
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include
)

install(DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch
)

# ── Python scripts ──────────────────────────────────────────────────────────
install(PROGRAMS
  scripts/demo_screen_node.py
  DESTINATION lib/${PROJECT_NAME}
)

# ── Web assets for demo_screen_node ─────────────────────────────────────────
install(DIRECTORY resource/
  DESTINATION share/${PROJECT_NAME}/resource
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
