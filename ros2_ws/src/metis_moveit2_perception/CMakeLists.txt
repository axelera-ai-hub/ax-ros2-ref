cmake_minimum_required(VERSION 3.12)
project(metis_moveit2_perception VERSION 0.1.0)
set(CMAKE_CXX_STANDARD 20)
set(CMAKE_POSITION_INDEPENDENT_CODE ON)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(vision_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(niryo_ned_ros2_interfaces REQUIRED)

# -- DetectionToTargetNode ----------------------------------------------------
add_library(detection_to_target_node SHARED
  src/detection_to_target_node.cpp)
target_include_directories(detection_to_target_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
ament_target_dependencies(detection_to_target_node
  rclcpp rclcpp_components geometry_msgs vision_msgs
  tf2_ros tf2_geometry_msgs)
rclcpp_components_register_node(detection_to_target_node
  PLUGIN "metis_moveit2_perception::DetectionToTargetNode"
  EXECUTABLE detection_to_target_node_exe)

# -- ArmControllerNode (standalone executable — MoveIt2 Jazzy pattern) -------
add_executable(arm_controller_node_exe
  src/arm_controller_node.cpp)
target_include_directories(arm_controller_node_exe PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
ament_target_dependencies(arm_controller_node_exe
  rclcpp geometry_msgs moveit_ros_planning_interface)

install(TARGETS
  detection_to_target_node detection_to_target_node_exe
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME})
# -- PickAndPlaceNode (standalone executable) ---------------------------------
add_executable(pick_and_place_node_exe
  src/pick_and_place_node.cpp)
target_include_directories(pick_and_place_node_exe PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
ament_target_dependencies(pick_and_place_node_exe
  rclcpp rclcpp_action geometry_msgs vision_msgs
  tf2_ros tf2_geometry_msgs
  moveit_ros_planning_interface niryo_ned_ros2_interfaces)

install(TARGETS arm_controller_node_exe pick_and_place_node_exe
  DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY include/ DESTINATION include)
install(DIRECTORY launch/  DESTINATION share/${PROJECT_NAME}/launch)
install(PROGRAMS
  scripts/test_safe_motion.sh
  scripts/yellow_detector_node.py
  scripts/red_detector_node.py
  DESTINATION lib/${PROJECT_NAME})
ament_package()
