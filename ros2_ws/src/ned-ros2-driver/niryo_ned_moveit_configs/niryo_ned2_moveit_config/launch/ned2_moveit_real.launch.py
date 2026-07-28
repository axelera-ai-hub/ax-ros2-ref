"""
Real-hardware MoveIt2 launch for Niryo NED2.

Replaces the simulation's ros2_control mock hardware with the ROSBridge
driver that connects to the real robot. The real NED2 provides:
  - /joint_states (via joint_state_broadcaster bridged from ROS1)
  - /niryo_robot_follow_joint_trajectory_controller/follow_joint_trajectory

Usage:
    ros2 launch niryo_ned2_moveit_config ned2_moveit_real.launch.py

Make sure drivers_list.yaml has the correct robot hostname/IP before launching.
"""

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory
from moveit_configs_utils import MoveItConfigsBuilder


def generate_launch_description():
    rviz_config_arg = DeclareLaunchArgument(
        "rviz_config",
        default_value="moveit.rviz",
        description="RViz configuration file",
    )
    # RViz is a CPU/GPU hog on the Pi 5 (~1 core just for the 3D scene).
    # Disable it with launch_rviz:=false when running headless or under
    # VNC where its preview is wasted bandwidth anyway.
    launch_rviz_arg = DeclareLaunchArgument(
        "launch_rviz",
        default_value="true",
        description="Start RViz alongside move_group. Set to 'false' on "
                    "the Pi to reclaim ~1 CPU core during pick-and-place.",
    )

    ned2_moveit_pkg = get_package_share_directory("niryo_ned2_moveit_config")

    # Use the original URDF (without ros2_control tags) for the real robot.
    # robot_state_publisher only needs the kinematic chain, not the
    # mock hardware plugin definition.
    urdf_file = os.path.join(ned2_moveit_pkg, "config", "niryo_ned2.urdf.xacro")

    moveit_config = (
        MoveItConfigsBuilder("niryo_ned2")
        .robot_description(file_path=urdf_file)
        .joint_limits(file_path="config/joint_limits.yaml")
        .robot_description_semantic(file_path="config/niryo_ned2.srdf")
        .robot_description_kinematics(file_path="config/kinematics.yaml")
        .trajectory_execution(file_path="config/moveit_controllers.yaml")
        .planning_pipelines(
            pipelines=["ompl", "chomp", "pilz_industrial_motion_planner", "stomp"]
        )
        .to_moveit_configs()
    )

    # ── ROSBridge driver (replaces ros2_control for real hardware) ─────────
    # This bridges /joint_states, /follow_joint_trajectory, and other
    # interfaces from the NED2's internal ROS1 stack to ROS2.
    # The whitelist excludes /tf to avoid conflicts with robot_state_publisher.
    driver_pkg = get_package_share_directory("niryo_ned_ros2_driver")
    whitelist_file = os.path.join(driver_pkg, "config", "whitelist.yaml")
    driver_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(driver_pkg, "launch", "driver.launch.py")),
        launch_arguments={"whitelist_params_file": whitelist_file}.items(),
    )

    # ── robot_state_publisher ─────────────────────────────────────────────
    # Subscribes to /joint_states (from ROSBridge driver) and publishes TF.
    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="screen",
        parameters=[moveit_config.robot_description],
    )

    # ── move_group ────────────────────────────────────────────────────────
    # MoveIt's SimpleControllerManager sends trajectories to the
    # niryo_robot_follow_joint_trajectory_controller action server,
    # which is bridged from the real robot by the ROSBridge driver.
    move_group_node = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        output="screen",
        parameters=[moveit_config.to_dict()],
        arguments=["--ros-args", "--log-level", "info"],
    )

    # ── RViz ──────────────────────────────────────────────────────────────
    rviz_base = LaunchConfiguration("rviz_config")
    rviz_config = PathJoinSubstitution(
        [FindPackageShare("niryo_ned2_moveit_config"), "config", rviz_base]
    )
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="log",
        arguments=["-d", rviz_config],
        parameters=[
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
            moveit_config.planning_pipelines,
            moveit_config.robot_description_kinematics,
            moveit_config.joint_limits,
        ],
        condition=IfCondition(LaunchConfiguration("launch_rviz")),
    )

    return LaunchDescription([
        rviz_config_arg,
        launch_rviz_arg,
        driver_launch,
        robot_state_publisher,
        move_group_node,
        rviz_node,
    ])
