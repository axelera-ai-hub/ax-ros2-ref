import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, RegisterEventHandler
from launch.event_handlers import OnProcessStart
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory
from moveit_configs_utils import MoveItConfigsBuilder

from rclpy import logging

logger = logging.get_logger("ned2_moveit.launch")

def generate_launch_description():
    rviz_config_arg = DeclareLaunchArgument(
        "rviz_config",
        default_value="moveit.rviz",
        description="RViz configuration file",
    )

    ned2_moveit_pkg = get_package_share_directory("niryo_ned2_moveit_config")

    # Use the MoveIt config URDF which includes ros2_control tags
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

    # ── robot_state_publisher ─────────────────────────────────────────────
    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="screen",
        parameters=[moveit_config.robot_description],
    )

    # ── ros2_control (mock hardware) ──────────────────────────────────────
    ros2_controllers_file = os.path.join(
        ned2_moveit_pkg, "config", "ros2_controllers.yaml")

    ros2_control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        output="screen",
        parameters=[
            moveit_config.robot_description,
            ros2_controllers_file,
        ],
    )

    # Spawn controllers after ros2_control_node starts
    spawn_jsb = ExecuteProcess(
        cmd=["ros2", "run", "controller_manager", "spawner",
             "joint_state_broadcaster", "--controller-manager", "/controller_manager"],
        output="screen",
    )
    spawn_jtc = ExecuteProcess(
        cmd=["ros2", "run", "controller_manager", "spawner",
             "niryo_robot_follow_joint_trajectory_controller",
             "--controller-manager", "/controller_manager"],
        output="screen",
    )

    # ── move_group ────────────────────────────────────────────────────────
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
    )

    return LaunchDescription([
        rviz_config_arg,
        robot_state_publisher,
        ros2_control_node,
        # Spawn controllers after ros2_control_node starts
        RegisterEventHandler(
            OnProcessStart(
                target_action=ros2_control_node,
                on_start=[spawn_jsb, spawn_jtc],
            )
        ),
        move_group_node,
        rviz_node,
    ])
