# (C) Copyright Axelera AI 2026
# All Rights Reserved
import os
import yaml
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import xacro

def generate_launch_description():

    ned2_description_pkg = get_package_share_directory('niryo_ned_description')
    ned2_moveit_pkg      = get_package_share_directory('niryo_ned2_moveit_config')

    # Speed parameters — override for real robot first tests:
    #   ros2 launch ... velocity:=0.05 acceleration:=0.05
    velocity    = float(os.environ.get('ARM_VELOCITY', '0.3'))
    acceleration = float(os.environ.get('ARM_ACCELERATION', '0.3'))
    target_class = os.environ.get('TARGET_CLASS', 'cat')

    xacro_file = os.path.join(
        ned2_moveit_pkg, 'config', 'niryo_ned2.urdf.xacro')
    robot_description_content = xacro.process_file(xacro_file).toxml()

    srdf_file = os.path.join(
        ned2_moveit_pkg, 'config', 'niryo_ned2.srdf')
    with open(srdf_file, 'r') as f:
        robot_description_semantic_content = f.read()

    kinematics_file = os.path.join(
        ned2_moveit_pkg, 'config', 'kinematics.yaml')
    with open(kinematics_file, 'r') as f:
        kinematics_config = yaml.safe_load(f)

    return LaunchDescription([

        # ── Static TF: camera → world ──────────────────────────────────────
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='camera_to_world_tf',
            arguments=[
                '0.275', '0.03', '0.42',
                '0.707', '0.707', '0.0', '0.0',
                'world', 'camera'
            ],
            output='screen',
        ),

        # ── Static TF: world → hand_camera_link (BRIO body frame) ──────────
        # BRIO mounted on robot's right side (Y = -0.70 m), level, rotated
        # +π/2 about world Z so the camera body +X axis points toward the
        # robot base (world -Y direction).
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='brio_body_tf',
            arguments=[
                '--x',     '0.165',
                '--y',    '-0.70',
                '--z',     '0.20',
                '--yaw',   '1.5708',
                '--pitch', '0',
                '--roll',  '0',
                '--frame-id',       'world',
                '--child-frame-id', 'hand_camera_link',
            ],
            output='screen',
        ),

        # ── Static TF: hand_camera_link → hand_camera (ROS optical frame) ──
        # REP-103 body→optical rotation (X right, Y down, Z forward).
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='brio_optical_tf',
            arguments=[
                '--x', '0', '--y', '0', '--z', '0',
                '--qx', '-0.5', '--qy', '0.5', '--qz', '-0.5', '--qw', '0.5',
                '--frame-id',       'hand_camera_link',
                '--child-frame-id', 'hand_camera',
            ],
            output='screen',
        ),

        # ── DetectionToTargetNode ──────────────────────────────────────────
        Node(
            package='metis_moveit2_perception',
            executable='detection_to_target_node_exe',
            name='detection_to_target',
            output='screen',
            parameters=[{
                'detections_topic':   '/detections',
                'target_class':       target_class,
                'table_height_m':     0.0,
                'hover_offset_m':     0.11,
                'camera_fx':          1360.0,
                'camera_fy':          1360.0,
                'camera_cx':          960.0,
                'camera_cy':          540.0,
                'assumed_depth_m':    0.42,
                'camera_frame':       'camera',
                'robot_base_frame':   'world',
                'max_reach_m':        0.38,
                'target_pose_topic':  '/target_hover_pose',
            }],
        ),

        # ── ArmControllerNode ──────────────────────────────────────────────
        Node(
            package='metis_moveit2_perception',
            executable='arm_controller_node_exe',
            name='arm_controller',
            output='screen',
            parameters=[
                {
                    'target_pose_topic':       '/target_hover_pose',
                    'planning_group':          'arm',
                    'end_effector_link':       'tool_link',
                    'planning_time_s':         5.0,
                    'planning_attempts':       10,
                    'goal_tolerance_m':        0.02,
                    'min_replan_interval_s':   3.0,
                    'velocity_scaling':        velocity,
                    'acceleration_scaling':    acceleration,
                    'robot_description':       robot_description_content,
                    'robot_description_semantic': robot_description_semantic_content,
                },
                {'robot_description_kinematics': kinematics_config},
            ],
        ),
    ])
