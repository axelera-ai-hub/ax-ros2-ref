# (C) Copyright Axelera AI 2026
# All Rights Reserved
"""Standalone launch for the BRIO pose inference node.

Brings up:
  - Static TFs world → hand_camera_link → hand_camera (mirrors perception_arm.launch.py)
  - robot_state_publisher + joint_state_publisher so rviz can render the NED2
  - PoseInferenceNode consuming /dev/video2 via voyager-sdk

Run separately from perception_arm.launch.py while validating; integrate later
if everything checks out.
"""

import os

import xacro
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    voyager_sdk_path = os.environ.get(
        'VOYAGER_SDK_DIR', os.path.expanduser('~/voyager-sdk'))

    ned2_moveit_pkg = get_package_share_directory('niryo_ned2_moveit_config')
    xacro_file = os.path.join(ned2_moveit_pkg, 'config', 'niryo_ned2.urdf.xacro')
    robot_description_content = xacro.process_file(xacro_file).toxml()

    y_plane_arg = DeclareLaunchArgument(
        'y_plane_m',
        default_value='0.0',
        description='Fixed Y (world frame, metres) the operator holds their '
                    'hand at. Wrist X and Z are recovered from the pixel; Y '
                    'is pinned to this value via plane intersection.',
    )

    return LaunchDescription([
        y_plane_arg,


        # ── Robot description publishers (for rviz RobotModel display) ──
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            parameters=[{'robot_description': robot_description_content}],
            output='screen',
        ),
        Node(
            package='joint_state_publisher',
            executable='joint_state_publisher',
            name='joint_state_publisher',
            output='screen',
        ),


        # ── Static TF: world → hand_camera_link (BRIO body frame) ──
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

        # ── Static TF: hand_camera_link → hand_camera (optical frame) ──
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

        # ── Pose inference + wrist TF broadcaster ──────────────────
        Node(
            package='metis_pose_inference',
            executable='pose_inference_node',
            name='pose_inference',
            output='screen',
            parameters=[{
                'voyager_sdk_path':     voyager_sdk_path,
                'network':              'yolov8npose-coco-onnx',
                'source':               'usb:2',
                'world_frame':          'world',
                'camera_frame':         'hand_camera',
                'y_plane_m':              LaunchConfiguration('y_plane_m'),
                'visibility_threshold':   0.7,
                'person_score_threshold': 0.7,
                'camera_fx':            1152.7,
                'camera_fy':            1152.7,
                'camera_cx':            640.0,
                'camera_cy':            360.0,
            }],
        ),
    ])
