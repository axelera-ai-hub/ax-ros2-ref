# (C) Copyright Axelera AI 2026
# All Rights Reserved
"""Wrist-tracking validation launch.

What this brings up:
  - Static TFs world → hand_camera_link → hand_camera (BRIO)
  - PoseInferenceNode (publishes wrist TFs + annotated image)
  - WristTrackerNode (republishes wrist TF as /target_hover_pose)
  - ArmControllerNode (plans to /target_hover_pose via MoveIt)

Run the NED2 hardware driver + MoveIt move_group separately beforehand
(same assumption as perception_arm.launch.py). robot_state_publisher and
joint_state_publisher come from that launch — duplicating them here would
fight the hardware driver's /joint_states.

The annotated image is published on /hand_camera/annotated but rqt_image_view
is NOT launched here: its shebang is `#!/usr/bin/env python3` which resolves
to the activated voyager-sdk venv's Python, which lacks PyQt5/PySide2. Run it
from a separate terminal that does NOT have the venv sourced:

    source /opt/ros/jazzy/setup.bash
    cd ~/experimental.axros/ros2_ws && source install/setup.bash
    ros2 run rqt_image_view rqt_image_view /hand_camera/annotated
"""

import os

import xacro
import yaml
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

    srdf_file = os.path.join(ned2_moveit_pkg, 'config', 'niryo_ned2.srdf')
    with open(srdf_file, 'r') as f:
        robot_description_semantic_content = f.read()

    kinematics_file = os.path.join(ned2_moveit_pkg, 'config', 'kinematics.yaml')
    with open(kinematics_file, 'r') as f:
        kinematics_config = yaml.safe_load(f)

    velocity = float(os.environ.get('ARM_VELOCITY', '0.15'))
    acceleration = float(os.environ.get('ARM_ACCELERATION', '0.15'))

    y_plane_arg = DeclareLaunchArgument(
        'y_plane_m',
        default_value='0.15',
        description='Fixed Y (metres) where the operator holds their hand.',
    )
    z_offset_arg = DeclareLaunchArgument(
        'z_offset_m',
        default_value='0.05',
        description='Vertical offset above the wrist the arm tracks to.',
    )

    return LaunchDescription([
        y_plane_arg,
        z_offset_arg,

        # NOTE: robot_state_publisher + joint_state_publisher are NOT started
        # here — ned2_moveit_real.launch.py (run in a separate terminal before
        # this one) already brings them up. Starting a duplicate would fail
        # with a node-name collision.

        # ── BRIO static TFs ──────────────────────────────────────────
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='brio_body_tf',
            arguments=[
                '--x',     '0.165',
                '--y',    '-0.70',
                '--z',     '0.35',
                '--yaw',   '1.5708',
                '--pitch', '0',
                '--roll',  '0',
                '--frame-id',       'world',
                '--child-frame-id', 'hand_camera_link',
            ],
            output='screen',
        ),
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

        # ── Pose inference (also publishes /hand_camera/annotated) ───
        Node(
            package='metis_pose_inference',
            executable='pose_inference_node',
            name='pose_inference',
            output='screen',
            parameters=[{
                'voyager_sdk_path':        voyager_sdk_path,
                'network':                 'yolov8npose-coco-onnx',
                'source':                  'usb:2',
                'world_frame':             'world',
                'camera_frame':            'hand_camera',
                'y_plane_m':               LaunchConfiguration('y_plane_m'),
                'visibility_threshold':    0.7,
                'person_score_threshold':  0.81,
                'camera_fx':               1152.7,
                'camera_fy':               1152.7,
                'camera_cx':               640.0,
                'camera_cy':               360.0,
                'publish_annotated_image': True,
                'annotated_image_topic':   '/hand_camera/annotated',
                # Dwell / lock gating
                'dwell_time_s':            3.0,
                'dwell_tolerance_m':       0.05,
                'unlock_distance_m':       0.10,
                'lock_frame_name':         'right_wrist_locked',
                'lock_redetect_grace_s':   2.0,
                'lock_hold_s':             2.0,
                # Reach gate — NED2 envelope is ~0.44 m. With horizontal
                # approach the TCP sits closer to the base than the wrist
                # (standoff on -Y), so a wrist at 0.50 m corresponds to a
                # TCP target inside 0.45 m. Bounded loosely here so only
                # clearly-unreachable targets are blocked.
                'max_reach_m':             0.50,
            }],
        ),

        # ── Wrist tracker: TF → /target_hover_pose ───────────────────
        Node(
            package='metis_pose_inference',
            executable='wrist_tracker_node',
            name='wrist_tracker',
            output='screen',
            parameters=[{
                'world_frame':     'world',
                # Follow the locked TF, not the raw right_wrist — the arm only
                # moves once the pose node has held a stable lock for
                # dwell_time_s. When the lock drops the TF stops refreshing
                # and max_tf_age_s gates further commands.
                'wrist_frame':     'right_wrist_locked',
                'output_topic':    '/target_hover_pose',
                'z_offset_m':      LaunchConfiguration('z_offset_m'),
                'publish_rate_hz': 2.0,
                'max_tf_age_s':    1.0,
                'min_movement_m':  0.03,
                # 'horizontal': gripper points at the hand (perpendicular to
                #               a hand held out; standoff along -Y). Larger
                #               reachable workspace than 'down'.
                # 'down':       gripper points -Z, hovers above the wrist.
                'approach_mode':   'horizontal',
            }],
        ),

        # ── Arm controller: /target_hover_pose → MoveIt plan ─────────
        Node(
            package='metis_moveit2_perception',
            executable='arm_controller_node_exe',
            name='arm_controller',
            output='screen',
            parameters=[
                {
                    'target_pose_topic':         '/target_hover_pose',
                    'planning_group':            'arm',
                    'end_effector_link':         'tool_link',
                    'planning_time_s':           5.0,
                    'planning_attempts':         10,
                    'goal_tolerance_m':          0.02,
                    'min_replan_interval_s':     3.0,
                    'velocity_scaling':          velocity,
                    'acceleration_scaling':      acceleration,
                    'robot_description':         robot_description_content,
                    'robot_description_semantic': robot_description_semantic_content,
                },
                {'robot_description_kinematics': kinematics_config},
            ],
        ),

        # rqt_image_view is intentionally NOT launched here — see module
        # docstring. Launch it from a separate non-venv terminal subscribing
        # to /hand_camera/annotated.
    ])
