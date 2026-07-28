# (C) Copyright Axelera AI 2026
# All Rights Reserved
"""Unified wrist-tracking demo (single launch, no venv shell activation).

What this brings up:
  - NED2 hardware driver + MoveIt move_group + RViz (via
    niryo_ned2_moveit_config/launch/ned2_moveit_real.launch.py)
  - Static TFs world → hand_camera_link → hand_camera (BRIO)
  - PoseInferenceNode (with voyager-sdk env injected per-node)
  - WristTrackerNode
  - ArmControllerNode
  - rqt_image_view pre-subscribed to /hand_camera/annotated

How this replaces the three-terminal workflow:
  `additional_env` on the pose_inference Node injects the voyager-sdk
  `LD_LIBRARY_PATH` / `GST_PLUGIN_PATH` / `AXELERA_FRAMEWORK` so its dynamic
  linker can resolve libaxstreamer.so + GStreamer plugins at startup. The
  other nodes (rqt_image_view in particular) inherit the parent's unmodified
  env and keep working — PyQt5 stays visible from apt's system site-packages,
  MoveIt and the NED2 driver stay on their normal paths.

Launch with:

    source /opt/ros/jazzy/setup.bash
    cd ~/experimental.axros/ros2_ws && source install/setup.bash
    ros2 launch metis_pose_inference wrist_tracking_full.launch.py

Do NOT activate the voyager venv in this shell — that was the whole point.
"""

import os

import xacro
import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, IncludeLaunchDescription,
                            LogInfo)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _voyager_additional_env(voyager_sdk_path: str, runtime_dir: str) -> dict:
    """Env vars required by axelera.app / GStreamer plugins.

    Mirrors the subset of voyager-sdk/venv/bin/activate that the dynamic
    linker and GStreamer need *before* Python starts — everything Python-
    level (sys.path, AXELERA_FRAMEWORK) is still handled inside the node.

    Prepended to any existing values rather than replaced, so ROS2 / system
    library paths aren't clobbered.
    """
    operators_lib = os.path.join(voyager_sdk_path, 'operators', 'lib')
    runtime_lib = os.path.join(runtime_dir, 'lib')
    gstreamer_lib = os.path.join(runtime_lib, 'gstreamer-1.0')

    def prepend(var: str, values: list) -> str:
        existing = os.environ.get(var, '')
        return os.pathsep.join([*values, existing]) if existing \
            else os.pathsep.join(values)

    return {
        'LD_LIBRARY_PATH':    prepend('LD_LIBRARY_PATH', [runtime_lib, operators_lib]),
        'GST_PLUGIN_PATH':    prepend('GST_PLUGIN_PATH', [gstreamer_lib, operators_lib]),
        'AXELERA_FRAMEWORK':  voyager_sdk_path,
        'AXELERA_RUNTIME_DIR': runtime_dir,
    }


def generate_launch_description():
    voyager_sdk_path = os.environ.get(
        'VOYAGER_SDK_DIR', os.path.expanduser('~/voyager-sdk'))
    runtime_dir = os.environ.get(
        'AXELERA_RUNTIME_DIR', '/opt/axelera/runtime-1.5.3-1')

    # Soft-validate the runtime path so a missing SDK upgrade path yields a
    # useful launch-time log rather than a cryptic dlopen error downstream.
    runtime_warning = None
    if not os.path.isfile(os.path.join(runtime_dir, 'lib', 'libaxruntime.so')):
        runtime_warning = (
            f'[wrist_tracking_full] WARNING: libaxruntime.so not found at '
            f'{runtime_dir}/lib — pose_inference_node will likely fail. '
            f'Set AXELERA_RUNTIME_DIR to the correct /opt/axelera/runtime-* '
            f'path before launching.')

    ned2_moveit_pkg = get_package_share_directory('niryo_ned2_moveit_config')

    xacro_file = os.path.join(ned2_moveit_pkg, 'config', 'niryo_ned2.urdf.xacro')
    robot_description_content = xacro.process_file(xacro_file).toxml()

    srdf_file = os.path.join(ned2_moveit_pkg, 'config', 'niryo_ned2.srdf')
    with open(srdf_file, 'r') as f:
        robot_description_semantic_content = f.read()

    kinematics_file = os.path.join(ned2_moveit_pkg, 'config', 'kinematics.yaml')
    with open(kinematics_file, 'r') as f:
        kinematics_config = yaml.safe_load(f)

    velocity = float(os.environ.get('ARM_VELOCITY', '0.40'))
    acceleration = float(os.environ.get('ARM_ACCELERATION', '0.40'))

    # ── Launch args ────────────────────────────────────────────────────────
    y_plane_arg = DeclareLaunchArgument(
        'y_plane_m',
        default_value='0.20',
        description='Fixed Y (metres) where the operator holds their hand.',
    )
    z_offset_arg = DeclareLaunchArgument(
        'z_offset_m',
        default_value='0.05',
        description='Standoff from the wrist along the gripper approach axis.',
    )

    # ── NED2 hardware driver + MoveIt ──────────────────────────────────────
    ned2_stack = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
            ned2_moveit_pkg, 'launch', 'ned2_moveit_real.launch.py')),
    )

    # ── BRIO static TFs ────────────────────────────────────────────────────
    brio_body_tf = Node(
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
    )
    brio_optical_tf = Node(
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
    )

    # ── Pose inference (voyager env injected here, and here only) ──────────
    pose_inference = Node(
        package='metis_pose_inference',
        executable='pose_inference_node',
        name='pose_inference',
        output='screen',
        additional_env=_voyager_additional_env(voyager_sdk_path, runtime_dir),
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
            # Workspace gate — intentionally loose so MoveIt can decide
            # reachability for anything inside the physically plausible
            # envelope. Real unreachable targets still surface as
            # planning timeouts downstream.
            'max_reach_m':             0.70,
        }],
    )

    # ── Wrist tracker: locked TF → /target_hover_pose ─────────────────────
    wrist_tracker = Node(
        package='metis_pose_inference',
        executable='wrist_tracker_node',
        name='wrist_tracker',
        output='screen',
        parameters=[{
            'world_frame':     'world',
            'wrist_frame':     'right_wrist_locked',
            'output_topic':    '/target_hover_pose',
            # Slight +X bias on the drop position so the TCP hovers just
            # forward of the tracked wrist rather than directly on top.
            'x_offset_m':      0.03,
            'z_offset_m':      LaunchConfiguration('z_offset_m'),
            'publish_rate_hz': 2.0,
            'max_tf_age_s':    1.0,
            'min_movement_m':  0.03,
            'approach_mode':   'down',
        }],
    )

    # ── Arm controller: /target_hover_pose → MoveIt plan ──────────────────
    arm_controller = Node(
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
    )

    # ── rqt_image_view — runs with parent env (no venv), so apt's PyQt5 ───
    # resolves via system python. Gets the pose_inference stream the moment
    # the node starts publishing.
    rqt_image_view = Node(
        package='rqt_image_view',
        executable='rqt_image_view',
        name='rqt_image_view',
        arguments=['/hand_camera/annotated'],
        output='screen',
    )

    actions = [y_plane_arg, z_offset_arg]
    if runtime_warning:
        actions.append(LogInfo(msg=runtime_warning))
    actions += [
        ned2_stack,
        brio_body_tf,
        brio_optical_tf,
        pose_inference,
        wrist_tracker,
        arm_controller,
        rqt_image_view,
    ]
    return LaunchDescription(actions)
