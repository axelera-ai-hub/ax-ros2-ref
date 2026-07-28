# (C) Copyright Axelera AI 2026
# All Rights Reserved
"""Duck + hand combined demo. Single launch, no venv shell activation.

End-to-end flow:
  1. The arm picks a yellow duck off the table (duck camera, /dev/video0).
  2. After lift + grip verification, it enters DETECTING_PLACE looking for
     the TF `right_wrist_locked` instead of a `cow` detection.
  3. Meanwhile, the BRIO camera watches the operator; once the user dwells
     their right wrist for `dwell_time_s` and the pose is reachable,
     pose_inference_node broadcasts `world → right_wrist_locked`.
  4. pick_and_place consumes that TF, approaches the hand, and opens the
     gripper — dropping the duck into the operator's hand.
  5. Loop back to pick the next duck.

What's in this launch:
  - niryo_ned2_moveit_config/launch/ned2_moveit_real.launch.py (NED2 driver,
    MoveIt, RViz, robot_state_publisher, joint_state_publisher)
  - Duck camera + /dev/video0 publisher + AxROS Live Inference display
    window (via axros_helper_nodes, with SKIP_YOLO=1 so COCO YOLO doesn't
    monopolise the AIPU — the duck is picked up by yellow_detector_node
    on the CPU instead).
  - yellow_detector_node (class_id='duck' on /detections)
  - pick_and_place_node with place_mode='tf', place_tf_frame='right_wrist_locked'
  - BRIO static TFs + pose_inference_node (with voyager-sdk env injected
    per-node, as in wrist_tracking_full.launch.py). This gets the AIPU to
    itself now that COCO YOLO is skipped.
  - rqt_image_view on /hand_camera/annotated (pose overlay + lock ring).
    The duck camera viz comes from AxROS Live Inference's own window.

The combined launch is NOT started with the voyager venv active. voyager
paths are injected only onto pose_inference_node via additional_env, so
rqt and ROS-system Python stay healthy.

Environment variables (inherited conventions from pick_and_place.launch.py
and wrist_tracking_full.launch.py):
    PICK_CLASS        — default 'duck'
    ARM_VELOCITY      — default 0.30
    ARM_ACCELERATION  — default 0.30
    DWELL_TIME_S      — default 3.0 (how long the user holds still before drop)
    Y_PLANE_M         — default 0.20
    Z_OFFSET_M        — default 0.05
    VOYAGER_SDK_DIR   — default ~/voyager-sdk
    AXELERA_RUNTIME_DIR — default /opt/axelera/runtime-1.5.3-1
    PICK_OFFSET_X, PICK_OFFSET_Y — per-run pick bias overrides
    SELECT_MODE       — yellow-detector select_mode (default 'largest')
"""

import os

import xacro
import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (IncludeLaunchDescription, LogInfo, TimerAction)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


def _voyager_additional_env(voyager_sdk_path: str, runtime_dir: str) -> dict:
    """Env vars needed by axelera.app / GStreamer plugins inside pose_inference.

    Mirrors the subset of voyager-sdk/venv/bin/activate that the dynamic
    linker and GStreamer need *before* Python starts. Prepended to any
    existing values rather than replaced.
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
    # ── Paths and env ──────────────────────────────────────────────────────
    voyager_sdk_path = os.environ.get(
        'VOYAGER_SDK_DIR', os.path.expanduser('~/voyager-sdk'))
    runtime_dir = os.environ.get(
        'AXELERA_RUNTIME_DIR', '/opt/axelera/runtime-1.5.3-1')

    runtime_warning = None
    if not os.path.isfile(os.path.join(runtime_dir, 'lib', 'libaxruntime.so')):
        runtime_warning = (
            f'[duck_hand_demo] WARNING: libaxruntime.so not found at '
            f'{runtime_dir}/lib — pose_inference_node will likely fail. '
            f'Override AXELERA_RUNTIME_DIR before launching.')

    ned2_moveit_pkg = get_package_share_directory('niryo_ned2_moveit_config')
    helper_share = get_package_share_directory('axros_helper_nodes')
    inference_pkg = get_package_share_directory('metis_ros2_inference')

    # MoveIt + robot description bits required by pick_and_place.
    xacro_file = os.path.join(ned2_moveit_pkg, 'config', 'niryo_ned2.urdf.xacro')
    robot_description_content = xacro.process_file(xacro_file).toxml()

    srdf_file = os.path.join(ned2_moveit_pkg, 'config', 'niryo_ned2.srdf')
    with open(srdf_file, 'r') as f:
        robot_description_semantic_content = f.read()

    kinematics_file = os.path.join(ned2_moveit_pkg, 'config', 'kinematics.yaml')
    with open(kinematics_file, 'r') as f:
        kinematics_config = yaml.safe_load(f)

    # ── Runtime knobs ──────────────────────────────────────────────────────
    velocity        = float(os.environ.get('ARM_VELOCITY', '0.30'))
    acceleration    = float(os.environ.get('ARM_ACCELERATION', '0.30'))
    pick_class      = os.environ.get('PICK_CLASS', 'duck')
    dwell_time_s    = float(os.environ.get('DWELL_TIME_S', '3.0'))
    y_plane_m       = float(os.environ.get('Y_PLANE_M', '0.20'))
    z_offset_m      = float(os.environ.get('Z_OFFSET_M', '0.05'))
    # Delay pick_and_place_node so MoveIt + the NED2 ros2_driver have time
    # to publish /joint_states first. Without this, the initial plan-and-
    # execute fires before joint_states arrive and move_group aborts with
    # "Didn't receive robot state" / "start point deviates" for ~5 seconds.
    pnp_start_delay_s = float(os.environ.get('PNP_START_DELAY_S', '20.0'))

    # TF-place geometry: where above the operator's wrist the TCP goes.
    # Smaller offsets keep the target inside NED2's reach envelope when the
    # hand is held near the workspace edge.
    # In TF mode the descent step is skipped, so the hover offset *is* the
    # height above the wrist where the gripper opens. drop_offset is kept
    # declared for backward compatibility but no longer used by the state
    # machine in TF mode.
    place_tf_hover_offset = float(os.environ.get('PLACE_TF_HOVER_OFFSET_M', '0.02'))
    place_tf_drop_offset  = float(os.environ.get('PLACE_TF_DROP_OFFSET_M', '0.05'))
    place_tf_lift_offset  = float(os.environ.get('PLACE_TF_LIFT_OFFSET_M', '0.15'))
    offset_x_override = os.environ.get('PICK_OFFSET_X')
    offset_y_override = os.environ.get('PICK_OFFSET_Y')

    # Duck tuning (same as pick_and_place.launch.py duck profile)
    grasp_height  = 0.090
    pick_offset_x = 0.0
    pick_offset_y = 0.0
    max_torque    = 100
    hold_torque   = 100
    if offset_x_override is not None:
        pick_offset_x = float(offset_x_override)
    if offset_y_override is not None:
        pick_offset_y = float(offset_y_override)

    # ── Stage 1: NED2 hardware driver + MoveIt + RViz ─────────────────────
    ned2_stack = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
            ned2_moveit_pkg, 'launch', 'ned2_moveit_real.launch.py')),
    )

    # ── Stage 2: Duck camera + AxROS Live Inference display ───────────────
    # live_inference.launch.py starts the /dev/video0 camera publisher and
    # (by default) the Metis COCO YOLO inference node. The COCO node claims
    # all 4 AIPU cores — which starves the pose model. For this demo we
    # only need the duck stream (detected by yellow_detector_node on the
    # CPU), so SKIP_YOLO=1 suppresses COCO YOLO and hands the AIPU to the
    # pose network. The AxROS Live Inference display window still comes up.
    os.environ.setdefault('USE_LIVE_CAMERA', '1')
    os.environ.setdefault('SKIP_YOLO', '1')
    inference_stack = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
            inference_pkg, 'launch', 'metis_inference.launch.py')),
    )

    # ── Yellow detector for duck (subscribes to image_raw) ─────────────────
    yellow_detector = Node(
        package='metis_moveit2_perception',
        executable='yellow_detector_node.py',
        name='yellow_detector',
        output='screen',
        parameters=[{
            'image_topic': 'image_raw',
            'label':       'duck',
            'h_min':       20,
            'h_max':       35,
            's_min':       100,
            's_max':       255,
            'v_min':       100,
            'v_max':       255,
            'min_area':    1000,
            'select_mode': os.environ.get('SELECT_MODE', 'largest'),
        }],
    )

    # ── Duck camera static TF (world → camera) ─────────────────────────────
    duck_camera_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='camera_to_world_tf',
        arguments=[
            '0.275', '0.028', '0.42',
            '0.707', '0.707', '0.0', '0.0',
            'world', 'camera'
        ],
        output='screen',
    )

    # ── BRIO static TFs (world → hand_camera_link → hand_camera) ───────────
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

    # ── Pose inference (BRIO + voyager env) ────────────────────────────────
    # Only the TF `right_wrist_locked` is consumed downstream. wrist_tracker
    # and arm_controller from the wrist-tracking stack are NOT launched — the
    # pick_and_place_node is the sole arm controller in this demo.
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
            'y_plane_m':               y_plane_m,
            'visibility_threshold':    0.7,
            'person_score_threshold':  0.81,
            'camera_fx':               1152.7,
            'camera_fy':               1152.7,
            'camera_cx':               640.0,
            'camera_cy':               360.0,
            'publish_annotated_image': True,
            'annotated_image_topic':   '/hand_camera/annotated',
            # Dwell / lock gating — user presents their hand, dwells 3s,
            # gets a locked TF. pick_and_place picks that up as its place
            # target only after VERIFYING_GRIP transitions to DETECTING_PLACE.
            'dwell_time_s':            dwell_time_s,
            'dwell_tolerance_m':       0.05,
            'unlock_distance_m':       0.10,
            'lock_frame_name':         'right_wrist_locked',
            'lock_redetect_grace_s':   2.0,
            'lock_hold_s':             2.0,
            # Keep reach gate loose — per user preference.
            'max_reach_m':             0.70,
        }],
    )

    # ── Pick-and-place with tf-place mode ──────────────────────────────────
    pick_and_place = Node(
        package='metis_moveit2_perception',
        executable='pick_and_place_node_exe',
        name='pick_and_place',
        output='screen',
        parameters=[
            {
                'pick_class':         pick_class,
                'place_class':        'cow',              # unused under place_mode='tf'
                'place_mode':         'tf',
                'place_tf_frame':     'right_wrist_locked',
                'place_tf_max_age_s': 1.0,
                'place_tf_hover_offset_m': place_tf_hover_offset,
                'place_tf_drop_offset_m':  place_tf_drop_offset,
                'place_tf_lift_offset_m':  place_tf_lift_offset,
                'camera_fx':          1360.0,
                'camera_fy':          1360.0,
                'camera_cx':          960.0,
                'camera_cy':          540.0,
                'assumed_depth_m':    0.42,
                'camera_frame':       'camera',
                'robot_base_frame':   'world',
                'max_reach_m':        0.38,
                'hover_height_m':      0.15,
                'grasp_height_m':      grasp_height,
                'place_drop_height_m': 0.11,
                'lift_height_m':       0.20,
                'pick_offset_x':       pick_offset_x,
                'pick_offset_y':       pick_offset_y,
                'min_detection_score': 0.45,
                'pick_ema_alpha':      0.40,
                'tool_id':             11,
                'max_torque_pct':      max_torque,
                'hold_torque_pct':     hold_torque,
                'grasp_min_height_m':  0.09,
                'verify_duration_s':   3.0,
                'verify_radius_m':     0.15,
                'max_grip_attempts':   1,
                'use_fixed_drop_pose': False,
                'fixed_drop_x':        0.05,
                'fixed_drop_y':        0.34,
                'loop_mode':           True,            # auto-repeat for next duck
                'loop_scene_timeout_s': 5.0,
                'pick_selection_mode':  'nearest_to_base',
                'pick_accept_radius_m': 0.03,
                'drop_stack_increment_m': 0.0,
                'min_pick_detections':  10,
                'planning_group':     'arm',
                'end_effector_link':  'tool_link',
                'planning_time_s':    5.0,
                'planning_attempts':  10,
                'velocity_scaling':   velocity,
                'acceleration_scaling': acceleration,
                'robot_description':  robot_description_content,
                'robot_description_semantic': robot_description_semantic_content,
            },
            {'robot_description_kinematics': kinematics_config},
        ],
    )

    # ── Visualiser — pose stream only ──────────────────────────────────────
    # Duck camera viz comes from the AxROS Live Inference window that
    # live_inference_node_exe opens on its own (cv2.imshow-based), so we
    # don't need a second rqt for /image_raw.
    rqt_pose_view = Node(
        package='rqt_image_view',
        executable='rqt_image_view',
        name='rqt_pose_view',
        arguments=['/hand_camera/annotated'],
        output='screen',
    )

    actions = []
    if runtime_warning:
        actions.append(LogInfo(msg=runtime_warning))
    actions += [
        ned2_stack,
        inference_stack,
        yellow_detector,
        duck_camera_tf,
        brio_body_tf,
        brio_optical_tf,
        pose_inference,
        # pick_and_place is gated on a delay so the NED2 driver has time
        # to bring /joint_states online; otherwise the first plan+execute
        # cycle aborts repeatedly until the state monitor catches up.
        TimerAction(period=pnp_start_delay_s, actions=[
            LogInfo(msg=f'[duck_hand_demo] Starting pick_and_place after {pnp_start_delay_s:.0f}s delay'),
            pick_and_place,
        ]),
        rqt_pose_view,
    ]
    return LaunchDescription(actions)
