# (C) Copyright Axelera AI 2026
# All Rights Reserved
"""
Pick-and-place perception launch.

Starts the static camera TF and the PickAndPlaceNode.
When PICK_CLASS is 'duck', also launches the yellow color detector.
When PICK_CLASS is 'red_toy', also launches the red color detector.
Per-object tuning profiles are applied based on PICK_CLASS.
Requires MoveIt + inference to be running already (via demo.launch.py or manually).

Environment variables:
    PICK_CLASS       — 'duck' | 'red_toy' | 'carrot' | 'banana' |
                       any COCO class (default: duck)
    PLACE_CLASS      — COCO class to place on (default: cow)
    ARM_VELOCITY     — velocity scaling 0.0-1.0 (default: 0.15)
    ARM_ACCELERATION — acceleration scaling 0.0-1.0 (default: 0.15)
    SELECT_MODE      — 'largest' | 'smallest' for color detectors (default: largest)
    PICK_OFFSET_X    — override X offset in metres for any class (e.g. -0.02)
    PICK_OFFSET_Y    — override Y offset in metres for any class (e.g.  0.03)

Supported pick profiles (grasp_height / pick_offset_x / max_torque / hold_torque):
    duck:               0.092 / -0.025 /  70 / 100   (soft body, neck grip, yellow detector)
    red_toy:            0.088 /   0.0  /  70 / 100   (soft body, center grip, red detector)
    carrot:             0.092 /   0.0  / 100 / 100   (center grip, max hold)
    banana:             0.085 /   0.0  /  70 /  60   (center grip, moderate)
    <other>:            0.092 /   0.0  /  80 /  50   (sensible defaults)
"""

import os
import yaml
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import xacro


def generate_launch_description():

    ned2_moveit_pkg = get_package_share_directory('niryo_ned2_moveit_config')

    velocity     = float(os.environ.get('ARM_VELOCITY', '0.15'))
    acceleration = float(os.environ.get('ARM_ACCELERATION', '0.15'))
    pick_class   = os.environ.get('PICK_CLASS', 'duck')
    place_class  = os.environ.get('PLACE_CLASS', 'cow')
    # Optional per-run offset overrides — useful for tuning without editing this file.
    # e.g.  PICK_OFFSET_Y=0.02 ros2 launch ...
    offset_x_override = os.environ.get('PICK_OFFSET_X')
    offset_y_override = os.environ.get('PICK_OFFSET_Y')

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

    # ── Per-object pick profile ────────────────────────────────────────
    # Override params based on which object we're picking up.
    if pick_class == 'duck':
        grasp_height  = 0.090
        pick_offset_x = 0.0
        pick_offset_y = 0.0
        max_torque    = 100
        hold_torque   = 100
    elif pick_class == 'carrot':
        # Carrot: slim, rigid but fragile — center grip, max hold (slippery)
        grasp_height  = 0.092
        pick_offset_x = 0.0
        pick_offset_y = 0.0
        max_torque    = 100
        hold_torque   = 100
    elif pick_class == 'banana':
        # Banana: curved, firm but bruises easily — center grip, moderate
        grasp_height  = 0.090
        pick_offset_x = 0.0
        pick_offset_y = -0.025
        max_torque    = 90
        hold_torque   = 100
    elif pick_class == 'red_toy':
        # Red plush/soft toy: small, squishy — center grip, gentle close, firm hold
        grasp_height  = 0.091
        pick_offset_x = 0.0
        pick_offset_y = 0.0
        max_torque    = 80
        hold_torque   = 100
    else:
        # Sensible defaults for unknown classes
        grasp_height  = 0.092
        pick_offset_x = 0.0
        pick_offset_y = 0.0
        max_torque    = 80
        hold_torque   = 50

    # Apply env-var overrides (PICK_OFFSET_X / PICK_OFFSET_Y) after profile selection
    if offset_x_override is not None:
        pick_offset_x = float(offset_x_override)
    if offset_y_override is not None:
        pick_offset_y = float(offset_y_override)

    # Per-profile extensions (safety floor, grip attempts, verify radius,
    # fixed-drop, loop). Default to the values that preserve the legacy
    # single-pick behaviour.
    grasp_min_height     = 0.09
    verify_duration_s    = 3.0
    verify_radius_m      = 0.15
    max_grip_attempts    = 1
    use_fixed_drop_pose  = False
    # Default drop is at the extreme -Y (robot's left) side of the workspace,
    # just inside the max_reach=0.38 clamp so the planner doesn't rescale it.
    # (Verified on hardware: -Y = left, +Y = right in this frame.)
    fixed_drop_x         = 0.05
    fixed_drop_y         = 0.34
    loop_mode            = False
    loop_scene_timeout_s = 5.0

    # Multi-object scheduling: when several pick-class objects are on the
    # table in one frame, commit to one instead of EMA-blending them.
    pick_selection_mode   = 'nearest_to_base'  # | rightmost | leftmost | nearest_to_x_axis
    pick_accept_radius_m  = 0.03               # only update EMA within this radius

    # Drop-stack offset: metres added to all place-phase heights per completed
    # drop so successive stacked objects don't crash the tower.
    drop_stack_increment_m = 0.0

    # Frames of a target we require before latching it. Default 10 is fine for
    # YOLO at 5+ Hz; override for detectors that emit sparsely.
    min_pick_detections   = 10

    # ── Optional: color detectors for non-COCO objects ──────────────────
    nodes = []
    if pick_class == 'duck':
        nodes.append(Node(
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
        ))
    elif pick_class == 'red_toy':
        nodes.append(Node(
            package='metis_moveit2_perception',
            executable='red_detector_node.py',
            name='red_detector',
            output='screen',
            parameters=[{
                'image_topic': 'image_raw',
                'label':       'red_toy',
                # Red wraps around hue — two ranges
                'h_low_min':   0,
                'h_low_max':   10,
                'h_high_min':  160,
                'h_high_max':  180,
                's_min':       100,
                's_max':       255,
                'v_min':       80,
                'v_max':       255,
                'min_area':    800,
                'select_mode': os.environ.get('SELECT_MODE', 'largest'),
            }],
        ))

    return LaunchDescription([

        # ── Static TF: camera → world ──────────────────────────────────────
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='camera_to_world_tf',
            arguments=[
                '0.275', '0.028', '0.42',
                '0.707', '0.707', '0.0', '0.0',
                'world', 'camera'
            ],
            output='screen',
        ),

        # ── PickAndPlaceNode ──────────────────────────────────────────────
        Node(
            package='metis_moveit2_perception',
            executable='pick_and_place_node_exe',
            name='pick_and_place',
            output='screen',
            parameters=[
                {
                    'pick_class':         pick_class,
                    'place_class':        place_class,
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
                    'grasp_min_height_m':  grasp_min_height,
                    'verify_duration_s':   verify_duration_s,
                    'verify_radius_m':     verify_radius_m,
                    'max_grip_attempts':   max_grip_attempts,
                    'use_fixed_drop_pose': use_fixed_drop_pose,
                    'fixed_drop_x':        fixed_drop_x,
                    'fixed_drop_y':        fixed_drop_y,
                    'loop_mode':           loop_mode,
                    'loop_scene_timeout_s': loop_scene_timeout_s,
                    'pick_selection_mode':  pick_selection_mode,
                    'pick_accept_radius_m': pick_accept_radius_m,
                    'drop_stack_increment_m': drop_stack_increment_m,
                    'min_pick_detections':  min_pick_detections,
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
        ),

        # ── Yellow detector (only when pick_class == 'duck') ─────────
        *nodes,
    ])
