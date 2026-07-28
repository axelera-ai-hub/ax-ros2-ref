# (C) Copyright Axelera AI 2026
# All Rights Reserved
"""
Unified demo launch. Single terminal, staged startup.

Usage (simulation — hover mode):
    ros2 launch metis_moveit2_perception demo.launch.py

Usage (real robot — hover mode):
    DEMO_SIM=false ros2 launch metis_moveit2_perception demo.launch.py

Usage (real robot — pick and place):
    DEMO_MODE=pick_and_place PICK_CLASS=banana PLACE_CLASS=cow DEMO_SIM=false \
        ros2 launch metis_moveit2_perception demo.launch.py

Environment variables:
    DEMO_SIM              — true (simulation) or false (real robot). Default: true
    DEMO_MODE             — 'hover' or 'pick_and_place'. Default: hover
    TARGET_CLASS          — target class for hover mode. Default: cat
    PICK_CLASS            — pick target for pick_and_place mode. Default: banana
    PLACE_CLASS           — place target for pick_and_place mode. Default: cow
    ARM_VELOCITY          — velocity scaling 0.0-1.0. Default: 0.3 (hover) / 0.15 (pnp)
    ARM_ACCELERATION      — acceleration scaling 0.0-1.0. Default: 0.3 (hover) / 0.15 (pnp)
    USE_LIVE_CAMERA       — 1 for USB camera, 0 for video file. Default: 1
    DEMO_MOVEIT_DELAY     — seconds to wait for MoveIt. Default: 12
    DEMO_INFERENCE_DELAY  — seconds after MoveIt before inference. Default: 8
    DEMO_PERCEPTION_DELAY — seconds after inference before arm. Default: 15
"""

import os
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    LogInfo,
    TimerAction,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():

    # ── Launch arguments ──────────────────────────────────────────────────
    sim_arg = DeclareLaunchArgument(
        'sim', default_value='true',
        description='Use simulation (true) or real robot (false)')

    use_live_camera_arg = DeclareLaunchArgument(
        'use_live_camera', default_value='true',
        description='Use live USB camera (true) or video file (false)')

    moveit_delay_arg = DeclareLaunchArgument(
        'moveit_delay', default_value='12.0',
        description='Seconds to wait for MoveIt + RViz to initialize')

    inference_delay_arg = DeclareLaunchArgument(
        'inference_delay', default_value='8.0',
        description='Seconds after MoveIt before starting inference')

    perception_delay_arg = DeclareLaunchArgument(
        'perception_delay', default_value='15.0',
        description='Seconds after inference before starting perception arm')

    # ── Package paths ─────────────────────────────────────────────────────
    ned2_moveit_pkg = get_package_share_directory('niryo_ned2_moveit_config')
    inference_pkg   = get_package_share_directory('metis_ros2_inference')
    perception_pkg  = get_package_share_directory('metis_moveit2_perception')

    # ── Mode selection ────────────────────────────────────────────────────
    demo_mode = os.environ.get('DEMO_MODE', 'hover')
    use_sim = os.environ.get('DEMO_SIM', 'true').lower() in ('true', '1', 'yes')

    if use_sim:
        moveit_launch_file = os.path.join(
            ned2_moveit_pkg, 'launch', 'ned2_moveit_launch.py')
        mode_label = 'SIMULATION'
    else:
        moveit_launch_file = os.path.join(
            ned2_moveit_pkg, 'launch', 'ned2_moveit_real.launch.py')
        mode_label = 'REAL ROBOT'

    if demo_mode == 'pick_and_place':
        pick_class  = os.environ.get('PICK_CLASS', 'bottle')
        place_class = os.environ.get('PLACE_CLASS', 'cow')
        mode_label += f' | PICK & PLACE: {pick_class} -> {place_class}'
        perception_launch_file = os.path.join(
            perception_pkg, 'launch', 'pick_and_place.launch.py')
    else:
        target_class = os.environ.get('TARGET_CLASS', 'cat')
        mode_label += f' | HOVER: {target_class}'
        perception_launch_file = os.path.join(
            perception_pkg, 'launch', 'perception_arm.launch.py')

    # ── Stage 1: MoveIt + RViz ────────────────────────────────────────────
    moveit_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(moveit_launch_file),
    )

    # ── Stage 2: Inference ────────────────────────────────────────────────
    if os.environ.get('USE_LIVE_CAMERA') is None:
        os.environ['USE_LIVE_CAMERA'] = '1'

    inference_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(inference_pkg, 'launch', 'metis_inference.launch.py')),
    )

    # ── Stage 3: Perception ───────────────────────────────────────────────
    perception_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(perception_launch_file),
    )

    # ── Compute cumulative delays ─────────────────────────────────────────
    moveit_delay_s     = float(os.environ.get('DEMO_MOVEIT_DELAY', '12.0'))
    inference_delay_s  = float(os.environ.get('DEMO_INFERENCE_DELAY', '8.0'))
    perception_delay_s = float(os.environ.get('DEMO_PERCEPTION_DELAY', '15.0'))

    inference_start  = moveit_delay_s
    perception_start = moveit_delay_s + inference_delay_s

    return LaunchDescription([
        # Arguments
        sim_arg,
        use_live_camera_arg,
        moveit_delay_arg,
        inference_delay_arg,
        perception_delay_arg,

        # Stage 1: MoveIt + RViz — immediate
        LogInfo(msg='\n══════════════════════════════════════════════'),
        LogInfo(msg=f'  Stage 1/3: Starting MoveIt + RViz'),
        LogInfo(msg=f'  Mode: {mode_label}'),
        LogInfo(msg='══════════════════════════════════════════════\n'),
        moveit_launch,

        # Stage 2: Inference — delayed
        TimerAction(
            period=inference_start,
            actions=[
                LogInfo(msg='\n══════════════════════════════════════════════'),
                LogInfo(msg='  Stage 2/3: Starting AxROS Live Inference...'),
                LogInfo(msg='══════════════════════════════════════════════\n'),
                inference_launch,
            ],
        ),

        # Stage 3: Perception / Pick-and-Place — delayed further
        TimerAction(
            period=perception_start,
            actions=[
                LogInfo(msg='\n══════════════════════════════════════════════'),
                LogInfo(msg=f'  Stage 3/3: Starting {demo_mode.replace("_", " ").title()} Pipeline...'),
                LogInfo(msg='══════════════════════════════════════════════\n'),
                perception_launch,
            ],
        ),
    ])
