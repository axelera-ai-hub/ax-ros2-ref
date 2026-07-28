# (C) Copyright Axelera AI 2026
# All Rights Reserved
"""Launch a single save_annotated node.

All parameters are exposed as launch arguments so this file can be
included from a parent launch and overridden:

    ros2 launch axros_helper_nodes save_annotated.launch.py \\
        output_dir:=/tmp/run1 \\
        save_every_n:=4

Default save_every_n is 1 (save every synchronised frame).
"""
import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


_DEFAULT_OUTPUT_DIR = os.path.expanduser('~/axros_results')


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'image_topic',
            default_value='image_raw',
            description='Topic carrying the input frames.',
        ),
        DeclareLaunchArgument(
            'detections_topic',
            default_value='detections',
            description='Topic carrying vision_msgs/Detection2DArray output.',
        ),
        DeclareLaunchArgument(
            'output_dir',
            default_value=_DEFAULT_OUTPUT_DIR,
            description='Directory where annotated PNGs are written.',
        ),
        DeclareLaunchArgument(
            'annotated_topic',
            default_value='camera_frame_annotated',
            description='Topic to publish annotated images on.',
        ),
        DeclareLaunchArgument(
            'save_every_n',
            default_value='1',
            description='Save 1 in every N synchronised frames (1 = save all).',
        ),
        DeclareLaunchArgument(
            'save_frames',
            default_value='true',
            description='Whether to write PNGs to disk.',
        ),
        DeclareLaunchArgument(
            'publish_annotated',
            default_value='true',
            description='Whether to publish annotated images on annotated_topic.',
        ),
        Node(
            package='axros_helper_nodes',
            executable='save_annotated_node_exe',
            name='save_annotated',
            output='screen',
            parameters=[{
                'image_topic':       LaunchConfiguration('image_topic'),
                'detections_topic':  LaunchConfiguration('detections_topic'),
                'output_dir':        LaunchConfiguration('output_dir'),
                'annotated_topic':   LaunchConfiguration('annotated_topic'),
                'save_every_n':      LaunchConfiguration('save_every_n'),
                'save_frames':       LaunchConfiguration('save_frames'),
                'publish_annotated': LaunchConfiguration('publish_annotated'),
            }],
        ),
    ])
