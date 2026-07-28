# (C) Copyright Axelera AI 2026
# All Rights Reserved
"""Launch a single video_publisher node.

All parameters are exposed as launch arguments so this file can be
included from a parent launch and overridden:

    ros2 launch axros_helper_nodes video_publisher.launch.py \\
        video_path:=/path/to/clip.mp4 \\
        publish_fps:=5.0

When launched standalone with no overrides, the video path falls back
to a clip bundled with the Voyager SDK; the SDK location is taken
from $VOYAGER_SDK_DIR (set by experimental.axros/workspace_setup.sh)
and otherwise defaults to ~/voyager-sdk.
"""
import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


_DEFAULT_VOYAGER_SDK = os.environ.get(
    'VOYAGER_SDK_DIR',
    os.path.expanduser('~/voyager-sdk'),
)
_DEFAULT_VIDEO = os.path.join(
    _DEFAULT_VOYAGER_SDK, 'media', 'traffic1_1080p.mp4'
)


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'video_path',
            default_value=_DEFAULT_VIDEO,
            description='Absolute path to the video file to publish.',
        ),
        DeclareLaunchArgument(
            'output_topic',
            default_value='image_raw',
            description='Topic name to publish frames on.',
        ),
        DeclareLaunchArgument(
            'publish_fps',
            default_value='3.0',
            description='Publish rate in Hz (0.0 = video native FPS).',
        ),
        DeclareLaunchArgument(
            'loop',
            default_value='true',
            description='Loop the video when it ends.',
        ),
        DeclareLaunchArgument(
            'frame_id',
            default_value='camera',
            description='frame_id stamped on outgoing Image messages.',
        ),
        Node(
            package='axros_helper_nodes',
            executable='video_publisher_node_exe',
            name='video_publisher',
            output='screen',
            parameters=[{
                'video_path':   LaunchConfiguration('video_path'),
                'output_topic': LaunchConfiguration('output_topic'),
                'publish_fps':  LaunchConfiguration('publish_fps'),
                'loop':         LaunchConfiguration('loop'),
                'frame_id':     LaunchConfiguration('frame_id'),
            }],
        ),
    ])
