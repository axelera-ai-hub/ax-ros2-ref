# (C) Copyright Axelera AI 2026
# All Rights Reserved
"""End-to-end Metis inference pipeline launch.

Default behaviour: read a video file from the Voyager SDK media folder,
run YOLOv5l-COCO inference on the Metis AIPU, and save annotated frames
to disk. Every parameter has a sensible default; override any of them
on the command line:

    ros2 launch metis_ros2_inference metis_inference.launch.py
    ros2 launch metis_ros2_inference metis_inference.launch.py \\
        model_name:=yolov5s-v7-coco \\
        media_file:=traffic2_720p.mp4 \\
        save_every_n:=4

The Voyager SDK location is taken from $VOYAGER_SDK_DIR (set by
experimental.axros/workspace_setup.sh); falls back to ~/voyager-sdk.
"""
import os

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    OpaqueFunction,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


_DEFAULT_VOYAGER_SDK = os.environ.get(
    'VOYAGER_SDK_DIR',
    os.path.expanduser('~/voyager-sdk'),
)


def _launch_setup(context, *_, **__):
    """Resolve substitutions, compute paths, and build the node graph."""
    voyager_sdk    = LaunchConfiguration('voyager_sdk').perform(context)
    model_name     = LaunchConfiguration('model_name').perform(context)
    media_file     = LaunchConfiguration('media_file').perform(context)
    results_dir    = LaunchConfiguration('results_dir').perform(context)
    save_every_n   = LaunchConfiguration('save_every_n').perform(context)
    aipu_cores     = int(LaunchConfiguration('aipu_cores').perform(context))
    conf_threshold = float(LaunchConfiguration('conf_threshold').perform(context))
    nms_threshold  = float(LaunchConfiguration('nms_threshold').perform(context))
    use_live       = LaunchConfiguration('use_live_camera').perform(context) == '1'

    # Allow build/<parent>/<actual>/1/ layouts (e.g. hand-detection ships as
    # build/hand-detection/yolov10n-coco-onnx/1/). Defaults to MODEL_NAME for
    # the nested layout so existing launches keep working.
    model_subdir = os.environ.get('MODEL_SUBDIR', model_name)
    model_dir    = os.path.join(voyager_sdk, 'build', model_name, model_subdir, '1')
    labels       = os.environ.get(
        'LABELS_PATH',
        os.path.join(voyager_sdk, 'ax_datasets', 'labels', 'coco.names'),
    )
    video_path   = os.path.join(voyager_sdk, 'media', media_file)

    helper_share = get_package_share_directory('axros_helper_nodes')

    if use_live:
        # live_inference.launch.py starts both the camera/display node
        # and the metis inference node — no other nodes are added here.
        live_launch = os.path.join(helper_share, 'launch', 'live_inference.launch.py')
        return [
            IncludeLaunchDescription(PythonLaunchDescriptionSource(live_launch)),
        ]

    # ── Video-file mode (default) ──────────────────────────────────────────
    video_publisher = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(helper_share, 'launch', 'video_publisher.launch.py')),
        launch_arguments={
            'video_path':   video_path,
            'output_topic': 'image_raw',
        }.items(),
    )

    save_annotated = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(helper_share, 'launch', 'save_annotated.launch.py')),
        launch_arguments={
            'image_topic':      'image_raw',
            'detections_topic': 'detections',
            'output_dir':       results_dir,
            'save_every_n':     save_every_n,
        }.items(),
    )

    inference = Node(
        package='metis_ros2_inference',
        executable='metis_inference_node',
        name='metis_inference',
        output='screen',
        parameters=[{
            'model_name':     model_name,
            'model_dir':      model_dir,
            'labels_path':    labels,
            'aipu_cores':     aipu_cores,
            'input_topic':    'image_raw',
            'output_topic':   'detections',
            'conf_threshold': conf_threshold,
            'nms_threshold':  nms_threshold,
            'normalize':      False,
            'mean':           [0.485, 0.456, 0.406],
            'stddev':         [0.229, 0.224, 0.225],
        }],
    )

    return [video_publisher, save_annotated, inference]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'voyager_sdk',
            default_value=_DEFAULT_VOYAGER_SDK,
            description='Path to the Voyager SDK root directory.',
        ),
        DeclareLaunchArgument(
            'model_name',
            default_value='yolov5l-v7-coco',
            description='Voyager-built model directory name (under <voyager_sdk>/build/).',
        ),
        DeclareLaunchArgument(
            'media_file',
            default_value='traffic1_1080p.mp4',
            description='Filename under <voyager_sdk>/media/ to use as input.',
        ),
        DeclareLaunchArgument(
            'results_dir',
            default_value=os.path.expanduser('~/axros_results'),
            description='Directory to save annotated frames.',
        ),
        DeclareLaunchArgument(
            'save_every_n',
            default_value='1',
            description='Save 1 in every N synchronised frames (1 = save all).',
        ),
        DeclareLaunchArgument(
            'aipu_cores',
            default_value='4',
            description='Number of AIPU cores to allocate (1-4).',
        ),
        DeclareLaunchArgument(
            'conf_threshold',
            default_value='0.25',
            description='Detection confidence threshold.',
        ),
        DeclareLaunchArgument(
            'nms_threshold',
            default_value='0.45',
            description='Non-Maximum Suppression IoU threshold.',
        ),
        DeclareLaunchArgument(
            'use_live_camera',
            default_value=os.environ.get('USE_LIVE_CAMERA', '0'),
            description='Set to 1 to use a live USB camera instead of a video file.',
        ),
        OpaqueFunction(function=_launch_setup),
    ])
