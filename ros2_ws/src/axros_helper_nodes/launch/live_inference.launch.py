# (C) Copyright Axelera AI 2026
# All Rights Reserved
"""
AxROS live-inference launcher.

Supports one or two camera streams:
  - Main camera (default /dev/video2, 1920x1080 @ 30fps) — always launched.
    YOLO optionally runs on it (disabled by SKIP_YOLO=1).
  - Hand camera (default /dev/video2, MJPG 1280x720 @ 60fps) — opted in with
    ENABLE_HAND_CAMERA=1. YOLO always runs on this stream when enabled, with
    its own output topic (/hand_detections) so it stays decoupled from the
    main detector's /detections.

Every topic the display syncs on (image_raw, detections, camera_frame_annotated,
/color_detections) is remapped for the hand camera so the two displays never
mix up streams.

Environment variables:
    SKIP_YOLO              — '1' to skip YOLO on main camera. Default: 0
    MAIN_CAMERA_DEVICE     — default /dev/video2
    MAIN_CAMERA_FOURCC     — capture FourCC for main camera, e.g. 'MJPG'.
                             Default: '' (driver picks; YUYV on most webcams).
                             Set to 'MJPG' for high-res over USB 2.0.
    MAIN_CAMERA_DISPLAY    — '1' (default) to open the AxROS preview window
                             and republish camera_frame_annotated; '0' to
                             skip both (recommended on Pi — keeps the 30 Hz
                             capture from being starved by display work).
    MAIN_WINDOW_NAME       — window title. Default 'AxROS Live Inference'
    ENABLE_HAND_CAMERA     — '1' to also launch the hand camera+YOLO. Default: 0
    HAND_CAMERA_DEVICE     — default /dev/video2
    HAND_CAMERA_WIDTH      — default 1280
    HAND_CAMERA_HEIGHT     — default 720
    HAND_CAMERA_FPS        — default 60.0
    HAND_CAMERA_FOURCC     — default 'MJPG'
    HAND_WINDOW_NAME       — window title. Default 'AxROS Live Inference'
    DEMO_SCREEN            — '1' to launch the demo_screen_node viewer
                             (Axelera partner-demo template wrapping the live
                             inference feed via cv::imshow). Default: 0.
                             When on, set MAIN_CAMERA_DISPLAY=0 so the
                             cv::imshow window in live_inference_node doesn't
                             fight for the screen.
    DEMO_SCREEN_FULLSCREEN — '1' to put the OpenCV demo window in fullscreen
                             mode. Default: 0.

Secondary model on the main camera stream (e.g. kitkat-YOLO + hand-YOLO
simultaneously). When MODEL_NAME_2 is set, a second metis_inference_node
is launched on the SAME image_raw input. Its detections are merged into
/detections by default, so pick_and_place_node — which filters by
class_id string — sees both models' outputs in one stream. Metis has 4
AIPU cores; each model takes 1 by default, so two models coexist with
spare headroom.

    MODEL_NAME_2           — directory under voyager-sdk/build/ for the
                             secondary model. Unset = no secondary model.
    MODEL_SUBDIR_2         — subdir inside MODEL_NAME_2/. Defaults to
                             MODEL_NAME_2 (same-name nested layout).
    LABELS_PATH_2          — labels file for the secondary model.
    CONF_THRESHOLD_2       — secondary model conf threshold. Default 0.50.
    NMS_THRESHOLD_2        — secondary model NMS IoU. Default 0.45.
    AIPU_CORES_2           — cores reserved for the secondary model.
                             Default 1.
"""

import os
from launch import LaunchDescription
from launch.actions import TimerAction
from launch_ros.actions import Node

# Voyager SDK location is set by experimental.axros/workspace_setup.sh; falls
# back to ~/voyager-sdk if the script wasn't sourced. Same pattern as
# metis_inference.launch.py and video_publisher.launch.py.
VOYAGER_SDK = os.environ.get(
    'VOYAGER_SDK_DIR',
    os.path.expanduser('~/voyager-sdk'),
)
MODEL_NAME   = os.environ.get('MODEL_NAME',   'yolov5l-v7-coco')
# Some Voyager builds land under build/<parent>/<actual>/1/ rather than the
# nested build/<name>/<name>/1/ default — e.g. hand-detection ships as
# build/hand-detection/yolov10n-coco-onnx/1/. MODEL_SUBDIR lets you point
# at that without renaming directories. Defaults to MODEL_NAME for the
# nested layout so existing launches keep working.
MODEL_SUBDIR = os.environ.get('MODEL_SUBDIR', MODEL_NAME)
MODEL_DIR    = os.path.join(VOYAGER_SDK, 'build', MODEL_NAME, MODEL_SUBDIR, '1')
LABELS       = os.environ.get(
    'LABELS_PATH',
    os.path.join(VOYAGER_SDK, 'ax_datasets', 'labels', 'coco.names'),
)

SKIP_YOLO          = os.environ.get('SKIP_YOLO', '0') == '1'
MAIN_CAMERA_DEVICE  = os.environ.get('MAIN_CAMERA_DEVICE',  '/dev/video0')
MAIN_CAMERA_FOURCC  = os.environ.get('MAIN_CAMERA_FOURCC',  '')
MAIN_CAMERA_DISPLAY = os.environ.get('MAIN_CAMERA_DISPLAY', '1') == '1'
MAIN_WINDOW_NAME    = os.environ.get('MAIN_WINDOW_NAME',    'AxROS Live Inference')

ENABLE_HAND_CAMERA = os.environ.get('ENABLE_HAND_CAMERA', '0') == '1'
HAND_CAMERA_DEVICE = os.environ.get('HAND_CAMERA_DEVICE', '/dev/video2')
HAND_CAMERA_WIDTH  = int(os.environ.get('HAND_CAMERA_WIDTH',  '1280'))
HAND_CAMERA_HEIGHT = int(os.environ.get('HAND_CAMERA_HEIGHT', '720'))
HAND_CAMERA_FPS    = float(os.environ.get('HAND_CAMERA_FPS',  '60.0'))
HAND_CAMERA_FOURCC = os.environ.get('HAND_CAMERA_FOURCC', 'MJPG')
HAND_WINDOW_NAME   = os.environ.get('HAND_WINDOW_NAME',   'AxROS Live Inference')

# ── Demo screen (cv::imshow) ────────────────────────────────────────────────
# Off by default — opt in with DEMO_SCREEN=1. When on, demo_screen_node.py
# renders the Axelera partner-demo template around the live feed and shows
# it via cv::imshow. Footer asset paths (logo, QR, chip icon) are hardcoded
# in the node and not configurable here.
DEMO_SCREEN            = os.environ.get('DEMO_SCREEN',      '0') == '1'
DEMO_SCREEN_FULLSCREEN = os.environ.get('DEMO_SCREEN_FULLSCREEN', '0') == '1'

# ── Secondary model (optional) ──────────────────────────────────────────────
# When MODEL_NAME_2 is set, a second metis_inference_node runs on the SAME
# main-camera stream, publishing to /detections alongside the primary model.
# Used by pick_and_place_pi when KITKAT_DETECTOR=yolo and PLACE_CLASS=palm-style
# to run kitkat-detection + hand-detection simultaneously, each on 1 AIPU core.
MODEL_NAME_2   = os.environ.get('MODEL_NAME_2', '')
MODEL_SUBDIR_2 = os.environ.get('MODEL_SUBDIR_2', MODEL_NAME_2)
MODEL_DIR_2    = (
    os.path.join(VOYAGER_SDK, 'build', MODEL_NAME_2, MODEL_SUBDIR_2, '1')
    if MODEL_NAME_2 else ''
)
LABELS_2       = os.environ.get(
    'LABELS_PATH_2',
    os.path.join(VOYAGER_SDK, 'ax_datasets', 'labels', 'coco.names'),
)


def _yolo_params(input_topic: str, output_topic: str,
                 model_name:    str   = None,
                 model_dir:     str   = None,
                 labels_path:   str   = None,
                 conf_threshold: float = None,
                 nms_threshold:  float = None,
                 aipu_cores:    int   = 1) -> dict:
    """
    Build the parameter dict for a metis_inference_node. Defaults match the
    primary model env vars (MODEL_NAME / MODEL_DIR / LABELS / CONF_THRESHOLD)
    so existing call sites keep working unchanged. Pass model_name/etc. to
    point a *different* instance at a second model on the same camera.
    """
    return {
        'model_name':     model_name if model_name is not None else MODEL_NAME,
        'model_dir':      model_dir  if model_dir  is not None else MODEL_DIR,
        'labels_path':    labels_path if labels_path is not None else LABELS,
        'aipu_cores':     aipu_cores,
        'input_topic':    input_topic,
        'output_topic':   output_topic,
        'conf_threshold': conf_threshold if conf_threshold is not None
                          else float(os.environ.get('CONF_THRESHOLD', '0.65')),
        'nms_threshold':  nms_threshold if nms_threshold is not None
                          else float(os.environ.get('NMS_THRESHOLD',  '0.45')),
        'normalize':      False,
        'mean':           [0.485, 0.456, 0.406],
        'stddev':         [0.229, 0.224, 0.225],
    }


def generate_launch_description():
    nodes = []

    # ── Main camera ─────────────────────────────────────────────────────────
    nodes.append(Node(
        package='axros_helper_nodes',
        executable='live_inference_node_exe',
        name='live_inference',
        output='screen',
        parameters=[{
            'camera_device':   MAIN_CAMERA_DEVICE,
            'output_topic':    'image_raw',
            'fps':             30.0,
            'frame_id':        'camera',
            'summary_every_s': 10,
            'window_name':     MAIN_WINDOW_NAME,
            'frame_width':     1920,
            'frame_height':    1080,
            'fourcc':          MAIN_CAMERA_FOURCC,
            'enable_display':  MAIN_CAMERA_DISPLAY,
        }],
    ))

    # ── Main-camera YOLO (optional) ─────────────────────────────────────────
    if not SKIP_YOLO:
        nodes.append(Node(
            package='metis_ros2_inference',
            executable='metis_inference_node',
            name='metis_inference',
            output='screen',
            parameters=[_yolo_params('image_raw', 'detections')],
        ))

        # ── Secondary main-camera model (optional, e.g. kitkat + hand) ──────
        # Same image_raw input, same /detections output. The two models'
        # detections are merged into one Detection2DArray stream because
        # pick_and_place_node filters by class_id string, not by source.
        #
        # IMPORTANT — secondary model is delayed (default 3 s) before
        # axr_device_connect / axr_load_model_instance fire. Loading both
        # models concurrently against the same Metis device races the
        # kernel-side DMA buffer setup and intermittently triggers
        # "DMABUF_METIS_XFER failed: Input/output error" /
        # "Level-zero memory operation failed: 0x70010001", killing both
        # processes. The primary model is small enough to fully load in
        # <2 s on Pi, so a 3 s stagger reliably lets it finish first.
        # Override with INFERENCE_2_START_DELAY_S=<seconds> if you need
        # to tune it on a different host.
        if MODEL_NAME_2:
            secondary_node = Node(
                package='metis_ros2_inference',
                executable='metis_inference_node',
                name='metis_inference_2',
                output='screen',
                parameters=[_yolo_params(
                    'image_raw', 'detections',
                    model_name=MODEL_NAME_2,
                    model_dir=MODEL_DIR_2,
                    labels_path=LABELS_2,
                    conf_threshold=float(os.environ.get('CONF_THRESHOLD_2', '0.50')),
                    nms_threshold=float(os.environ.get('NMS_THRESHOLD_2',  '0.45')),
                    aipu_cores=int(os.environ.get('AIPU_CORES_2', '1')),
                )],
            )
            secondary_delay = float(
                os.environ.get('INFERENCE_2_START_DELAY_S', '3.0'))
            nodes.append(TimerAction(
                period=secondary_delay,
                actions=[secondary_node]))

    # ── Hand camera (second stream for person/hand view) ────────────────────
    if ENABLE_HAND_CAMERA:
        # The node internally subscribes to hardcoded "image_raw" and
        # "detections" for its display overlay. We remap those so the hand
        # display syncs against the hand topics, not the main ones.
        # /color_detections is also hardcoded in the node → remap to a dead
        # topic so the color detector's crosshairs don't leak onto the
        # hand view.
        nodes.append(Node(
            package='axros_helper_nodes',
            executable='live_inference_node_exe',
            name='hand_live_inference',
            output='screen',
            parameters=[{
                'camera_device':   HAND_CAMERA_DEVICE,
                'output_topic':    'image_raw',   # remapped below
                'fps':             HAND_CAMERA_FPS,
                'frame_id':        'hand_camera',
                'summary_every_s': 10,
                'window_name':     HAND_WINDOW_NAME,
                'frame_width':     HAND_CAMERA_WIDTH,
                'frame_height':    HAND_CAMERA_HEIGHT,
                'fourcc':          HAND_CAMERA_FOURCC,
            }],
            remappings=[
                ('image_raw',              'hand_image_raw'),
                ('detections',             'hand_detections'),
                ('camera_frame_annotated', 'hand_camera_frame_annotated'),
                ('/color_detections',      '/hand_color_detections'),
            ],
        ))

        nodes.append(Node(
            package='metis_ros2_inference',
            executable='metis_inference_node',
            name='hand_metis_inference',
            output='screen',
            parameters=[_yolo_params('hand_image_raw', 'hand_detections')],
        ))

    # ── Demo screen (cv::imshow) ────────────────────────────────────────────
    # Optional partner-demo viewer. Subscribes to image_raw / detections /
    # color_detections / pick_roi_polygon, draws the same overlays as the
    # C++ live_inference window, and shows the result in the Axelera
    # partner-demo template via cv::imshow. Footer asset paths are
    # hardcoded inside the node and not configurable from here.
    if DEMO_SCREEN:
        nodes.append(Node(
            package='axros_helper_nodes',
            executable='demo_screen_node.py',
            name='demo_screen',
            output='screen',
            parameters=[{
                'window_fullscreen': DEMO_SCREEN_FULLSCREEN,
            }],
        ))

    return LaunchDescription(nodes)
