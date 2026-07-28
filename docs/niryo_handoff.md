# ax-ros2-ref Handoff for Niryo

> ⚠️ **Reference implementation only.** This document is part of [`ax-ros2-ref`](../README.md), an experimental, unsupported reference implementation — **not** an Axelera product. It is provided as-is with no warranty or support, and APIs and behavior may change without notice.

This document summarises the Axelera authored ROS2 packages in this repository and explains how to set the project up on a fresh machine so the work can be reproduced.

The repository targets ROS2 Jazzy on Ubuntu 24.04, with an Axelera Metis AIPU (M.2 or PCIe) for inference and a Niryo NED2 arm for the manipulation demos.

## Package overview

The Niryo driver packages (`niryo_ned_ros2_driver`, `niryo_ned_ros2_interfaces`, `niryo_ned_description`, and the `niryo_ned2_moveit_config` / `niryo_ned3pro_moveit_config` MoveIt configs) are your own packages and are vendored here as a dependency, so they are not described below.

| Package | Language | What it does |
| :------ | :------- | :----------- |
| `metis_ros2_inference` | C++ | Core inference node. Subscribes to `sensor_msgs/Image`, runs object detection on the Metis AIPU through the AxRuntime C API, runs the ONNX post process graph, and publishes `vision_msgs/Detection2DArray`. The runtime context and ONNX session are created once and reused on every frame, and inference runs on a dedicated worker thread so it never blocks the ROS executor. |
| `axros_helper_nodes` | C++ and Python | Utility nodes that surround the inference node. `VideoPublisherNode` publishes frames from a video file as `image_raw`; `SaveAnnotatedNode` syncs images with detections and writes annotated frames to disk; `LiveInferenceNode` renders a live camera feed with detection overlays; `demo_screen_node.py` is a viewer for demos. |
| `metis_moveit2_perception` | C++ and Python | The bridge from perception to arm control on the NED2. `DetectionToTargetNode` converts a `Detection2DArray` into a 3D `PoseStamped` target using a pinhole model and a table height assumption; `ArmControllerNode` plans and executes motion to those poses via the MoveIt MoveGroupInterface; `PickAndPlaceNode` runs the full pick and place state machine (detect, approach, descend, grip, verify, place) and drives the gripper through the Niryo `Tool` action. HSV color detectors (`red_detector_node.py`, `yellow_detector_node.py`) are included for non COCO objects. |
| `metis_pose_inference` | Python | Human pose pipeline. `PoseInferenceNode` runs a YOLO pose model on a second camera, back projects wrist keypoints onto a fixed plane in the world frame, and broadcasts `world` to `left_wrist` / `right_wrist` TFs plus a dwell gated `right_wrist_locked` TF. `WristTrackerNode` republishes the locked TF as a `PoseStamped` target that the arm controller consumes, enabling hand follow and hand drop demos. |

Typical data flow: a camera or video source feeds `image_raw`; `metis_ros2_inference` publishes `detections`; `metis_moveit2_perception` turns detections into arm targets and commands the NED2 through your driver and MoveIt configs.

## How to recreate it on your setup

### 1. Prerequisites

- Ubuntu 24.04 with ROS2 Jazzy installed.
- An Axelera Metis device (M.2 or PCIe) with the Voyager SDK installed. Follow the Voyager SDK instructions and run its installer with the media option so the sample models and videos are present.
- A Niryo NED2 (and your driver packages) for the manipulation demos. The inference and helper packages run without a robot.

### 2. Directory layout

The setup script expects the Voyager SDK to sit next to this repository, as a sibling directory:

```
<parent>/
  ax-ros2-ref/      this repository
  voyager-sdk/             the Voyager SDK checkout and install
```

Launch files resolve the SDK through `VOYAGER_SDK_DIR` (set by the setup script) and fall back to `~/voyager-sdk`.

### 3. Source the environment

```bash
source ax-ros2-ref/workspace_setup.sh
```

This sources ROS2 Jazzy, installs any missing ROS apt dependencies (vision_msgs, cv_bridge, MoveIt, and similar), sets the Axelera runtime and ONNX paths, and exposes `axruntime` to pkg-config. By default it does not activate the Voyager Python virtual environment, which is the correct mode for building (it keeps the system Python on the path for the build tooling).

### 4. Build

Always build from the `ros2_ws` directory, not the repository root:

```bash
cd ax-ros2-ref/ros2_ws
colcon build --symlink-install
```

On machines with limited memory, the MoveIt heavy `metis_moveit2_perception` can exhaust RAM when many packages compile in parallel. If the build is killed, limit parallelism:

```bash
MAKEFLAGS="-j2" colcon build --symlink-install --parallel-workers 1
```

### 5. Run

After a successful build, overlay the workspace:

```bash
source ax-ros2-ref/ros2_ws/install/setup.bash
```

The C++ inference and perception nodes run as is. The Python pose nodes import the Voyager SDK at runtime, so for those launches re enable the virtual environment first:

```bash
export AXROS_ENABLE_VENV=1
source ax-ros2-ref/workspace_setup.sh
```

Example launches:

```bash
# Object detection on a sample video, saving annotated frames
ros2 launch metis_ros2_inference metis_inference.launch.py

# NED2 pick and place (requires your driver and MoveIt config running)
PICK_CLASS=duck/cocolabel ros2 launch metis_moveit2_perception pick_and_place.launch.py
```

## Inference API: AxRuntime and AxInferenceNet

A note on how inference is wired in this project, and the alternative the Voyager SDK offers, so you can choose the right approach when you adapt this on your own setup.

The core detection node (`metis_ros2_inference`) is built directly on **AxRuntime**, the low level C and C++ runtime API. The node loads the compiled model, connects to the Metis device, and on every frame does its own preprocessing (letterbox resize, color convert, INT8 quantize), runs the AIPU, dequantizes the output tensors, and runs a small ONNX graph for the decode and NMS step. In other words, the node owns every stage of the pipeline.

The pose pipeline (`metis_pose_inference`) takes the other route: it opens the camera through `create_inference_stream` from the Voyager application layer, which is built on **AxInferenceNet**. There the full preprocess, inference, and postprocess graph is described by the model YAML (the model card) and executed by the SDK, so the node only consumes decoded results such as keypoints and boxes.

So the repository already demonstrates both styles. If you wanted to rebuild the core detection node on AxInferenceNet instead of AxRuntime, you would remove the manual pre and post processing code, point AxInferenceNet at the model card, and publish the decoded detections it returns.

### When to prefer each

**AxRuntime (the low level API):**

- Full control over preprocessing, postprocessing, threading, and buffer management; it drops cleanly into a custom C++ ROS2 node with your own queueing and timing.
- Small dependency footprint: only the runtime library plus pkg-config, with no GStreamer or Python application layer required, so it is easy to build and ship as a pure C++ component.
- Transparent and easy to debug at the tensor level, since you can see exactly what happens to each frame.
- Cost: more code, and you are responsible for the correctness of the preprocessing and decode steps for each model family.

**AxInferenceNet (the pipeline API):**

- Much less code: the preprocessing and the decoder for the model come from its YAML model card, so adding or swapping a model is mostly a configuration change.
- Optimized execution: the SDK runs the stages as a pipelined, multi threaded graph that overlaps host preprocessing with AIPU inference, which generally gives higher throughput and better device utilization out of the box.
- Built in decoders for common model families (the YOLO detectors and pose models used here), which reduces the chance of pre and post processing bugs.
- Cost: a heavier dependency surface (the Voyager application stack, GStreamer plugins, and the Python virtual environment), and less fine grained control over individual stages. This is why the pose nodes activate the Voyager venv at runtime.

Rule of thumb: choose AxRuntime when you need tight control or a minimal C++ footprint inside an existing ROS pipeline; choose AxInferenceNet when you want to integrate or swap models quickly and let the SDK handle the pipeline and throughput.

## Where to read more

- `docs/quickstart.md`: full installation walkthrough and environment reference.
- `docs/overview.md`: architecture and package descriptions.
- `docs/model_swapping.md`: how to change or compile models.
- `docs/ned2_guide.md`: the NED2 arm control demo end to end.
- `docs/pose_estimation.md`: the wrist tracking and hand drop demos.
