# ax-ros2-ref — Overview

> ⚠️ **Reference implementation only.** This document is part of [`ax-ros2-ref`](../README.md), an experimental, unsupported reference implementation — **not** an Axelera product. It is provided as-is with no warranty or support, and APIs and behavior may change without notice.

## Contents

- [What is ax-ros2-ref?](#what-is-ax-ros2-ref)
- [Why Metis + ROS2?](#why-metis--ros2)
- [Architecture](#architecture)
- [Repository Structure](#repository-structure)
- [Links](#links)

---

## What is ax-ros2-ref?

ax-ros2-ref is a [ROS2](https://docs.ros.org/en/jazzy/) integration layer for the
[Axelera Metis AIPU](https://www.axelera.ai), a 214 TOPS, 5–9 W AI processing unit
available in M.2 and PCIe form factors. It exposes Metis inference directly as composable
ROS2 nodes, publishing standard [`vision_msgs/Detection2DArray`](https://github.com/ros-perception/vision_msgs)
messages that any downstream ROS2 stack (Nav2, MoveIt2, Foxglove, custom nodes) can consume
without modification. The goal is to position Metis as an open, hardware-agnostic alternative
to NVIDIA Isaac ROS: bring your own model, run it on any Linux robot, and keep the full ROS2
ecosystem intact.

The runtime is built on the [Voyager SDK](https://github.com/axelera-ai-hub/voyager-sdk),
Axelera's open-source software stack that provides the DKMS kernel driver, AxRuntime C/C++
library, RISC-V JIT toolchain, and a pre-compiled model zoo.

---

## Why Metis + ROS2?

### Hardware comparison

| Specification        | Axelera Metis M.2     | Jetson Orin Nano Super | Hailo-8 M.2   |
| :------------------- | :-------------------- | :--------------------- | :------------ |
| Peak TOPS (INT8)     | **214**               | 67                     | 26            |
| Typical power        | **5–9 W**             | 15–25 W                | 2.5–5 W       |
| Efficiency (TOPS/W)  | **~24**               | ~2.7                   | 8.7–10.4      |
| ROS2 integration     | **ax-ros2-ref (this repo)** | Isaac ROS (NVIDIA)     | Community     |
| Host OS lock-in      | **None** (Ubuntu 24.04) | Jetpack / L4T        | None          |

### Why not Isaac ROS?

- **No CUDA lock-in.** Isaac ROS requires NVIDIA hardware throughout your stack. This reference implementation runs on
  any x86_64 or aarch64 host with a PCIe or M.2 slot.
- **Lower power envelope.** Metis delivers 3–5× the TOPS at a fraction of the wattage,
  enabling mobile and battery-constrained platforms.
- **Simpler deployment.** One `source workspace_setup.sh` and one `colcon build` — no JetPack,
  no CUDA toolkit, no proprietary container registry required.

---

## Architecture

### Processing stack

```
┌─────────────────────────────────────────────┐
│  Camera / Video Source                      |
│  (USB camera · CSI · video file)            |
└────────────────────┬────────────────────────┘
                     │  sensor_msgs/Image
                     ▼
┌─────────────────────────────────────────────┐
│  metis_ros2_inference                       │
│  AxRuntime → Metis AIPU (214 TOPS, 5–9 W)   │
└────────────────────┬────────────────────────┘
                     │  vision_msgs/Detection2DArray
                     ▼
┌─────────────────────────────────────────────┐
│  DetectionToTargetNode                      │
│  Selects best detection → 3D target pose    │
└────────────────────┬────────────────────────┘
                     │  geometry_msgs/PoseStamped
                     ▼
┌─────────────────────────────────────────────┐
│  ArmControllerNode                          │
│  MoveIt2 motion planning + execution        │
└────────────────────┬────────────────────────┘
                     │  MoveIt2 trajectory
                     ▼
┌─────────────────────────────────────────────┐
│  Niryo NED2                                 │
│  6-DOF robotic arm (real or simulated)      │
└─────────────────────────────────────────────┘
```

### Package descriptions

| Package | Language | Description |
| :------ | :------- | :---------- |
| `metis_ros2_inference` | C++ | Core composable inference node. Reads `sensor_msgs/Image`, runs the model on the Metis AIPU via AxRuntime, publishes `vision_msgs/Detection2DArray`. ONNX post-processing session created once in the constructor and reused every frame. |
| `axros_helper_nodes` | C++ | Utility composable nodes. `VideoPublisherNode` reads a video file and publishes frames at native FPS. `SaveAnnotatedNode` subscribes to image + detections, draws bounding boxes, and saves annotated PNGs to disk. |
| `metis_moveit2_perception` | C++ | MoveIt2 perception bridge. `DetectionToTargetNode` converts `Detection2DArray` into a `geometry_msgs/PoseStamped` target. `ArmControllerNode` plans and executes pick motions via MoveIt2. `PickAndPlaceNode` runs the full pick-and-place state machine, with three place-target sources: `detection`, `fixed`, and `tf` (consumes the wrist TF for hand-drop demos). |
| `metis_pose_inference` | Python | Human-pose pipeline. `PoseInferenceNode` runs `yolo11lpose-coco-onnx` on a second BRIO camera and broadcasts `world → right_wrist` / `left_wrist` TFs plus a stationary-target `right_wrist_locked` TF gated by a dwell state machine. `WristTrackerNode` republishes the locked TF as a `geometry_msgs/PoseStamped` target. |
| `niryo_ned2_moveit_config` | Config | MoveIt2 configuration for the Niryo NED2 6-DOF arm. URDF/SRDF, kinematics, joint limits, Pilz planner cartesian limits, and ros2_control hardware interface definitions. Part of `ned-ros2-driver`. |

---

## Repository Structure

```
ax-ros2-ref/
├── README.md                        — Landing page and quick-start
├── LICENSE                          — Apache-2.0
├── CLAUDE.md                        — Claude Code project context (internal)
├── workspace_setup.sh               — Sources ROS2, Axelera env vars, workspace overlay
├── docs/
│   ├── overview.md                  — This file
│   ├── quickstart.md                — Installation and first run
│   ├── model_swapping.md            — How to change or add models
│   ├── ned2_guide.md                — NED2 arm control demo guide
│   └── pose_estimation.md           — Pose-tracking + duck/hand-drop demo guide
└── ros2_ws/
    └── src/
        ├── metis_ros2_inference/
        │   ├── CMakeLists.txt
        │   ├── package.xml
        │   ├── include/metis_ros2_inference/
        │   │   └── metis_inference_node.hpp
        │   ├── src/
        │   │   ├── metis_inference_node.cpp   — Core inference logic
        │   │   └── metis_inference_node_main.cpp
        │   └── launch/
        │       └── metis_inference.launch.py  — Full pipeline launch
        ├── axros_helper_nodes/
        │   ├── CMakeLists.txt
        │   ├── package.xml
        │   ├── include/axros_helper_nodes/
        │   │   ├── video_publisher_node.hpp
        │   │   └── save_annotated_node.hpp
        │   ├── src/
        │   │   ├── video_publisher_node.cpp   — Video file → image_raw
        │   │   └── save_annotated_node.cpp    — Annotated frame saver
        │   └── launch/
        │       ├── video_publisher.launch.py
        │       └── save_annotated.launch.py
        ├── metis_moveit2_perception/
        │   ├── CMakeLists.txt
        │   ├── package.xml
        │   ├── src/
        │   │   ├── detection_to_target_node.cpp — Detection → PoseStamped
        │   │   ├── arm_controller_node.cpp      — MoveIt2 pick execution
        │   │   └── pick_and_place_node.cpp      — Pick-place state machine (detection / fixed / tf place modes)
        │   ├── launch/
        │   │   ├── perception_arm.launch.py     — Perception + arm pipeline
        │   │   ├── pick_and_place.launch.py     — Pick-place demo (per-object profiles)
        │   │   └── demo.launch.py               — Full NED2 demo launch
        │   └── scripts/
        │       ├── yellow_detector_node.py      — HSV duck detector (CPU)
        │       ├── red_detector_node.py         — HSV red-toy detector (CPU)
        │       └── test_safe_motion.sh          — Safe motion smoke test
        ├── metis_pose_inference/
        │   ├── pyproject.toml
        │   ├── package.xml
        │   ├── metis_pose_inference/
        │   │   ├── pose_inference_node.py       — yolo11lpose @ AIPU + lock state machine
        │   │   └── wrist_tracker_node.py        — locked TF → /target_hover_pose
        │   └── launch/
        │       ├── wrist_tracking_test.launch.py — Three-terminal validation flow
        │       ├── wrist_tracking_full.launch.py — Single-terminal pose tracking
        │       └── duck_hand_demo.launch.py      — Combined duck pick + hand-drop demo
        └── ned-ros2-driver/
            ├── niryo_ned_description/           — URDF, meshes, RViz config
            ├── niryo_ned_moveit_configs/        — MoveIt2 configs (NED2 + NED3 Pro)
            └── niryo_ned_ros2_driver/           — ROSBridge driver node
```

---



## Links

### ax-ros2-ref documentation

- [Quick Start Guide](quickstart.md) — Get from zero to running detections
- [Model Swapping Guide](model_swapping.md) — Change or add models
- [NED2 Arm Control Demo](ned2_guide.md) — End-to-end pick demo with Niryo NED2
- [Pose Estimation Guide](pose_estimation.md) — Wrist tracking + duck/hand-drop demo
- [Raspberry Pi 5 Tuning Guide](pi_tuning_guide.md): tuning the reference implementation for Raspberry Pi 5 deployments

### External resources

- [Axelera AI](https://www.axelera.ai) — Company website, product page
- [Voyager SDK](https://github.com/axelera-ai-hub/voyager-sdk) — Runtime, driver, model zoo
- [Voyager SDK model zoo](https://github.com/axelera-ai-hub/voyager-sdk/tree/main/ax_models/zoo) — Pre-compiled models
- [ROS2 Jazzy documentation](https://docs.ros.org/en/jazzy/) — Official ROS2 Jazzy docs
- [vision_msgs](https://github.com/ros-perception/vision_msgs) — Standard perception message types
- [Axelera Community Forum](https://community.axelera.ai) — Technical support and blog posts
- [Axelera Customer Portal](https://support.axelera.ai) — Technical documents and guides
