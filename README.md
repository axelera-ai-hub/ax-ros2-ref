# ax-ros2-ref

> ⚠️ **Experimental reference integration with ROS2.**
> `ax-ros2-ref` is a reference implementation provided as-is to demonstrate how the
> Axelera AIPU can be integrated with ROS2. It is **not** part of an Axelera product,
> and is **not** officially supported, and carries **no warranty or maintenance
> guarantees**. APIs, packages, and behavior may change or break without notice. Use it
> as a starting point and reference for your own integration.

**ROS2 Jazzy integration for [Axelera AIPUs](https://www.axelera.ai)**

This reference implementation connects the Metis AI processing unit (214 TOPS, 5 to 9 W, M.2/PCIe) to the ROS2
ecosystem. Drop it into any ROS2 robot stack to get real-time object detection, compatible with Nav2, MoveIt2, and Foxglove out of the box.

Built on the [Voyager SDK](https://github.com/axelera-ai-hub/voyager-sdk).

---

## Hardware at a glance

| | Axelera Metis M.2 | Jetson Orin Nano Super | Hailo-8 M.2 |
| :--- | :---: | :---: | :---: |
| Peak TOPS (INT8) | **214** | 67 | 26 |
| Typical power | **5–9 W** | 15–25 W | 2.5–5 W |
| ROS2 integration | **ax-ros2-ref** | Isaac ROS | Community |

---

## Tested versions

`ax-ros2-ref` has been tested and validated against the following versions:

| Component | Tested versions |
| :-------- | :-------------- |
| ROS2 | Jazzy Jalisco (on Ubuntu 24.04) |
| Voyager SDK / Axelera runtime | `1.5.3-1` and `1.6.0-1` |

The Voyager SDK version is pinned to `1.5.3-1` by default in
[`workspace_setup.sh`](workspace_setup.sh) (see line 54). The pin is overridable — export
`AXELERA_SDK_VERSION` before sourcing the script to select another installed version:

```bash
export AXELERA_SDK_VERSION=1.6.0-1
source ~/ax-ros2-ref/workspace_setup.sh
```

Other versions may work but are untested. As this is an experimental reference
implementation, no compatibility guarantees are made outside the versions listed above.

---

## Quick start (3 commands)

```bash
# 1. Clone
git clone https://github.com/axelera-ai-hub/ax-ros2-ref.git ~/ax-ros2-ref

# 2. Source the workspace (sets ROS2 + Axelera env vars)
source ~/ax-ros2-ref/workspace_setup.sh

# 3. Build and launch
cd ~/ax-ros2-ref/ros2_ws && colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release
ros2 launch metis_ros2_inference metis_inference.launch.py
```

> **Prerequisites:** [Voyager SDK](https://github.com/axelera-ai-hub/voyager-sdk) installed
> (`./install.sh --all --media`) and [ROS2 Jazzy](https://docs.ros.org/en/jazzy/Installation.html)
> on Ubuntu 24.04. See [docs/quickstart.md](docs/quickstart.md) for the full setup walkthrough.

---

## What runs

| ROS2 node | Subscribes | Publishes |
| :-------- | :--------- | :-------- |
| `video_publisher` | — | `/image_raw` (sensor_msgs/Image) |
| `metis_inference` | `/image_raw` | `/detections` (vision_msgs/Detection2DArray) |
| `save_annotated` | `/image_raw` + `/detections` | `/camera_frame_annotated` + PNG files |

Annotated frames are saved to `~/ax-ros2-ref/results/frame_XXXXXX.png`.

---

## NED2 Pick-and-Place Demo

The `metis_moveit2_perception` package extends the inference pipeline to drive a Niryo NED2
6-DOF robotic arm through full pick-and-place cycles: detect, approach, grip, verify, place.

| ROS2 node | Role |
| :-------- | :--- |
| `pick_and_place` | 14-state machine — detects pick/place targets, plans MoveIt2 trajectories, controls gripper, verifies grip |
| `yellow_detector` | HSV color detector for rubber duck (launched automatically with `PICK_CLASS=duck`) |
| `red_detector` | HSV color detector for red toys (launched automatically with `PICK_CLASS=red_toy`) |

```bash
# Pick a rubber duck, place on the cow mat
PICK_CLASS=duck ros2 launch metis_moveit2_perception pick_and_place.launch.py

# Pick a banana with position offset tuning
PICK_CLASS=banana PICK_OFFSET_Y=0.02 \
  ros2 launch metis_moveit2_perception pick_and_place.launch.py
```

See [docs/ned2_guide.md](docs/ned2_guide.md) for the complete setup and usage walkthrough.

---

## Documentation

| Document | Description |
| :------- | :---------- |
| [docs/overview.md](docs/overview.md) | Architecture, package descriptions, repository structure |
| [docs/quickstart.md](docs/quickstart.md) | Full installation walkthrough and environment reference |
| [docs/model_swapping.md](docs/model_swapping.md) | How to change models, compile custom models, normalization reference |
| [docs/ned2_guide.md](docs/ned2_guide.md) | NED2 arm control demo setup, launch, and troubleshooting |
| [docs/pose_estimation.md](docs/pose_estimation.md) | Pose inference and wrist tracking; duck and hand-drop demo |
| [docs/pi_tuning_guide.md](docs/pi_tuning_guide.md) | Tuning the reference implementation for Raspberry Pi 5 deployments |

---

## Licensing

The root [LICENSE](LICENSE) (MIT, © Axelera AI) covers repository-level files. It does
**not** blanket the whole tree: each ROS2 package declares its own license in its
`package.xml`, and bundled third-party code carries its own license file which governs
that subtree.

| Path | License |
| :--- | :--- |
| `ros2_ws/src/metis_ros2_inference/` | Apache-2.0 |
| `ros2_ws/src/metis_pose_inference/` | Apache-2.0 |
| `ros2_ws/src/axros_helper_nodes/` | Apache-2.0 |
| `ros2_ws/src/metis_moveit2_perception/` | MIT |
| `ros2_ws/src/ned-ros2-driver/` | BSD-3-Clause, © Niryo Robotics — bundled and modified |

See [THIRD_PARTY_NOTICES.md](THIRD_PARTY_NOTICES.md) for the origin, version, and record
of modifications to bundled third-party code.

---

## Contributing

Contributions are welcome! Please read [CONTRIBUTING.md](CONTRIBUTING.md) before opening a
pull request. All contributions are made under the [Developer Certificate of Origin](DCO),
so every commit must be signed off (`git commit -s`).

---

## Links

- [Axelera AI](https://www.axelera.ai) — product page and company info
- [Voyager SDK](https://github.com/axelera-ai-hub/voyager-sdk) — runtime, driver, model zoo
- [Axelera Community Forum](https://community.axelera.ai) — technical support and blog posts
- [Axelera Customer Portal](https://support.axelera.ai) — firmware and technical guides
