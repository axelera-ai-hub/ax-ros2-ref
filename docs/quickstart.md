# ax-ros2-ref — Quick Start Guide

> ⚠️ **Reference implementation only.** This document is part of [`ax-ros2-ref`](../README.md), an experimental, unsupported reference implementation — **not** an Axelera product. It is provided as-is with no warranty or support, and APIs and behavior may change without notice.

## Contents

- [Prerequisites](#prerequisites)
- [Installation](#installation)
- [Running the Demo](#running-the-demo)
- [Verifying the Installation](#verifying-the-installation)
- [Environment Variables Reference](#environment-variables-reference)
- [Next Steps](#next-steps)

---

## Prerequisites

### Hardware

| Requirement | How to check |
| :---------- | :----------- |
| Axelera Metis card (M.2 or PCIe) installed | `ls /dev/metis*` → should show `/dev/metis-0:X:X` |
| Metis kernel driver loaded | `lsmod \| grep metis` → should show `metis` module |
| x86_64 host running Ubuntu 24.04 | `uname -m && lsb_release -rs` |
| PCIe x4 or M.2 2280 slot available | Physical inspection |

> ⚠️ **Warning:** This reference implementation targets **Ubuntu 24.04** with **ROS2 Jazzy Jalisco**. Ubuntu 22.04
> (ROS2 Humble) is not supported — the Python packaging, kernel version, and colcon toolchain
> differ in incompatible ways.

### Software dependencies

#### a) Axelera drivers + Voyager SDK (required first)

The [Voyager SDK](https://github.com/axelera-ai-hub/voyager-sdk) is Axelera's open-source
software stack. It provides everything the reference implementation depends on at runtime:

- **DKMS kernel driver** — creates the `/dev/metis` device node and handles PCIe DMA
- **AxRuntime C/C++ library** — the inference API used by `metis_ros2_inference`
- **RISC-V JIT toolchain** — cross-compiles on-device control kernels at model load time
- **Pre-compiled model zoo** — YOLOv5, YOLOv8, YOLOX and more, ready to run
- **Sample media** — `traffic1_1080p.mp4` and other test videos

Install the Voyager SDK before cloning `ax-ros2-ref`:

```bash
git clone https://github.com/axelera-ai-hub/voyager-sdk.git ~/voyager-sdk
cd ~/voyager-sdk
./install.sh --all --media
```

After installation, the following paths must exist:

```bash
/opt/axelera/runtime-1.x.x/
/opt/axelera/device-1.x.x/
/opt/axelera/riscv-gnu-newlib-toolchain-*/
```

> 💡 **Tip:** Run `axdevice` after installation to confirm the Metis card is detected.
> You should see output like `metis-0:00:00.0 Metis PCIe [214 TOPS]`.

#### b) ROS2 Jazzy Jalisco

Follow the [official ROS2 Jazzy installation guide](https://docs.ros.org/en/jazzy/Installation.html).
The minimum install is `ros-jazzy-ros-base`. Then add the perception packages `ax-ros2-ref` needs:

```bash
sudo apt install \
  ros-jazzy-vision-msgs \
  ros-jazzy-cv-bridge \
  ros-jazzy-image-transport \
  ros-jazzy-message-filters \
  ros-jazzy-foxglove-bridge
```

#### c) Build tools

```bash
sudo apt install \
  python3-colcon-common-extensions \
  libopencv-dev \
  pkg-config
```

---

## Installation

### Step 1 — Clone the repository

```bash
git clone https://github.com/axelera-ai-hub/ax-ros2-ref.git ~/ax-ros2-ref
```

### Step 2 — Inspect `workspace_setup.sh`

`workspace_setup.sh` is the single script that configures your shell for both ROS2 and
the Axelera runtime. Source it once per terminal session. Here is the complete file with
annotations:

```bash
#!/bin/bash

# ── 1. ROS2 ─────────────────────────────────────────────────────────────────
source /opt/ros/jazzy/setup.bash

# ── 2. Axelera environment ───────────────────────────────────────────────────
# AxRuntime: C/C++ headers + shared libraries (.so) for the inference node.
export AXELERA_RUNTIME_DIR="/opt/axelera/runtime-1.5.3-1"

# Device directory: firmware blobs uploaded to the Metis card at model load time.
export AXELERA_DEVICE_DIR="/opt/axelera/device-1.5.3-1/omega"

# RISC-V cross-compiler: compiles on-device control kernels at model load time (JIT).
export AXELERA_RISCV_TOOLCHAIN_DIR="/opt/axelera/riscv-gnu-newlib-toolchain-409b951ba662-7"

# Firmware ELF image loaded onto the AIPU at startup.
export AIPU_FIRMWARE_OMEGA="${AXELERA_DEVICE_DIR}/bin/start_axelera_runtime.elf"

# Stage-0 bootloader — initialises the AIPU before the main firmware runs.
export AIPU_RUNTIME_STAGE0_OMEGA="${AXELERA_DEVICE_DIR}/bin/start_axelera_runtime_stage0.bin"

export PATH="${AXELERA_RUNTIME_DIR}/bin:${AXELERA_RISCV_TOOLCHAIN_DIR}/bin:${PATH}"

# pkg-config: lets CMake find axruntime.pc at build time.
export PKG_CONFIG_PATH="${AXELERA_RUNTIME_DIR}/lib/pkgconfig:${PKG_CONFIG_PATH}"

# Shared library search path: axruntime + the ONNX Runtime used for post-processing.
export LD_LIBRARY_PATH="${AXELERA_RUNTIME_DIR}/lib:\
$HOME/voyager-sdk/operators/onnxruntime/onnxruntime-linux-x64-1.17.1/lib:\
${LD_LIBRARY_PATH}"

# ── 3. Workspace install overlay ─────────────────────────────────────────────
cd ~/ax-ros2-ref/ros2_ws
source install/setup.bash 2>/dev/null || true

# ── 4. Voyager SDK venv (Python nodes only — skip for C++ builds) ─────────
if [[ "${AXROS_ENABLE_VENV}" == "1" ]]; then
  cd ~/voyager-sdk && source venv/bin/activate
  export PYTHONPATH=$VIRTUAL_ENV/lib/python3.12/site-packages${PYTHONPATH:+:$PYTHONPATH}
fi
```

> 💡 **Tip:** If you are only building and running the C++ inference node, you do not need
> to activate the Voyager SDK Python venv. Set `AXROS_ENABLE_VENV=1` only if you are using
> Python-based pipeline scripts from the Voyager SDK.

### Step 3 — Build

```bash
source ~/ax-ros2-ref/workspace_setup.sh
cd ~/ax-ros2-ref/ros2_ws
colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release
```

<details>
<summary>Build a single package (faster iteration)</summary>

```bash
colcon build --packages-select metis_ros2_inference \
  --cmake-args -DCMAKE_BUILD_TYPE=Release

colcon build --packages-select axros_helper_nodes \
  --cmake-args -DCMAKE_BUILD_TYPE=Release
```

</details>

<details>
<summary>Debug build (verbose CMake output)</summary>

```bash
colcon build --packages-select metis_ros2_inference axros_helper_nodes \
  --cmake-args -DCMAKE_BUILD_TYPE=Debug \
  --event-handlers console_direct+
```

</details>

> ⚠️ **Warning:** Always source `workspace_setup.sh` **before** running `colcon build`.
> Without `PKG_CONFIG_PATH` set, CMake cannot find `axruntime.pc` and the build will fail
> with `Could not find package axruntime`.

---

## Running the Demo

### Prerequisites check

```bash
# Metis hardware detected
ls /dev/metis*
# Expected: /dev/metis-0:00:00.0

# Kernel driver loaded
lsmod | grep metis
# Expected: metis  <size>  0

# Model compiled and ready
ls ~/voyager-sdk/build/yolov5s-v7-coco/yolov5s-v7-coco/4/
# Expected: model.json  postprocess_graph.onnx
```

### Launch everything

```bash
source ~/ax-ros2-ref/workspace_setup.sh
ros2 launch metis_ros2_inference metis_inference.launch.py
```

This single command starts three nodes:

1. **`video_publisher`** — reads `~/voyager-sdk/media/traffic1_1080p.mp4` and publishes
   frames to `/image_raw` at the video's native FPS (capped to 3 FPS by default)
2. **`metis_inference`** — subscribes to `/image_raw`, runs YOLOv5s on the Metis AIPU,
   publishes `vision_msgs/Detection2DArray` to `/detections`
3. **`save_annotated`** — synchronises `/image_raw` with `/detections`, draws bounding boxes,
   and saves annotated PNGs to `~/ax-ros2-ref/results/`

### Expected startup output

```
[video_publisher]: VideoPublisherNode:
  video_path:    /home/<user>/voyager-sdk/media/traffic1_1080p.mp4
  output_topic:  image_raw
  fps:           3.00
  loop:          true
  resolution:    1920 x 1080
  total_frames:  900

[metis_inference]: Loading model: yolov5s-v7-coco
[metis_inference]: ONNX session created (postprocess_graph.onnx)
[metis_inference]: Metis inference node ready — publishing to /detections

[save_annotated]: Saving annotated frames to: /home/<user>/ax-ros2-ref/results
[save_annotated]: SaveAnnotatedNode:
  image_topic:       image_raw
  detections_topic:  detections
  annotated_topic:   camera_frame_annotated
  save_every_n:      12
  save_frames:       true
  publish_annotated: true
```

### Verify detections are publishing

In a second terminal (with the workspace sourced):

```bash
ros2 topic echo /detections --once
```

Expected output includes a `detections` array with bounding box centers, sizes, and class
hypotheses (e.g., `class_id: "car"`, `score: 0.87`).

### View annotated frames

```bash
ls ~/ax-ros2-ref/results/
# frame_000000.png  frame_000001.png  ...
```

Open any PNG to see the annotated output. Green boxes = high confidence (≥0.7),
orange = medium (≥0.4), red = low (<0.4).

---

## Verifying the Installation

Run through this checklist if anything is not working:

```bash
# 1. Hardware device node exists
ls /dev/metis*

# 2. Kernel module loaded
lsmod | grep metis

# 3. AxRuntime discoverable by pkg-config
pkg-config --modversion axruntime

# 4. ROS2 package visible
ros2 pkg list | grep -E 'metis|axros'

# 5. Composable nodes registered
ros2 component types | grep -E 'metis|axros'

# 6. Topics advertised after launch (run while launch is active)
ros2 topic list
# Expected: /image_raw  /detections  /camera_frame_annotated

# 7. Topic rates
ros2 topic hz /image_raw
ros2 topic hz /detections
```

<details>
<summary>Common errors and fixes</summary>

**`Could not find package axruntime`**
`PKG_CONFIG_PATH` is not set. Source `workspace_setup.sh` before building.

**`[ERROR] Failed to open video file`**
The `video_path` parameter points to a file that does not exist. Run
`ls ~/voyager-sdk/media/traffic1_1080p.mp4` to confirm the media was downloaded during
Voyager SDK installation (`./install.sh --all --media`).

**`[ERROR] No package 'ros-jazzy-vision-msgs' found`**
Run `sudo apt install ros-jazzy-vision-msgs` and rebuild.

**`/dev/metis*: No such file or directory`**
The Metis PCIe driver is not loaded. Check that the Voyager SDK DKMS driver was installed:
`dkms status | grep metis`. If not present, re-run `./install.sh --all` from the Voyager SDK.

**`sync_callback` never fires (save_annotated silent)**
The `ApproximateTimeSynchronizer` requires both `/image_raw` and `/detections` to have
matching timestamps within 0.1 s. Check that both topics are publishing with
`ros2 topic hz /image_raw` and `ros2 topic hz /detections`.

</details>

---

## Environment Variables Reference

All variables are set by `workspace_setup.sh`. The version strings shown here match SDK
release 1.5.3; adjust if you installed a different release.

| Variable | Default value | Purpose |
| :------- | :------------ | :------ |
| `AXELERA_RUNTIME_DIR` | `/opt/axelera/runtime-1.5.3-1` | AxRuntime headers and `.so` libraries; required by CMake `pkg-config` at build time and by the linker at runtime |
| `AXELERA_DEVICE_DIR` | `/opt/axelera/device-1.5.3-1/omega` | Firmware binaries and RISC-V kernel headers uploaded to the Metis card at model load time |
| `AXELERA_RISCV_TOOLCHAIN_DIR` | `/opt/axelera/riscv-gnu-newlib-toolchain-409b951ba662-7` | RISC-V cross-compiler used by AxRuntime to JIT-compile on-device control kernels |
| `AIPU_FIRMWARE_OMEGA` | `$AXELERA_DEVICE_DIR/bin/start_axelera_runtime.elf` | Main firmware ELF loaded onto the AIPU at `ctx.device_connect()` time |
| `AIPU_RUNTIME_STAGE0_OMEGA` | `$AXELERA_DEVICE_DIR/bin/start_axelera_runtime_stage0.bin` | Stage-0 bootloader that initialises the AIPU before the main firmware |
| `LD_LIBRARY_PATH` | `axruntime/lib` + `onnxruntime/lib` | Tells the dynamic linker where to find `libaxruntime.so` and `libonnxruntime.so` at process start |
| `PKG_CONFIG_PATH` | `axruntime/lib/pkgconfig` | Lets `colcon build` locate `axruntime.pc` so CMake can find include paths and link flags automatically |
| `AXROS_ENABLE_VENV` | _(unset)_ | Set to `1` before sourcing to also activate the Voyager SDK Python venv (needed for Python-based Voyager scripts, not for C++ node builds) |

---

## Next Steps

- **Change the model**: see [Model Swapping Guide](model_swapping.md)
- **Understand the architecture**: see [Overview](overview.md)
- **Visualise in Foxglove**: `ros2 launch foxglove_bridge foxglove_bridge_launch.xml` —
  subscribe to `/camera_frame_annotated` for a live annotated stream
- **Community support**: [Axelera Community Forum](https://community.axelera.ai)
