# ax-ros2-ref — Niryo NED2 Pick-and-Place Guide

> ⚠️ **Reference implementation only.** This document is part of [`ax-ros2-ref`](../README.md), an experimental, unsupported reference implementation — **not** an Axelera product. It is provided as-is with no warranty or support, and APIs and behavior may change without notice.

## Contents

- [Introduction](#introduction)
- [What You Need](#what-you-need)
- [How It Works](#how-it-works)
- [Setup — Simulation (No Robot Required)](#setup--simulation-no-robot-required)
- [Setup — Real Robot](#setup--real-robot)
  - [Step 1: First-Time Robot Setup](#step-1-first-time-robot-setup)
  - [Step 2: Network and Clock](#step-2-network-and-clock)
  - [Step 3: Verify the Connection](#step-3-verify-the-connection)
  - [Step 4: First Safe Motion](#step-4-first-safe-motion)
  - [Step 5: Camera Calibration](#step-5-camera-calibration)
  - [Step 6: Full Pipeline](#step-6-full-pipeline)
- [Running the Demo](#running-the-demo)
- [Pick Profiles](#pick-profiles)
- [Color Detectors](#color-detectors)
- [Environment Variables Reference](#environment-variables-reference)
- [Tuning Parameters](#tuning-parameters)
- [Troubleshooting](#troubleshooting)

---

## Introduction

This guide walks you through setting up and running the reference implementation's pick-and-place demo: a Niryo NED2
6-axis robot arm that detects objects with a camera powered by the Axelera Metis AIPU, picks
them up, and places them on a target surface.

The full pipeline is:

1. A USB camera feeds 1080p frames to a YOLOv5 model running on the Metis AIPU at 5+ FPS
2. Detections are published as `vision_msgs/Detection2DArray`
3. The `PickAndPlaceNode` tracks the pick object, plans trajectories with MoveIt2, descends
   with a straight Cartesian path, grips, lifts, then moves to the place target
4. A visual grip-verification step checks whether the object was successfully picked before
   proceeding to place — if not, it retries at the original position

The system works in two modes:

- **Simulation** — runs in software. The arm moves in RViz. No robot hardware needed.
- **Real robot** — the physical NED2 arm executes live pick-and-place cycles.

---

## What You Need

### For simulation only

| Requirement | Details |
| :---------- | :------ |
| Ubuntu 24.04 host | x86_64 or aarch64 |
| ROS2 Jazzy installed | `sudo apt install ros-jazzy-desktop` |
| Voyager SDK installed | [Installation guide](quickstart.md) |
| ax-ros2-ref workspace built | `colcon build` from `ros2_ws/` |
| ros2_control packages | `sudo apt install ros-jazzy-controller-manager ros-jazzy-joint-trajectory-controller ros-jazzy-joint-state-broadcaster` |

### For real robot (in addition to above)

| Requirement | Details |
| :---------- | :------ |
| Niryo NED2 robot | Powered on, calibrated, connected via Ethernet |
| USB camera | Logitech C925e or similar, mounted above the workspace pointing straight down |
| Metis AIPU card | Installed in the host machine |
| Pick objects | Physical objects from the COCO dataset or the supported pick profiles |
| Place target | A large flat object the model can detect (default: `cow` mat) |

---

## How It Works

```
USB Camera
    |
    |  sensor_msgs/Image (1920×1080, 30 FPS)
    v
Metis Inference Node (YOLOv5 on Metis AIPU, 214 TOPS)
    |
    |  vision_msgs/Detection2DArray  (/detections)
    |
    +── [optional] Yellow Detector Node  ─── duck detections
    +── [optional] Red Detector Node     ─── red_toy detections
    v
PickAndPlaceNode
    |  State machine (14 states):
    |
    |  DETECTING_PICK
    |    ↓  EMA-filtered target with MIN_PICK_DETECTIONS convergence
    |  APPROACHING_PICK   → hover above pick target (joint-space plan)
    |  OPENING_GRIPPER
    |  DESCENDING_TO_PICK → straight Cartesian descent to grasp height
    |  GRIPPING
    |  LIFTING            → straight up to lift height
    |  VERIFYING_GRIP     → arm moves clear; camera checks if object still on table
    |    ↓ success              ↓ failed (object still visible near pick spot)
    |  DETECTING_PLACE    APPROACHING_PICK (retry at original position)
    |  APPROACHING_PLACE
    |  DESCENDING_TO_PLACE
    |  RELEASING
    |  RETRACTING
    |  DONE
    v
NED2 Robot (simulation or real hardware)
```

### Key design decisions

| Decision | Reason |
| :------- | :----- |
| Cartesian descent (computeCartesianPath) | Joint-space OMPL plans can dip below the target Z mid-trajectory; Cartesian planning guarantees a straight vertical path |
| EMA position filtering | Smooths out per-frame bbox jitter before committing to a pick position |
| MIN_PICK_DETECTIONS = 10 | Ensures the EMA has converged (~2 s at 5 FPS inference) before the arm moves |
| Grip verification | Arm moves to a clear pose; camera checks the pick spot for up to 3 s; detection near the gripper is filtered (that's the held object, not a missed pick) |
| Original-position restore on retry | Re-detection during verification may see the held object at the gripper position — the retry always uses the pre-lift pick location |

---

## Setup — Simulation (No Robot Required)

### Build

```bash
source ~/ax-ros2-ref/workspace_setup.sh
cd ~/ax-ros2-ref/ros2_ws
colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release
```

### Run the simulation demo

```bash
source ~/ax-ros2-ref/workspace_setup.sh
source install/setup.bash
ros2 launch metis_moveit2_perception demo.launch.py
```

This launches three stages automatically with delays between them:

| Time | Stage | What starts |
| :--- | :---- | :---------- |
| 0 s | Stage 1 | MoveIt2 + RViz + simulated robot (mock hardware) |
| 12 s | Stage 2 | Camera inference (Metis AIPU) + Live Inference window |
| 20 s | Stage 3 | Pick-and-place node + optional color detector |

After ~25 seconds the arm in RViz will begin executing pick-and-place cycles toward detected objects.

---

## Setup — Real Robot

### Step 1: First-Time Robot Setup

This step requires **physical access** to the robot. It only needs to be done once.

1. **Power on the NED2** — press the power button, wait 1-2 minutes for boot
2. **Connect via Ethernet** — plug the robot into the same network as the host machine
3. **Find the robot** — check the label on the robot for the SSID (e.g., `Niryo Hotspot f2-f85-bb2`). The hostname is derived from this: `ned2-f2-f85-bb2.local`
4. **Test the connection**:
   ```bash
   ping ned2-f2-f85-bb2.local
   ```
5. **Calibrate via Niryo Studio** — download [Niryo Studio](https://niryo.com/niryostudio/),
   connect to the robot, and run auto-calibration.

> [!WARNING]
> Someone must be physically present during calibration — the robot moves unpredictably.

> [!NOTE]
> If the robot was set up on its WiFi hotspot first, configure it to use Ethernet in
> Niryo Studio before proceeding.

### Step 2: Network and Clock

The robot's clock must match the host machine. SSH in and fix the timezone:

```bash
ssh niryo@ned2-f2-f85-bb2.local
# Password: robotics

date                                    # Check current time
timedatectl set-timezone Europe/Zurich  # Set to your timezone
sudo service niryo_robot_ros restart    # Restart the robot's ROS1 stack
exit
```

### Step 3: Verify the Connection

Launch the ROSBridge driver and check that real joint states come through:

**Terminal 1:**
```bash
source ~/ax-ros2-ref/workspace_setup.sh
source install/setup.bash
ros2 launch niryo_ned_ros2_driver driver.launch.py
```

**Terminal 2:**
```bash
source ~/ax-ros2-ref/workspace_setup.sh
source install/setup.bash

# Should show 6 joint positions from the real robot
ros2 topic echo /joint_states --once

# Check calibration status
ros2 topic echo /niryo_robot_hardware_interface/hardware_status --once
# Look for: calibration_needed: false

# Verify trajectory action exists
ros2 action list | grep trajectory
# Should show: /niryo_robot_follow_joint_trajectory_controller/follow_joint_trajectory
```

If `calibration_needed: true`, recalibrate (someone must watch the robot):

```bash
ros2 service call /niryo_robot/joints_interface/calibrate_motors \
  niryo_ned_ros2_interfaces/srv/SetInt "{value: 1}"
```

### Step 4: First Safe Motion

Kill the standalone driver, then launch MoveIt with the real robot:

**Terminal 1:**
```bash
source ~/ax-ros2-ref/workspace_setup.sh
source install/setup.bash
ros2 launch niryo_ned2_moveit_config ned2_moveit_real.launch.py
```

Wait for `You can start planning now!`

**Terminal 2** (slow speed for safety):
```bash
source ~/ax-ros2-ref/workspace_setup.sh
source install/setup.bash
ARM_VELOCITY=0.05 ARM_ACCELERATION=0.05 \
  ros2 launch metis_moveit2_perception perception_arm.launch.py
```

**Terminal 3** (test motion):
```bash
source ~/ax-ros2-ref/workspace_setup.sh
source install/setup.bash
ros2 run metis_moveit2_perception test_safe_motion.sh forward
```

The real robot should slowly move forward. Test all positions before proceeding:

```bash
ros2 run metis_moveit2_perception test_safe_motion.sh left
ros2 run metis_moveit2_perception test_safe_motion.sh right
ros2 run metis_moveit2_perception test_safe_motion.sh home
```

### Step 5: Camera Calibration

Mount the USB camera on a stand above the robot's workspace, pointing **straight down** at the
table. Then measure:

| Measurement | What to measure | Parameter |
| :---------- | :-------------- | :-------- |
| **X** | Horizontal distance from robot base center to directly below the camera (toward camera = positive) | Static TF x |
| **Y** | Left/right offset (robot's left = positive) | Static TF y |
| **Z** | Camera height above the table | Static TF z and `assumed_depth_m` |

Update these values in `pick_and_place.launch.py` under the static TF arguments and
`assumed_depth_m` parameter.

**Verify the projection** by placing a known object on the table and checking that the logged
`Moving to (x, y, z)` position matches the physical location within a few centimetres.

### Step 6: Full Pipeline

Once the projection is accurate, run the full pick-and-place pipeline:

> [!IMPORTANT]
> Keep `ARM_VELOCITY` at `0.05` for the first run on real hardware. Only increase speed once
> you have confirmed correct motion through the full pick-place cycle at least twice.

```bash
# Start MoveIt (Terminal 1 — keep running throughout)
ros2 launch niryo_ned2_moveit_config ned2_moveit_real.launch.py

# Start inference + pick-and-place (Terminal 2)
PICK_CLASS=duck DEMO_SIM=false \
  ARM_VELOCITY=0.05 ARM_ACCELERATION=0.05 \
  ros2 launch metis_moveit2_perception demo.launch.py
```

Gradually increase speed after confirming correct motion:

```bash
PICK_CLASS=duck DEMO_SIM=false \
  ARM_VELOCITY=0.15 ARM_ACCELERATION=0.15 \
  ros2 launch metis_moveit2_perception demo.launch.py
```

---

## Running the Demo

### Stage 3 only (pick-and-place node)

If MoveIt and inference are already running, launch only the pick-and-place stage:

```bash
source ~/ax-ros2-ref/workspace_setup.sh
source install/setup.bash
PICK_CLASS=duck ros2 launch metis_moveit2_perception pick_and_place.launch.py
```

### Common launch examples

```bash
# Pick the rubber duck, place on the cow mat
PICK_CLASS=duck ros2 launch metis_moveit2_perception pick_and_place.launch.py

# Pick a carrot
PICK_CLASS=carrot ros2 launch metis_moveit2_perception pick_and_place.launch.py

# Pick a banana with a 2 cm Y offset (tune until gripper centres on the fruit)
PICK_CLASS=banana PICK_OFFSET_Y=0.02 \
  ros2 launch metis_moveit2_perception pick_and_place.launch.py

# Pick the smallest detected duck (useful when multiple ducks are visible)
PICK_CLASS=duck SELECT_MODE=smallest \
  ros2 launch metis_moveit2_perception pick_and_place.launch.py

# Slow speed for first-time testing
PICK_CLASS=duck ARM_VELOCITY=0.05 ARM_ACCELERATION=0.05 \
  ros2 launch metis_moveit2_perception pick_and_place.launch.py
```

---

## Pick Profiles

Each supported object has a pre-tuned parameter profile. The profile sets the grasp height,
position offsets, and gripper torques for that object's size, shape, and material.

| `PICK_CLASS` | Grasp height | Offset X / Y | Max torque | Hold torque | Detector |
| :----------- | :----------: | :----------: | :--------: | :---------: | :------- |
| `duck` | 0.090 m | 0.0 / 0.0 | 80 % | 100 % | Yellow HSV |
| `carrot` | 0.092 m | 0.0 / 0.0 | 100 % | 100 % | YOLO (COCO) |
| `banana` | 0.090 m | 0.0 / −0.025 | 90 % | 100 % | YOLO (COCO) |
| `red_toy` | 0.091 m | 0.0 / 0.0 | 80 % | 100 % | Red HSV |
| *(other)* | 0.092 m | 0.0 / 0.0 | 80 % | 50 % | YOLO (COCO) |

> [!NOTE]
> Offsets correct for systematic detection bias. For example, YOLO may centre the banana
> bounding box on the visible yellow surface rather than the physical midpoint. Use
> `PICK_OFFSET_Y` to tune without modifying the launch file.

**Grasp height** is measured from the robot's world frame origin (robot base = table level) to
the tool link. **Lower = arm descends further.** Never go below 0.09 m (`GRASP_MIN_HEIGHT_M`).

---

## Color Detectors

Some objects (rubber duck, red toy) are not reliably detected by YOLO on the COCO dataset.
This reference implementation ships dedicated HSV-threshold detectors for these:

### Yellow detector (`duck`)

Launched automatically when `PICK_CLASS=duck`. Thresholds yellow in the HSV range
H: 20–35, S: 100–255, V: 100–255 and publishes detections with class `duck` and score 0.9.

### Red detector (`red_toy`)

Launched automatically when `PICK_CLASS=red_toy`. Red wraps around hue in HSV, so two masks
are combined: H: 0–10 and H: 160–180 (union). Publishes class `red_toy` with score 0.9.

Both detectors also publish to `/color_detections` so the Live Inference window can
overlay a crosshair on the main camera feed in real-time.

### `SELECT_MODE`

When multiple blobs are above the minimum area threshold, the color detectors select:

| `SELECT_MODE` | Behaviour |
| :------------ | :-------- |
| `largest` (default) | Pick the largest detected blob |
| `smallest` | Pick the smallest detected blob |

---

## Environment Variables Reference

All variables are passed on the command line before the launch command.

### Pick-and-place launch (`pick_and_place.launch.py`)

| Variable | Default | Description |
| :------- | :------ | :---------- |
| `PICK_CLASS` | `duck` | Object class to pick. Use a COCO class name or one of the supported profiles. |
| `PLACE_CLASS` | `cow` | Object class to place on. Must be visible to the camera. |
| `ARM_VELOCITY` | `0.15` | MoveIt velocity scaling (0.0–1.0). Start at `0.05` for first real-robot tests. |
| `ARM_ACCELERATION` | `0.15` | MoveIt acceleration scaling (0.0–1.0). |
| `SELECT_MODE` | `largest` | `largest` or `smallest` — which blob to pick when multiple are detected (color detectors only). |
| `PICK_OFFSET_X` | *(profile)* | Override the X offset in metres for this run (e.g. `-0.02`). Useful for tuning without editing the launch file. |
| `PICK_OFFSET_Y` | *(profile)* | Override the Y offset in metres for this run (e.g. `0.03`). +Y = robot's left, −Y = robot's right. |

### Full demo launch (`demo.launch.py`)

| Variable | Default | Description |
| :------- | :------ | :---------- |
| `DEMO_SIM` | `true` | `true` for simulation, `false` for real robot |
| `PICK_CLASS` | `duck` | Passed through to `pick_and_place.launch.py` |
| `PLACE_CLASS` | `cow` | Passed through to `pick_and_place.launch.py` |
| `ARM_VELOCITY` | `0.15` | Passed through to `pick_and_place.launch.py` |
| `ARM_ACCELERATION` | `0.15` | Passed through to `pick_and_place.launch.py` |

---

## Tuning Parameters

These parameters live in `pick_and_place.launch.py` and are passed to `PickAndPlaceNode`.
The per-class profiles already set the most object-specific values; the parameters below
apply globally across all pick classes.

### Height parameters

| Parameter | Default | Description |
| :-------- | :------ | :---------- |
| `hover_height_m` | `0.15` | Height above table where the arm pauses before descending. |
| `grasp_height_m` | *(profile)* | Height at which the gripper closes. Set per profile; never goes below 0.09 m. |
| `place_drop_height_m` | `0.11` | Height at which the gripper opens over the place target. |
| `lift_height_m` | `0.20` | Height the arm lifts to after gripping, and uses for the grip verification clear pose. |

### Position parameters

| Parameter | Default | Description |
| :-------- | :------ | :---------- |
| `pick_offset_x` | *(profile)* | X correction added to the projected pick position. +X = further from robot. |
| `pick_offset_y` | *(profile)* | Y correction added to the projected pick position. +Y = robot's left, −Y = robot's right. |
| `max_reach_m` | `0.38` | Maximum workspace radius. Targets beyond this are clamped toward the robot centre. NED2 max reach is ~0.44 m. |

### Detection parameters

| Parameter | Default | Description |
| :-------- | :------ | :---------- |
| `min_detection_score` | `0.45` | Minimum YOLO confidence to accept a detection. |
| `pick_ema_alpha` | `0.40` | EMA smoothing factor for the pick target position. Higher = more responsive but noisier. |

### Gripper parameters (set per profile, can be overridden)

| Parameter | Default | Description |
| :-------- | :------ | :---------- |
| `max_torque_pct` | *(profile)* | Maximum closing torque (0–100 %). Higher = stronger grip. |
| `hold_torque_pct` | *(profile)* | Hold torque once gripper is closed (0–100 %). |
| `tool_id` | `11` | Niryo tool ID for the gripper attached to the robot. |

### Camera intrinsics

| Parameter | Default | Description |
| :-------- | :------ | :---------- |
| `assumed_depth_m` | `0.42` | Camera-to-table distance. Must match the physical setup. |
| `camera_fx` / `camera_fy` | `1360.0` | Focal length in pixels (Logitech C925e at 1080p). |
| `camera_cx` / `camera_cy` | `960.0` / `540.0` | Principal point (image centre at 1920×1080). |

### MoveIt planning

| Parameter | Default | Description |
| :-------- | :------ | :---------- |
| `planning_time_s` | `5.0` | Maximum time for OMPL to find a plan. |
| `planning_attempts` | `10` | Number of planning retries before failing a move. |

---

## Troubleshooting

### Robot not found on network

```
ping: ned2-f2-f85-bb2.local: Name or service not known
```

- Check the Ethernet cable is plugged in
- Wait 1-2 minutes after power-on for the robot to boot fully
- Try the IP address directly if mDNS is not working: `ping 192.168.1.101` — substitute
  your robot's address, which Niryo Studio shows in its connection panel

### ROSBridge driver fails to connect

```
[ros2_driver]: Creating driver for robot with IP: ned2-f2-f85-bb2.local and port: 9090
```
(hangs without "Bridge node initialized")

- The robot's ROS1 stack may not be running. SSH in and restart:
  ```bash
  ssh niryo@ned2-f2-f85-bb2.local
  sudo service niryo_robot_ros restart
  ```

### Calibration needed

```
calibration_needed: true
```

Someone must be physically present. Then:

```bash
ros2 service call /niryo_robot/joints_interface/calibrate_motors \
  niryo_ned_ros2_interfaces/srv/SetInt "{value: 1}"
```

### Planning fails (code: -1 or 99999)

- Target may be outside the workspace. Check `max_reach_m`.
- Target may be at a kinematic singularity (directly above the robot base). Add a small X offset.
- Kinematics solver not loaded. Check that `robot_description_kinematics` is set in the launch.

### Execution aborted

```
Execute request aborted
```

- On real robot: the ROSBridge driver may not be bridging the trajectory action.
  Check `ros2 action list | grep trajectory`.
- Timing issue: the robot's clock may be out of sync with the host. Fix timezone (see Step 2).

### TF transform failed: "world" frame does not exist

```
TF transform failed: "world" passed to lookupTransform argument target_frame does not exist.
```

This is normal for the first few seconds after launch — the static TF publisher takes a moment
to start. If it persists, check that `pick_and_place.launch.py` includes the
`static_transform_publisher` node.

### Arm descends to correct height on first attempt but crashes on retry

This was caused by the adaptive grasp-height step-down that has since been removed. If you see
this on an older build, update to the current version.

### Duck picked up but dropped during grip verification

The camera detected the held duck at the gripper's world position during the verification
phase, causing the node to think the grip failed and open the gripper at the arm's clear
pose. This is fixed — detections more than 15 cm from the original pick position are filtered
during verification.

### Grip verification always fails (duck still detected on table)

- `VERIFY_DURATION_S` (3 s) may not be long enough if inference is running slowly.
  Increase `pick_ema_alpha` so the EMA converges faster, or lengthen the verification window
  in the source code.
- Check that the arm actually moves to the clear pose (`(0.12, 0.0, lift_height_m)`) before
  the verification timer starts — look for `Arm clear — watching for 'duck' for 3.0s...`
  in the logs.

### Object not detected (no pick target found)

- For COCO objects: check `min_detection_score`. Lower it (e.g. to `0.35`) if detections are
  intermittent.
- For `duck` / `red_toy`: check the color detector is running. You should see
  `YellowDetectorNode ready` or `RedDetectorNode ready` in the launch output.
- Use `ros2 topic hz /detections` to verify detections are being published at all.

### Gripper misses the object (lands to one side)

Adjust `pick_offset_x` and `pick_offset_y`. Run with the offset as an env var first:

```bash
PICK_CLASS=banana PICK_OFFSET_Y=0.02 \
  ros2 launch metis_moveit2_perception pick_and_place.launch.py
```

Once the correct value is found, update the per-class profile in `pick_and_place.launch.py`.

---

## Related Documentation

- [Quick Start Guide](quickstart.md) — Installation and first inference run
- [Overview](overview.md) — Architecture and package descriptions
- [Model Swapping Guide](model_swapping.md) — Change or add detection models
