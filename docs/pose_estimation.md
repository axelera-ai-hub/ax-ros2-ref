# ax-ros2-ref — Pose estimation guide

> ⚠️ **Reference implementation only.** This document is part of [`ax-ros2-ref`](../README.md), an experimental, unsupported reference implementation — **not** an Axelera product. It is provided as-is with no warranty or support, and APIs and behavior may change without notice.

## Contents

- [Prerequisites](#prerequisites)
- [Level](#level)
- [Introduction](#introduction)
- [Architecture](#architecture)
  - [Pipeline overview](#pipeline-overview)
  - [Lock / dwell state machine](#lock--dwell-state-machine)
  - [Frame conventions](#frame-conventions)
- [Setup](#setup)
  - [Hardware](#hardware)
  - [Camera intrinsics](#camera-intrinsics)
  - [Static TFs](#static-tfs)
- [Running the demos](#running-the-demos)
  - [Demo 1 — Standalone wrist tracking (single terminal)](#demo-1--standalone-wrist-tracking-single-terminal)
  - [Demo 2 — Three-terminal validation flow](#demo-2--three-terminal-validation-flow)
  - [Demo 3 — Combined duck pick + hand-drop demo](#demo-3--combined-duck-pick--hand-drop-demo)
- [Node reference — `pose_inference_node`](#node-reference--pose_inference_node)
- [Node reference — `wrist_tracker_node`](#node-reference--wrist_tracker_node)
- [Pick-and-place — `place_mode='tf'`](#pick-and-place--place_modetf)
- [Tuning](#tuning)
- [Troubleshooting](#troubleshooting)
- [Related documentation](#related-documentation)
- [Further support](#further-support)

## Prerequisites

- ax-ros2-ref workspace built and the Niryo NED2 demo running end-to-end.
  Complete the [Quick Start Guide](quickstart.md) and [NED2 Arm Control Demo](ned2_guide.md)
  first — those cover the toolchain, the workspace, the NED2 hardware bridge, MoveIt2,
  and the duck pick-and-place pipeline that this guide builds on.
- Voyager SDK 1.5.3 installed at `~/voyager-sdk` and the Metis AIPU detected
  (`axdevice --refresh` returns one device).
- An additional **Logitech BRIO** USB camera (or any UVC camera at 1280×720) mounted in
  a fixed position observing the operator. The on-table duck camera (Logitech C925e at
  1920×1080) used by the existing pick demo stays where it is — pose tracking adds a
  *second* camera.
- The pose model `yolo11lpose-coco-onnx` deployed in `~/voyager-sdk/build/`. If the SDK
  installer didn't already build it, run `make NN=yolo11lpose-coco-onnx` from the
  voyager-sdk root before launching this demo.

> [!NOTE]
> The pose pipeline runs on the same Metis device as the COCO YOLO detector used for
> the duck pick. The two models cannot share the AIPU's four cores at the same time —
> the combined launch handles this automatically by skipping COCO YOLO when pose is
> active. See [Demo 3](#demo-3--combined-duck-pick--hand-drop-demo) for details.

## Level

**Intermediate** — Familiarity with ROS2 launches, TF, and MoveIt2 planning concepts.

## Introduction

`metis_pose_inference` adds human-in-the-loop control to the reference implementation's arm pipeline.
A second camera observes the operator, the
[YOLO11l-pose](https://docs.ultralytics.com/tasks/pose/) model running on the
Metis AIPU detects the operator's wrists, and the pipeline broadcasts a `right_wrist`
TF frame in the robot's `world` coordinate system every frame.

A second node, `WristTrackerNode`, watches that TF and republishes it as a
`/target_hover_pose` message that any of the existing arm controllers — including
the `PickAndPlaceNode` from `metis_moveit2_perception` — can consume directly.

Two pieces of behaviour are added on top of the raw detection:

1. **Stationary-target gating ("lock").** The arm only moves when the operator's right
   wrist has held position within a small tolerance for a configurable dwell window.
   Random hand motion is ignored; an intentional pose hold commits.
2. **TF-driven place mode for `pick_and_place_node`.** The pick-and-place state machine
   gains a third place-target source: `tf`. In this mode the arm picks a duck, verifies
   the grip, then waits for the operator to present a stable hand. Once the hand locks,
   the arm flies to it and opens the gripper — dropping the duck into the operator's
   palm — then loops back to the table to pick the next duck.

The result is a hand-off demo that exercises the full Metis stack
(two simultaneous camera streams, pose model + COCO YOLO disambiguation, MoveIt2
planning to a moving target, gripper coordination) in a single launch.

## Architecture

### Pipeline overview

```
┌──────────────────┐   /image_raw       ┌────────────────────────┐
│ Logitech C925e   ├───────────────────▶│ metis_inference_node   │
│ (table camera)   │   (1080p)          │  (COCO YOLO @ AIPU)    │
└──────────────────┘                    └──────────|────────────-
                                                    │ /detections
        ┌──────────────────┐                        │
        │ yellow_detector_ │ ── /detections ────────┤
        │ node (HSV, CPU)  │                        │
        └──────────────────┘                        ▼
                                          ┌─────────────────────┐
┌──────────────────┐                      │ pick_and_place_node │
│ Logitech BRIO    │   /hand_camera_raw   │  (state machine)    │
│ (operator cam.)  ├──┐                   └──────────┬──────────┘
└──────────────────┘  │                              │
                      │   ┌─────────────────────┐    │ MoveIt2
                      └──▶│ pose_inference_node │    │ goals
                          │  (yolo11lpose@AIPU) │    ▼
                          └─────────┬───────────┘  ┌────────────┐
                                    │  TF          │ NED2 +     │
                                    │  world →     │ MoveIt2    │
                                    │  right_wrist │ move_group │
                                    │  right_wrist_└────────────┘
                                    │  _locked              │
                                    ▼                       │
                          ┌─────────────────────┐           │
                          │ wrist_tracker_node  │ optional  │
                          │  (TF → PoseStamped) │           │
                          └─────────────────────┘           │
                                    │                       │
                                    └─ /target_hover_pose ──┘
                                       (alternative path)
```

There are **two paths** through the pipeline:

- **Direct TF path** (used by [Demo 3](#demo-3--combined-duck-pick--hand-drop-demo)).
  `pose_inference_node` broadcasts the `right_wrist_locked` TF; `pick_and_place_node`
  with `place_mode='tf'` picks it up directly via its TF listener. `wrist_tracker_node`
  is **not** launched.
- **Tracker path** (used by [Demo 1](#demo-1--standalone-wrist-tracking-single-terminal)
  and [Demo 2](#demo-2--three-terminal-validation-flow)). `wrist_tracker_node` listens
  to the locked TF, applies a configurable approach offset, and publishes
  `/target_hover_pose`. A simple arm controller plans to that pose. Used for tuning the
  pose pipeline without the full pick-and-place state machine in the loop.

### Lock / dwell state machine

The locking logic lives inside `pose_inference_node`. Three states:

```
                                  wrist seen,
                                  vis ≥ vis_thresh
                              ┌────────────────────┐
                              │                    ▼
                ┌─────────┐   │   ┌──────────────────────┐
   start ─────▶ │  IDLE   │───┘   │      DWELLING        │
                │         │       │  anchor xyz captured │
                │         │       │  progress 0 → 1.0    │
                └────▲────┘       └──────────┬───────────┘
                     │                       │  elapsed ≥ dwell_time_s
                     │  wrist lost           │  AND |xyz - anchor| ≤ tol
                     │  > lock_redetect_     │
                     │   grace_s             ▼
                     │            ┌─────────────────────┐
                     │            │       LOCKED        │ ──▶ broadcasts
                     │            │  locked_xyz frozen  │     world → right_wrist_locked
                     │            │  (broadcast only    │
                     │            │   if reachable)     │
                     │            └──────────┬──────────┘
                     │                       │
                     └───────────────────────┘
                       wrist moves > unlock_distance_m
                       OR wrist lost > lock_hold_s
```

**Visualised in the annotated stream** (`/hand_camera/annotated`):

| State    | Overlay at the right-wrist pixel                              |
| :------- | :------------------------------------------------------------ |
| IDLE     | (none)                                                        |
| DWELLING | Orange arc filling clockwise 0° → 360° as `progress` reaches 1.0; label `DWELL XX%` |
| LOCKED, in reach    | Solid green ring + crosshair + `LOCKED`            |
| LOCKED, out of reach| Solid red ring + crosshair + `OUT OF REACH`        |

> [!TIP]
> The dwell ring is the single most useful debug surface in this pipeline. If you don't
> see it filling when you hold your hand still, either the model isn't detecting your
> wrist (vis < `visibility_threshold`) or the back-projected XYZ is jittering more than
> `dwell_tolerance_m` between frames. The throttled `right_wrist world xyz=(...)` log
> line tells you which.

### Frame conventions

- **`world`** — NED2 robot base origin.
  `+X` = robot forward, `+Y` = robot's **right**, `-Y` = robot's **left**, `+Z` = up.
- **`hand_camera_link`** — BRIO body frame. Static TF set in the launch file
  (default position `(0.165, -0.70, 0.35)` m relative to the robot base, yaw 90°).
- **`hand_camera`** — BRIO optical frame, related to `hand_camera_link` by the standard
  ROS optical-frame rotation `(qx, qy, qz, qw) = (-0.5, 0.5, -0.5, 0.5)`.
- **`right_wrist`**, **`left_wrist`** — broadcast every frame whenever a wrist keypoint
  has visibility ≥ `visibility_threshold`. Useful for `tf2_echo` debugging.
- **`right_wrist_locked`** — broadcast **only while `LOCKED` and within reach**.
  Frozen at the locked position; gates the downstream consumer naturally
  (the consumer's `max_tf_age_s` rejects the transform once the lock drops).

> [!IMPORTANT]
> Pose estimation back-projects the 2D wrist pixel through the BRIO's pinhole model
> and intersects the resulting ray with a **fixed plane at `Y = y_plane_m`** in the
> world frame. This means the operator must hold their hand at approximately that
> Y depth — a hand 10 cm closer or further from the camera than `y_plane_m` produces a
> proportional XZ error. If you see the arm targeting points that are systematically
> off, calibrate `y_plane_m` to match the average operator standoff distance from the
> robot.

## Setup

### Hardware

```
                        +Z (up)
                         │
                         │
                         │     ┌──────┐
                         │     │ BRIO │  observing
                         │     └──┬───┘  operator
                         │        │  (mounted on tripod or rig
                         │        │   facing the operator)
                         │        │
        +X (forward) ────┼────────┼─── operator stands here,
        ────────────────▶│        │    holding hand at depth Y ≈ y_plane_m
                         │        │
                         │        │     ┌──────────────┐
                  ┌──────┴──┐     │     │ Logitech     │
                  │  NED2   │     └────▶│ C925e        │  observing table
                  │  base   │           └──────┬───────┘
                  │         │                  │
                  └─────────┘                  ▼
                                         ┌─────────────┐
                                         │ duck on the │
                                         │ pick table  │
                                         └─────────────┘
```

Recommended physical setup:

| Component        | Position (world frame, m)             | Mount notes                                   |
| :--------------- | :------------------------------------ | :-------------------------------------------- |
| NED2 base        | `(0, 0, 0)`                            | Bolted to a stable surface                    |
| C925e (table)    | `(0.275, 0.028, 0.42)`, 90° roll      | Mounted overhead facing the table             |
| BRIO (operator)  | `(0.165, -0.70, 0.35)`, yaw 90°       | Mounted on a tripod facing `+Y` toward operator |
| Operator stand   | `Y ≈ 0.20` to `0.30`                   | Hand held in front of the body at chest height |

> [!WARNING]
> The default static TFs in `wrist_tracking_full.launch.py` and `duck_hand_demo.launch.py`
> assume the BRIO sits 70 cm to the **left** of the robot (-Y) and 35 cm above the base.
> If your physical mount differs, **measure the offset and update the static TF in the
> launch file** before running. A 10 cm error in camera position translates to a 5–15 cm
> error in computed wrist position.

### Camera intrinsics

The BRIO at 1280×720 with its 65° diagonal FoV gives the following pinhole approximation:

| Parameter   | Value      |
| :---------- | :--------- |
| `camera_fx` | `1152.7`   |
| `camera_fy` | `1152.7`   |
| `camera_cx` | `640.0`    |
| `camera_cy` | `360.0`    |

These are the launch-file defaults. Datasheet-derived values are good to ±5 % — for a
demo this is fine. For a deployed system, run a checkerboard calibration with
[`camera_calibration`](https://docs.ros.org/en/jazzy/Tutorials/Intermediate/Calibration.html)
and replace the values in the launch.

### Static TFs

The pose pipeline relies on two static transforms broadcast by `tf2_ros`:

```
world ──▶ hand_camera_link ──▶ hand_camera
         (BRIO body frame)     (BRIO optical frame)
```

The launch files publish both. The body→optical transform is the standard ROS optical
frame quaternion and shouldn't change. The world→body transform is the one you tune
when you reposition the camera.

## Running the demos

Three launches are shipped, each at a different point on the simplicity↔completeness
trade-off.

### Demo 1 — Standalone wrist tracking (single terminal)

The simplest end-to-end pose demo. The arm tracks the operator's right wrist with a
configurable hover offset. No duck, no pick-and-place, no second camera. Useful for
calibrating the pose pipeline before integrating it.

**Single terminal, no venv activation:**

```bash
source /opt/ros/jazzy/setup.bash
cd ~/ax-ros2-ref/ros2_ws && source install/setup.bash
ros2 launch metis_pose_inference wrist_tracking_full.launch.py
```

The launch injects voyager-sdk's `LD_LIBRARY_PATH` / `GST_PLUGIN_PATH` into
`pose_inference_node` only — other nodes (RViz, MoveIt, rqt_image_view) inherit a clean
ROS2 environment. **Do not** activate the voyager venv in this shell.

What comes up:

- The full NED2 hardware driver + MoveIt move_group + RViz (via the included
  `niryo_ned2_moveit_config/launch/ned2_moveit_real.launch.py`)
- BRIO static TFs
- `pose_inference_node` with voyager env injected per-node
- `wrist_tracker_node` republishing the locked TF as `/target_hover_pose`
- `arm_controller_node` from `metis_moveit2_perception` planning to that pose
- An rqt_image_view subscribed to `/hand_camera/annotated`

Override at launch time:

```bash
Y_PLANE_M=0.25 ARM_VELOCITY=0.25 \
  ros2 launch metis_pose_inference wrist_tracking_full.launch.py
```

See [Tuning](#tuning) for the full list of environment-variable overrides.

### Demo 2 — Three-terminal validation flow

Used during initial bring-up. Splits the pipeline across three terminals so each
component can be restarted independently while you tune the camera position,
intrinsics, and dwell timing.

**Terminal 1 — NED2 driver + MoveIt:**

```bash
ros2 launch niryo_ned2_moveit_config ned2_moveit_real.launch.py
```

**Terminal 2 — pose pipeline with voyager venv activated:**

```bash
cd ~/voyager-sdk && source venv/bin/activate
source /opt/ros/jazzy/setup.bash
cd ~/ax-ros2-ref/ros2_ws && source install/setup.bash
ros2 launch metis_pose_inference wrist_tracking_test.launch.py
```

**Terminal 3 — annotated stream viewer (no venv):**

```bash
source /opt/ros/jazzy/setup.bash
cd ~/ax-ros2-ref/ros2_ws && source install/setup.bash
ros2 run rqt_image_view rqt_image_view /hand_camera/annotated
```

> [!NOTE]
> rqt_image_view is launched in a **separate, non-venv** terminal because the voyager
> venv ships its own Python interpreter without `PyQt5` / `PySide2`. The packages only
> exist in the system apt install (`python3-pyqt5`). The unified Demo 1 and Demo 3
> launches solve this by injecting voyager paths only into the inference node — Demo 2
> is the manual equivalent.

### Demo 3 — Combined duck pick + hand-drop demo

The full demo: the arm picks a yellow duck off the table, then drops it into the
operator's hand once the operator dwells their wrist. Loops to pick the next duck.

```bash
source /opt/ros/jazzy/setup.bash
cd ~/ax-ros2-ref/ros2_ws && source install/setup.bash
ros2 launch metis_pose_inference duck_hand_demo.launch.py
```

What this brings up:

- NED2 driver + MoveIt + RViz
- C925e duck camera + Live Inference display window (`SKIP_YOLO=1` so COCO YOLO
  doesn't grab the AIPU)
- `yellow_detector_node` (HSV duck detector on the CPU)
- BRIO static TFs + `pose_inference_node` with voyager env injected
- `pick_and_place_node` with `place_mode='tf'`, `place_tf_frame='right_wrist_locked'`,
  loop mode on
- One `rqt_image_view` on `/hand_camera/annotated`

The duck camera viz comes from Live Inference's own native window — no second
rqt is needed. `pick_and_place_node` runs as the **sole** arm controller;
`wrist_tracker_node` is not launched in this demo.

**Pick-and-place flow under TF mode:**

```
DETECTING_PICK ─▶ APPROACHING_PICK ─▶ OPENING_GRIPPER ─▶ DESCENDING_TO_PICK ─▶
GRIPPING ─▶ LIFTING ─▶ VERIFYING_GRIP ─▶ DETECTING_PLACE
                                          │
                                          │  (waits for right_wrist_locked TF)
                                          ▼
                                  APPROACHING_PLACE  (move 2 cm above wrist)
                                          │
                                          │  (DESCENDING_TO_PLACE skipped in tf mode)
                                          ▼
                                      RELEASING  (open gripper, duck drops)
                                          │
                                          ▼
                                     RETRACTING  (move to home: 0.12, 0.0, lift_height_m)
                                          │
                                          ▼
                              LOOP_CHECKING_SCENE → DETECTING_PICK
```

Override at launch time:

```bash
PLACE_TF_HOVER_OFFSET_M=0.05 PNP_START_DELAY_S=15 \
  ros2 launch metis_pose_inference duck_hand_demo.launch.py
```

> [!TIP]
> The launch waits 20 s before starting `pick_and_place_node` (`PNP_START_DELAY_S=20`)
> so the NED2 driver has time to publish `/joint_states` before MoveIt is asked to plan.
> Without this delay, the first plan-and-execute cycle fails repeatedly with
> `Didn't receive robot state` for ~5 s. If your NED2 boots faster, lower the delay.

## Node reference — `pose_inference_node`

C++/Python: Python (`metis_pose_inference/pose_inference_node.py`).
Subscribes to: nothing (opens the BRIO directly via voyager's `create_inference_stream`).
Publishes:

- `/hand_camera/annotated` (`sensor_msgs/Image`) — rendered overlay
- TF `world → left_wrist` and `world → right_wrist` (per-frame, when `vis ≥ visibility_threshold`)
- TF `world → right_wrist_locked` (only while `LOCKED` and within reach)

### Parameters

| Parameter                  | Type   | Default                               | Purpose                                                                                       |
| :------------------------- | :----- | :------------------------------------ | :-------------------------------------------------------------------------------------------- |
| `voyager_sdk_path`         | str    | `~/voyager-sdk`                       | Path to the voyager-sdk installation. The node prepends `voyager_sdk_path/venv/...` to `sys.path` and sets `AXELERA_FRAMEWORK` so axelera.app imports resolve. Override via the `VOYAGER_SDK_DIR` env var (matches `workspace_setup.sh`). |
| `network`                  | str    | `yolo11lpose-coco-onnx`               | Voyager network name. Must be deployed in `voyager_sdk_path/build/`.                          |
| `source`                   | str    | `usb:2`                               | voyager source spec. `usb:N` opens `/dev/videoN`.                                              |
| `world_frame`              | str    | `world`                                | Frame the wrist TFs are published in.                                                         |
| `camera_frame`             | str    | `hand_camera`                          | Camera optical frame.                                                                         |
| `y_plane_m`                | double | `0.20`                                 | Fixed Y depth used for wrist back-projection. See [Frame conventions](#frame-conventions).    |
| `visibility_threshold`     | double | `0.7`                                  | Per-keypoint visibility floor.                                                                |
| `person_score_threshold`   | double | `0.81`                                 | Per-person confidence floor before keypoints are considered.                                  |
| `camera_fx` / `camera_fy`  | double | `1152.7` / `1152.7`                    | Pinhole focal lengths.                                                                        |
| `camera_cx` / `camera_cy`  | double | `640.0` / `360.0`                      | Pinhole principal point.                                                                      |
| `publish_annotated_image`  | bool   | `true`                                 | Toggle the rendered `/hand_camera/annotated` topic.                                           |
| `dwell_time_s`             | double | `3.0`                                  | How long the wrist must stay within `dwell_tolerance_m` to commit a lock.                     |
| `dwell_tolerance_m`        | double | `0.05`                                 | Maximum jitter from the dwell anchor allowed during DWELLING.                                 |
| `unlock_distance_m`        | double | `0.10`                                 | Distance from the locked point that triggers unlock.                                           |
| `lock_frame_name`          | str    | `right_wrist_locked`                   | TF frame published while LOCKED.                                                               |
| `lock_redetect_grace_s`    | double | `2.0`                                  | Grace period before DWELLING falls back to IDLE on a missed detection.                         |
| `lock_hold_s`              | double | `2.0`                                  | Grace period before LOCKED unlocks on prolonged loss.                                          |
| `max_reach_m`              | double | `0.42`                                 | Reachability gate. Locked targets beyond this are flagged `OUT OF REACH` and **not broadcast**. |

## Node reference — `wrist_tracker_node`

Python (`metis_pose_inference/wrist_tracker_node.py`).
Subscribes to TF.
Publishes: `/target_hover_pose` (`geometry_msgs/PoseStamped`).

### Parameters

| Parameter         | Type   | Default                | Purpose                                                                                       |
| :---------------- | :----- | :--------------------- | :-------------------------------------------------------------------------------------------- |
| `world_frame`     | str    | `world`                | Pose `header.frame_id`.                                                                       |
| `wrist_frame`     | str    | `right_wrist_locked`   | TF the tracker follows.                                                                       |
| `output_topic`    | str    | `/target_hover_pose`   | Output PoseStamped topic.                                                                     |
| `x_offset_m`      | double | `0.0`                  | Slight `+X` bias on the published TCP target.                                                 |
| `z_offset_m`      | double | `0.10`                 | Standoff above the wrist (in approach axis direction).                                         |
| `publish_rate_hz` | double | `2.0`                  | Tick rate.                                                                                     |
| `max_tf_age_s`    | double | `1.0`                  | Reject stale transforms — gates publishing when the lock drops.                                |
| `min_movement_m`  | double | `0.03`                 | Don't republish if the new pose is within this radius of the last publish.                     |
| `approach_mode`   | str    | `horizontal`           | `'horizontal'` — gripper points world `+Y` (toward operator); standoff applied in `-Y`.<br>`'down'` — gripper points world `-Z`; standoff applied in `+Z`. See [Architecture](#architecture). |

## Pick-and-place — `place_mode='tf'`

`pick_and_place_node` (`metis_moveit2_perception`) gains a third `place_mode`:

| `place_mode`  | Place target source                                                                |
| :------------ | :--------------------------------------------------------------------------------- |
| `detection`   | (default) Looks for a `place_class` detection on `/detections` and back-projects it. |
| `fixed`       | Drops at `(fixed_drop_x, fixed_drop_y)`. Equivalent to legacy `use_fixed_drop_pose=true`. |
| `tf`          | Polls the TF tree for `place_tf_frame` and uses its translation as the place target. |

### Additional parameters (TF mode only)

| Parameter                   | Type   | Default                | Purpose                                                                                       |
| :-------------------------- | :----- | :--------------------- | :-------------------------------------------------------------------------------------------- |
| `place_tf_frame`            | str    | `""`                   | TF child frame to use as the place target. e.g. `right_wrist_locked`.                          |
| `place_tf_max_age_s`        | double | `1.0`                  | Reject stale transforms older than this. Aligns with the wrist tracker's age gate.             |
| `place_tf_hover_offset_m`   | double | `0.05`                 | TCP hover height above the wrist. **In TF mode this is also where the gripper opens** — the descent step is skipped. |
| `place_tf_drop_offset_m`    | double | `0.00`                 | Declared for backward compatibility; **unused in TF mode** since DESCENDING_TO_PLACE is skipped. |
| `place_tf_lift_offset_m`    | double | `0.10`                 | Retract Z above the wrist. Only used if you re-enable descent via custom code.                  |

### Behavioural changes in TF mode

1. The place-class detection-callback path is skipped — `place_target_` is populated only
   from TF.
2. After `VERIFYING_GRIP`, the state machine enters `DETECTING_PLACE` and polls the TF
   tree at the tick rate (every 500 ms). It waits **indefinitely** for a fresh
   `place_tf_frame`. Hold your hand to start the dwell.
3. `APPROACHING_PLACE` moves the TCP to `(wrist.x, wrist.y, wrist.z + place_tf_hover_offset_m)`.
4. `DESCENDING_TO_PLACE` is **skipped**. The state machine transitions directly from
   `APPROACHING_PLACE` to `RELEASING`.
5. `RETRACTING` moves to a fixed home pose `(0.12, 0.0, lift_height_m)` with the
   canonical down orientation (via `make_down_pose`). The arm is now clear of the
   operator's hand and positioned to rescan the table for the next duck.

## Tuning

### When the lock fires too easily (random poses commit)

Increase `dwell_time_s` or shrink `dwell_tolerance_m`. The default `dwell_time_s=3.0` is
calibrated for an operator deliberately holding a pose; for fully random hand motion to
get filtered, 5–7 s with a 3 cm tolerance is better.

### When the lock never fires (you hold still and nothing happens)

Most often: the model isn't seeing your wrist consistently. Check the throttled INFO
log:

```
[pose_inference] right_wrist world xyz=(0.146, 0.200, 0.272) | reach=0.367m
```

If you see this line every 1–2 s, detection is fine. If it's missing, lower
`person_score_threshold` (e.g. 0.7) or `visibility_threshold` (e.g. 0.6). A noisy BRIO
in poor lighting can drop the right-wrist visibility below the default 0.7 even with a
clean detection.

### When the back-projected XYZ is biased

If `tf2_echo world right_wrist` reports a Z systematically lower (or higher) than your
physical wrist position, one of these is wrong:

1. **Camera height in the static TF.** Most common cause. Measure the BRIO lens
   height from the table and update `--z` in the launch.
2. **`y_plane_m` is wrong.** The default 0.20 m assumes the operator stands ~70 cm in
   front of the BRIO. If your operator standoff is different, computed XZ scales
   proportionally.
3. **Camera intrinsics are off by more than ~5 %.** Run a checkerboard calibration.

### When MoveIt fails to plan to the locked hand

The error pattern is typically `Unable to sample any valid states for goal tree` and a
5 s timeout. Two likely causes:

1. **Reach.** `axdevice` reach is ~0.44 m for the NED2. The reach gate
   (`pose_inference_node.max_reach_m`, default 0.42) blocks unreachable locks before
   they reach the arm — but if you've raised the gate, you'll see MoveIt timeouts
   instead. Lower the gate or ask the operator to hold their hand closer.
2. **Orientation pinch.** Specific (x, y, z) + orientation combinations have no IK
   solution. Lowering `place_tf_hover_offset_m` to 2 cm has historically made these
   pinched configurations feasible.

### When the duck is dropped from too high

In TF mode, the gripper opens at `place_tf_hover_offset_m` above the wrist. Default is
2 cm. To drop from higher, raise this value; to drop closer to the palm, lower it.

```bash
PLACE_TF_HOVER_OFFSET_M=0.05 ros2 launch metis_pose_inference duck_hand_demo.launch.py
```

## Troubleshooting

### `Failed to load plugin '~/voyager-sdk/operators/lib/libgstaxstreamer.so': libaxstreamer.so: cannot open shared object file`

`LD_LIBRARY_PATH` doesn't include voyager's `operators/lib`. The unified launches inject
this per-node — if you're running Demo 2 (three terminals), check that the voyager venv
is activated in the pose terminal:

```bash
echo $LD_LIBRARY_PATH | tr ':' '\n' | grep -E 'axelera|voyager'
# Should include /opt/axelera/runtime-1.5.3-1/lib and ~/voyager-sdk/operators/lib
```

### `create_inference_stream failed: "Invalid network 'yolo11lpose-coco-onnx'"`

`AXELERA_FRAMEWORK` isn't set, so voyager's yaml registry doesn't scan `ax_models/zoo`.
The unified launches set this automatically. For Demo 2, activate the voyager venv —
or run `make NN=yolo11lpose-coco-onnx` from the voyager-sdk root to confirm the model
is built.

### `axr_device_connect failed ... ZE_RESULT_ERROR_INVALID_NULL_POINTER`

AIPU contention. Two voyager processes are competing for the four AIPU cores. In Demo 3
this is handled by `SKIP_YOLO=1` (the COCO YOLO node is suppressed). If you launch the
pose pipeline manually alongside an existing inference stack, kill the COCO YOLO
process first.

### Pose pipeline starts but rqt_image_view dies with `No module named 'PyQt5'`

The voyager venv's Python doesn't ship `PyQt5`. **rqt_image_view must be launched from
a non-venv shell.** Demo 1 / Demo 3 launches handle this with per-node `additional_env`;
Demo 2 requires you to launch rqt manually in a third terminal.

### MoveIt logs `Didn't receive robot state` repeatedly at startup

The NED2 ros2_driver hasn't published `/joint_states` yet when `pick_and_place_node`
fires its first plan request. Demo 3 wraps the node in a 20 s `TimerAction` to give the
driver time. Raise `PNP_START_DELAY_S` if your NED2 boots more slowly.

### `Place TF 'right_wrist_locked' is X.XXs stale` and the arm waits

The lock dropped — the operator's hand moved or disappeared from view. Re-present the
hand and dwell; once the lock fires again, the broadcast resumes and the arm proceeds.
This is the intended, fail-safe behaviour: the arm never moves toward an unlocked or
stale target.

## Related documentation

- [ax-ros2-ref Quick Start Guide](quickstart.md) — base workspace and toolchain
- [ax-ros2-ref Overview](overview.md) — package structure, KR mapping
- [NED2 Arm Control Demo](ned2_guide.md) — pick-and-place pipeline that this guide builds on
- [Model Swapping Guide](model_swapping.md) — adding or replacing models
- [Voyager SDK Quick Start](https://github.com/axelera-ai-hub/voyager-sdk/blob/main/docs/tutorials/quick_start_guide.md)
- [Voyager SDK Application Integration](https://github.com/axelera-ai-hub/voyager-sdk/blob/main/docs/tutorials/application.md)
- [Voyager SDK Custom Model](https://github.com/axelera-ai-hub/voyager-sdk/blob/main/docs/tutorials/custom_model.md) — for swapping the pose model

## Further support

- [Axelera Community Forum](https://community.axelera.ai)
- [Axelera Customer Portal](https://support.axelera.ai)
- File an issue on the ax-ros2-ref repository for ROS2-specific questions.
