# ax-ros2-ref — Orange Pi Pick-and-Place Tuning Guide

> ⚠️ **Reference implementation only.** This document is part of [`ax-ros2-ref`](../README.md), an experimental, unsupported reference implementation — **not** an Axelera product. It is provided as-is with no warranty or support, and APIs and behavior may change without notice.

A reference for the parameters in
[`pick_and_place_pi.launch.py`](../ros2_ws/src/metis_moveit2_perception/launch/pick_and_place_pi.launch.py)
that need adjustment when the physical setup changes: the camera mount,
the table height, the lighting, the layout of objects, or the operator's
desired arm speed.

This document targets the **Orange Pi 5 + Niryo NED2 + Logitech C920**
bring-up. The x86 launch (`pick_and_place.launch.py`) has its own defaults
and is **not** covered here.

## Contents

- [Camera mount](#camera-mount)
- [Pinhole back-projection depth](#pinhole-back-projection-depth)
- [Grasp height](#grasp-height)
- [HSV colour detector (KitKat profile)](#hsv-colour-detector-kitkat-profile)
- [Pick-verify radius](#pick-verify-radius)
- [Hand-place mode (HAGRID gestures)](#hand-place-mode-hagrid-gestures)
- [Arm speed](#arm-speed)
- [What does NOT change with the camera mount](#what-does-not-change-with-the-camera-mount)
- [Validation checklist](#validation-checklist)

---

## Camera mount

The static TF between `world` and `camera` is published from three
translation strings and a quaternion at the top of the launch file:

```python
CAMERA_TX, CAMERA_TY, CAMERA_TZ = '0.225', '0.0', '1.01'
CAMERA_QX, CAMERA_QY, CAMERA_QZ, CAMERA_QW = '-0.707', '-0.707', '0.0', '0.0'
```

| Parameter | Meaning | Typical value |
|-----------|---------|---------------|
| `CAMERA_TX` | Camera position along robot +X (in front of base), metres | `0.225` |
| `CAMERA_TY` | Camera position along robot +Y (robot's left), metres | `0.0` if optically centred on the base |
| `CAMERA_TZ` | Camera height above the table top, metres | `1.01` |
| Quaternion | Overhead-vertical orientation (looking straight down at the table) | `(-0.707, -0.707, 0, 0)` |

The quaternion only stays valid as long as the camera is mounted
overhead and pointing straight down. If you tilt it at all, recompute the
quaternion — don't just edit translation.

> 💡 **Tip:** Measure `CAMERA_TX` and `CAMERA_TZ` from the **robot base
> centre** and the **table top**, not from the floor or the camera body.
> A 1 cm measurement error propagates linearly into back-projected XY.

---

## Pinhole back-projection depth

When `detection_to_target_node` converts a pixel `(u, v)` into a world XY
without a depth sensor, it assumes the object lies on a known horizontal
plane and uses the pinhole model:

```
world_X = (u − cx) * depth / fx
world_Y = (v − cy) * depth / fy
```

| Parameter | Plane | Formula |
|-----------|-------|---------|
| `assumed_depth_m` | Table top (Z = 0) | = `CAMERA_TZ` |
| `place_assumed_depth_m` | Hand plane (operator's palm at ~Z = 0.20 m) | = `CAMERA_TZ − 0.20` |

> ⚠️ **Warning:** Using the table depth for a detection that's actually at
> hand height scales XY by `1.01 / 0.81 ≈ 1.25×`. A hand 30 cm from the
> optical centre back-projects to ~37 cm and the arm overshoots. Always
> update **both** depths when `CAMERA_TZ` changes.

---

## Grasp height

`grasp_height_m` is the Z height of `tool_link` (not the fingertip) at
the bottom of the grasp descent. `tool_link` sits **~9 cm above the
fingertips** on the Niryo 2-finger gripper, so `grasp_height_m = 0.09`
puts the fingertips at the table top (Z = 0).

Adjust `grasp_height_m` whenever the table itself is raised or lowered:

| Table change | `grasp_height_m` |
|--------------|-------------------|
| Original height (Z = 0) | `0.09` |
| Raised 13 mm (Z = 0.013) | `0.103` |
| Raised by `Δ` | `0.09 + Δ` |

Set `grasp_min_height_m` to the same value as `grasp_height_m` unless
you specifically want a minimum-Z floor below the planned grasp.

The KitKat profile in particular runs the gripper close to the table —
flat objects skid if `grasp_height_m` is too high. Run the descent with
the gripper open to confirm fingertip contact before enabling grasps.

---

## HSV colour detector (KitKat profile)

The KitKat mini bar is detected by `red_detector_node.py`, not the AIPU.
The launch passes the following HSV gates:

| Parameter | Current default | Env override | Purpose |
|-----------|------------------|--------------|---------|
| `h_low_min` / `h_low_max` | `0` / `8` | `KITKAT_H_LOW_MIN/MAX` | Low hue band (red wraps around 0) |
| `h_high_min` / `h_high_max` | `168` / `180` | `KITKAT_H_HIGH_MIN/MAX` | High hue band |
| `s_min` / `s_max` | `115` / `255` | `KITKAT_S_MIN/MAX` | Saturation gate — rejects washed-out reds |
| `v_min` / `v_max` | `65` / `255` | `KITKAT_V_MIN/MAX` | Brightness gate — rejects shadows |
| `min_area` | `2500` | `KITKAT_MIN_AREA` | Pixel area floor — rejects red speckles |

### Tuning rules of thumb

- **Bars under bright light are getting rejected** → lower `s_min`
  and/or `v_min` by 10-15 each. Real bars under shadow have lower
  saturation and value than under direct light.
- **Orange/peach noise leaks into the mask** → raise `s_min` and/or
  tighten the hue bands (e.g. `KITKAT_H_LOW_MAX=6`,
  `KITKAT_H_HIGH_MIN=172`).
- **Multiple small red specks (cables, edges) are detected** →
  raise `min_area`. A real bar at table depth from a 1.01 m mount
  with `fx ≈ 1482` projects to ~4500 px².

Use the env overrides to iterate quickly without rebuilding:

```bash
KITKAT_S_MIN=125 KITKAT_V_MIN=70 ros2 launch \
    metis_moveit2_perception pick_and_place_pi.launch.py
```

---

## Pick-verify radius

After a pick, the node saves the original pick XY and watches
`/detections` for `verify_duration_s` seconds. Any detection within
`verify_radius_m` of that XY is treated as **"object still on the table
→ pick failed → drop and retry."**

| Parameter | Default | Meaning |
|-----------|---------|---------|
| `verify_radius_m` | `0.015` (1.5 cm) | Spatial radius for "still there" |
| `verify_duration_s` | `2.0` | How long to watch |

### Choosing the radius

The radius must be **smaller than the inter-object spacing** in
multi-object layouts. Otherwise a neighbouring object trips a false-fail
and the robot drops the object it successfully picked.

| Object layout | Recommended `verify_radius_m` |
|---------------|--------------------------------|
| Single object on table | `0.10`–`0.15` (catches a slipped pick that rolled a few cm) |
| Multiple objects, 10-15 cm apart | `0.06` |
| Multiple objects, 6-10 cm apart | `0.04` |
| Multiple objects, touching / 3-6 cm apart | `0.015`–`0.025` |

> ⚠️ **Warning:** Tightening the radius weakens the verify-by-absence
> failsafe. If a failed pick drops an object that then rolls further
> than `verify_radius_m`, the pipeline will incorrectly declare success.
> When tight, lean on torque-feedback grip verification
> (`max_torque_pct`, `hold_torque_pct`) as the primary failure signal.

---

## Hand-place mode (HAGRID gestures)

Setting `PLACE_CLASS` to a HAGRID gesture name (or a comma-separated
list of them) switches the AIPU pipeline to YOLOv10 hand detection and
configures the place-side geometry to drop the object onto the
operator's open palm.

```bash
PLACE_CLASS=palm ros2 launch metis_moveit2_perception pick_and_place_pi.launch.py
PLACE_CLASS=palm,stop ros2 launch metis_moveit2_perception pick_and_place_pi.launch.py
PLACE_CLASS='*' ros2 launch metis_moveit2_perception pick_and_place_pi.launch.py   # any gesture
```

The launch file looks up each comma-separated token in `HAGRID_GESTURES`.
If **any** token matches, hand mode is enabled and the HAGRID model
is loaded.

### Geometry differences vs. table-place mode

| Parameter | Table-place default | Hand-place override |
|-----------|--------------------|--------------------|
| `hover_height_m` | `0.15` | `0.25` |
| `place_drop_height_m` | `0.11` | `0.20` |
| `lift_height_m` | `0.20` | `0.25` |
| `place_assumed_depth_m` | (inherits `assumed_depth_m`) | `0.81` (= 1.01 − 0.20) |
| `place_dwell_time_s` | `0.0` (first-frame latch) | `2.0` |
| `place_dwell_tolerance_m` | `0.05` | `0.05` |
| `place_max_reach_m` | (inherits `max_reach_m`) | `0.60` |

### Caveats

- **Mixed HAGRID + COCO classes are not rejected.** Setting
  `PLACE_CLASS=palm,cat` loads the HAGRID model and silently ignores
  `cat`. Use one model family at a time.
- **YOLOv10 hand detection runs at threshold 0.25.** Override with
  `CONF_THRESHOLD=0.35` if you see too many transient false positives.
- The dwell-lock (`place_dwell_time_s = 2.0`) means the operator must
  hold their hand still for 2 seconds before the drop fires. This is
  intentional — without it, transient detections trigger premature
  placements.

---

## Arm speed

```python
velocity     = float(os.environ.get('ARM_VELOCITY',     '0.50'))
acceleration = float(os.environ.get('ARM_ACCELERATION', '0.50'))
```

Both are MoveIt joint scaling factors in the 0–1 range, multiplied
against the robot's published velocity/acceleration limits.

| Scaling | Use case |
|---------|----------|
| `1.0` | Maximum speed — usually too jerky for tabletop demos |
| `0.5` | Default — brisk demo pace |
| `0.15`–`0.3` | Comfortable for filming or first bring-up |
| `< 0.1` | Diagnostic / teaching — very slow |

Per-run override:

```bash
ARM_VELOCITY=0.3 ARM_ACCELERATION=0.3 ros2 launch \
    metis_moveit2_perception pick_and_place_pi.launch.py
```

> 💡 **Tip:** If the motion looks snappy or jerky, lower
> **acceleration** before lowering velocity. Perceived "ouch" factor
> tracks acceleration more than top speed.

---

## What does NOT change with the camera mount

When you move the camera, these parameters stay valid and should not
be re-tuned just because the mount moved:

- **Camera intrinsics** (`CAMERA_FX`, `CAMERA_FY`, `CAMERA_CX`,
  `CAMERA_CY`) — these are properties of the lens + sensor. Only
  re-run `camera_calibration` if you swap the camera or the lens.
- **Camera rotation quaternion** — only valid for an
  overhead-vertical mount. Untouched if the camera still points
  straight down.
- **Robot reach gates** (`max_reach_m`, `place_max_reach_m`) — these
  are in the robot's frame and are independent of the camera.
- **Grasp heights** — these are in the table's frame and are
  independent of the camera.

Parameters that **do** need attention after a mount move:

- `CAMERA_TX`, `CAMERA_TY`, `CAMERA_TZ` (the new measurement)
- `assumed_depth_m` (matches `CAMERA_TZ`)
- `place_assumed_depth_m` (= `CAMERA_TZ` − hand-plane height)
- `pick_offset_x`, `pick_offset_y`, `place_offset_x`,
  `place_offset_y` — these are empirical residuals; re-tune via the
  `PICK_OFFSET_X` / `PICK_OFFSET_Y` / `PLACE_OFFSET_X` /
  `PLACE_OFFSET_Y` env vars before editing the file.

---

## Validation checklist

After any non-trivial parameter change, run through this list before
committing:

1. **Build**:
   ```bash
   cd ~/ax-ros2-ref/ros2_ws
   colcon build --packages-select metis_moveit2_perception
   source install/setup.bash
   ```
2. **Gate B back-projection check**: place an object at a known world
   XY, run the inference stack, confirm `detection_to_target` publishes
   a world point within ±2 cm of the known location.
3. **Single-object pick**: clear the table, place one target object,
   confirm the full pick-place cycle completes without verify
   false-fails.
4. **Multi-object pick** (if applicable): place 2-3 target objects at
   the layout density you intend to use, confirm `verify_radius_m`
   is tight enough to avoid neighbour-trips.
5. **Hand-mode smoke test** (if applicable):
   ```bash
   PLACE_CLASS=palm ros2 launch \
       metis_moveit2_perception pick_and_place_pi.launch.py
   ```
   Confirm the HAGRID model loads (not COCO) and a held palm triggers
   the place after the 2 s dwell.
