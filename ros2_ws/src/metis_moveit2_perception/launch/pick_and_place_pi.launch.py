# (C) Copyright Axelera AI 2026
# All Rights Reserved
"""
Raspberry Pi 5 pick-and-place launch.

One overhead Logitech C920 (mounted at world (0.225, 0.0, 1.01), looking straight
down at the table) plus the Niryo NED2 with the 2-finger gripper. Designed for
the AxROS Pi bring-up; x86 demos should keep using pick_and_place.launch.py,
which is left untouched.

Differences vs the x86 launch:
  - Camera at 101 cm above the table instead of 42 cm (different mount).
  - C920 instead of C925e — calibrated intrinsics baked in below.
  - MJPG capture forced (USB 2.0 on the Pi can't sustain 1080p YUYV).
  - In-node display republisher disabled by default (Pi 5 single-threaded
    executor starves capture otherwise). Override with MAIN_CAMERA_DISPLAY=1.
  - planning_time raised to 8 s (MoveIt RRTConnect is ~3x slower on Pi).

Supported pick profiles (set via PICK_CLASS env var):
    kitkat:    KitKat mini bar — red HSV detector, low grasp height, fixed drop
    <COCO>:    Any COCO class (cat / bottle / cup / cell phone / mouse / ...)
               via the AIPU YOLO pipeline. Default grasp params; tune at run.

Environment variables:
    PICK_CLASS              — 'kitkat' or any COCO class (default: kitkat)
    PLACE_CLASS             — COCO class to drop on. If set, runs YOLO and
                              drops on the detected object. If unset, the
                              kitkat profile uses a fixed drop pose instead.
    ARM_VELOCITY            — joint velocity scaling 0-1 (default 0.15)
    ARM_ACCELERATION        — joint accel scaling 0-1 (default 0.15)
    PNP_START_DELAY_S       — seconds to wait before starting pick-and-place
                              (default 30 — Pi-side joint_states startup)
    DROP_X / DROP_Y         — fixed drop XY in metres, only used when
                              PLACE_CLASS is unset (defaults 0.05 / 0.25)
    PICK_OFFSET_X           — metres added to the detected pick X
                              (overrides the profile default; negative
                              moves the grip closer to the robot base)
    PICK_OFFSET_Y           — metres added to the detected pick Y
                              (overrides the profile default; +Y is the
                              robot's left in REP-103 convention)
    MAIN_CAMERA_DISPLAY     — '1' to enable the in-node preview window
                              (default '0' on Pi for performance)
    MAIN_CAMERA_FOURCC      — capture FourCC (default 'MJPG' on Pi)
    DEMO_SCREEN             — '1' to launch the demo_screen_node viewer
                              (Axelera partner-demo template wrapping the
                              live inference feed via cv::imshow). Default '0'.
                              Pairs well with MAIN_CAMERA_DISPLAY=0.
    DEMO_SCREEN_FULLSCREEN  — '1' for fullscreen demo window (default '0')

KitKat ROI (only used by the 'kitkat' pick profile — no effect for
other pick classes). Defines a world-frame rectangle on the table
inside which red blobs are accepted as KitKats. Outside the rectangle,
HSV mask is zeroed so emergency buttons / cables / shirts can't be
mistaken for a bar. The rectangle is overlaid on the live inference
window for visual tuning.

    KITKAT_ROI_SIZE_X       — forward extent (m, world X). Default 0.42
                              (sized to match NED2's ~0.40 m table reach
                              with a 2 cm cushion at the far edge).
    KITKAT_ROI_SIZE_Y       — sideways extent (m, world Y). Default 0.40
    KITKAT_ROI_CENTER_X     — centre X in m. Default = camera mount X
    KITKAT_ROI_CENTER_Y     — centre Y in m. Default = camera mount Y
    KITKAT_ROI_Z            — table plane Z in m. Default 0.0

KitKat detector backend (only used by the 'kitkat' pick profile). The
HSV path is the default — bright crimson red blob detector with the
ROI mask baked in upstream. The YOLO path swaps the upstream HSV in
for the AIPU-compiled yolo26l-kitkat-detection model and gates its
hits to the same world-frame ROI in pick_and_place. Hand-place still
works in either mode — the hand-detection model runs as the primary
YOLO in HSV mode and as the secondary YOLO (1 AIPU core each) in YOLO
mode.

    KITKAT_DETECTOR         — 'hsv' (default) or 'yolo'
    KITKAT_MODEL_NAME       — voyager-sdk build dir name. Default
                              'yolo26l-kitkat-detection'
    KITKAT_MODEL_SUBDIR     — subdir inside the build dir. Defaults to
                              KITKAT_MODEL_NAME (same-name nested layout)
    KITKAT_CONF_THRESHOLD   — kitkat YOLO conf threshold. Default 0.50

Hand-gated pick mode (operator-triggered). When enabled, the robot
idles in a new WAITING_FOR_HAND state until a PLACE_CLASS gesture has
been continuously detected for ~1 s, then runs the normal pick cycle
and drops on the hand. In loop mode each cycle re-gates — show palm,
get one bar, show palm again. Requires PLACE_CLASS to be a HAGRID
gesture so the hand-detection model is actually loaded.

    WAIT_FOR_HAND           — '1' to enable the pre-pick hand gate,
                              '0' (default) for immediate pick scanning.
    HAND_GATE_DURATION_S    — how long the hand must be visible before
                              the gate fires. Default 1.0
    HAND_GATE_STALE_S       — max gap between detections inside one
                              streak. Default 0.3 (tolerates ~9 missed
                              frames at 30 fps without breaking the
                              streak — robust to YOLO flicker).

Adaptive place mode (hover-track the hand before dropping). When
enabled, after the lift the arm follows the operator's hand around
in discrete moves (each ~3 s on Pi due to synchronous MoveIt plan +
execute). The descent commits only once the hand has been stable for
place_dwell_time_s AND the arm has caught up to within
ADAPTIVE_ARM_CLOSE_M of it. The arm only re-plans toward a new hover
when the dwell-centre has drifted > ADAPTIVE_FOLLOW_THRESHOLD_M from
its last commanded position (a small dead-zone so a steady-but-jittery
hand doesn't trigger spurious replans). Requires PLACE_CLASS to be a
HAGRID gesture. Composes cleanly with KITKAT_DETECTOR and WAIT_FOR_HAND.

    ADAPTIVE_PLACE                  — '1' to enable adaptive hover-track,
                                      '0' (default) keeps the existing
                                      lock-then-approach flow.
    ADAPTIVE_FOLLOW_THRESHOLD_M     — hand must drift more than this from
                                      the last commanded hover before a
                                      fresh chase is triggered. Default 0.04
                                      (4 cm).
    ADAPTIVE_ARM_CLOSE_M            — descent commit requires the arm to be
                                      within this distance of the dwell-
                                      centre. Default 0.06 (6 cm).

Plan B pick fallback (fixed pick pose). Used when camera calibration
is too unreliable for kitkat back-projection but the hand-place flow
still works. The state machine skips detection-based pick targeting
and grasps at a fixed world XY every cycle. The kitkat YOLO model can
keep running in the background — it's just ignored for the pick. The
verify-grip move to home still runs (and is what the customer success
hand-find reset hooks onto), and the entire hand-place / adaptive
flow downstream is untouched.

    USE_FIXED_PICK_POSE     — '1' to enable Plan B pick, '0' (default)
                              keeps detection-based picking.
    FIXED_PICK_X            — fixed pick X in m (world frame). Default 0.30.
    FIXED_PICK_Y            — fixed pick Y in m (world frame). Default 0.0.
    MAX_FIXED_PICK_ATTEMPTS — cap on retry attempts when verify says the
                              grip failed (e.g. the bar shifted off the
                              fixed spot). After this many full pick
                              attempts the state machine gives up and
                              proceeds to the place phase. Default 3.
    VERIFY_RADIUS_M         — widens the verify-grip "is the bar still
                              there?" radius. Fixed-pick mode auto-defaults
                              to 0.20 m (20 cm) because back-projection is
                              unreliable; the default elsewhere stays
                              0.03 m. Override here to tune.
"""

import os
import yaml
import xacro
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


# ── HAGRID gesture set ───────────────────────────────────────────────────────
# Names match hagrid.names (index 0..33). PLACE_CLASS set to any of these
# triggers hand-mode: loads the YOLOv10 hand-detection model on the AIPU,
# raises hover/drop heights, and tightens the back-projection depth.
HAGRID_GESTURES = frozenset({
    'grabbing', 'grip', 'holy', 'point', 'call', 'three3', 'timeout', 'xsign',
    'hand_heart', 'hand_heart2', 'little_finger', 'middle_finger',
    'take_picture', 'dislike', 'fist', 'four', 'like', 'mute', 'ok', 'one',
    'palm', 'peace', 'peace_inverted', 'rock', 'stop', 'stop_inverted',
    'three', 'three2', 'two_up', 'two_up_inverted', 'three_gun',
    'thumb_index', 'thumb_index2', 'no_gesture',
})


# ── Camera intrinsics ────────────────────────────────────────────────────────
# Calibrated 2026-05-18 on this Pi with the C920 at 1920x1080 MJPG, using
# camera_calibration cameracalibrator and an 8x6-corner / 25 mm chessboard
# (35 samples, all four coverage axes full).
#
# camera matrix K (row-major):
#   1482.086956   0.000000   991.520441
#      0.000000 1484.542023  545.063905
#      0.000000   0.000000    1.000000
#
# Distortion D = [0.0738, -0.1167, -0.0065, 0.0047, 0.0] is moderate radial
# pincushion and not currently consumed by detection_to_target back-projection
# (single-plane pinhole). Re-run cameracalibrator if you change the camera or
# the mount.
CAMERA_FX = 1482.087
CAMERA_FY = 1484.542
CAMERA_CX =  991.520
CAMERA_CY =  545.064

# ── Camera mount (overhead, vertical, looking straight down) ────────────────
# Translation: 22.5 cm in front of robot base (+X), centred (Y=0), 101 cm above
# the table (+Z). Rotation rotates the world frame onto the camera frame for
# an overhead-vertical mount; the sign convention is verified at Gate B by
# back-projecting a known world point and checking ±2 cm agreement.
CAMERA_TX, CAMERA_TY, CAMERA_TZ = '0.37', '0.0', '0.84'
CAMERA_QX, CAMERA_QY, CAMERA_QZ, CAMERA_QW = '0.0', '-1.0', '0.0', '0.0'

# Float duplicates of the translation for the ROI center default below.
# Keeping the strings above unchanged because static_transform_publisher
# takes its args as strings on the CLI.
CAMERA_TX_F = float(CAMERA_TX)
CAMERA_TY_F = float(CAMERA_TY)


def generate_launch_description():
    pick_class       = os.environ.get('PICK_CLASS', 'kitkat')
    # KitKat detector backend: 'hsv' (default, in-image red-blob detector)
    # or 'yolo' (run the AIPU-compiled yolo26l-kitkat-detection model). YOLO
    # mode runs the kitkat model on the main camera stream and — if hand-
    # place mode is also on — runs hand-detection alongside as a second
    # AIPU instance (each on 1 core, well within the Metis 4-core budget).
    # Only honoured when pick_class == 'kitkat'; ignored for COCO picks.
    kitkat_detector_mode = os.environ.get('KITKAT_DETECTOR', 'hsv').lower()
    if kitkat_detector_mode not in ('hsv', 'yolo'):
        raise ValueError(
            f"KITKAT_DETECTOR must be 'hsv' or 'yolo', got "
            f"'{kitkat_detector_mode}'")
    use_yolo_kitkat = (pick_class == 'kitkat' and
                       kitkat_detector_mode == 'yolo')
    # Pre-pick hand gate. When '1', the robot does nothing until a
    # PLACE_CLASS gesture has been continuously detected for ~1 s, then
    # starts the normal pick cycle and drops on the hand. Loops re-gate
    # before each cycle. Validation below (after place_to_hand is set)
    # ensures PLACE_CLASS is a hand gesture — without one, hand-detection
    # isn't loaded and the gate would wait forever.
    wait_for_hand = os.environ.get('WAIT_FOR_HAND', '0') == '1'
    # Adaptive place mode. When '1', after the lift the arm hover-tracks
    # the operator's hand wherever it moves, and only descends + drops
    # when the dwell condition satisfies AND the arm has caught up. Works
    # independently of (and alongside) WAIT_FOR_HAND and KITKAT_DETECTOR.
    # Still requires PLACE_CLASS to be a HAGRID gesture so the hand model
    # is loaded — same validation rule as the hand-gate above.
    adaptive_place = os.environ.get('ADAPTIVE_PLACE', '0') == '1'
    # Plan B fallback for the PICK side. When '1', the state machine
    # skips kitkat detection entirely and grasps at a fixed world XY
    # every cycle. Used when camera calibration is unreliable but the
    # hand-place drop logic still works. The kitkat YOLO model can keep
    # running in the background — it's just ignored for picking.
    # retry_on_failed_verify is forced off in this mode because the
    # background YOLO with poor calibration could falsely re-detect a
    # kitkat near the fixed pick spot and trigger an infinite retry.
    use_fixed_pick_pose = os.environ.get('USE_FIXED_PICK_POSE', '0') == '1'
    fixed_pick_x = float(os.environ.get('FIXED_PICK_X', '0.30'))
    fixed_pick_y = float(os.environ.get('FIXED_PICK_Y', '0.0'))
    # Retry cap for fixed-pick mode. The robot's grasp reproducibility
    # isn't perfect at sub-cm levels and the kitkat may shift slightly
    # off the fixed spot, so a single attempt sometimes misses. We let
    # the state machine retry up to MAX_FIXED_PICK_ATTEMPTS times before
    # giving up and proceeding to the place phase.
    max_fixed_pick_attempts = int(os.environ.get('MAX_FIXED_PICK_ATTEMPTS', '1'))
    # Detect whether the user explicitly asked for a COCO place class
    # (e.g. PLACE_CLASS=cat) vs. not setting it at all. When unset, color-
    # detector pick profiles fall back to a fixed drop pose so we don't need
    # YOLO at all; when set, we run YOLO too and drop on the detected place
    # object — same pattern as the x86 duck+cow launch.
    place_class_env  = os.environ.get('PLACE_CLASS')
    place_class      = place_class_env if place_class_env else 'cow'
    use_coco_place   = place_class_env is not None and place_class_env != ''
    velocity         = float(os.environ.get('ARM_VELOCITY',     '0.50'))
    acceleration     = float(os.environ.get('ARM_ACCELERATION', '0.50'))
    pnp_delay        = float(os.environ.get('PNP_START_DELAY_S', '5'))

    # ── Per-pick-class profile ──────────────────────────────────────────────
    # Defaults sensible for a generic small COCO object (cat/bottle/cup/etc.);
    # overridden below for color-detector pick classes.
    grasp_height_m       = 0.09
    grasp_min_height_m   = 0.09
    max_grip_attempts    = 1
    max_torque_pct       = 80
    hold_torque_pct      = 50
    verify_radius_m      = 0.15
    verify_duration_s    = 3.0
    use_fixed_drop_pose  = False
    fixed_drop_x         = 0.05
    fixed_drop_y         = 0.25
    min_pick_detections  = 10
    loop_mode            = True
    use_color_detector   = False    # if True, also skip YOLO
    color_detector_node  = None     # filled in below for color picks
    pick_offset_x        = 1.00
    pick_offset_y        = 0.4
    place_offset_x       = 0.0
    place_offset_y       = 0.0

    if pick_class == 'kitkat':
        # KitKat mini bar — ~7×3×1 cm, bright red foil wrapper.
        # The bar is thin and flat, so the gripper has to come close to the
        # table. grasp_height_m is the Z height of tool_link (not the finger
        # tip — tool_link sits ~9 cm above the fingers, so Z=0.09 puts the
        # fingertips near the table). Uses thin-flat-object tuning
        # validated on similar small flat objects.
        # Table is back at its original height (the 2026-05-20 13 mm raise
        # was reverted). tool_link sits ~9 cm above the fingertips, so 0.09
        # lands the fingers at the table top (Z=0).
        grasp_height_m       = 0.09
        grasp_min_height_m   = 0.09
        max_grip_attempts    = 2     # flat face can skid on first pinch
        max_torque_pct       = 100
        hold_torque_pct      = 100
        # Tightened from 15 cm to 1.5 cm for multi-bar layouts where
        # adjacent kitkats sit 3-6 cm apart: a 15 cm radius wrongly caught
        # a neighbouring bar as "still there" after a successful pick and
        # the robot then dropped its prize back on the table. 1.5 cm only
        # catches a near-stationary failed pick; rely on max_torque /
        # hold_torque feedback if a bar rolls further than that.
        verify_radius_m      = 0.03
        verify_duration_s    = 2.0
        # First-pass empirical offsets for the current camera mount.
        # Observed: the planned grasp landed too far forward and too far
        # to one side of the actual bar; we want the gripper 6 cm closer
        # to the robot base and 2 cm further to the robot's left.
        # +Y is the robot's left in REP-103 right-hand convention; if the
        # robot moves the wrong way after this change, override with
        # PICK_OFFSET_Y=-0.02 (or whatever sign you need).
        pick_offset_x        = 0.0
        pick_offset_y        = 0.0
        # Drop / place X back by 2 cm so the bar lands closer to the
        # robot base — same sign convention as pick_offset_x (−X is
        # toward base). Applies to both fixed-drop (no PLACE_CLASS) and
        # detection-based (e.g. PLACE_CLASS=dislike) place modes. Override
        # with PLACE_OFFSET_X / PLACE_OFFSET_Y if you need a different bias.
        place_offset_x       = -0.10
        place_offset_y       = 0.0
        # Fixed drop only when the user didn't set PLACE_CLASS. If they did
        # (e.g. PLACE_CLASS=cat), drop on the COCO-detected place object.
        use_fixed_drop_pose  = not use_coco_place
        fixed_drop_x         = float(os.environ.get('DROP_X', '0.05'))
        fixed_drop_y         = float(os.environ.get('DROP_Y', '0.25'))
        # red_detector is deterministic + min_area=2000 already rejects red
        # noise; 2 frames is enough debounce against single-frame glints
        # while keeping perception-to-action latency near minimum.
        min_pick_detections  = 2
        use_color_detector   = True

        # ── World-frame ROI gate ────────────────────────────────────────
        # Restrict KitKat detection to a 60 cm (X / forward) × 40 cm
        # (Y / sideways) rectangle on the table plane, centred under the
        # camera by default. Anything red outside this zone (emergency
        # button, cables, operator's shirt) is masked out at the HSV
        # stage so it never even produces a contour. Env overrides let
        # you tune live without rebuilding; the projected polygon is
        # drawn in the live_inference window so you can verify the
        # extent visually before grasping anything.
        # 42 x 40 cm. X (forward) trimmed from 60 to 42 cm so the ROI
        # matches NED2's actual table-reach envelope (~0.40 m horizontal
        # at Z=0) with a 2 cm cushion — no point detecting bars the arm
        # can't physically grasp, and a smaller ROI also reduces false
        # positives proportionally to its area.
        roi_size_x = float(os.environ.get('KITKAT_ROI_SIZE_X', '0.40'))
        roi_size_y = float(os.environ.get('KITKAT_ROI_SIZE_Y', '0.40'))
        roi_cx     = float(os.environ.get('KITKAT_ROI_CENTER_X', str(CAMERA_TX_F)))
        roi_cy     = float(os.environ.get('KITKAT_ROI_CENTER_Y', str(CAMERA_TY_F)))
        roi_z      = float(os.environ.get('KITKAT_ROI_Z', '0.0'))
        roi_x_min  = roi_cx - roi_size_x / 2.0
        roi_x_max  = roi_cx + roi_size_x / 2.0
        roi_y_min  = roi_cy - roi_size_y / 2.0
        roi_y_max  = roi_cy + roi_size_y / 2.0

        # KitKat HSV: bright crimson red, two hue ranges to cover the wrap.
        # Bar dimensions: 7 × 3 × 1 cm. At 101 cm overhead depth + fx=1482,
        # the bar projects to ~103 × 44 = ~4500 px² when fully visible. The
        # white "KitKat" branding bites holes in the mask, dropping the
        # detected blob to ~2500-3500 px². min_area=2000 keeps real bars in
        # and rejects background red noise (cables, painted edges, etc.).
        color_detector_node = Node(
            package='metis_moveit2_perception',
            executable='red_detector_node.py',
            name='red_detector',
            output='screen',
            parameters=[{
                'image_topic': 'image_raw',
                'label':       'kitkat',
                # Red wraps around hue — two ranges combined. Slightly
                # tightened from the original (0-10 / 160-180) but still
                # permissive enough to pick up matte areas of the foil
                # where the print bites into saturation. Every knob is
                # env-overridable so you can dial in live: set e.g.
                # KITKAT_S_MIN=130 to force a stricter saturation gate
                # without rebuilding.
                # Hue bands a touch wider than the 5/172 attempt (which
                # killed real bars under softer lighting) but still tight
                # enough that an orange ethernet cable at hue ~10-15
                # doesn't pass.
                'h_low_min':   int(os.environ.get('KITKAT_H_LOW_MIN',   '0')),
                'h_low_max':   int(os.environ.get('KITKAT_H_LOW_MAX',   '8')),
                'h_high_min':  int(os.environ.get('KITKAT_H_HIGH_MIN',  '168')),
                'h_high_max':  int(os.environ.get('KITKAT_H_HIGH_MAX',  '180')),
                # Middle-ground saturation/brightness — loosened from the
                # previous 130/75 so shadowed / matte regions of the foil
                # still pass the mask, while staying well above the original
                # 110/70 / 80/60 settings where oranges leaked in.
                's_min':       int(os.environ.get('KITKAT_S_MIN',       '115')),
                's_max':       int(os.environ.get('KITKAT_S_MAX',       '255')),
                'v_min':       int(os.environ.get('KITKAT_V_MIN',       '65')),
                'v_max':       int(os.environ.get('KITKAT_V_MAX',       '255')),
                # Up from 2000 — kills isolated red specks but keeps real
                # bars. A bar at table depth from this camera projects to
                # ~4500 px² when fully visible.
                'min_area':    int(os.environ.get('KITKAT_MIN_AREA',    '2500')),
                'select_mode': 'largest',
                # World-frame ROI gate. The detector projects these 4
                # corners through the world->camera TF + intrinsics,
                # builds a binary mask, and applies it to the HSV mask
                # before contour-finding. Out-of-zone red things are
                # silently dropped.
                'roi_enabled':      True,
                'roi_x_min_m':      roi_x_min,
                'roi_x_max_m':      roi_x_max,
                'roi_y_min_m':      roi_y_min,
                'roi_y_max_m':      roi_y_max,
                'roi_z_m':          roi_z,
                'camera_fx':        CAMERA_FX,
                'camera_fy':        CAMERA_FY,
                'camera_cx':        CAMERA_CX,
                'camera_cy':        CAMERA_CY,
                'camera_frame':     'camera',
                'robot_base_frame': 'world',
                # HSV detection is the source of /detections in this branch.
                # In YOLO mode (see below) the same node runs in overlay-
                # only mode so it can still publish /pick_roi_polygon.
                'detect_enabled':   not use_yolo_kitkat,
            }],
        )

        if use_yolo_kitkat:
            # ── YOLO kitkat mode ───────────────────────────────────────
            # Configure live_inference's primary metis_inference_node to
            # load the AIPU-compiled kitkat-detection model. The hand-
            # place block below detects use_yolo_kitkat and routes the
            # hand-detection model to MODEL_NAME_2 (secondary) instead of
            # MODEL_NAME, so kitkat + hand run side by side on 1 AIPU
            # core each. pick_and_place's pick_roi_* params are also
            # turned on (see further down) so out-of-zone YOLO hits are
            # rejected at the consumer side — the upstream image mask
            # only filters the HSV path.
            voyager_sdk_root = os.environ.get(
                'VOYAGER_SDK_DIR', os.path.expanduser('~/voyager-sdk'))
            kitkat_model_name   = os.environ.get(
                'KITKAT_MODEL_NAME',   'yolo26l-kitkat-detection')
            kitkat_model_subdir = os.environ.get(
                'KITKAT_MODEL_SUBDIR', kitkat_model_name)
            os.environ['MODEL_NAME']    = kitkat_model_name
            os.environ['MODEL_SUBDIR']  = kitkat_model_subdir
            os.environ['LABELS_PATH']   = os.path.join(
                voyager_sdk_root, 'ax_datasets', 'labels', 'kitkat.names')
            os.environ.setdefault('CONF_THRESHOLD',
                                  os.environ.get('KITKAT_CONF_THRESHOLD',
                                                 '0.50'))
            os.environ['SKIP_YOLO']     = '0'
        else:
            # HSV path: when no PLACE_CLASS is set, the color detector
            # is the only source of /detections, so we can skip YOLO on
            # the AIPU entirely. If the user did set PLACE_CLASS, YOLO
            # still has to run for the place object — leave SKIP_YOLO
            # untouched and let live_inference launch it.
            if not use_coco_place:
                os.environ.setdefault('SKIP_YOLO', '1')

    # Apply env-var overrides after the per-profile defaults so the user
    # can tune offsets without editing this file. Useful at first bring-up.
    if os.environ.get('PICK_OFFSET_X') is not None:
        pick_offset_x = float(os.environ['PICK_OFFSET_X'])
    if os.environ.get('PICK_OFFSET_Y') is not None:
        pick_offset_y = float(os.environ['PICK_OFFSET_Y'])
    if os.environ.get('PLACE_OFFSET_X') is not None:
        place_offset_x = float(os.environ['PLACE_OFFSET_X'])
    if os.environ.get('PLACE_OFFSET_Y') is not None:
        place_offset_y = float(os.environ['PLACE_OFFSET_Y'])

    # In fixed-pick mode the kitkat detection is unreliable for the verify
    # phase too — the camera calibration error means a still-there bar
    # back-projects ~10–30 cm away from its true location, well outside the
    # default 3 cm verify radius. Widen the check so any kitkat detected
    # roughly near the fixed pick spot counts as "still there, retry".
    # Env-overridable for tuning. Only applied if user didn't already set
    # VERIFY_RADIUS_M themselves.
    if use_fixed_pick_pose and os.environ.get('VERIFY_RADIUS_M') is None:
        verify_radius_m = 0.01  # 1 cm — tight enough that YOLO back-projection
                                # of the bar-in-gripper never lands within it,
                                # effectively disabling verify-driven retries
    if os.environ.get('VERIFY_RADIUS_M') is not None:
        verify_radius_m = float(os.environ['VERIFY_RADIUS_M'])

    # Force MJPG + display-off into the included live_inference launch
    # (env-var pass-through; users can still override at the command line).
    os.environ.setdefault('MAIN_CAMERA_FOURCC',  'MJPG')
    os.environ.setdefault('MAIN_CAMERA_DISPLAY', '0')

    # ── Place-target geometry defaults (overridden by hand mode below) ───────
    # Used by the pick_and_place parameter block. Existing kitkat-on-table
    # behaviour stays at these heights; the hand-place mode below bumps
    # them so the gripper hovers/drops/lifts above an operator's palm.
    hover_height_m        = 0.15
    pick_hover_height_m   = -1.0       # sentinel → node uses hover_height_m
    place_drop_height_m   = 0.11
    lift_height_m         = 0.20
    place_assumed_depth_m = -1.0      # sentinel → node uses assumed_depth_m
    place_edge_margin_px  = 0          # 0 disables the edge gate
    image_width           = 1920
    image_height          = 1080
    # Dwell-lock disabled by default → first-frame latch (legacy behaviour
    # used by kitkat/COCO place flows). Enabled in the hand profile so the
    # operator must hold their hand still before the drop fires.
    place_dwell_time_s    = 0.0
    place_dwell_tolerance_m = 0.05
    # -1 sentinel = inherit max_reach_m. Set tighter (~0.30) in hand mode
    # so the operator's hand has to be inside the high-Z reachable cone
    # before the lock can form.
    place_max_reach_m     = -1.0

    # ── Hand-place profile (PLACE_CLASS=*) ───────────────────────────────────
    # Triggered when the user wants the bar dropped onto their hand. We:
    #  1. Load the YOLOv10 hand-detection model in the inference stack
    #     (env-var pass-through to live_inference.launch.py before include).
    #  2. Raise the place-side heights so the arm hovers above a hand held
    #     at world Z≈0.20 m instead of just above the table.
    #  3. Use a tighter assumed depth for the place back-projection (0.81 m
    #     = 1.01 m camera height − 0.20 m hand plane) so the XY isn't scaled
    #     up by the table-depth factor.
    #  4. Reject detections within 150 px of the image border and prefer
    #     the most central hand when multiple gestures fire.
    # Hand mode triggers on '*' (any gesture) or any HAGRID class name
    # (palm, dislike, like, peace, ...). New gestures added to hagrid.names
    # only need to appear in HAGRID_GESTURES above for the launch to pick
    # them up. PLACE_CLASS also accepts a comma-separated list (e.g.
    # 'palm,stop') so the operator can drop on either gesture — the C++
    # node already parses this into place_class_set_; here we just need
    # to recognise multi-token inputs as hand mode too. Without the split,
    # 'palm,stop' wouldn't be a member of HAGRID_GESTURES and the launch
    # would silently fall back to the COCO YOLO model.
    place_class_tokens = (
        [t.strip() for t in place_class_env.split(',') if t.strip()]
        if place_class_env else []
    )
    place_to_hand = (
        place_class_env == '*'
        or any(t in HAGRID_GESTURES for t in place_class_tokens)
    )
    # Hand-gate validation. The gate only fires on PLACE_CLASS detections,
    # which only exist if the hand-detection model is loaded — and that
    # only happens when place_to_hand is True. Fail loudly rather than
    # silently waiting forever for a hand that nobody is detecting.
    if wait_for_hand and not place_to_hand:
        raise ValueError(
            "WAIT_FOR_HAND=1 requires PLACE_CLASS to be set to a HAGRID "
            "gesture (e.g. PLACE_CLASS=palm or PLACE_CLASS=palm,stop) so "
            "the hand-detection model is loaded and publishes the gating "
            "detections. Got PLACE_CLASS='%s'." % (place_class_env or ''))
    if adaptive_place and not place_to_hand:
        raise ValueError(
            "ADAPTIVE_PLACE=1 requires PLACE_CLASS to be set to a HAGRID "
            "gesture (e.g. PLACE_CLASS=palm or PLACE_CLASS=palm,stop) so "
            "hand-detection runs and produces the moving target the arm "
            "tracks. Got PLACE_CLASS='%s'." % (place_class_env or ''))
    if place_to_hand:
        hover_height_m        = 0.25   # hover ABOVE the operator's palm (5 cm clearance at palm Z=0.20; ≈0.36 m horizontal reach)
        pick_hover_height_m   = 0.162  # 15 cm above the raised table top (+13 mm, −1 mm tune)
        place_drop_height_m   = 0.20
        lift_height_m         = 0.25
        place_assumed_depth_m = 0.41
        # Edge gate disabled — the dwell-lock below is the only place
        # filter we want now. A hand near the FOV edge that holds steady
        # is fine; reflections / transient detections fail the dwell.
        place_edge_margin_px  = 0
        # Hold within 5 cm for 1.0 s before locking.
        place_dwell_time_s    = 1.0
        place_dwell_tolerance_m = 0.05
        # Hand-place reach gate: 0.50 m. Tighter than the previous 0.60 m
        # so fewer obvious-false-positive locks form, but still 14 cm past
        # NED2's actual reach at hover Z=0.25 (~0.36 m). Locks formed in
        # the 0.36–0.50 m band will still fail at plan time with code
        # 99999 — bring your hand inside ~36 cm horizontally of the base
        # for the place plan to actually succeed.
        place_max_reach_m     = 0.60

        voyager_sdk_root = os.environ.get(
            'VOYAGER_SDK_DIR',
            os.path.expanduser('~/voyager-sdk'),
        )
        hand_labels_path = os.path.join(
            voyager_sdk_root, 'ax_datasets', 'labels', 'hagrid.names')
        if use_yolo_kitkat:
            # YOLO kitkat is already the primary model — load hand-detection
            # as the SECONDARY AIPU instance. live_inference.launch.py
            # launches a second metis_inference_node on the same image_raw
            # stream when MODEL_NAME_2 is set, with detections merged into
            # /detections so pick_and_place sees both models' outputs.
            os.environ['MODEL_NAME_2']    = 'hand-detection'
            os.environ['MODEL_SUBDIR_2']  = 'yolov10n-coco-onnx'
            os.environ['LABELS_PATH_2']   = hand_labels_path
            # Hand-detection (YOLOv10) trains/evals at the Ultralytics
            # default 0.25, much more permissive than YOLOv5's 0.65.
            os.environ.setdefault('CONF_THRESHOLD_2', '0.25')
        else:
            # Single-model hand-place flow (existing behaviour): hand-
            # detection is the *primary* main-camera YOLO model. Used both
            # by the HSV kitkat + hand-place combo and by non-kitkat picks
            # that route through the COCO pipeline.
            os.environ.setdefault('MODEL_NAME',   'hand-detection')
            os.environ.setdefault('MODEL_SUBDIR', 'yolov10n-coco-onnx')
            os.environ.setdefault('LABELS_PATH',  hand_labels_path)
            os.environ.setdefault('CONF_THRESHOLD', '0.25')
        # Hand model has its own AIPU pipeline — make sure YOLO is enabled
        # in live_inference even if a color-detector pick profile set
        # SKIP_YOLO=1 above.
        os.environ['SKIP_YOLO'] = '0'

    ned2_moveit_pkg = get_package_share_directory('niryo_ned2_moveit_config')

    xacro_file = os.path.join(
        ned2_moveit_pkg, 'config', 'niryo_ned2.urdf.xacro')
    robot_description_content = xacro.process_file(xacro_file).toxml()

    srdf_file = os.path.join(
        ned2_moveit_pkg, 'config', 'niryo_ned2.srdf')
    with open(srdf_file, 'r') as f:
        robot_description_semantic_content = f.read()

    kinematics_file = os.path.join(
        ned2_moveit_pkg, 'config', 'kinematics.yaml')
    with open(kinematics_file, 'r') as f:
        kinematics_config = yaml.safe_load(f)

    # ── Inference stack: camera + (optional) YOLO ────────────────────────────
    inference_stack = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
            get_package_share_directory('axros_helper_nodes'),
            'launch', 'live_inference.launch.py')),
    )

    # ── Static TF: world → camera ────────────────────────────────────────────
    static_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='camera_to_world_tf',
        arguments=[CAMERA_TX, CAMERA_TY, CAMERA_TZ,
                   CAMERA_QX, CAMERA_QY, CAMERA_QZ, CAMERA_QW,
                   'world', 'camera'],
        output='screen',
    )

    # ── Pick-and-place node ──────────────────────────────────────────────────
    pick_and_place = Node(
        package='metis_moveit2_perception',
        executable='pick_and_place_node_exe',
        name='pick_and_place',
        output='screen',
        parameters=[
            {
                'pick_class':            pick_class,
                'place_class':           place_class,
                'camera_fx':             CAMERA_FX,
                'camera_fy':             CAMERA_FY,
                'camera_cx':             CAMERA_CX,
                'camera_cy':             CAMERA_CY,
                # Camera height above the table — used by the pinhole back-
                # projection in detection_to_target_node when no per-detection
                # depth is available. Matches CAMERA_TZ above.
                'assumed_depth_m':       0.64,
                'camera_frame':          'camera',
                'robot_base_frame':      'world',
                'max_reach_m':           0.90,
                # World-frame pick ROI. Active in YOLO-kitkat mode so the
                # neural-network detector's hits are gated to the same
                # 60x40 cm patch the HSV branch enforces via image mask.
                # In HSV mode the gate is upstream (red_detector zeroes
                # out-of-zone pixels before contour-finding), so this stays
                # off to avoid double-filtering with a tighter copy of the
                # same rectangle. Hand / place candidates are NEVER gated
                # — the operator can stand wherever the camera sees them.
                'pick_roi_enabled':      use_yolo_kitkat and (pick_class == 'kitkat'),
                'pick_roi_x_min_m':      roi_x_min if pick_class == 'kitkat' else -1.0,
                'pick_roi_x_max_m':      roi_x_max if pick_class == 'kitkat' else  1.0,
                'pick_roi_y_min_m':      roi_y_min if pick_class == 'kitkat' else -1.0,
                'pick_roi_y_max_m':      roi_y_max if pick_class == 'kitkat' else  1.0,
                # Pre-pick hand gate. Inserts WAITING_FOR_HAND before every
                # DETECTING_PICK entry (initial + each loop iteration).
                'wait_for_hand_before_pick': wait_for_hand,
                'hand_gate_duration_s':     float(os.environ.get('HAND_GATE_DURATION_S', '1.0')),
                'hand_gate_stale_s':        float(os.environ.get('HAND_GATE_STALE_S',    '0.3')),
                # Adaptive place — hover-track the hand, commit on dwell+arm-close.
                'adaptive_place_enabled':      adaptive_place,
                'adaptive_follow_threshold_m': float(os.environ.get('ADAPTIVE_FOLLOW_THRESHOLD_M', '0.04')),
                'adaptive_arm_close_m':        float(os.environ.get('ADAPTIVE_ARM_CLOSE_M',        '0.06')),
                'hover_height_m':        hover_height_m,
                # Pick uses its own (lower) hover when the launch sets it;
                # otherwise the node falls back to hover_height_m.
                'pick_hover_height_m':   pick_hover_height_m,
                'grasp_height_m':        grasp_height_m,
                'place_drop_height_m':   place_drop_height_m,
                'lift_height_m':         lift_height_m,
                # Separate place-side depth + edge gate (hand profile only;
                # sentinel and zeros are no-ops for the kitkat-on-table flow).
                'place_assumed_depth_m': place_assumed_depth_m,
                'image_width':           image_width,
                'image_height':          image_height,
                'place_edge_margin_px':  place_edge_margin_px,
                # Dwell-lock for the place candidate (mirrors the wrist
                # lock in pose_inference_node.py). 0 disables.
                'place_dwell_time_s':       place_dwell_time_s,
                'place_dwell_tolerance_m':  place_dwell_tolerance_m,
                # Place-side reach gate (sentinel -1 inherits max_reach_m).
                'place_max_reach_m':        place_max_reach_m,
                'pick_offset_x':         pick_offset_x,
                'pick_offset_y':         pick_offset_y,
                'place_offset_x_m':      place_offset_x,
                'place_offset_y_m':      place_offset_y,
                'min_detection_score':   0.45,
                'pick_ema_alpha':        0.40,
                'tool_id':               11,    # Niryo 2-finger gripper
                'max_torque_pct':        max_torque_pct,
                'hold_torque_pct':       hold_torque_pct,
                'grasp_min_height_m':    grasp_min_height_m,
                'verify_duration_s':     verify_duration_s,
                'verify_radius_m':       verify_radius_m,
                # When verify clearly shows the pick object is still on the
                # table (i.e. grip failed), retry the pick instead of going
                # to the drop pose with an empty gripper.
                # retry_on_failed_verify drives the retry-on-grip-miss
                # loop. In HSV/YOLO kitkat mode it's tied to use_color_detector
                # (kitkat profile retries; COCO classes don't). In fixed-pick
                # mode we ALSO enable it so the cap at max_fixed_pick_attempts
                # gets a chance to fire — otherwise retries never start.
                'retry_on_failed_verify': use_color_detector or use_fixed_pick_pose,
                'max_grip_attempts':     max_grip_attempts,
                'use_fixed_drop_pose':   use_fixed_drop_pose,
                'fixed_drop_x':          fixed_drop_x,
                'fixed_drop_y':          fixed_drop_y,
                # Plan B pick fallback — skip detection-based pick targeting.
                'use_fixed_pick_pose':   use_fixed_pick_pose,
                'fixed_pick_x':          fixed_pick_x,
                'fixed_pick_y':          fixed_pick_y,
                'max_fixed_pick_attempts': max_fixed_pick_attempts,
                'loop_mode':             loop_mode,
                'min_pick_detections':   min_pick_detections,
                'planning_group':        'arm',
                'end_effector_link':     'tool_link',
                # Raised from x86's 5.0 — RRTConnect is ~3x slower on Pi 5.
                # At 12.0 the planner has enough headroom to find paths
                # into the wider hand-place workspace; earlier 8.0 setting
                # was timing out on borderline-reach lock targets.
                'planning_time_s':       12.0,
                'planning_attempts':     10,
                'velocity_scaling':      velocity,
                'acceleration_scaling':  acceleration,
                'robot_description':     robot_description_content,
                'robot_description_semantic': robot_description_semantic_content,
            },
            {'robot_description_kinematics': kinematics_config},
        ],
    )

    nodes = [inference_stack, static_tf]
    if color_detector_node is not None:
        nodes.append(color_detector_node)
    # Pi-side joint_states comes up slowly via rosbridge → ned-ros2-driver;
    # delay pick_and_place so it doesn't start with stale or empty TF.
    nodes.append(TimerAction(period=pnp_delay, actions=[pick_and_place]))

    return LaunchDescription(nodes)
