# (C) Copyright Axelera AI 2026
# All Rights Reserved
"""
BRIO pose inference to wrist TF broadcaster.

Runs the voyager-sdk yolov8npose-coco-onnx pose model on a USB camera stream
(Logitech BRIO, typically /dev/video2), extracts left_wrist (kpt 9) and
right_wrist (kpt 10) pixel coordinates from the highest-confidence person
detection, back-projects each through a pinhole camera model, intersects the
ray with a fixed Y=y_plane plane in the world frame, and broadcasts dynamic
TFs world → left_wrist and world → right_wrist.

Heavy inference loop runs on a dedicated thread so it never stalls the
executor.
"""

import os
import sys
import threading

import cv2
import numpy as np

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TransformStamped
from sensor_msgs.msg import Image
from tf2_ros import TransformBroadcaster, Buffer, TransformListener, LookupException


# COCO keypoint indices for human body (17-point model)
KPT_LEFT_WRIST = 9
KPT_RIGHT_WRIST = 10

# Subset of COCO skeleton edges we draw (keeps the overlay readable)
SKELETON_EDGES = (
    (5, 7), (7, 9),      # left shoulder → elbow → wrist
    (6, 8), (8, 10),     # right shoulder → elbow → wrist
    (5, 6),              # shoulders
    (5, 11), (6, 12),    # shoulders → hips
    (11, 12),            # hips
    (11, 13), (13, 15),  # left hip → knee → ankle
    (12, 14), (14, 16),  # right hip → knee → ankle
)


def quat_to_rotation_matrix(qx: float, qy: float, qz: float, qw: float) -> np.ndarray:
    """Convert a unit quaternion (x, y, z, w) to a 3×3 rotation matrix."""
    return np.array([
        [1 - 2 * (qy * qy + qz * qz),  2 * (qx * qy - qz * qw),      2 * (qx * qz + qy * qw)],
        [2 * (qx * qy + qz * qw),      1 - 2 * (qx * qx + qz * qz),  2 * (qy * qz - qx * qw)],
        [2 * (qx * qz - qy * qw),      2 * (qy * qz + qx * qw),      1 - 2 * (qx * qx + qy * qy)],
    ])


class PoseInferenceNode(Node):

    def __init__(self):
        super().__init__('pose_inference_node')

        # ── Parameters ──────────────────────────────────────────────────────
        self.declare_parameter('voyager_sdk_path', os.path.expanduser('~/voyager-sdk'))
        self.declare_parameter('network', 'yolov8npose-coco-onnx')
        self.declare_parameter('source', 'usb:2')
        self.declare_parameter('world_frame', 'world')
        self.declare_parameter('camera_frame', 'hand_camera')
        self.declare_parameter('y_plane_m', 0.0)
        self.declare_parameter('visibility_threshold', 0.7)
        self.declare_parameter('person_score_threshold', 0.7)
        # BRIO @ 1280×720, 65° diagonal FoV (datasheet approximation).
        # Replace with checkerboard-calibrated values when available.
        self.declare_parameter('camera_fx', 1152.7)
        self.declare_parameter('camera_fy', 1152.7)
        self.declare_parameter('camera_cx', 640.0)
        self.declare_parameter('camera_cy', 360.0)
        self.declare_parameter('publish_annotated_image', True)
        self.declare_parameter('annotated_image_topic', '/hand_camera/annotated')
        # Lock / dwell parameters — gate arm movement on a stationary right wrist.
        self.declare_parameter('dwell_time_s', 5.0)
        self.declare_parameter('dwell_tolerance_m', 0.05)
        self.declare_parameter('unlock_distance_m', 0.10)
        self.declare_parameter('lock_frame_name', 'right_wrist_locked')
        self.declare_parameter('lock_redetect_grace_s', 2.0)
        self.declare_parameter('lock_hold_s', 2.0)
        # Reach gate: reject locked targets whose norm from the base origin
        # exceeds this. NED2 reach envelope is ~0.44 m; 0.42 leaves a small
        # margin so MoveIt doesn't spend its full planning_time_s hunting
        # for a plan that can't exist.
        self.declare_parameter('max_reach_m', 0.42)

        self.voyager_sdk_path = self.get_parameter('voyager_sdk_path').value
        self.network = self.get_parameter('network').value
        self.source = self.get_parameter('source').value
        self.world_frame = self.get_parameter('world_frame').value
        self.camera_frame = self.get_parameter('camera_frame').value
        self.y_plane = float(self.get_parameter('y_plane_m').value)
        self.vis_thresh = float(self.get_parameter('visibility_threshold').value)
        self.score_thresh = float(self.get_parameter('person_score_threshold').value)
        self.fx = float(self.get_parameter('camera_fx').value)
        self.fy = float(self.get_parameter('camera_fy').value)
        self.cx = float(self.get_parameter('camera_cx').value)
        self.cy = float(self.get_parameter('camera_cy').value)
        self.publish_annotated = bool(
            self.get_parameter('publish_annotated_image').value)
        annotated_topic = self.get_parameter('annotated_image_topic').value
        self.dwell_time_s = float(self.get_parameter('dwell_time_s').value)
        self.dwell_tolerance_m = float(self.get_parameter('dwell_tolerance_m').value)
        self.unlock_distance_m = float(self.get_parameter('unlock_distance_m').value)
        self.lock_frame_name = self.get_parameter('lock_frame_name').value
        self.lock_redetect_grace_s = float(
            self.get_parameter('lock_redetect_grace_s').value)
        self.lock_hold_s = float(self.get_parameter('lock_hold_s').value)
        self.max_reach_m = float(self.get_parameter('max_reach_m').value)

        # ── TF ──────────────────────────────────────────────────────────────
        self.tf_broadcaster = TransformBroadcaster(self)
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # ── Annotated image publisher (for rqt_image_view / rviz) ───────────
        self._image_pub = (
            self.create_publisher(Image, annotated_topic, 5)
            if self.publish_annotated else None
        )

        # ── Lock state machine ──────────────────────────────────────────────
        # IDLE     : no active candidate — waiting for first right-wrist sight.
        # DWELLING : right wrist seen; anchor captured; accumulating dwell time.
        #            If wrist leaves `dwell_tolerance_m` the anchor resets.
        # LOCKED   : anchor stayed put for `dwell_time_s`; broadcasting a fixed
        #            `lock_frame_name` TF which the wrist_tracker follows. Lock
        #            breaks when the wrist moves > `unlock_distance_m` from the
        #            locked point, or the wrist is lost for > `lock_hold_s`.
        self._lock_state = 'IDLE'
        self._dwell_anchor_xyz = None
        self._dwell_start_time = None
        self._locked_xyz = None
        self._last_right_wrist_seen_time = None
        self._last_right_wrist_pixel = None

        # ── Background inference thread ─────────────────────────────────────
        self._running = True
        self._stream = None
        self._frames_seen = 0
        self._wrists_broadcast = 0
        self._worker = threading.Thread(target=self._inference_loop, daemon=True)
        self._worker.start()

        self.get_logger().info('PoseInferenceNode ready')
        self.get_logger().info(f'  voyager_sdk_path : {self.voyager_sdk_path}')
        self.get_logger().info(f'  network          : {self.network}')
        self.get_logger().info(f'  source           : {self.source}')
        self.get_logger().info(f'  world_frame      : {self.world_frame}')
        self.get_logger().info(f'  camera_frame     : {self.camera_frame}')
        self.get_logger().info(f'  y_plane_m        : {self.y_plane}')
        self.get_logger().info(f'  vis_thresh       : {self.vis_thresh}')
        self.get_logger().info(f'  person_score_thr : {self.score_thresh}')
        self.get_logger().info(f'  fx, fy           : {self.fx}, {self.fy}')
        self.get_logger().info(f'  cx, cy           : {self.cx}, {self.cy}')
        self.get_logger().info(f'  dwell_time_s     : {self.dwell_time_s}')
        self.get_logger().info(f'  dwell_tol_m      : {self.dwell_tolerance_m}')
        self.get_logger().info(f'  unlock_dist_m    : {self.unlock_distance_m}')
        self.get_logger().info(f'  lock_frame_name  : {self.lock_frame_name}')
        self.get_logger().info(f'  max_reach_m      : {self.max_reach_m}')

    # ── Inference thread ────────────────────────────────────────────────────
    def _inference_loop(self):
        # voyager-sdk's model registry resolves `network` names relative to
        # its own working directory — chdir there so "yolov8npose-coco-onnx"
        # resolves without env-var juggling.
        try:
            os.chdir(self.voyager_sdk_path)
        except OSError as e:
            self.get_logger().error(f'chdir({self.voyager_sdk_path}) failed: {e}')
            return

        # Ensure voyager-sdk's venv site-packages are visible. Safe no-op if
        # the node is already launched inside an activated venv.
        venv_site = os.path.join(
            self.voyager_sdk_path, 'venv', 'lib', 'python3.12', 'site-packages')
        if os.path.isdir(venv_site) and venv_site not in sys.path:
            sys.path.insert(0, venv_site)

        # `axelera.app` lives in the voyager-sdk source tree (not the venv) and
        # is stitched into the `axelera` namespace package via PEP 420. The
        # root must be on sys.path for `from axelera.app import …` to resolve.
        if os.path.isdir(self.voyager_sdk_path) and self.voyager_sdk_path not in sys.path:
            sys.path.insert(0, self.voyager_sdk_path)

        # voyager's yaml registry expands $AXELERA_FRAMEWORK to locate zoo/
        # model_cards model definitions. When it's unset the zoo is silently
        # skipped and `create_inference_stream` rejects zoo models with
        # "Invalid network". The venv's activate script sets this — but this
        # node may be launched without activation, so set it here as a fallback.
        os.environ.setdefault('AXELERA_FRAMEWORK', self.voyager_sdk_path)

        try:
            from axelera.app import create_inference_stream
        except ImportError as e:
            self.get_logger().error(
                f'Failed to import axelera.app: {e}. '
                'Make sure the voyager-sdk venv is installed at '
                f'{self.voyager_sdk_path}/venv and Python 3.12 is in use.')
            return

        try:
            self._stream = create_inference_stream(
                network=self.network,
                sources=[self.source],
            )
        except Exception as e:
            self.get_logger().error(f'create_inference_stream failed: {e}')
            return

        self.get_logger().info('Inference stream opened — consuming frames')

        for frame_result in self._stream:
            if not self._running:
                break
            try:
                self._handle_frame(frame_result)
            except Exception as e:
                self.get_logger().warning(
                    f'Frame handler raised: {e}', throttle_duration_sec=5)

    def _handle_frame(self, frame_result):
        self._frames_seen += 1

        # voyager-sdk task name for yolov8npose is 'keypoint_detections'.
        # __getattr__ raises AttributeError if the task isn't in meta.
        try:
            persons = frame_result.keypoint_detections
        except AttributeError:
            available = list(getattr(frame_result, 'meta', {}).keys())
            self.get_logger().warning(
                f'frame_result has no .keypoint_detections — meta keys: {available}',
                throttle_duration_sec=5)
            return

        self.get_logger().info(
            f'frame #{self._frames_seen}: {len(persons)} person(s) detected',
            throttle_duration_sec=2)

        # Drop low-confidence people entirely — their keypoints tend to be
        # noise and pollute the wrist TF with jittery positions.
        valid_persons = [p for p in persons
                         if float(getattr(p, 'score', 0.0)) >= self.score_thresh]
        if persons and not valid_persons:
            self.get_logger().info(
                f'all person detections below score threshold '
                f'({self.score_thresh:.2f}) — skipping frame',
                throttle_duration_sec=2)

        now_s = self._lock_now()
        right_wrist_xyz = None
        right_wrist_pixel = None

        if valid_persons:
            # Current world ← hand_camera transform. Camera is rigidly mounted,
            # so this is effectively constant — we re-lookup each frame for
            # correctness (and to pick up static-TF publisher after startup).
            try:
                tf_wc = self.tf_buffer.lookup_transform(
                    self.world_frame,
                    self.camera_frame,
                    rclpy.time.Time(),  # latest available
                )
            except LookupException as e:
                self.get_logger().warning(
                    f'TF lookup {self.world_frame} ← {self.camera_frame} failed: {e}',
                    throttle_duration_sec=5)
                tf_wc = None

            if tf_wc is not None:
                origin, rotation = self._tf_to_origin_rotation(tf_wc)

                # Pick the highest-confidence person in the frame.
                best = max(valid_persons, key=lambda p: getattr(p, 'score', 0.0))
                kpts = np.asarray(best.keypoints, dtype=np.float64)  # (17, 3)

                self.get_logger().info(
                    f'best person score={float(best.score):.2f} | '
                    f'L-wrist px=({kpts[KPT_LEFT_WRIST, 0]:.0f},{kpts[KPT_LEFT_WRIST, 1]:.0f}) '
                    f'vis={kpts[KPT_LEFT_WRIST, 2]:.2f} | '
                    f'R-wrist px=({kpts[KPT_RIGHT_WRIST, 0]:.0f},{kpts[KPT_RIGHT_WRIST, 1]:.0f}) '
                    f'vis={kpts[KPT_RIGHT_WRIST, 2]:.2f}',
                    throttle_duration_sec=2)

                stamp = self.get_clock().now().to_msg()
                for name, idx in (('left_wrist', KPT_LEFT_WRIST),
                                  ('right_wrist', KPT_RIGHT_WRIST)):
                    u, v, vis = kpts[idx]
                    if vis < self.vis_thresh:
                        continue

                    xyz = self._backproject(u, v, origin, rotation)
                    if xyz is None:
                        self.get_logger().warning(
                            f'{name} back-projection failed '
                            f'(ray parallel to Y-plane or behind camera)',
                            throttle_duration_sec=5)
                        continue

                    self._broadcast_wrist_tf(name, xyz, stamp)
                    self._wrists_broadcast += 1

                    if idx == KPT_RIGHT_WRIST:
                        right_wrist_xyz = xyz
                        right_wrist_pixel = (float(u), float(v))
                        # Diagnostic: world-frame back-projection. Compare
                        # against a physically measured wrist position to
                        # check camera TF / intrinsics / y_plane_m.
                        self.get_logger().info(
                            f'right_wrist world xyz=({xyz[0]:.3f}, '
                            f'{xyz[1]:.3f}, {xyz[2]:.3f}) | '
                            f'reach={float(np.linalg.norm(xyz)):.3f}m',
                            throttle_duration_sec=2)

        # Drive the lock state machine. This runs on every frame — including
        # frames with no persons and frames where the right wrist was
        # occluded — so grace/hold timers advance correctly.
        if right_wrist_xyz is not None:
            self._update_lock_state_seen(
                right_wrist_xyz, right_wrist_pixel, now_s)
        else:
            self._update_lock_state_not_seen(now_s)

        # While LOCKED, re-broadcast the locked TF at the frozen position so
        # the wrist_tracker sees a fresh transform every frame. When lock
        # drops we simply stop broadcasting — max_tf_age_s in the tracker
        # ensures the arm stops receiving new targets. Out-of-reach locks
        # are NOT broadcast — the lock state stays LOCKED (so the overlay
        # keeps showing "OUT OF REACH") until the operator's hand drifts
        # > unlock_distance_m and a new dwell begins.
        if (self._lock_state == 'LOCKED'
                and self._locked_xyz is not None
                and self._lock_target_reachable()):
            stamp = self.get_clock().now().to_msg()
            self._broadcast_wrist_tf(self.lock_frame_name, self._locked_xyz, stamp)

        # Annotated overlay — always attempt if enabled, even when no persons
        # detected, so the live stream is visible in rqt_image_view.
        if self._image_pub is not None:
            self._publish_annotated(frame_result, persons)

    # ── Geometry helpers ────────────────────────────────────────────────────
    @staticmethod
    def _tf_to_origin_rotation(tf_msg: TransformStamped):
        t = tf_msg.transform.translation
        q = tf_msg.transform.rotation
        origin = np.array([t.x, t.y, t.z])
        rotation = quat_to_rotation_matrix(q.x, q.y, q.z, q.w)
        return origin, rotation

    def _backproject(self, u: float, v: float,
                     origin: np.ndarray, rotation: np.ndarray):
        """Ray from pixel (u, v) intersected with the Y = y_plane plane.

        Returns a (3,) world-frame numpy array, or None if the ray is parallel
        to the plane or intersects behind the camera.
        """
        d_cam = np.array([
            (u - self.cx) / self.fx,
            (v - self.cy) / self.fy,
            1.0,
        ])
        d_world = rotation @ d_cam

        if abs(d_world[1]) < 1e-6:
            return None  # ray parallel to Y-plane

        t = (self.y_plane - origin[1]) / d_world[1]
        if t <= 0.0:
            return None  # intersection is behind the camera

        return origin + t * d_world

    def _publish_annotated(self, frame_result, persons):
        """Draw bounding box + keypoints + skeleton on the frame and publish."""
        try:
            img = frame_result.image.asarray()
        except Exception as e:
            self.get_logger().warning(
                f'Could not read frame_result.image.asarray(): {e}',
                throttle_duration_sec=5)
            return

        # voyager-sdk returns RGB; OpenCV draws in BGR. Convert + make writable.
        fmt = str(getattr(frame_result.image, 'color_format', 'RGB')).upper()
        if 'RGB' in fmt and 'BGR' not in fmt:
            img = cv2.cvtColor(img, cv2.COLOR_RGB2BGR)
            encoding = 'bgr8'
        else:
            img = np.ascontiguousarray(img)
            encoding = 'bgr8' if 'BGR' in fmt else 'rgb8'

        for person in persons:
            score = float(getattr(person, 'score', 0.0))
            box = np.asarray(person.box, dtype=np.int32).flatten()
            if box.size >= 4:
                x1, y1, x2, y2 = int(box[0]), int(box[1]), int(box[2]), int(box[3])
                cv2.rectangle(img, (x1, y1), (x2, y2), (0, 255, 0), 2)
                cv2.putText(img, f'person {score:.2f}', (x1, max(0, y1 - 8)),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1, cv2.LINE_AA)

            kpts = np.asarray(person.keypoints, dtype=np.float32)  # (17, 3)

            for a, b in SKELETON_EDGES:
                if kpts[a, 2] > self.vis_thresh and kpts[b, 2] > self.vis_thresh:
                    cv2.line(img,
                             (int(kpts[a, 0]), int(kpts[a, 1])),
                             (int(kpts[b, 0]), int(kpts[b, 1])),
                             (255, 128, 0), 2)

            for idx in range(kpts.shape[0]):
                u, v, vis = kpts[idx]
                if vis < self.vis_thresh:
                    continue
                if idx in (KPT_LEFT_WRIST, KPT_RIGHT_WRIST):
                    cv2.circle(img, (int(u), int(v)), 8, (0, 0, 255), -1)
                    label = 'L-wrist' if idx == KPT_LEFT_WRIST else 'R-wrist'
                    cv2.putText(img, label, (int(u) + 10, int(v) - 10),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.5,
                                (0, 0, 255), 1, cv2.LINE_AA)
                else:
                    cv2.circle(img, (int(u), int(v)), 3, (0, 255, 255), -1)

        # ── Lock / dwell overlay ───────────────────────────────────────────
        # Drawn at the most recent right-wrist pixel. During DWELLING a ring
        # fills clockwise proportional to progress (orange); during LOCKED a
        # solid green ring + crosshair + "LOCKED" label signals that the arm
        # is being commanded to this point.
        if self._last_right_wrist_pixel is not None and self._lock_state != 'IDLE':
            cx_px = int(self._last_right_wrist_pixel[0])
            cy_px = int(self._last_right_wrist_pixel[1])
            radius = 32
            if self._lock_state == 'DWELLING':
                _, progress = self._lock_progress()
                cv2.circle(img, (cx_px, cy_px), radius,
                           (60, 60, 60), 2, cv2.LINE_AA)
                end_angle = int(360 * progress)
                # start at 12 o'clock, sweep clockwise
                cv2.ellipse(img, (cx_px, cy_px), (radius, radius),
                            -90, 0, end_angle,
                            (0, 165, 255), 5, cv2.LINE_AA)
                cv2.putText(img, f'DWELL {int(progress * 100)}%',
                            (cx_px - 48, cy_px - radius - 10),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.6,
                            (0, 165, 255), 2, cv2.LINE_AA)
            elif self._lock_state == 'LOCKED':
                reachable = self._lock_target_reachable()
                color = (0, 255, 0) if reachable else (0, 0, 255)
                label = 'LOCKED' if reachable else 'OUT OF REACH'
                label_offset = 36 if reachable else 70
                cv2.circle(img, (cx_px, cy_px), radius,
                           color, 5, cv2.LINE_AA)
                cv2.line(img, (cx_px - radius - 8, cy_px),
                         (cx_px + radius + 8, cy_px), color, 2)
                cv2.line(img, (cx_px, cy_px - radius - 8),
                         (cx_px, cy_px + radius + 8), color, 2)
                cv2.putText(img, label,
                            (cx_px - label_offset, cy_px - radius - 10),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.7,
                            color, 2, cv2.LINE_AA)

        msg = Image()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.camera_frame
        msg.height = img.shape[0]
        msg.width = img.shape[1]
        msg.encoding = encoding
        msg.is_bigendian = 0
        msg.step = img.shape[1] * img.shape[2]
        msg.data = img.tobytes()
        self._image_pub.publish(msg)

    # ── Lock state machine ─────────────────────────────────────────────────
    def _lock_now(self) -> float:
        return self.get_clock().now().nanoseconds / 1e9

    def _lock_target_reachable(self) -> bool:
        """True iff the current locked xyz is inside the reach envelope."""
        if self._locked_xyz is None:
            return True
        return float(np.linalg.norm(self._locked_xyz)) <= self.max_reach_m

    def _lock_progress(self):
        """Return (state, progress_0_to_1) for the annotated overlay."""
        if (self._lock_state == 'DWELLING'
                and self._dwell_start_time is not None):
            elapsed = self._lock_now() - self._dwell_start_time
            return 'DWELLING', max(0.0, min(1.0, elapsed / self.dwell_time_s))
        if self._lock_state == 'LOCKED':
            return 'LOCKED', 1.0
        return 'IDLE', 0.0

    def _update_lock_state_seen(self, xyz: np.ndarray,
                                pixel: tuple, now_s: float):
        """Right wrist was detected this frame at world `xyz` / image `pixel`."""
        self._last_right_wrist_seen_time = now_s
        self._last_right_wrist_pixel = pixel

        if self._lock_state == 'IDLE':
            self._dwell_anchor_xyz = xyz.copy()
            self._dwell_start_time = now_s
            self._lock_state = 'DWELLING'
            self.get_logger().info('lock: DWELL started')
            return

        if self._lock_state == 'DWELLING':
            dist = float(np.linalg.norm(xyz - self._dwell_anchor_xyz))
            if dist > self.dwell_tolerance_m:
                # wrist drifted beyond tolerance — re-anchor, restart timer.
                self._dwell_anchor_xyz = xyz.copy()
                self._dwell_start_time = now_s
                return
            elapsed = now_s - self._dwell_start_time
            if elapsed >= self.dwell_time_s:
                self._locked_xyz = self._dwell_anchor_xyz.copy()
                self._lock_state = 'LOCKED'
                reach = float(np.linalg.norm(self._locked_xyz))
                if reach > self.max_reach_m:
                    self.get_logger().warning(
                        f'lock: LOCKED at ({self._locked_xyz[0]:.3f}, '
                        f'{self._locked_xyz[1]:.3f}, '
                        f'{self._locked_xyz[2]:.3f}) but OUT OF REACH '
                        f'({reach:.2f}m > max_reach={self.max_reach_m:.2f}m) — '
                        'arm will not move; hold hand closer to the base')
                else:
                    self.get_logger().info(
                        f'lock: LOCKED at ({self._locked_xyz[0]:.3f}, '
                        f'{self._locked_xyz[1]:.3f}, '
                        f'{self._locked_xyz[2]:.3f}) — {reach:.2f}m from base, '
                        'arm will head here')
            return

        if self._lock_state == 'LOCKED':
            dist = float(np.linalg.norm(xyz - self._locked_xyz))
            if dist > self.unlock_distance_m:
                self.get_logger().info(
                    f'lock: UNLOCKED — wrist moved {dist * 100:.0f} cm '
                    'from locked point; re-dwelling at new position')
                self._lock_state = 'DWELLING'
                self._dwell_anchor_xyz = xyz.copy()
                self._dwell_start_time = now_s
                self._locked_xyz = None
            return

    def _update_lock_state_not_seen(self, now_s: float):
        """Right wrist missing this frame — advance grace timers."""
        last_seen = self._last_right_wrist_seen_time
        if last_seen is None:
            return
        elapsed_since_seen = now_s - last_seen

        if (self._lock_state == 'DWELLING'
                and elapsed_since_seen > self.lock_redetect_grace_s):
            self.get_logger().info('lock: DWELL reset — right wrist lost')
            self._lock_state = 'IDLE'
            self._dwell_anchor_xyz = None
            self._dwell_start_time = None
            return

        if (self._lock_state == 'LOCKED'
                and elapsed_since_seen > self.lock_hold_s):
            self.get_logger().info(
                'lock: UNLOCKED — right wrist lost for too long')
            self._lock_state = 'IDLE'
            self._locked_xyz = None
            self._dwell_anchor_xyz = None
            self._dwell_start_time = None

    def _broadcast_wrist_tf(self, child_frame: str, xyz: np.ndarray, stamp):
        msg = TransformStamped()
        msg.header.stamp = stamp
        msg.header.frame_id = self.world_frame
        msg.child_frame_id = child_frame
        msg.transform.translation.x = float(xyz[0])
        msg.transform.translation.y = float(xyz[1])
        msg.transform.translation.z = float(xyz[2])
        msg.transform.rotation.w = 1.0  # point — orientation is not meaningful
        self.tf_broadcaster.sendTransform(msg)

    # ── Shutdown ────────────────────────────────────────────────────────────
    def shutdown(self):
        self._running = False
        stream = self._stream
        if stream is not None:
            stop = getattr(stream, 'stop', None)
            if callable(stop):
                try:
                    stop()
                except Exception as e:
                    self.get_logger().warning(f'stream.stop() raised: {e}')


def main():
    rclpy.init()
    node = PoseInferenceNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.shutdown()
        node.destroy_node()
        # rclpy's default signal handler may have already called shutdown()
        # when Ctrl+C arrived. Skip the second call to avoid an RCLError.
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
