#!/usr/bin/env python3
# (C) Copyright Axelera AI 2026
# All Rights Reserved
"""
Red object detector using HSV color thresholding.

Subscribes to image_raw, detects the largest red blob, and publishes
a Detection2DArray on /detections with a configurable class label.
Also publishes a crosshair overlay via /color_detections so that the
live_inference window draws it on top of the main feed.

Red hue wraps around in HSV (both 0-10 and 160-180), so two masks
are combined.

Optional world-frame ROI gate
-----------------------------
When roi_enabled=True, the HSV mask is multiplied by a binary mask
covering only the projection of a world-frame rectangle (default Z=0,
i.e. the table plane). This lets the KitKat profile reject every red
thing outside a 60x40 cm pick zone in front of the robot — emergency
buttons, red cables, the operator's shirt, etc. The 4 world-frame
corners are projected through the pinhole model using the world->camera
TF + the standard intrinsics (fx, fy, cx, cy), then cached after the
first successful TF lookup (TF is static for the overhead mount).

The projected quad is also published as a geometry_msgs/PolygonStamped
on /pick_roi_polygon (TRANSIENT_LOCAL QoS) so the live_inference window
can draw the ROI on top of the main inference feed for visual tuning.
"""

import rclpy
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSDurabilityPolicy
from sensor_msgs.msg import Image
from geometry_msgs.msg import Point32, PolygonStamped
from vision_msgs.msg import (
    Detection2D,
    Detection2DArray,
    ObjectHypothesisWithPose,
)
from tf2_ros import Buffer, TransformListener, TransformException
from tf2_geometry_msgs import do_transform_point
from geometry_msgs.msg import PointStamped
import cv2
import numpy as np


class RedDetectorNode(Node):
    def __init__(self):
        super().__init__('red_detector')

        # Parameters
        self.declare_parameter('image_topic', 'image_raw')
        self.declare_parameter('label', 'red_toy')
        # Red wraps around — two hue ranges
        self.declare_parameter('h_low_min', 0)
        self.declare_parameter('h_low_max', 10)
        self.declare_parameter('h_high_min', 160)
        self.declare_parameter('h_high_max', 180)
        self.declare_parameter('s_min', 100)
        self.declare_parameter('s_max', 255)
        self.declare_parameter('v_min', 80)
        self.declare_parameter('v_max', 255)
        self.declare_parameter('min_area', 800)
        self.declare_parameter('select_mode', 'largest')  # 'largest' | 'smallest'
        # When False the node skips all HSV / contour work and only computes
        # and publishes the world-frame ROI polygon (used in YOLO-kitkat mode
        # to keep the live_inference overlay alive without running an HSV
        # detector that the pipeline doesn't consume). Detection topics
        # (/detections, /color_detections) and /red_detector/image are not
        # published in this mode.
        self.declare_parameter('detect_enabled', True)

        # ── World-frame ROI gate ────────────────────────────────────────────
        # When enabled, only red contours whose centers lie inside a
        # rectangle on a world-frame Z plane (table by default) are
        # accepted. Defaults are harmless no-ops — without an explicit
        # roi_enabled=True the detector behaves exactly as before.
        self.declare_parameter('roi_enabled',       False)
        self.declare_parameter('roi_x_min_m',       -1.0)   # world X (forward)
        self.declare_parameter('roi_x_max_m',        1.0)
        self.declare_parameter('roi_y_min_m',       -1.0)   # world Y (sideways)
        self.declare_parameter('roi_y_max_m',        1.0)
        self.declare_parameter('roi_z_m',            0.0)   # table plane
        self.declare_parameter('camera_fx',       1482.087)
        self.declare_parameter('camera_fy',       1484.542)
        self.declare_parameter('camera_cx',        991.520)
        self.declare_parameter('camera_cy',        545.064)
        self.declare_parameter('camera_frame',    'camera')
        self.declare_parameter('robot_base_frame','world')

        self.label = self.get_parameter('label').value
        self.h_low_min = self.get_parameter('h_low_min').value
        self.h_low_max = self.get_parameter('h_low_max').value
        self.h_high_min = self.get_parameter('h_high_min').value
        self.h_high_max = self.get_parameter('h_high_max').value
        self.s_min = self.get_parameter('s_min').value
        self.s_max = self.get_parameter('s_max').value
        self.v_min = self.get_parameter('v_min').value
        self.v_max = self.get_parameter('v_max').value
        self.min_area = self.get_parameter('min_area').value
        self.select_mode = self.get_parameter('select_mode').value
        if self.select_mode not in ('largest', 'smallest'):
            self.get_logger().warn(
                f"Invalid select_mode '{self.select_mode}' — falling back to 'largest'")
            self.select_mode = 'largest'

        self.detect_enabled = bool(self.get_parameter('detect_enabled').value)
        self.roi_enabled  = bool(self.get_parameter('roi_enabled').value)
        self.roi_x_min    = float(self.get_parameter('roi_x_min_m').value)
        self.roi_x_max    = float(self.get_parameter('roi_x_max_m').value)
        self.roi_y_min    = float(self.get_parameter('roi_y_min_m').value)
        self.roi_y_max    = float(self.get_parameter('roi_y_max_m').value)
        self.roi_z        = float(self.get_parameter('roi_z_m').value)
        self.camera_fx    = float(self.get_parameter('camera_fx').value)
        self.camera_fy    = float(self.get_parameter('camera_fy').value)
        self.camera_cx    = float(self.get_parameter('camera_cx').value)
        self.camera_cy    = float(self.get_parameter('camera_cy').value)
        self.camera_frame = self.get_parameter('camera_frame').value
        self.base_frame   = self.get_parameter('robot_base_frame').value

        # World-frame corners in CCW order. Used both for the binary
        # mask and the overlay polygon.
        self.roi_world_corners = [
            (self.roi_x_min, self.roi_y_min, self.roi_z),
            (self.roi_x_max, self.roi_y_min, self.roi_z),
            (self.roi_x_max, self.roi_y_max, self.roi_z),
            (self.roi_x_min, self.roi_y_max, self.roi_z),
        ]

        # Cached pixel-space ROI — populated lazily on the first frame
        # where TF lookup succeeds, then frozen (the world->camera TF is
        # published as a static_transform, so it never changes).
        self.roi_pixel_corners = None   # list[(int,int)] or None
        self.roi_mask          = None   # np.uint8 (H,W) or None
        self._roi_lookup_warned = False

        image_topic = self.get_parameter('image_topic').value

        # Publishers
        self.det_pub = self.create_publisher(
            Detection2DArray, '/detections', 10)
        self.color_det_pub = self.create_publisher(
            Detection2DArray, '/color_detections', 10)
        self.annotated_pub = self.create_publisher(
            Image, '/red_detector/image', 10)
        # Latched-style QoS so any display subscriber (live_inference)
        # gets the polygon on connect, not only after the next message.
        roi_qos = QoSProfile(depth=1)
        roi_qos.durability = QoSDurabilityPolicy.TRANSIENT_LOCAL
        self.roi_polygon_pub = self.create_publisher(
            PolygonStamped, '/pick_roi_polygon', roi_qos)

        # TF (only used when roi_enabled — but cheap to always set up).
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # Subscriber — shares the camera feed from live_inference_node
        self.image_sub = self.create_subscription(
            Image, image_topic, self.on_image, 10)

        if self.detect_enabled:
            self.get_logger().info(
                f'RedDetectorNode ready — label={self.label}, '
                f'H=[{self.h_low_min}-{self.h_low_max}] U [{self.h_high_min}-{self.h_high_max}], '
                f'S=[{self.s_min}-{self.s_max}], V=[{self.v_min}-{self.v_max}], '
                f'min_area={self.min_area}')
        else:
            self.get_logger().info(
                'RedDetectorNode in overlay-only mode — HSV detection disabled, '
                'publishing /pick_roi_polygon only.')
        if self.roi_enabled:
            self.get_logger().info(
                f'ROI gate ON — world X=[{self.roi_x_min:.3f},{self.roi_x_max:.3f}] '
                f'Y=[{self.roi_y_min:.3f},{self.roi_y_max:.3f}] Z={self.roi_z:.3f} '
                f'(frames {self.base_frame}->{self.camera_frame})')

    # ------------------------------------------------------------------

    def _try_build_roi_mask(self, image_shape, stamp):
        """
        Project the 4 world-frame ROI corners through the world->camera
        TF and the pinhole intrinsics. On success, populate
        self.roi_pixel_corners + self.roi_mask + publish the polygon
        (once — TF is static).
        """
        if self.roi_mask is not None:
            return  # already cached
        try:
            # Get the world->camera transform (so points expressed in
            # 'base_frame' can be re-expressed in 'camera_frame').
            transform = self.tf_buffer.lookup_transform(
                self.camera_frame, self.base_frame,
                rclpy.time.Time(),
                timeout=Duration(seconds=0.0))
        except TransformException as ex:
            if not self._roi_lookup_warned:
                self.get_logger().info(
                    f'Waiting for TF {self.base_frame}->{self.camera_frame} '
                    f'before computing ROI mask ({ex})')
                self._roi_lookup_warned = True
            return

        pixel_pts = []
        for (xw, yw, zw) in self.roi_world_corners:
            p_world = PointStamped()
            p_world.header.frame_id = self.base_frame
            p_world.point.x = float(xw)
            p_world.point.y = float(yw)
            p_world.point.z = float(zw)
            p_cam = do_transform_point(p_world, transform)
            zc = p_cam.point.z
            if zc <= 1e-3:
                self.get_logger().error(
                    f'ROI corner at world ({xw:.3f},{yw:.3f},{zw:.3f}) '
                    f'projects to camera Z={zc:.3f}m — behind / on the '
                    f'image plane. Check camera frame orientation.')
                return
            u = self.camera_fx * p_cam.point.x / zc + self.camera_cx
            v = self.camera_fy * p_cam.point.y / zc + self.camera_cy
            pixel_pts.append((int(round(u)), int(round(v))))

        h, w = image_shape[:2]
        mask = np.zeros((h, w), dtype=np.uint8)
        poly = np.array(pixel_pts, dtype=np.int32).reshape((-1, 1, 2))
        cv2.fillPoly(mask, [poly], 255)

        self.roi_pixel_corners = pixel_pts
        self.roi_mask = mask

        # Publish polygon for the live_inference overlay (TRANSIENT_LOCAL
        # — late joiners still receive the last value).
        poly_msg = PolygonStamped()
        poly_msg.header.stamp = stamp
        # frame_id is informational here — coords are already pixels,
        # not a TF frame — but mark it as the image frame for context.
        poly_msg.header.frame_id = self.camera_frame
        for (u, v) in pixel_pts:
            pt = Point32()
            pt.x = float(u)
            pt.y = float(v)
            pt.z = 0.0
            poly_msg.polygon.points.append(pt)
        self.roi_polygon_pub.publish(poly_msg)

        self.get_logger().info(
            f'ROI projected to pixels: {pixel_pts} '
            f'(image {w}x{h})')

    def on_image(self, msg: Image):
        # Convert ROS Image → numpy
        if msg.encoding == 'bgr8':
            frame = np.frombuffer(msg.data, dtype=np.uint8).reshape(
                msg.height, msg.width, 3)
        elif msg.encoding == 'rgb8':
            raw = np.frombuffer(msg.data, dtype=np.uint8).reshape(
                msg.height, msg.width, 3)
            frame = cv2.cvtColor(raw, cv2.COLOR_RGB2BGR)
        else:
            self.get_logger().warn(
                f'Unsupported encoding: {msg.encoding}',
                throttle_duration_sec=5.0)
            return

        # Lazy ROI projection: needs the image shape, and needs TF to be
        # up. Cheap no-op once cached.
        if self.roi_enabled:
            self._try_build_roi_mask(frame.shape, msg.header.stamp)

        # Overlay-only mode (YOLO kitkat path): we exist solely to keep
        # /pick_roi_polygon latched for the live_inference window. No HSV,
        # no /detections, no annotated republish.
        if not self.detect_enabled:
            return

        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

        # Two red ranges — combine them
        mask_low = cv2.inRange(
            hsv,
            np.array([self.h_low_min, self.s_min, self.v_min]),
            np.array([self.h_low_max, self.s_max, self.v_max]))
        mask_high = cv2.inRange(
            hsv,
            np.array([self.h_high_min, self.s_min, self.v_min]),
            np.array([self.h_high_max, self.s_max, self.v_max]))
        mask = cv2.bitwise_or(mask_low, mask_high)

        # World-frame ROI gate — restrict the candidate region to the
        # projected rectangle. Anything red outside is zeroed out before
        # contour-finding, so red toys, emergency buttons, shirts, etc.
        # never become detections at all.
        if self.roi_enabled and self.roi_mask is not None:
            mask = cv2.bitwise_and(mask, self.roi_mask)

        # Clean up noise
        kernel = np.ones((5, 5), np.uint8)
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)

        contours, _ = cv2.findContours(
            mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        det_array = Detection2DArray()
        det_array.header.stamp = msg.header.stamp
        det_array.header.frame_id = msg.header.frame_id

        annotated = frame.copy()

        # Draw the ROI polygon on the local annotated view so /red_detector/image
        # is self-contained for tuning even if the main display isn't running.
        if self.roi_enabled and self.roi_pixel_corners is not None:
            poly = np.array(self.roi_pixel_corners, dtype=np.int32).reshape((-1, 1, 2))
            cv2.polylines(annotated, [poly], isClosed=True,
                          color=(0, 255, 255), thickness=2)
            anchor = self.roi_pixel_corners[0]
            cv2.putText(
                annotated, 'KitKat ROI',
                (anchor[0] + 6, max(0, anchor[1] - 8)),
                cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 255), 2)

        # Filter candidates above min_area, then pick largest or smallest
        candidates = [c for c in contours if cv2.contourArea(c) >= self.min_area]
        if candidates:
            picker = max if self.select_mode == 'largest' else min
            chosen = picker(candidates, key=cv2.contourArea)
            area = cv2.contourArea(chosen)

            x, y, w, h = cv2.boundingRect(chosen)
            cx = x + w / 2.0
            cy = y + h / 2.0

            det = Detection2D()
            det.bbox.center.position.x = cx
            det.bbox.center.position.y = cy
            det.bbox.size_x = float(w)
            det.bbox.size_y = float(h)

            hyp = ObjectHypothesisWithPose()
            hyp.hypothesis.class_id = self.label
            # Any blob above min_area is a confident detection — fixed high score
            hyp.hypothesis.score = 0.9
            det.results.append(hyp)
            det_array.detections.append(det)

            # Draw crosshair + label on local annotated image
            ci = (int(cx), int(cy))
            size = 30
            color = (0, 0, 255)  # red in BGR
            cv2.line(annotated, (ci[0] - size, ci[1]),
                     (ci[0] + size, ci[1]), color, 2)
            cv2.line(annotated, (ci[0], ci[1] - size),
                     (ci[0], ci[1] + size), color, 2)
            cv2.circle(annotated, ci, 6, color, -1)
            cv2.putText(
                annotated,
                f'{self.label} [{self.select_mode}] ({area:.0f}px)',
                (ci[0] + 20, ci[1] - 15),
                cv2.FONT_HERSHEY_SIMPLEX, 0.6, color, 2)

        self.det_pub.publish(det_array)
        self.color_det_pub.publish(det_array)

        # Publish annotated image (viewable in rqt_image_view)
        out_msg = Image()
        out_msg.header = det_array.header
        out_msg.height = annotated.shape[0]
        out_msg.width = annotated.shape[1]
        out_msg.encoding = 'bgr8'
        out_msg.step = annotated.shape[1] * 3
        out_msg.data = annotated.tobytes()
        self.annotated_pub.publish(out_msg)


def main(args=None):
    rclpy.init(args=args)
    node = RedDetectorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
