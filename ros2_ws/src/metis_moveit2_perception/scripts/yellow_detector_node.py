#!/usr/bin/env python3
# (C) Copyright Axelera AI 2026
# All Rights Reserved
"""
Yellow object detector using HSV color thresholding.

Subscribes to image_raw, detects the largest yellow blob, and publishes
a Detection2DArray on /detections with a configurable class label.
Draws a crosshair on the annotated image.

Works alongside the YOLO inference node — both publish to /detections,
and the pick_and_place_node filters by class_id.
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from vision_msgs.msg import (
    Detection2D,
    Detection2DArray,
    ObjectHypothesisWithPose,
)
import cv2
import numpy as np


class YellowDetectorNode(Node):
    def __init__(self):
        super().__init__('yellow_detector')

        # Parameters
        self.declare_parameter('image_topic', 'image_raw')
        self.declare_parameter('label', 'duck')
        self.declare_parameter('h_min', 20)
        self.declare_parameter('h_max', 35)
        self.declare_parameter('s_min', 100)
        self.declare_parameter('s_max', 255)
        self.declare_parameter('v_min', 100)
        self.declare_parameter('v_max', 255)
        self.declare_parameter('min_area', 1000)
        self.declare_parameter('select_mode', 'largest')  # 'largest' | 'smallest'

        self.label = self.get_parameter('label').value
        self.h_min = self.get_parameter('h_min').value
        self.h_max = self.get_parameter('h_max').value
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

        image_topic = self.get_parameter('image_topic').value

        # Publishers
        self.det_pub = self.create_publisher(
            Detection2DArray, '/detections', 10)
        self.color_det_pub = self.create_publisher(
            Detection2DArray, '/color_detections', 10)
        self.annotated_pub = self.create_publisher(
            Image, '/yellow_detector/image', 10)

        # Subscriber — shares the camera feed from live_inference_node
        self.image_sub = self.create_subscription(
            Image, image_topic, self.on_image, 10)

        self.get_logger().info(
            f'YellowDetectorNode ready — label={self.label}, '
            f'HSV=[{self.h_min}-{self.h_max}, {self.s_min}-{self.s_max}, '
            f'{self.v_min}-{self.v_max}], min_area={self.min_area}')

    # ------------------------------------------------------------------

    def on_image(self, msg: Image):
        # Convert ROS Image → numpy (avoids cv_bridge dependency)
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

        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        mask = cv2.inRange(
            hsv,
            np.array([self.h_min, self.s_min, self.v_min]),
            np.array([self.h_max, self.s_max, self.v_max]))

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

            # Draw crosshair + label on chosen blob
            ci = (int(cx), int(cy))
            size = 30
            cv2.line(annotated, (ci[0] - size, ci[1]),
                     (ci[0] + size, ci[1]), (0, 255, 0), 2)
            cv2.line(annotated, (ci[0], ci[1] - size),
                     (ci[0], ci[1] + size), (0, 255, 0), 2)
            cv2.circle(annotated, ci, 6, (0, 255, 0), -1)
            cv2.putText(
                annotated,
                f'{self.label} [{self.select_mode}] ({area:.0f}px)',
                (ci[0] + 20, ci[1] - 15),
                cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)

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
    node = YellowDetectorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
