# (C) Copyright Axelera AI 2026
# All Rights Reserved
"""
Wrist tracker. Converts a dynamic wrist TF into a MoveIt target pose.

Periodically looks up `world → <wrist_frame>` from the TF tree. If the
transform is fresh (younger than `max_tf_age_s`) and has moved at least
`min_movement_m` since the last publication, a PoseStamped is published to
`/target_hover_pose` with Z offset by `z_offset_m`. The downstream
ArmControllerNode (metis_moveit2_perception) consumes this and plans to it
via MoveIt.

This node doesn't command the arm directly — it reuses the existing
/target_hover_pose → ArmControllerNode interface, so planning cadence,
velocity/acceleration scaling, and safety checks stay in one place.
"""

import math

import rclpy
from rclpy.node import Node
from rclpy.duration import Duration

from geometry_msgs.msg import PoseStamped
from tf2_ros import Buffer, TransformListener, LookupException, ExtrapolationException


class WristTrackerNode(Node):

    def __init__(self):
        super().__init__('wrist_tracker_node')

        self.declare_parameter('world_frame', 'world')
        self.declare_parameter('wrist_frame', 'right_wrist')
        self.declare_parameter('output_topic', '/target_hover_pose')
        self.declare_parameter('x_offset_m', 0.0)
        self.declare_parameter('z_offset_m', 0.10)
        self.declare_parameter('publish_rate_hz', 2.0)
        self.declare_parameter('max_tf_age_s', 1.0)
        self.declare_parameter('min_movement_m', 0.03)
        # Approach mode:
        #   'horizontal' — gripper points +Y (toward operator), standoff
        #                  applied in -Y (hover 5cm closer to the base than
        #                  the wrist). Perpendicular to a hand held out in
        #                  front, and dramatically expands the reachable
        #                  workspace compared to 'down'.
        #   'down'       — legacy: gripper points -Z, standoff applied in +Z
        #                  (hover 5cm above the wrist).
        self.declare_parameter('approach_mode', 'horizontal')

        self.world_frame = self.get_parameter('world_frame').value
        self.wrist_frame = self.get_parameter('wrist_frame').value
        self.x_offset = float(self.get_parameter('x_offset_m').value)
        self.z_offset = float(self.get_parameter('z_offset_m').value)
        self.rate_hz = float(self.get_parameter('publish_rate_hz').value)
        self.max_age = float(self.get_parameter('max_tf_age_s').value)
        self.min_move = float(self.get_parameter('min_movement_m').value)
        self.approach_mode = self.get_parameter('approach_mode').value
        output_topic = self.get_parameter('output_topic').value

        if self.approach_mode not in ('horizontal', 'down'):
            self.get_logger().warning(
                f"unknown approach_mode='{self.approach_mode}', "
                "falling back to 'horizontal'")
            self.approach_mode = 'horizontal'

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.pose_pub = self.create_publisher(PoseStamped, output_topic, 1)

        self._last_published_xyz = None
        self.timer = self.create_timer(1.0 / self.rate_hz, self._tick)

        self.get_logger().info('WristTrackerNode ready')
        self.get_logger().info(f'  wrist_frame     : {self.wrist_frame}')
        self.get_logger().info(f'  output_topic    : {output_topic}')
        self.get_logger().info(f'  z_offset_m      : {self.z_offset}')
        self.get_logger().info(f'  publish_rate_hz : {self.rate_hz}')
        self.get_logger().info(f'  max_tf_age_s    : {self.max_age}')
        self.get_logger().info(f'  min_movement_m  : {self.min_move}')
        self.get_logger().info(f'  x_offset_m      : {self.x_offset}')
        self.get_logger().info(f'  approach_mode   : {self.approach_mode}')

    def _tick(self):
        try:
            tf = self.tf_buffer.lookup_transform(
                self.world_frame,
                self.wrist_frame,
                rclpy.time.Time(),  # latest available
            )
        except (LookupException, ExtrapolationException):
            self.get_logger().warning(
                f'{self.world_frame} ← {self.wrist_frame} TF not yet available',
                throttle_duration_sec=5)
            return

        # Reject stale transforms (operator left the frame and we shouldn't
        # keep commanding the arm to their last-seen position).
        tf_stamp = rclpy.time.Time.from_msg(tf.header.stamp)
        now = self.get_clock().now()
        age_s = (now - tf_stamp).nanoseconds / 1e9
        if age_s > self.max_age:
            self.get_logger().warning(
                f'Wrist TF is stale ({age_s:.2f}s > {self.max_age:.2f}s) — '
                'not publishing target',
                throttle_duration_sec=5)
            return

        t = tf.transform.translation
        wx, wy, wz = float(t.x), float(t.y), float(t.z)

        # Standoff and orientation per approach mode. The TCP (tool_link)
        # lands `z_offset` metres short of the wrist along the approach axis,
        # so the gripper tip ends up near — not on top of — the hand.
        if self.approach_mode == 'horizontal':
            # Gripper points world +Y; approach along +Y ⇒ standoff on -Y.
            # Rotation -90° about world X takes tool +Z → world +Y.
            x, y, z = wx + self.x_offset, wy - self.z_offset, wz
            qx, qy, qz, qw = -0.70710678, 0.0, 0.0, 0.70710678
        else:  # 'down'
            # Gripper points world -Z; approach downward ⇒ standoff on +Z.
            # Finger-opening axis rotated 90° CCW about world Z (viewed from
            # above) relative to the naive (1,0,0,0) down orientation, so
            # the fingers open along world +Y instead of +X.
            # Composed quaternion: q_z_90 * q_x_180.
            x, y, z = wx + self.x_offset, wy, wz + self.z_offset
            qx, qy, qz, qw = 0.70710678, 0.70710678, 0.0, 0.0

        # Dedupe: don't spam replans if the hand hasn't moved meaningfully.
        if self._last_published_xyz is not None:
            dx = x - self._last_published_xyz[0]
            dy = y - self._last_published_xyz[1]
            dz = z - self._last_published_xyz[2]
            if math.sqrt(dx * dx + dy * dy + dz * dz) < self.min_move:
                return

        msg = PoseStamped()
        msg.header.stamp = now.to_msg()
        msg.header.frame_id = self.world_frame
        msg.pose.position.x = x
        msg.pose.position.y = y
        msg.pose.position.z = z
        msg.pose.orientation.x = qx
        msg.pose.orientation.y = qy
        msg.pose.orientation.z = qz
        msg.pose.orientation.w = qw

        self.pose_pub.publish(msg)
        self._last_published_xyz = (x, y, z)

        self.get_logger().info(
            f'target pose published: ({x:.3f}, {y:.3f}, {z:.3f})',
            throttle_duration_sec=2)


def main():
    rclpy.init()
    node = WristTrackerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
