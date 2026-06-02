#!/usr/bin/env python3
"""
tf_node — pixel → world coordinate transform node
===================================================
Subscribes to pixel-coordinate topics (e.g. /box_center from hsv_image_node),
applies a configurable camera-offset + pixel/mm scale conversion, and publishes
the result as physical coordinates in the robot-arm base frame.

Transformation model (pinhole flat-plane projection)
----------------------------------------------------
  dx_pix = pixel_x - image_width/2       (pixel offset from optical centre)
  dy_pix = pixel_y - image_height/2
  dx_mm  = dx_pix * scale_x              (physical offset on target plane)
  dy_mm  = dy_pix * scale_y
  target_x = camera_x + dx_mm            (absolute position in arm base frame)
  target_y = camera_y + dy_mm
  target_z = target_z                    (configured plane height)

All linear dimensions are in **millimetres** (params & published messages).
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Point, PointStamped, TransformStamped
from tf2_ros import TransformBroadcaster
import math


class TFNode(Node):
    """Convert pixel detection coordinates → physical arm-base coordinates."""

    def __init__(self):
        super().__init__('tf_node')

        # ── topics ──────────────────────────────────────────
        self.declare_parameter('input_topic', '/box_center')
        self.declare_parameter('output_topic', '/world_xyz')

        # ── camera position in arm base frame (mm) ──────────
        self.declare_parameter('camera_x', -26.0)
        self.declare_parameter('camera_y', 270.0)
        self.declare_parameter('camera_z', 365.0)

        # ── pixel → mm scale (mm / pixel) ──────────────────
        self.declare_parameter('scale_x', 0.5)
        self.declare_parameter('scale_y', 0.5)

        # ── image geometry ──────────────────────────────────
        self.declare_parameter('image_width', 640)
        self.declare_parameter('image_height', 480)

        # ── target-plane height in arm base frame (mm) ──────
        self.declare_parameter('target_z', 0.0)

        # ── stale timeout: 超过此时间无新检测则清零 (s) ─────
        self.declare_parameter('stale_timeout', 0.5)

        # ── TF frame names ──────────────────────────────────
        self.declare_parameter('arm_base_frame', 'Base')
        self.declare_parameter('target_frame', 'target')

        # ── load params ─────────────────────────────────────
        self.input_topic      = self.get_parameter('input_topic').value
        self.output_topic     = self.get_parameter('output_topic').value

        self.camera_x = self.get_parameter('camera_x').value
        self.camera_y = self.get_parameter('camera_y').value
        self.camera_z = self.get_parameter('camera_z').value

        self.scale_x = self.get_parameter('scale_x').value
        self.scale_y = self.get_parameter('scale_y').value

        self.image_w = self.get_parameter('image_width').value
        self.image_h = self.get_parameter('image_height').value
        self.cx = self.image_w / 2.0  # optical centre (pixels)
        self.cy = self.image_h / 2.0

        self.target_z      = self.get_parameter('target_z').value
        self.stale_timeout  = self.get_parameter('stale_timeout').value

        self.arm_base_frame = self.get_parameter('arm_base_frame').value
        self.target_frame   = self.get_parameter('target_frame').value

        # ── pub / sub / tf ──────────────────────────────────
        self._create_sub = self.create_subscription(
            Point, self.input_topic, self._callback, 10)

        self._point_pub = self.create_publisher(
            PointStamped, self.output_topic, 10)

        self._tf_broadcaster = TransformBroadcaster(self)

        self._latest_pixel = None       # most recent pixel detection
        self._last_pixel_time = 0.0     # timestamp of last pixel callback

        # periodic publish (10 Hz) so downstream consumers see steady data
        self._timer = self.create_timer(0.1, self._publish_loop)

        self._log_config()

    # ── helpers ────────────────────────────────────────────

    def _log_config(self):
        self.get_logger().info('=' * 60)
        self.get_logger().info('  TF Node — pixel → world transform')
        self.get_logger().info('=' * 60)
        self.get_logger().info(f'  Input:          {self.input_topic}')
        self.get_logger().info(f'  Output:         {self.output_topic} (PointStamped)')
        self.get_logger().info(f'  Arm base frame: {self.arm_base_frame}')
        self.get_logger().info(f'  Target frame:   {self.target_frame}')
        self.get_logger().info('-' * 60)
        self.get_logger().info(f'  Camera pos (mm):   '
                               f'X={self.camera_x:.1f}, Y={self.camera_y:.1f}, Z={self.camera_z:.1f}')
        self.get_logger().info(f'  Scale (mm/px):     '
                               f'X={self.scale_x:.4f}, Y={self.scale_y:.4f}')
        self.get_logger().info(f'  Image size:        {self.image_w}×{self.image_h}')
        self.get_logger().info(f'  Target plane Z:    {self.target_z:.1f} mm')
        self.get_logger().info(f'  Stale timeout:     {self.stale_timeout:.1f} s')
        self.get_logger().info('=' * 60)

    # ── pixel → world transform ────────────────────────────

    def pixel_to_world(self, px, py):
        """Convert a single pixel coordinate to physical arm-base coordinates (mm)."""
        dx = (px - self.cx) * self.scale_x
        dy = (py - self.cy) * self.scale_y
        return (self.camera_x + dx, self.camera_y + dy, self.target_z)

    # ── callback ───────────────────────────────────────────

    def _callback(self, msg: Point):
        """Buffer the latest pixel detection."""
        self._latest_pixel = (msg.x, msg.y)
        self._last_pixel_time = self.get_clock().now().nanoseconds * 1e-9

    # ── publish loop ───────────────────────────────────────

    def _publish_loop(self):
        # 超时清零：目标丢失后停止发布残留坐标
        now = self.get_clock().now().nanoseconds * 1e-9
        if self._latest_pixel is not None:
            if now - self._last_pixel_time > self.stale_timeout:
                self._latest_pixel = None
                return

        if self._latest_pixel is None:
            return

        px, py = self._latest_pixel
        wx, wy, wz = self.pixel_to_world(px, py)

        stamp = self.get_clock().now().to_msg()

        # PointStamped
        ps = PointStamped()
        ps.header.stamp    = stamp
        ps.header.frame_id = self.arm_base_frame
        ps.point.x = float(wx)
        ps.point.y = float(wy)
        ps.point.z = float(wz)
        self._point_pub.publish(ps)

        # TF broadcast
        tf_msg = TransformStamped()
        tf_msg.header.stamp    = stamp
        tf_msg.header.frame_id = self.arm_base_frame
        tf_msg.child_frame_id  = self.target_frame
        tf_msg.transform.translation.x = wx / 1000.0   # mm → m for TF
        tf_msg.transform.translation.y = wy / 1000.0
        tf_msg.transform.translation.z = wz / 1000.0
        tf_msg.transform.rotation.w    = 1.0            # identity rotation
        self._tf_broadcaster.sendTransform(tf_msg)


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = TFNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        print('\nTF Node stopped.')
    except Exception as e:
        print(f'Error: {e}')
        import traceback
        traceback.print_exc()
    finally:
        if node:
            node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
