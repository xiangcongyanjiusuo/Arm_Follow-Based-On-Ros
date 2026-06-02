#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from geometry_msgs.msg import Point
from cv_bridge import CvBridge
import cv2
import numpy as np

# ── color palette ──────────────────────────────────────────────
ACCENT  = (0, 220, 255)     # gold/amber for highlights
GREEN   = (0, 255, 128)     # mint green
RED     = (50, 50, 255)     # warm red
WHITE   = (255, 255, 255)
DIM     = (180, 180, 180)
PANEL   = (30, 30, 30)      # dark panel background


class HSVImageNode(Node):

    def __init__(self):
        super().__init__('hsv_image_node')

        self.declare_parameter('image_topic', '/image_raw')

        self.declare_parameter('h_min', 110)
        self.declare_parameter('h_max', 117)
        self.declare_parameter('s_min', 146)
        self.declare_parameter('s_max', 255)
        self.declare_parameter('v_min', 73)
        self.declare_parameter('v_max', 255)
        self.declare_parameter('erode_iter', 3)
        self.declare_parameter('dilate_iter', 5)
        self.declare_parameter('min_area', 500)
        self.declare_parameter('publish_topic', '/box_center')

        self.image_topic = self.get_parameter('image_topic').value

        self.h_min = self.get_parameter('h_min').value
        self.h_max = self.get_parameter('h_max').value
        self.s_min = self.get_parameter('s_min').value
        self.s_max = self.get_parameter('s_max').value
        self.v_min = self.get_parameter('v_min').value
        self.v_max = self.get_parameter('v_max').value
        self.erode_iter = self.get_parameter('erode_iter').value
        self.dilate_iter = self.get_parameter('dilate_iter').value
        self.min_area = self.get_parameter('min_area').value

        self.bridge = CvBridge()
        self.latest_frame = None
        self.frame_count = 0

        # ── window: main detection view ──
        self.WIN_MAIN = 'HSV Tuner'
        cv2.namedWindow(self.WIN_MAIN, cv2.WINDOW_NORMAL)
        cv2.resizeWindow(self.WIN_MAIN, 960, 640)

        # ── window: compact slider panel ──
        self.WIN_CTRL = 'Controls'
        cv2.namedWindow(self.WIN_CTRL, cv2.WINDOW_NORMAL)
        cv2.resizeWindow(self.WIN_CTRL, 420, 320)

        cv2.createTrackbar('H Min', self.WIN_CTRL, self.h_min, 179, self._nop)
        cv2.createTrackbar('H Max', self.WIN_CTRL, self.h_max, 179, self._nop)
        cv2.createTrackbar('S Min', self.WIN_CTRL, self.s_min, 255, self._nop)
        cv2.createTrackbar('S Max', self.WIN_CTRL, self.s_max, 255, self._nop)
        cv2.createTrackbar('V Min', self.WIN_CTRL, self.v_min, 255, self._nop)
        cv2.createTrackbar('V Max', self.WIN_CTRL, self.v_max, 255, self._nop)
        cv2.createTrackbar('Erode',  self.WIN_CTRL, self.erode_iter, 10, self._nop)
        cv2.createTrackbar('Dilate', self.WIN_CTRL, self.dilate_iter, 10, self._nop)

        self.image_sub = self.create_subscription(
            Image, self.image_topic, self.image_callback, 10)

        publish_topic = self.get_parameter('publish_topic').value
        self.center_pub = self.create_publisher(Point, publish_topic, 10)

        self.latest_objects = []
        self.display_timer = self.create_timer(0.033, self.display_callback)

        self.get_logger().info('HSV Tuner ready.')
        self.get_logger().info(f'Image topic:  {self.image_topic}')
        self.get_logger().info(f'Publish:      {publish_topic}')
        self.get_logger().info('Keys: [q]uit  [s]creenshot')

    # ── helpers ────────────────────────────────────────────────

    def _nop(self, x):
        pass

    def _read_sliders(self):
        self.h_min       = cv2.getTrackbarPos('H Min',  self.WIN_CTRL)
        self.h_max       = cv2.getTrackbarPos('H Max',  self.WIN_CTRL)
        self.s_min       = cv2.getTrackbarPos('S Min',  self.WIN_CTRL)
        self.s_max       = cv2.getTrackbarPos('S Max',  self.WIN_CTRL)
        self.v_min       = cv2.getTrackbarPos('V Min',  self.WIN_CTRL)
        self.v_max       = cv2.getTrackbarPos('V Max',  self.WIN_CTRL)
        self.erode_iter  = cv2.getTrackbarPos('Erode',  self.WIN_CTRL)
        self.dilate_iter = cv2.getTrackbarPos('Dilate', self.WIN_CTRL)

    @staticmethod
    def _draw_panel(img, x, y, w, h, color=PANEL, alpha=0.75):
        """Draw a semi-transparent panel on a BGR image."""
        overlay = img.copy()
        cv2.rectangle(overlay, (x, y), (x + w, y + h), color, -1)
        cv2.addWeighted(overlay, alpha, img, 1 - alpha, 0, img)

    @staticmethod
    def _draw_label(img, text, pos, font=cv2.FONT_HERSHEY_SIMPLEX,
                    scale=0.55, color=WHITE, thickness=1, shadow=True):
        x, y = pos
        if shadow:
            cv2.putText(img, text, (x + 1, y + 1), font, scale, (0, 0, 0), thickness + 1)
        cv2.putText(img, text, (x, y), font, scale, color, thickness)

    # ── callbacks ──────────────────────────────────────────────

    def image_callback(self, msg):
        try:
            self.latest_frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            self.frame_count += 1
        except Exception as e:
            self.get_logger().warn(f'Image conversion error: {e}')

    # ── processing ─────────────────────────────────────────────

    def process_frame(self, frame):
        self._read_sliders()

        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        lower = np.array([self.h_min, self.s_min, self.v_min])
        upper = np.array([self.h_max, self.s_max, self.v_max])
        mask = cv2.inRange(hsv, lower, upper)

        kernel = np.ones((5, 5), np.uint8)
        if self.erode_iter > 0:
            mask = cv2.erode(mask, kernel, iterations=self.erode_iter)
        if self.dilate_iter > 0:
            mask = cv2.dilate(mask, kernel, iterations=self.dilate_iter)

        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        result = frame.copy()
        detected_objects = []

        for cnt in contours:
            area = cv2.contourArea(cnt)
            if area > self.min_area:
                x, y, w, h = cv2.boundingRect(cnt)
                cx, cy = x + w // 2, y + h // 2

                # bounding box — rounded-corner look via thicker rect
                cv2.rectangle(result, (x, y), (x + w, y + h), GREEN, 2)
                # corner accents
                for (px, py) in [(x, y), (x + w, y), (x, y + h), (x + w, y + h)]:
                    cv2.line(result, (px, py),
                             (px + (8 if px == x else -8), py), GREEN, 2)
                    cv2.line(result, (px, py),
                             (px, py + (8 if py == y else -8)), GREEN, 2)

                # crosshair at center
                cv2.drawMarker(result, (cx, cy), ACCENT,
                               cv2.MARKER_TILTED_CROSS, 12, 2)

                # compact label above box
                label = f'{cx},{cy}'
                (tw, th), _ = cv2.getTextSize(label, cv2.FONT_HERSHEY_SIMPLEX, 0.45, 1)
                label_y = y - 6 if y > th + 6 else y + h + th + 6
                self._draw_panel(result, x, label_y - th - 2, tw + 6, th + 4)
                self._draw_label(result, label, (x + 3, label_y),
                                 scale=0.45, color=WHITE, shadow=False)

                detected_objects.append({
                    'center': (cx, cy),
                    'bbox': (x, y, w, h),
                    'area': area,
                })

        # ── info bar (top) ──
        fw = frame.shape[1]
        BAR_H = 56
        GAP = 10
        self._draw_panel(result, 0, 0, fw, BAR_H)

        # ─ row 1: HSV range (left)  ─  key hints (right) ─
        y1 = 18
        r1_left = f'H[{self.h_min}..{self.h_max}]   S[{self.s_min}..{self.s_max}]   V[{self.v_min}..{self.v_max}]'
        self._draw_label(result, r1_left, (GAP, y1), scale=0.42, color=DIM, shadow=False)

        keys = '[q]uit  [s]ave'
        (kw, _), _ = cv2.getTextSize(keys, cv2.FONT_HERSHEY_SIMPLEX, 0.42, 1)
        self._draw_label(result, keys, (fw - kw - GAP, y1), scale=0.42, color=DIM, shadow=False)

        # ─ row 2: object count (left)  ─  brightness (right, conditional) ─
        y2 = 40
        count_str = f'Objects: {len(detected_objects)}'
        self._draw_label(result, count_str, (GAP, y2), scale=0.5, color=GREEN, shadow=False)

        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        mean_brightness = np.mean(gray)
        if mean_brightness < 60:
            hint = f'Low light ({mean_brightness:.0f})  —  check env / exposure'
        else:
            hint = f'Brightness: {mean_brightness:.0f}'
        (hw, _), _ = cv2.getTextSize(hint, cv2.FONT_HERSHEY_SIMPLEX, 0.42, 1)
        hint_color = (80, 160, 255) if mean_brightness < 60 else DIM
        self._draw_label(result, hint, (fw - hw - GAP, y2), scale=0.42, color=hint_color, shadow=False)

        # ── mask pip (bottom-right) ──
        mask_color = cv2.cvtColor(mask, cv2.COLOR_GRAY2BGR)
        # tint detected areas green so they pop
        mask_color[mask > 0] = (0, 180, 0)

        pip_w, pip_h = 180, 135
        pip = cv2.resize(mask_color, (pip_w, pip_h))
        px0 = frame.shape[1] - pip_w - 8
        py0 = frame.shape[0] - pip_h - 8
        # thin border
        cv2.rectangle(result, (px0 - 1, py0 - 1),
                      (px0 + pip_w + 1, py0 + pip_h + 1), WHITE, 1)
        result[py0:py0 + pip_h, px0:px0 + pip_w] = pip
        self._draw_label(result, 'Mask', (px0 + 4, py0 - 6),
                         scale=0.35, color=DIM, shadow=False)

        return result, mask, detected_objects

    # ── publish ────────────────────────────────────────────────

    def _publish_centers(self, objects):
        """Publish detected object centers as Point messages (x=pixel_x, y=pixel_y, z=-1 placeholder)."""
        for obj in objects:
            msg = Point()
            msg.x = float(obj['center'][0])
            msg.y = float(obj['center'][1])
            msg.z = -1.0
            self.center_pub.publish(msg)

    # ── display loop ───────────────────────────────────────────

    def display_callback(self):
        if self.latest_frame is None:
            return

        result, _mask, objects = self.process_frame(self.latest_frame)
        self.latest_objects = objects
        self._publish_centers(objects)
        cv2.imshow(self.WIN_MAIN, result)

        key = cv2.waitKey(1) & 0xFF

        if key == ord('q'):
            self.get_logger().info('Quit.')
            raise KeyboardInterrupt()
        elif key == ord('s'):
            fname = f'hsv_tuner_{self.frame_count}.png'
            cv2.imwrite(fname, result)
            self.get_logger().info(f'Screenshot saved → {fname}')

    def destroy_node(self):
        self.get_logger().info('Shutting down HSV Tuner...')
        cv2.destroyWindow(self.WIN_MAIN)
        cv2.destroyWindow(self.WIN_CTRL)
        cv2.destroyAllWindows()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = HSVImageNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        print('\nHSV Tuner stopped.')
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
