#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2


class CameraNativeNode(Node):

    def __init__(self):
        super().__init__('camera_native_node')
        
        self.declare_parameter('image_topic', '/image_raw')
        self.declare_parameter('window_name', 'Camera Display')
        self.declare_parameter('show_info', True)
        
        self.image_topic = self.get_parameter('image_topic').value
        self.window_name = self.get_parameter('window_name').value
        self.show_info = self.get_parameter('show_info').value
        
        self.bridge = CvBridge()
        self.latest_frame = None
        self.frame_count = 0
        
        cv2.namedWindow(self.window_name, cv2.WINDOW_NORMAL)
        cv2.resizeWindow(self.window_name, 640, 480)
        
        self.image_sub = self.create_subscription(
            Image,
            self.image_topic,
            self.image_callback,
            10
        )
        
        self.display_timer = self.create_timer(0.033, self.display_callback)
        
        self.get_logger().info(f'Camera Native Node has been initialized.')
        self.get_logger().info(f'Subscribing to: {self.image_topic}')
        self.get_logger().info(f'Press "q" to quit, "s" to save screenshot')

    def image_callback(self, msg):
        try:
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            self.latest_frame = frame
            self.frame_count += 1
        except Exception as e:
            self.get_logger().warn(f'Error converting image: {e}')

    def display_callback(self):
        if self.latest_frame is None:
            return
        
        display_frame = self.latest_frame.copy()
        
        if self.show_info:
            info_text = f'Frame: {self.frame_count} | Topic: {self.image_topic}'
            cv2.putText(display_frame, info_text, (10, 30),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
            
            fps_text = f'Press "q" to quit, "s" to save'
            cv2.putText(display_frame, fps_text, (10, 60),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 0), 2)
        
        cv2.imshow(self.window_name, display_frame)
        
        key = cv2.waitKey(1) & 0xFF
        
        if key == ord('q'):
            self.get_logger().info('Quit key pressed. Shutting down...')
            raise KeyboardInterrupt()
        elif key == ord('s'):
            filename = f'screenshot_{self.frame_count}.png'
            cv2.imwrite(filename, self.latest_frame)
            self.get_logger().info(f'Screenshot saved: {filename}')
        elif key == ord('f'):
            cv2.setWindowProperty(self.window_name, 
                                 cv2.WND_PROP_FULLSCREEN,
                                 cv2.WINDOW_FULLSCREEN)

    def destroy_node(self):
        self.get_logger().info('Shutting down camera node...')
        cv2.destroyWindow(self.window_name)
        cv2.destroyAllWindows()
        self.get_logger().info('Display window closed.')
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    
    camera_native_node = None
    try:
        camera_native_node = CameraNativeNode()
        rclpy.spin(camera_native_node)
    except KeyboardInterrupt:
        print("\nKeyboard interrupt received. Shutting down...")
    except Exception as e:
        print(f'Error: {e}')
    finally:
        if camera_native_node:
            camera_native_node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
