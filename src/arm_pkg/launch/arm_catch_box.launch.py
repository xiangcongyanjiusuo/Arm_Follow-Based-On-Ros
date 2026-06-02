#!/usr/bin/env python3
"""
arm_catch_box.launch.py — 机械臂视觉抓取完整流程
=================================================
启动节点链:
  usb_cam → camera_native_node (查看)
          → hsv_image_node → tf_node → arm_catch_box_node → arm_joint_node

数据流:
  usb_cam:/image_raw → /box_center → /world_xyz → cmd_angles → 舵机
"""

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([

        # ── 0. USB 相机驱动 (发布 /image_raw) ─────────────────
        Node(
            package='usb_cam',
            executable='usb_cam_node_exe',
            name='usb_cam',
            output='screen',
            parameters=[{
                'video_device': '/dev/video0',
                'image_width': 640,
                'image_height': 480,
                'framerate': 30.0,
            }],
        ),

        # ── 1. 相机查看窗口 (订阅 /image_raw) ─────────────────
        Node(
            package='vision_pkg',
            executable='camera_native_node',
            name='camera_native_node',
            output='screen',
        ),

        # ── 2. HSV 颜色检测 → /box_center (像素坐标) ─────────
        Node(
            package='vision_pkg',
            executable='hsv_image_node',
            name='hsv_image_node',
            output='screen',
            parameters=[{
                'image_topic': '/image_raw',
                'publish_topic': '/box_center',
            }],
        ),

        # ── 3. 像素→世界坐标 → /world_xyz (PointStamped, mm) ─
        Node(
            package='vision_pkg',
            executable='tf_node',
            name='tf_node',
            output='screen',
            parameters=[{
                'input_topic': '/box_center',
                'output_topic': '/world_xyz',
                'stale_timeout': 0.5,               # 目标丢失 0.5s 后停止发布
            }],
        ),

        # ── 4. 世界坐标→IK→关节角度 → cmd_angles ─────────────
        Node(
            package='arm_pkg',
            executable='arm_catch_box_node',
            name='arm_catch_box_node',
            output='screen',
            parameters=[{
                'input_topic': '/world_xyz',
                'output_topic': 'cmd_angles',
                'z_offset': 0.15,                   # Z 轴抬高 15cm
            }],
        ),

        # ── 5. 舵机驱动 ───────────────────────────────────────
        Node(
            package='arm_pkg',
            executable='arm_joint_node',
            name='arm_joint_node',
            output='screen',
            parameters=[{
                'port': '/dev/ttyUSB0',
                'baudrate': 1000000,
                'servo_ids': [1, 2, 3, 4, 5, 6],
            }],
        ),

    ])
