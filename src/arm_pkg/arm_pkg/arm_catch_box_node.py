#!/usr/bin/env python3
"""
arm_catch_box_node — 世界坐标订阅 → IK 解算 → 关节角度发布
============================================================
订阅 tf_node 发布的 /world_xyz (PointStamped, 单位 mm),
调用 world_joint.world_to_joint() 转换为关节角度,
以上一帧角度为种子保证 IK 解连续性, 避免多解振荡.
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PointStamped
from arm_msg.msg import ArmJointAngles

from arm_pkg.world_joint import world_to_joint


class ArmCatchBoxNode(Node):
    """订阅目标世界坐标, IK 解算后发布关节角度."""

    def __init__(self):
        super().__init__('arm_catch_box_node')

        # ── 参数 ──────────────────────────────────────────
        self.declare_parameter('input_topic', '/world_xyz')
        self.declare_parameter('output_topic', 'cmd_angles')
        self.declare_parameter('z_offset', 0.15)        # Z 轴抬高量 (m)
        self.declare_parameter('j6_angle', -90.0)        # J6 夹爪固定角度 (度)

        self.input_topic = self.get_parameter('input_topic').value
        self.output_topic = self.get_parameter('output_topic').value
        self.z_offset = self.get_parameter('z_offset').value
        self.j6_angle = self.get_parameter('j6_angle').value

        # ── 状态 ──────────────────────────────────────────
        self._ik_result = None     # IK 原始结果 (J5=J6=0), 作为下帧种子

        # ── 订阅 ──────────────────────────────────────────
        self._sub = self.create_subscription(
            PointStamped, self.input_topic, self._callback, 10)

        # ── 发布 ──────────────────────────────────────────
        self._pub = self.create_publisher(
            ArmJointAngles, self.output_topic, 10)

        self.get_logger().info(
            f'Arm Catch Box Node started.\n'
            f'  input:     {self.input_topic} (PointStamped, mm)\n'
            f'  output:    {self.output_topic} (ArmJointAngles)\n'
            f'  z_offset:  +{self.z_offset*1000:.0f} mm\n'
            f'  j6_angle:  {self.j6_angle:.0f} deg\n'
            f'  IK seed:   previous-frame angles (stable tracking)')

    # ── 回调: 收到世界坐标 → IK → 发布 ─────────────────

    def _callback(self, msg: PointStamped):
        # mm → m, x 取负 (相机→机械臂坐标系)
        x_m = -msg.point.x / 1000.0
        y_m =  msg.point.y / 1000.0
        z_m =  msg.point.z / 1000.0 + self.z_offset

        # ── 6DOF IK (J5 自由 → 最高精度; J6 锁定) ─────────
        angles = world_to_joint((x_m, y_m, z_m), seed=self._ik_result)

        if angles is None:
            self.get_logger().warn(
                f'IK failed for ({x_m:.4f}, {y_m:.4f}, {z_m:.4f})')
            return

        # 保存 IK 原始输出作为下帧种子
        self._ik_result = [float(a) for a in angles]

        # ── 发布时归零: J5 归零消扭转, J6 设为夹爪角度 ─────
        angles[4] = 0.0             # J5 腕关节归零
        angles[5] = self.j6_angle   # J6 夹爪闭合

        # 发布
        cmd = ArmJointAngles()
        cmd.angles = [float(a) for a in angles]
        self._pub.publish(cmd)

        self.get_logger().info(
            f'({x_m:.4f}, {y_m:.4f}, {z_m:.4f}) m → '
            f'[{", ".join(f"{a:.2f}" for a in angles)}]')


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = ArmCatchBoxNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        print('\nArm Catch Box Node stopped.')
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
