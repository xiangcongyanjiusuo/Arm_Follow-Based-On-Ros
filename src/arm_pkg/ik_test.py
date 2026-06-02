#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
ik_test.py — URDF IK 逆解验证脚本
=================================
- 基于 urdf_ik 的 URDFInverseKinematics 求解器
- 通过 COM8 (1M 波特率) 控制 6 轴 GenkiArm 机械臂
- 串口协议完全对齐 04_read_all_student_angle.py / 05_write_all_student_angle.py
- 随机生成关节角 → FK 得可达目标位置 → IK 反解 → FK 验证精度
  → 发送舵机 → 回读角度验证 → 等待用户按 Enter 下一步
- J6 (夹爪) 锁定: IK 阶段限位设为 0 rad, 舵机固定发送 -90°
- 输入 q 退出

用法:
    python ik_test.py
"""

import os
import sys
import math
import time

import numpy as np

try:
    import serial
except ImportError:
    print("请先安装 pyserial: pip install pyserial")
    sys.exit(1)

# ---------------------------------------------------------------------------
# 路径
# ---------------------------------------------------------------------------
SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, SCRIPT_DIR)

from urdf_ik import URDFInverseKinematics
from calibration import urdf_to_real, real_to_urdf

# ===================================================================
# 用户可调配置
# ===================================================================

# ---- 串口 ----
COM_PORT = "COM8"
COM_BAUDRATE = 1000000

# ---- J6 夹爪锁定 ----
LOCK_J6 = True                     # True  = IK 不解 J6, 舵机固定角度
FIXED_J6_URDF_RAD = 0.0            # J6 在 URDF 中的固定值 (弧度), 下限为 0
FIXED_J6_SERVO_DEG = -90.0         # J6 舵机固定发送角度 (度). 因安装方向需 -90°

# ---- 测试参数 ----
MOVE_DELAY = 1.2                   # 发送后等待运动完成的时间 (秒)
SERVO_WRITE_DELAY = 0.03           # 舵机间发送间隔 (秒)
SERVO_READ_DELAY = 0.01            # 舵机间读取间隔 (秒)


# ===================================================================
# ServoController — 完全对齐 04/05 的协议实现
# ===================================================================

class ServoController:
    """
    GenkiArm 6 轴舵机串口控制器.

    协议 (与 04_read_all_student_angle.py / 05_write_all_student_angle.py 一致):
      - 包头: 0xFF 0xFF
      - 数据帧: [ID, Length, Instruction, Param1, ..., Checksum]
      - 校验和: (~sum(core)) & 0xFF
      - 角度映射: -90° → 1024,  +90° → 3072
    """

    # -- 寄存器常量 (来自 04/05) --
    ADDR_PRESENT_POSITION = 56
    ADDR_GOAL_POSITION = 42
    ADDR_TORQUE_ENABLE = 40

    INST_READ = 2
    INST_WRITE = 3

    # -- 通信状态常量 (来自 04/05) --
    COMM_SUCCESS = 0
    COMM_RX_TIMEOUT = -6
    COMM_RX_CORRUPT = -7

    def __init__(self, port=COM_PORT, baudrate=COM_BAUDRATE, timeout=0.1):
        self.port_name = port
        self.serial_port = None

        try:
            self.serial_port = serial.Serial(
                port, baudrate=baudrate, timeout=timeout
            )
            time.sleep(0.1)
            self.serial_port.reset_input_buffer()
            print(f"  ✓ 串口 {port} 已打开 (波特率 {baudrate})")
        except serial.SerialException as e:
            print(f"  ✗ 无法打开串口 {port}: {e}")
            sys.exit(1)

    def close(self):
        """关闭串口"""
        if self.serial_port and self.serial_port.is_open:
            self.serial_port.close()
            print(f"  串口 {self.port_name} 已关闭")

    # ================================================================
    # 协议底层 (直接来自 04/05)
    # ================================================================

    @staticmethod
    def _calculate_checksum(data):
        """校验和 = (~sum(data)) & 0xFF"""
        return (~sum(data)) & 0xFF

    def _send_packet(self, servo_id, instruction, parameters=None):
        """
        发送数据包. 返回 True/False.
        与 04/05 完全一致 — reset_input_buffer → write → flush.
        """
        if not self.serial_port or not self.serial_port.is_open:
            return False
        if parameters is None:
            parameters = []
        length = len(parameters) + 2
        packet_core = [servo_id, length, instruction] + parameters
        checksum = self._calculate_checksum(packet_core)
        packet = bytes([0xFF, 0xFF] + packet_core + [checksum])
        try:
            self.serial_port.reset_input_buffer()
            self.serial_port.write(packet)
            self.serial_port.flush()
            return True
        except Exception:
            return False

    def _read_packet(self):
        """
        读取并解析响应数据包.

        返回: (status, error_byte, data_list)
          status: COMM_SUCCESS / COMM_RX_TIMEOUT / COMM_RX_CORRUPT
          error_byte: 舵机错误字节
          data_list:  返回参数列表

        与 04/05 完全一致 — 含 0xFF 0xFF 重同步逻辑.
        """
        start_time = time.time()
        packet = []
        while (time.time() - start_time) < self.serial_port.timeout:
            if self.serial_port.in_waiting > 0:
                byte = self.serial_port.read(1)
                if not byte:
                    continue
                byte = byte[0]

                # 等待第一个 0xFF
                if not packet and byte != 0xFF:
                    continue

                packet.append(byte)

                # 检测到新包头 → 重同步
                if len(packet) >= 2 and packet[-2:] == [0xFF, 0xFF]:
                    if len(packet) > 2:
                        packet = [0xFF, 0xFF]
                    continue

                # 帧长度足够 → 尝试解析
                if len(packet) > 4:
                    pkt_len = packet[3]            # Length 字段
                    if len(packet) == pkt_len + 4:  # 收齐一帧
                        core_data = packet[2:-1]
                        calc_checksum = self._calculate_checksum(core_data)
                        if calc_checksum == packet[-1]:
                            return (
                                self.COMM_SUCCESS,
                                packet[4],          # error byte
                                packet[5:-1],       # params
                            )
                        else:
                            return self.COMM_RX_CORRUPT, 0, []

        return self.COMM_RX_TIMEOUT, 0, []

    # ================================================================
    # 寄存器读写 (来自 04/05)
    # ================================================================

    def _write_register(self, servo_id, address, value, size=2):
        """
        写入寄存器. 一发即忘 (不等待响应), 与 05 一致.

        参数:
            servo_id: 舵机 ID (1~6)
            address:  寄存器地址
            value:    写入值
            size:     1=单字节, 2=双字节(小端序)
        """
        params = [address]
        if size == 1:
            params.append(value & 0xFF)
        elif size == 2:
            params.extend([value & 0xFF, (value >> 8) & 0xFF])
        else:
            return False
        return self._send_packet(servo_id, self.INST_WRITE, params)

    # ================================================================
    # 高层 API
    # ================================================================

    def enable_torque(self, servo_id):
        """使能舵机扭矩 (一发即忘, 与 05 一致)."""
        return self._write_register(
            servo_id, self.ADDR_TORQUE_ENABLE, 1, size=1
        )

    def set_servo_angle(self, servo_id, angle):
        """
        设置舵机目标角度 (一发即忘, 与 05 一致).

        参数:
            servo_id: 舵机 ID (1~6)
            angle:    目标角度, 范围 -90° ~ +90°
        """
        angle = max(-90.0, min(90.0, angle))
        position = int(
            ((angle + 90.0) / 180.0) * (3072.0 - 1024.0) + 1024.0
        )
        position = max(1024, min(3072, position))
        return self._write_register(
            servo_id, self.ADDR_GOAL_POSITION, position, size=2
        )

    def get_servo_angle(self, servo_id):
        """
        读取舵机当前角度 (与 04 完全一致).

        返回:
            角度值 (度), 读取失败返回 None.
        """
        if not self._send_packet(
            servo_id, self.INST_READ, [self.ADDR_PRESENT_POSITION, 2]
        ):
            return None

        result, error, data = self._read_packet()

        if result != self.COMM_SUCCESS or error != 0:
            return None

        if data and len(data) >= 2:
            position = data[0] | (data[1] << 8)
            angle = ((position - 1024.0) / (3072.0 - 1024.0)) * 180.0 - 90.0
            angle = max(-90.0, min(90.0, angle))
            return angle

        return None

    def read_all_angles(self):
        """
        读取全部 6 个舵机的当前角度.

        返回:
            list[float | None]: 6 个角度 (度), 失败项为 None
        """
        angles = []
        for sid in range(1, 7):
            ang = self.get_servo_angle(sid)
            angles.append(ang)
            time.sleep(SERVO_READ_DELAY)
        return angles

    def set_all_angles(self, angles_deg):
        """
        一次性设置全部 6 个舵机角度.

        参数:
            angles_deg: 6 个目标角度 (度)
        """
        for i, ang in enumerate(angles_deg):
            self.set_servo_angle(i + 1, ang)
            time.sleep(SERVO_WRITE_DELAY)


# ===================================================================
# IK 辅助函数
# ===================================================================

def lock_j6(ik_solver):
    """
    锁定 J6 关节限位为 (0, 0) — IK 求解时不改变夹爪角度.

    返回: (original_lower, original_upper) 用于后续恢复.
    """
    j6 = ik_solver.robot.model.joint_chain[5]
    orig_lo, orig_hi = j6.lower_limit, j6.upper_limit
    j6.lower_limit = FIXED_J6_URDF_RAD
    j6.upper_limit = FIXED_J6_URDF_RAD
    return orig_lo, orig_hi


def unlock_j6(ik_solver, orig_lo, orig_hi):
    """恢复 J6 原始关节限位."""
    j6 = ik_solver.robot.model.joint_chain[5]
    j6.lower_limit = orig_lo
    j6.upper_limit = orig_hi


def random_joint_angles(ik_solver, rng):
    """在关节限位内均匀随机采样一组角度 (弧度)."""
    limits = ik_solver.joint_limits
    return [lo + rng.random() * (hi - lo) for lo, hi in limits]


def clamp_to_limits(angles_deg, ik_solver):
    """将角度裁剪到 IK 求解器的关节限位内 (度)."""
    limits_deg = [
        (math.degrees(lo), math.degrees(hi))
        for lo, hi in ik_solver.joint_limits
    ]
    return [
        max(lo, min(hi, a))
        for a, (lo, hi) in zip(angles_deg, limits_deg)
    ]


def fmt_angles(angles_deg, locked_idx=None):
    """
    格式化角度列表为字符串.
    locked_idx: 标记为锁定 (显示 LOCK) 的索引列表.
    """
    parts = []
    for i, a in enumerate(angles_deg):
        tag = " [LOCK]" if (locked_idx and i in locked_idx) else ""
        parts.append(f"J{i+1}:{a:+7.2f}°{tag}")
    return "  ".join(parts)


# ===================================================================
# 主程序
# ===================================================================

def main():
    print()
    print("=" * 62)
    print("   URDF IK 逆解验证 — ik_test")
    print("=" * 62)

    if LOCK_J6:
        print(f"\n  🔒 J6 夹爪已锁定")
        print(f"     URDF 角度: {math.degrees(FIXED_J6_URDF_RAD):.1f}° (固定)")
        print(f"     舵机角度:  {FIXED_J6_SERVO_DEG:.1f}° (固定)")
        print(f"     IK 不会求解 J6, J6 限位临时设为 "
              f"[{math.degrees(FIXED_J6_URDF_RAD):.1f}°, "
              f"{math.degrees(FIXED_J6_URDF_RAD):.1f}°]")

    # ==================================================================
    # 1. 初始化 IK 求解器
    # ==================================================================
    print("\n[1/5] 加载 URDF 模型 …")
    urdf_path = os.path.join(SCRIPT_DIR, "genkiarm.urdf")
    if not os.path.exists(urdf_path):
        print(f"  ✗ 找不到 URDF 文件: {urdf_path}")
        sys.exit(1)

    ik = URDFInverseKinematics(urdf_path)
    print(f"  机器人: so_arm100_simplified_core (GenkiArm)")
    print(f"  关节数: {ik.num_joints}")
    print(f"  关节名称及限位:")
    for i, (name, (lo, hi)) in enumerate(
        zip(ik.joint_names, ik.joint_limits)
    ):
        tag = " ← 夹爪" if i == 5 else ""
        print(f"    [{i+1}] {name:10s}  {math.degrees(lo):+6.1f}° ~ "
              f"{math.degrees(hi):+6.1f}°{tag}")

    # ==================================================================
    # 2. 连接串口
    # ==================================================================
    print(f"\n[2/5] 连接串口 …")
    servo = ServoController(port=COM_PORT, baudrate=COM_BAUDRATE)

    # ==================================================================
    # 3. 使能舵机扭矩
    # ==================================================================
    print(f"\n[3/5] 使能舵机扭矩 …")
    all_ok = True
    for sid in range(1, 7):
        ok = servo.enable_torque(sid)
        tag = " ✓" if ok else " ✗"
        lock_tag = " [LOCKED]" if (LOCK_J6 and sid == 6) else ""
        print(f"  舵机 {sid}:{tag}{lock_tag}")
        if not ok:
            all_ok = False
        time.sleep(0.05)
    if not all_ok:
        print("  ⚠ 部分舵机使能失败，请检查硬件连接")

    # ==================================================================
    # 4. 读取当前角度
    # ==================================================================
    print(f"\n[4/5] 读取当前舵机角度 …")
    cur_angles = servo.read_all_angles()
    valid = [a for a in cur_angles if a is not None]
    if len(valid) == 6:
        print(f"  当前角度:  {fmt_angles(cur_angles, locked_idx={5} if LOCK_J6 else None)}")
        cur_rad = [math.radians(a) if a is not None else 0.0 for a in cur_angles]
        if LOCK_J6:
            cur_rad[5] = FIXED_J6_URDF_RAD   # FK 用 URDF 值, 不是舵机角度
        x_cur, y_cur, z_cur = ik.fk(cur_rad)
        xr_cur, yr_cur, zr_cur = urdf_to_real(x_cur, y_cur, z_cur)
        print(f"  末端 URDF坐标 (m):  x={x_cur:.4f}  y={y_cur:.4f}  z={z_cur:.4f}")
        print(f"  末端 现实坐标 (m):  x={xr_cur:.4f}  y={yr_cur:.4f}  z={zr_cur:.4f}")
    else:
        missing = [i + 1 for i, a in enumerate(cur_angles) if a is None]
        print(f"  ⚠ 舵机 {missing} 读取失败, 当前角度未知")
        print(f"  已读到的: {fmt_angles(cur_angles, locked_idx={5} if LOCK_J6 else None)}")

    # ==================================================================
    # 5. 验证循环
    # ==================================================================
    print(f"\n[5/5] 开始 IK 验证测试")
    print("-" * 62)
    if LOCK_J6:
        print("  🔒 J6 锁定: IK 不解夹爪, 舵机固定发送 "
              f"{FIXED_J6_SERVO_DEG:.0f}°")
    print("  流程: 随机关节角 → FK 得目标 → IK 反解 (J1~J5)")
    print("        → FK 验证精度 → 发送舵机 → 回读确认")
    print("        → 等待 Enter 确认下一步")
    print("  按 Enter = 下一个目标  |  输入 q + Enter = 退出")
    print("-" * 62)

    rng = np.random.default_rng()
    test_count = 0
    total_ik_err_mm = 0.0
    max_ik_err_mm = 0.0
    total_rb_err_deg = 0.0      # 回读偏差 (只计 J1~J5)
    max_rb_err_deg = 0.0

    try:
        while True:
            test_count += 1

            # ----------------------------------------------------------
            # 5a. 随机生成可达目标
            #     关节限位内均匀采样 → FK → 保证 100% 可达
            # ----------------------------------------------------------
            if LOCK_J6:
                # 源关节角也要锁 J6
                j6_orig_lo, j6_orig_hi = lock_j6(ik)
                q_src = random_joint_angles(ik, rng)
                unlock_j6(ik, j6_orig_lo, j6_orig_hi)
            else:
                q_src = random_joint_angles(ik, rng)

            x_tgt, y_tgt, z_tgt = ik.fk(q_src)

            # 现实坐标约束: Y ≥ 0 (URDF X = 现实 Y)
            _, yr_tgt, _ = urdf_to_real(x_tgt, y_tgt, z_tgt)
            if yr_tgt < 0:
                test_count -= 1
                continue

            target = (x_tgt, y_tgt, z_tgt)

            q_src_deg = [math.degrees(a) for a in q_src]

            print(f"\n{'─' * 62}")
            print(f"  测试 #{test_count}")
            print(f"{'─' * 62}")
            print(f"  源关节角 (deg):  "
                  f"{fmt_angles(q_src_deg, locked_idx={5} if LOCK_J6 else None)}")
            xr_tgt, yr_tgt, zr_tgt = urdf_to_real(x_tgt, y_tgt, z_tgt)
            print(f"  目标 URDF (m):  x={x_tgt:.4f}  y={y_tgt:.4f}  "
                  f"z={z_tgt:.4f}")
            print(f"  目标 现实 (m):  x={xr_tgt:.4f}  y={yr_tgt:.4f}  "
                  f"z={zr_tgt:.4f}")

            # ----------------------------------------------------------
            # 5b. IK 反解 (J6 锁定)
            # ----------------------------------------------------------
            print(f"  求解 IK …", end=" ", flush=True)

            if LOCK_J6:
                orig_lo, orig_hi = lock_j6(ik)
            else:
                orig_lo = orig_hi = None

            q_ik = ik.ik(target, degrees=False)

            if LOCK_J6:
                unlock_j6(ik, orig_lo, orig_hi)

            if q_ik is None:
                print("✗ 失败 (返回 None)，跳过")
                test_count -= 1
                continue
            print("✓")

            # ----------------------------------------------------------
            # 5c. FK 验证 IK 解 (理论精度)
            # ----------------------------------------------------------
            x_vfy, y_vfy, z_vfy = ik.fk(q_ik)
            ik_err_mm = (
                math.sqrt(
                    (x_tgt - x_vfy) ** 2
                    + (y_tgt - y_vfy) ** 2
                    + (z_tgt - z_vfy) ** 2
                )
                * 1000.0
            )

            total_ik_err_mm += ik_err_mm
            if ik_err_mm > max_ik_err_mm:
                max_ik_err_mm = ik_err_mm
            avg_ik_err_mm = total_ik_err_mm / test_count

            q_ik_deg = [math.degrees(a) for a in q_ik]

            xr_vfy, yr_vfy, zr_vfy = urdf_to_real(x_vfy, y_vfy, z_vfy)

            print(f"  IK 解 (deg):      "
                  f"{fmt_angles(q_ik_deg, locked_idx={5} if LOCK_J6 else None)}")
            print(f"  FK 验证 URDF (m): x={x_vfy:.4f}  y={y_vfy:.4f}  "
                  f"z={z_vfy:.4f}")
            print(f"  FK 验证 现实 (m): x={xr_vfy:.4f}  y={yr_vfy:.4f}  "
                  f"z={zr_vfy:.4f}")
            print(f"  IK 理论误差: {ik_err_mm:.3f} mm  "
                  f"| 平均: {avg_ik_err_mm:.3f} mm  "
                  f"| 最大: {max_ik_err_mm:.3f} mm")

            if ik_err_mm > 1.0:
                print(f"  ⚠ IK 误差 > 1mm，精度偏低")
            elif ik_err_mm <= 0.1:
                print(f"  ✓ IK 误差 ≤ 0.1mm，精度优秀")

            # ----------------------------------------------------------
            # 5d. 构建舵机发送角度 & 发送
            # ----------------------------------------------------------
            q_send_deg = clamp_to_limits(q_ik_deg, ik)
            if LOCK_J6:
                q_send_deg[5] = FIXED_J6_SERVO_DEG   # J6 固定舵机角度

            print(f"  发送到舵机 …")
            print(f"    发送 →  {fmt_angles(q_send_deg, locked_idx={5} if LOCK_J6 else None)}")
            servo.set_all_angles(q_send_deg)
            print(f"  ✓ 已发送全部 6 路，等待运动完成 "
                  f"({MOVE_DELAY:.1f}s) …")
            time.sleep(MOVE_DELAY)

            # ----------------------------------------------------------
            # 5e. 回读舵机当前角度 (硬件验证)
            # ----------------------------------------------------------
            print(f"  回读舵机角度 …")
            readback = servo.read_all_angles()
            valid_rb = [a for a in readback if a is not None]

            if len(valid_rb) == 6:
                print(f"    回读 →  "
                      f"{fmt_angles(readback, locked_idx={5} if LOCK_J6 else None)}")

                # 每轴偏差 (J6 锁定则不计入)
                rb_errs = [
                    abs(readback[i] - q_send_deg[i]) for i in range(6)
                ]
                if LOCK_J6:
                    evaluate_idx = list(range(5))   # 只评估 J1~J5
                else:
                    evaluate_idx = list(range(6))

                rb_eval_errs = [rb_errs[i] for i in evaluate_idx]
                max_rb_err = max(rb_eval_errs) if rb_eval_errs else 0.0
                total_rb_err_deg += max_rb_err
                if max_rb_err > max_rb_err_deg:
                    max_rb_err_deg = max_rb_err
                avg_rb_err = total_rb_err_deg / test_count

                # 打印偏差
                err_parts = []
                for i in range(6):
                    if LOCK_J6 and i == 5:
                        err_parts.append(f"J{i+1}: -- (锁)")
                    else:
                        err_parts.append(f"J{i+1}:{rb_errs[i]:+.2f}°")
                print(f"    舵机偏差:  {'  '.join(err_parts)}")
                print(f"    J1~J5 最大偏差: {max_rb_err:.2f}°  "
                      f"| 平均: {avg_rb_err:.2f}°  "
                      f"| 历史最大: {max_rb_err_deg:.2f}°")

                if max_rb_err > 3.0:
                    print(f"  ⚠ 舵机偏差 > 3°，请检查机械或舵机映射方向!")
            else:
                missing = [i + 1 for i, a in enumerate(readback) if a is None]
                print(f"  ⚠ 舵机 {missing} 回读失败")
                if valid_rb:
                    print(f"    读到的: "
                          f"{fmt_angles(readback, locked_idx={5} if LOCK_J6 else None)}")

            # 计算实际末端位置
            # 注意: J6 锁定后舵机角度 ≠ URDF 角度, FK 计算时用固定 URDF 值
            if len(valid_rb) == 6:
                rb_rad = [math.radians(a) for a in readback]
                if LOCK_J6:
                    rb_rad[5] = FIXED_J6_URDF_RAD   # 用 URDF 固定值, 不是舵机角度
                x_rb, y_rb, z_rb = ik.fk(rb_rad)
                xr_rb, yr_rb, zr_rb = urdf_to_real(x_rb, y_rb, z_rb)
                rb_pos_err = math.sqrt(
                    (x_tgt - x_rb) ** 2
                    + (y_tgt - y_rb) ** 2
                    + (z_tgt - z_rb) ** 2
                ) * 1000.0
                print(f"    实际末端 URDF (m): x={x_rb:.4f}  y={y_rb:.4f}  "
                      f"z={z_rb:.4f}")
                print(f"    实际末端 现实 (m): x={xr_rb:.4f}  y={yr_rb:.4f}  "
                      f"z={zr_rb:.4f}")
                print(f"    末端偏差: {rb_pos_err:.2f} mm")

            # ----------------------------------------------------------
            # 5f. 等待用户确认
            # ----------------------------------------------------------
            print(f"\n  [Enter=下一个 | q=退出]: ", end="", flush=True)
            line = sys.stdin.readline().strip().lower()
            if line == "q":
                print("  用户请求退出")
                break

    except KeyboardInterrupt:
        print("\n\n  收到 Ctrl+C，正在退出 …")

    finally:
        # ==============================================================
        # 清理 & 总结
        # ==============================================================
        print(f"\n{'=' * 62}")
        if test_count > 0:
            print(f"  测试总结")
            print(f"{'=' * 62}")
            print(f"  总测试次数:    {test_count}")
            print(f"  IK 理论误差 —")
            print(f"    平均: {total_ik_err_mm / test_count:.3f} mm")
            print(f"    最大: {max_ik_err_mm:.3f} mm")
            print(f"  舵机回读偏差 (J1~J{'5' if LOCK_J6 else '6'}) —")
            print(f"    平均: {total_rb_err_deg / test_count:.2f}°")
            print(f"    最大: {max_rb_err_deg:.2f}°")
            if LOCK_J6:
                print(f"  🔒 J6 始终锁定: URDF={math.degrees(FIXED_J6_URDF_RAD):.0f}°  "
                      f"舵机={FIXED_J6_SERVO_DEG:.0f}°")
        else:
            print(f"  未完成任何测试")
        print(f"{'=' * 62}")

        servo.close()
        print("  程序结束")


# ===================================================================
# 入口
# ===================================================================

if __name__ == "__main__":
    main()
