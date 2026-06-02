#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
world_joint.py — 世界坐标 → 关节角度 (独立模块, 无需其他项目文件)
===============================================================

对外接口:
    world_to_joint(xyz) → [J1°, J2°, J3°, J4°, J5°, J6°]

用法:
    from world_joint import world_to_joint
    angles = world_to_joint((0.15, 0.20, 0.25))

依赖: numpy, 以及同目录下的 genkiarm.urdf 模型文件.
"""

import math
import os
import xml.etree.ElementTree as ET
from dataclasses import dataclass
from typing import Dict, List, Optional, Tuple

import numpy as np


# ============================================================
# 1. 数学工具
# ============================================================

def _rotation_x(angle: float) -> np.ndarray:
    c, s = math.cos(angle), math.sin(angle)
    return np.array([[1, 0, 0], [0, c, -s], [0, s, c]])


def _rotation_y(angle: float) -> np.ndarray:
    c, s = math.cos(angle), math.sin(angle)
    return np.array([[c, 0, s], [0, 1, 0], [-s, 0, c]])


def _rotation_z(angle: float) -> np.ndarray:
    c, s = math.cos(angle), math.sin(angle)
    return np.array([[c, -s, 0], [s, c, 0], [0, 0, 1]])


def rpy_to_rotation_matrix(roll: float, pitch: float, yaw: float) -> np.ndarray:
    """RPY → 3x3 旋转矩阵. R = Rz(yaw)·Ry(pitch)·Rx(roll)."""
    return _rotation_z(yaw) @ _rotation_y(pitch) @ _rotation_x(roll)


def axis_angle_rotation(axis: Tuple[float, float, float], angle: float) -> np.ndarray:
    """任意轴旋转 (Rodrigues)."""
    ax = np.array(axis, dtype=float)
    ax = ax / np.linalg.norm(ax)
    c, s, v = math.cos(angle), math.sin(angle), 1 - math.cos(angle)
    x, y, z = ax
    return np.array([
        [c + x*x*v,   x*y*v - z*s, x*z*v + y*s],
        [y*x*v + z*s, c + y*y*v,   y*z*v - x*s],
        [z*x*v - y*s, z*y*v + x*s, c + z*z*v],
    ])


def make_transform(R: np.ndarray, t: np.ndarray) -> np.ndarray:
    T = np.eye(4)
    T[:3, :3] = R
    T[:3, 3] = t
    return T


# ============================================================
# 2. 数据结构
# ============================================================

@dataclass
class JointInfo:
    name: str
    joint_type: str
    parent_link: str
    child_link: str
    origin_xyz: Tuple[float, float, float]
    origin_rpy: Tuple[float, float, float]
    axis_xyz: Tuple[float, float, float]
    lower_limit: float
    upper_limit: float
    effort_limit: float
    velocity_limit: float


@dataclass
class LinkInfo:
    name: str
    mesh_file: Optional[str] = None


@dataclass
class RobotModel:
    name: str
    links: Dict[str, LinkInfo]
    joints: List[JointInfo]
    joint_map: Dict[str, JointInfo]
    parent_map: Dict[str, Optional[str]]
    child_map: Dict[str, List[str]]
    joint_chain: List[JointInfo]


# ============================================================
# 3. URDF 解析器
# ============================================================

class URDFParser:

    @staticmethod
    def parse(filepath: str) -> RobotModel:
        tree = ET.parse(filepath)
        root = tree.getroot()
        robot_name = root.get("name", "robot")
        links: Dict[str, LinkInfo] = {}
        joints: List[JointInfo] = []

        for link_elem in root.findall("link"):
            name = link_elem.get("name", "")
            mesh_file = None
            visual = link_elem.find("visual")
            if visual is not None:
                geometry = visual.find("geometry")
                if geometry is not None:
                    mesh = geometry.find("mesh")
                    if mesh is not None:
                        mesh_file = mesh.get("filename", None)
            links[name] = LinkInfo(name=name, mesh_file=mesh_file)

        for joint_elem in root.findall("joint"):
            name = joint_elem.get("name", "")
            jtype = joint_elem.get("type", "fixed")
            parent = joint_elem.find("parent").get("link") if joint_elem.find("parent") is not None else ""
            child = joint_elem.find("child").get("link") if joint_elem.find("child") is not None else ""

            origin = joint_elem.find("origin")
            if origin is not None:
                xyz = tuple(float(v) for v in origin.get("xyz", "0 0 0").split())
                rpy = tuple(float(v) for v in origin.get("rpy", "0 0 0").split())
            else:
                xyz, rpy = (0.0, 0.0, 0.0), (0.0, 0.0, 0.0)

            axis_elem = joint_elem.find("axis")
            axis = tuple(float(v) for v in (axis_elem.get("xyz", "0 0 1") if axis_elem is not None else "0 0 1").split())

            limit_elem = joint_elem.find("limit")
            if limit_elem is not None:
                lower = float(limit_elem.get("lower", "0"))
                upper = float(limit_elem.get("upper", "0"))
                effort = float(limit_elem.get("effort", "0"))
                velocity = float(limit_elem.get("velocity", "0"))
            else:
                lower, upper, effort, velocity = 0.0, 0.0, 0.0, 0.0

            joints.append(JointInfo(
                name=name, joint_type=jtype, parent_link=parent, child_link=child,
                origin_xyz=xyz, origin_rpy=rpy, axis_xyz=axis,
                lower_limit=lower, upper_limit=upper,
                effort_limit=effort, velocity_limit=velocity,
            ))

        parent_map: Dict[str, Optional[str]] = {name: None for name in links}
        child_map: Dict[str, List[str]] = {name: [] for name in links}
        joint_map: Dict[str, JointInfo] = {}
        for j in joints:
            joint_map[j.name] = j
            parent_map[j.child_link] = j.parent_link
            child_map[j.parent_link].append(j.child_link)

        base_link = next((name for name, p in parent_map.items() if p is None), None)
        joint_chain = URDFParser._find_kinematic_chain(base_link, child_map, joint_map)

        return RobotModel(
            name=robot_name, links=links, joints=joints,
            joint_map=joint_map, parent_map=parent_map,
            child_map=child_map, joint_chain=joint_chain,
        )

    @staticmethod
    def _find_kinematic_chain(base, child_map, joint_map):
        end_links = [n for n, ch in child_map.items() if not ch]

        def chain_to(end_link):
            chain, cur = [], end_link
            while True:
                found = next((j for j in joint_map.values() if j.child_link == cur), None)
                if found is None:
                    break
                chain.insert(0, found)
                cur = found.parent_link
            return chain

        longest = []
        for el in end_links:
            c = chain_to(el)
            if len(c) > len(longest):
                longest = c
        return longest


# ============================================================
# 4. 正运动学 (FK)
# ============================================================

class URDFKinematics:

    def __init__(self, urdf_path: str):
        self.model = URDFParser.parse(urdf_path)
        self._num_joints = len(self.model.joint_chain)

    @property
    def num_joints(self) -> int:
        return self._num_joints

    @property
    def joint_names(self) -> List[str]:
        return [j.name for j in self.model.joint_chain]

    @property
    def joint_limits(self) -> List[Tuple[float, float]]:
        return [(j.lower_limit, j.upper_limit) for j in self.model.joint_chain]

    def joint_transform(self, joint: JointInfo, angle: float) -> np.ndarray:
        t = np.array(joint.origin_xyz)
        R_origin = rpy_to_rotation_matrix(*joint.origin_rpy)
        R_axis = axis_angle_rotation(joint.axis_xyz, angle)
        return make_transform(R_origin, t) @ make_transform(R_axis, np.zeros(3))

    def forward_kinematics(self, joint_angles: List[float]) -> np.ndarray:
        T = np.eye(4)
        for angle, joint in zip(joint_angles, self.model.joint_chain):
            T = T @ self.joint_transform(joint, angle)
        return T

    def fk(self, joint_angles: List[float], *, degrees: bool = False) -> Tuple[float, float, float]:
        ang = [math.radians(a) for a in joint_angles] if degrees else list(joint_angles)
        T = self.forward_kinematics(ang)
        return (float(T[0, 3]), float(T[1, 3]), float(T[2, 3]))

    def jacobian(self, joint_angles: List[float]) -> np.ndarray:
        N = self._num_joints
        J = np.zeros((6, N))
        T_end = self.forward_kinematics(joint_angles)
        p_end = T_end[:3, 3]

        T_cum = np.eye(4)
        for i in range(N):
            joint = self.model.joint_chain[i]

            t_origin = np.array(joint.origin_xyz)
            R_origin = rpy_to_rotation_matrix(*joint.origin_rpy)
            T_origin = make_transform(R_origin, t_origin)
            T_cum = T_cum @ T_origin

            p_joint = T_cum[:3, 3]
            z_axis = T_cum[:3, :3] @ np.array(joint.axis_xyz)

            R_axis = axis_angle_rotation(joint.axis_xyz, joint_angles[i])
            T_axis = make_transform(R_axis, np.zeros(3))
            T_cum = T_cum @ T_axis

            J[:3, i] = np.cross(z_axis, p_end - p_joint)
            J[3:, i] = z_axis

        return J


# ============================================================
# 5. 逆运动学 (IK) — DLS + 零空间 + 随机重启
# ============================================================

class URDFInverseKinematics:

    def __init__(self, urdf_path: str):
        self.robot = URDFKinematics(urdf_path)

    @property
    def num_joints(self) -> int:
        return self.robot.num_joints

    @property
    def joint_limits(self) -> List[Tuple[float, float]]:
        return self.robot.joint_limits

    @property
    def joint_names(self) -> List[str]:
        return self.robot.joint_names

    def fk(self, joint_angles: List[float], *, degrees: bool = False) -> Tuple[float, float, float]:
        return self.robot.fk(joint_angles, degrees=degrees)

    def ik(
        self,
        target: Tuple[float, float, float],
        *,
        degrees: bool = False,
        seed: Optional[List[float]] = None,
        max_iter: int = 300,
        tolerance: float = 1e-4,
        damping: float = 0.5,
        num_restarts: int = 200,
    ) -> Optional[List[float]]:
        target_pos = np.array(target, dtype=float)
        robot = self.robot

        if seed is not None and len(seed) != robot.num_joints:
            raise ValueError(f"seed 需要 {robot.num_joints} 个值，但收到 {len(seed)} 个")

        limits = robot.joint_limits
        low_arr = np.array([lo for lo, _ in limits])
        high_arr = np.array([hi for _, hi in limits])
        mid_arr = (low_arr + high_arr) / 2.0
        range_arr = high_arr - low_arr

        best_q = None
        best_error = float("inf")
        n_joints = robot.num_joints
        rng = np.random.default_rng()

        # ---- 构建初始猜测池 ----
        seeds = []
        if seed is not None:
            seeds.append(np.array(seed, dtype=float))

        seeds.append(np.zeros(n_joints))
        seeds.append(mid_arr.copy())
        for pose in [
            [0.0, 0.5, -0.8, 0.3, 0.0, 0.5],
            [0.0, 0.8, -0.5, -0.3, 0.0, 0.0],
            [0.3, 0.7, -1.0, 0.0, 0.0, 1.0],
            [-0.3, 0.5, -0.7, 0.2, 0.5, 0.3],
            [0.5, 1.0, -1.2, 0.5, 1.0, 0.8],
            [0.0, 1.0, -1.0, 0.5, 0.0, 0.0],
            [0.0, 0.3, -0.3, 0.0, 0.0, 1.5],
            [-1.0, 0.8, -1.0, -0.5, -0.5, 0.5],
            [1.0, 0.6, -0.6, 1.0, 0.5, 0.5],
            [0.0, -0.3, 0.3, 1.0, 0.0, 1.0],
            [1.5, 0.5, -0.5, 1.5, 1.5, 0.3],
            [-1.5, 0.5, -0.5, -1.5, -1.5, 0.3],
            [1.5, 1.5, -1.5, -1.5, -1.5, 1.5],
            [-1.5, -1.5, 1.5, 1.5, 1.5, 0.0],
            [1.5, -1.5, -1.5, 1.5, -1.5, 1.5],
            [-1.5, 1.5, 1.5, -1.5, 1.5, 0.0],
        ]:
            seeds.append(np.array(pose))

        for _ in range(num_restarts // 2):
            seeds.append(low_arr + rng.random(n_joints) * range_arr)
        for _ in range(num_restarts // 2):
            s = mid_arr + rng.standard_normal(n_joints) * range_arr * 0.35
            seeds.append(np.clip(s, low_arr, high_arr))

        tol_sq = tolerance * tolerance

        # ---- 单种子 DLS 求解 ----
        def solve_one(q_init):
            q = q_init.copy()
            lam = damping
            best_local_q = q.copy()
            best_local_err = float("inf")
            no_improve = 0

            for _ in range(max_iter):
                try:
                    T = robot.forward_kinematics(q.tolist())
                except Exception:
                    return best_local_q, best_local_err

                err = target_pos - T[:3, 3]
                e2 = float(err @ err)

                if e2 < tol_sq:
                    return q.copy(), e2

                if e2 < best_local_err:
                    best_local_err = e2
                    best_local_q = q.copy()
                    no_improve = 0
                else:
                    no_improve += 1

                if no_improve > 50:
                    q = best_local_q + rng.standard_normal(n_joints) * 0.1
                    q = np.clip(q, low_arr, high_arr)
                    lam = damping
                    no_improve = 0
                    continue

                J = robot.jacobian(q.tolist())[:3, :]
                JJT = J @ J.T
                damped = JJT + (lam * lam) * np.eye(3)

                try:
                    delta_q = J.T @ np.linalg.solve(damped, err)
                except np.linalg.LinAlgError:
                    delta_q = J.T @ np.linalg.pinv(damped) @ err

                ns_grad = np.zeros(n_joints)
                for j in range(n_joints):
                    half = range_arr[j] / 2.0
                    if half > 1e-8:
                        ns_grad[j] = -(q[j] - mid_arr[j]) / half
                ns_grad *= 0.003

                try:
                    _, _, Vt = np.linalg.svd(J, full_matrices=False)
                    null_proj = np.eye(n_joints) - Vt.T @ Vt
                except np.linalg.LinAlgError:
                    null_proj = np.eye(n_joints) * 0.1

                delta_q += null_proj @ ns_grad

                step = float(np.linalg.norm(delta_q))
                if step > 0.3:
                    delta_q *= 0.3 / step

                q_new = q + delta_q
                q_new = np.clip(q_new, low_arr, high_arr)

                try:
                    T_new = robot.forward_kinematics(q_new.tolist())
                    e2_new = float((target_pos - T_new[:3, 3]) @ (target_pos - T_new[:3, 3]))
                except Exception:
                    lam = min(lam * 2, 10.0)
                    continue

                if e2_new < e2:
                    q = q_new
                    lam = max(lam * 0.5, 0.001)
                else:
                    lam = min(lam * 2.0, 10.0)

            return best_local_q, best_local_err

        # ---- 遍历种子 ----
        for qi in seeds:
            if best_error < tol_sq:
                break
            q_init = np.clip(qi.copy(), low_arr, high_arr)
            q_res, e2 = solve_one(q_init)
            if e2 < best_error:
                best_error = e2
                best_q = q_res.copy()

        # ---- 终精细化 ----
        if best_q is not None and best_error > tol_sq:
            q = best_q.copy()
            lam = 0.01
            best_local_q = q.copy()
            best_local_e2 = best_error

            for _ in range(500):
                try:
                    T = robot.forward_kinematics(q.tolist())
                except Exception:
                    break
                err = target_pos - T[:3, 3]
                e2 = float(err @ err)

                if e2 < tol_sq:
                    best_error = e2
                    best_q = q.copy()
                    break

                if e2 < best_local_e2:
                    best_local_e2 = e2
                    best_local_q = q.copy()

                J = robot.jacobian(q.tolist())[:3, :]
                JJT = J @ J.T
                damped = JJT + (lam * lam) * np.eye(3)

                try:
                    delta_q = J.T @ np.linalg.solve(damped, err)
                except np.linalg.LinAlgError:
                    delta_q = J.T @ np.linalg.pinv(damped) @ err

                step = float(np.linalg.norm(delta_q))
                if step > 0.05:
                    delta_q *= 0.05 / step

                q_new = np.clip(q + delta_q, low_arr, high_arr)

                try:
                    T_new = robot.forward_kinematics(q_new.tolist())
                    e2_new = float((target_pos - T_new[:3, 3]) @ (target_pos - T_new[:3, 3]))
                except Exception:
                    lam = min(lam * 2, 5.0)
                    continue

                if e2_new < e2:
                    q = q_new
                    lam = max(lam * 0.5, 1e-6)
                else:
                    lam = min(lam * 1.5, 5.0)
            else:
                if best_local_e2 < best_error:
                    best_error = best_local_e2
                    best_q = best_local_q.copy()

        if best_q is None:
            return None

        result = best_q.tolist()
        if degrees:
            result = [math.degrees(a) for a in result]
        return result


# ============================================================
# 6. 坐标系校准 (URDF ↔ 现实世界)
# ============================================================

def _real_to_urdf(x_real, y_real, z_real):
    """现实世界坐标 → URDF 模型坐标 (XY 互换)."""
    return (y_real, x_real, z_real)


# ============================================================
# 7. IK 求解器单例 (延迟加载)
# ============================================================

_IK = None

# J6 锁定配置 (对齐 ik_test.py)
_FIXED_J6_URDF_RAD = 0.0   # J6 在 URDF 中固定为 0 rad = 0°
                            # 实际舵机安装偏移 -90°, 发送时由调用方映射


def _get_ik():
    global _IK
    if _IK is None:
        script_dir = os.path.dirname(os.path.abspath(__file__))
        urdf_path = os.path.join(script_dir, "genkiarm.urdf")
        _IK = URDFInverseKinematics(urdf_path)
    return _IK


# ============================================================
# 8. 对外接口
# ============================================================

def world_to_joint(xyz):
    """
    世界坐标 → 关节角度.  J6 (夹爪) 锁定为 0°.

    参数:
        xyz: (x, y, z) 现实世界坐标, 单位米, tuple/list/array 均可

    返回:
        [J1, J2, J3, J4, J5, J6] 6 个关节角度 (度), J6 恒为 0°.
        失败返回 None.

    用法:
        from world_joint import world_to_joint
        angles = world_to_joint((0.15, 0.20, 0.25))
    """
    ik = _get_ik()

    # 锁定 J6: 限位临时设为 (0, 0), IK 不改变夹爪角度
    j6 = ik.robot.model.joint_chain[5]
    orig_lo, orig_hi = j6.lower_limit, j6.upper_limit
    j6.lower_limit = _FIXED_J6_URDF_RAD
    j6.upper_limit = _FIXED_J6_URDF_RAD

    try:
        x_real, y_real, z_real = xyz
        x_urdf, y_urdf, z_urdf = _real_to_urdf(x_real, y_real, z_real)
        return ik.ik((x_urdf, y_urdf, z_urdf), degrees=True)
    finally:
        # 恢复 J6 原始限位
        j6.lower_limit = orig_lo
        j6.upper_limit = orig_hi


# ============================================================
# 9. 自检
# ============================================================

def demo():
    print("=" * 50)
    print("  world_joint 自检")
    print("=" * 50)

    ik = _get_ik()

    # 零位 FK → 世界坐标
    q_zero = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
    x_u, y_u, z_u = ik.fk(q_zero)
    x_w, y_w, z_w = _real_to_urdf(x_u, y_u, z_u)  # 用同一个函数往返

    print(f"\n  零位 FK (deg): {q_zero}")
    print(f"  → 世界坐标 (m): ({x_w:.4f}, {y_w:.4f}, {z_w:.4f})")

    # IK
    q_ik = world_to_joint((x_w, y_w, z_w))
    if q_ik:
        print(f"\n  world_to_joint(({x_w:.4f}, {y_w:.4f}, {z_w:.4f}))")
        print(f"  → {[f'{a:.2f}' for a in q_ik]}")

        # 验证
        x2_u, y2_u, z2_u = ik.fk(q_ik, degrees=True)
        x2_w, y2_w, z2_w = _real_to_urdf(x2_u, y2_u, z2_u)
        err = math.sqrt((x_w - x2_w)**2 + (y_w - y2_w)**2 + (z_w - z2_w)**2) * 1000
        print(f"  验证 → ({x2_w:.4f}, {y2_w:.4f}, {z2_w:.4f})  误差: {err:.3f} mm")
    else:
        print("  IK 失败!")

    print()


if __name__ == "__main__":
    demo()
