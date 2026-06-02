"""
URDF 逆运动学 (Inverse Kinematics) 工具模块
============================================
基于 urdf_fk 的正运动学引擎，实现数值反解。

算法: 阻尼最小二乘法 (DLS) + 零空间优化 + 大规模随机重启
  - 200 个随机种子覆盖 6D 关节空间
  - DLS 迭代保证局部收敛
  - 零空间项利用冗余 DOF 推离关节限位
  - 终精细化阶段高精度收敛

用法:
  from urdf_ik import URDFInverseKinematics

  ik_solver = URDFInverseKinematics("genkiarm.urdf")
  q = ik_solver.ik((0.2, 0.0, 0.3))           # 弧度输出
  q = ik_solver.ik((0.2, 0.0, 0.3), degrees=True)  # 度输出
"""

import math
from typing import List, Optional, Tuple

import numpy as np

from urdf_fk import URDFKinematics


class URDFInverseKinematics:
    """
    基于 URDF 模型的逆运动学求解器。

    用法:
        >>> ik_solver = URDFInverseKinematics("genkiarm.urdf")
        >>> q = ik_solver.ik((0.2, 0.0, 0.3))       # 弧度输出
        >>> q = ik_solver.ik((0.2, 0.0, 0.3), degrees=True)  # 度输出
        >>> q = ik_solver.ik((0.2, 0.0, 0.3), seed=[0, 0.3, -0.5, 0, 0, 0])
    """

    def __init__(self, urdf_path: str):
        """
        初始化逆运动学求解器。

        参数:
            urdf_path: URDF 文件路径
        """
        self.robot = URDFKinematics(urdf_path)

    # ----------------------------------------------------------
    # 反解 (IK) — 末端位置 → 关节角度
    # ----------------------------------------------------------

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
        """
        **反解方法** — 将末端目标位置转化为关节角度。

        使用"阻尼最小二乘法 (DLS) + 零空间优化 + 大规模随机重启":
        - 200 个随机种子覆盖 6D 关节空间
        - DLS 迭代保证局部收敛
        - 零空间项利用冗余 DOF 推离关节限位
        - 终精细化阶段做小步长高精度收敛

        参数:
            target:       末端目标位置 (x, y, z)，单位: 米
            degrees:      输出角度单位 — False=弧度(默认), True=度
            seed:         用户指定的初始关节角度 (弧度)，优先级最高
            max_iter:     单种子最大迭代次数 (默认 300)
            tolerance:    位置误差收敛阈值 (米，默认 0.1 mm)
            damping:      初始阻尼系数 (默认 0.5)
            num_restarts: 随机种子数量 (默认 200)

        返回:
            6 个关节角度列表，或 None

        用法:
            >>> ik_solver = URDFInverseKinematics("genkiarm.urdf")
            >>> q = ik_solver.ik((0.2, 0.0, 0.3))       # 弧度输出
            >>> q = ik_solver.ik((0.2, 0.0, 0.3), degrees=True)  # 度输出
            >>> q = ik_solver.ik((0.2, 0.0, 0.3), seed=[0, 0.3, -0.5, 0, 0, 0])
        """
        target_pos = np.array(target, dtype=float)
        robot = self.robot

        if seed is not None and len(seed) != robot.num_joints:
            raise ValueError(f"seed 需要 {robot.num_joints} 个值，但收到 {len(seed)} 个")

        limits = robot.joint_limits
        low_arr = np.array([l for l, h in limits])
        high_arr = np.array([h for l, h in limits])
        mid_arr = (low_arr + high_arr) / 2.0
        range_arr = high_arr - low_arr

        best_q = None
        best_error = float("inf")
        n_joints = robot.num_joints

        # ---- 构建初始猜测池 ----
        seeds = []

        # 用户种子最高优先级
        if seed is not None:
            seeds.append(np.array(seed, dtype=float))

        # 智能种子
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

        # 大规模随机种子
        rng = np.random.default_rng()
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

                # 长期无改善 → 扰动跳出
                if no_improve > 50:
                    q = best_local_q + rng.standard_normal(n_joints) * 0.1
                    q = np.clip(q, low_arr, high_arr)
                    lam = damping
                    no_improve = 0
                    continue

                # 位置雅可比 (3×N)
                J = robot.jacobian(q.tolist())[:3, :]

                # DLS: Δq = J^T (J J^T + λ²I)^(-1) e
                JJT = J @ J.T
                damped = JJT + (lam * lam) * np.eye(3)

                try:
                    delta_q = J.T @ np.linalg.solve(damped, err)
                except np.linalg.LinAlgError:
                    delta_q = J.T @ np.linalg.pinv(damped) @ err

                # 零空间: 推离限位
                ns_grad = np.zeros(n_joints)
                for j in range(n_joints):
                    half = range_arr[j] / 2.0
                    if half > 1e-8:
                        ns_grad[j] = -(q[j] - mid_arr[j]) / half
                ns_grad *= 0.003

                # SVD 零空间投影
                try:
                    _, _, Vt = np.linalg.svd(J, full_matrices=False)
                    null_proj = np.eye(n_joints) - Vt.T @ Vt
                except np.linalg.LinAlgError:
                    null_proj = np.eye(n_joints) * 0.1

                delta_q += null_proj @ ns_grad

                # 步长自适应
                step = float(np.linalg.norm(delta_q))
                if step > 0.3:
                    delta_q *= 0.3 / step

                q_new = q + delta_q
                q_new = np.clip(q_new, low_arr, high_arr)

                # 评估
                try:
                    T_new = robot.forward_kinematics(q_new.tolist())
                    e2_new = float(
                        (target_pos - T_new[:3, 3]) @ (target_pos - T_new[:3, 3])
                    )
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

        # ---- 终精细化阶段 ----
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
                    e2_new = float(
                        (target_pos - T_new[:3, 3]) @ (target_pos - T_new[:3, 3])
                    )
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

        # ---- 返回 ----
        if best_q is None:
            return None

        err_mm = math.sqrt(max(best_error, 0)) * 1000
        if err_mm > 1.0:
            print(
                f"[WARN] IK best error = {err_mm:.1f} mm "
                f"(threshold: {tolerance*1000:.1f} mm)"
            )

        result = best_q.tolist()
        if degrees:
            result = [math.degrees(a) for a in result]
        return result

    # ----------------------------------------------------------
    # 便捷委托方法
    # ----------------------------------------------------------

    def fk(
        self,
        joint_angles: List[float],
        *,
        degrees: bool = False,
    ) -> Tuple[float, float, float]:
        """
        正解 — 关节角度 → 末端位置 (委托给 URDFKinematics.fk)。
        """
        return self.robot.fk(joint_angles, degrees=degrees)

    @property
    def num_joints(self) -> int:
        """活动关节数量"""
        return self.robot.num_joints

    @property
    def joint_limits(self) -> List[Tuple[float, float]]:
        """关节限位列表 [(lower, upper), ...]"""
        return self.robot.joint_limits

    @property
    def joint_names(self) -> List[str]:
        """关节名称列表"""
        return self.robot.joint_names


# ============================================================
# 示例与演示
# ============================================================

def demo():
    """演示逆运动学求解"""
    import os

    script_dir = os.path.dirname(os.path.abspath(__file__))
    urdf_path = os.path.join(script_dir, "genkiarm.urdf")

    if not os.path.exists(urdf_path):
        print(f"错误: 找不到 URDF 文件 {urdf_path}")
        return

    ik_solver = URDFInverseKinematics(urdf_path)

    # 1. FK → IK 往返验证
    print("\n" + "=" * 60)
    print("FK → IK 往返验证")
    print("=" * 60)

    test_angles = [
        ("零位", [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]),
        ("组合1", [0.3, 0.5, -0.8, 0.4, 0.0, 0.0]),
        ("组合2", [-0.5, 0.7, -1.0, 0.3, 1.0, 0.5]),
    ]

    for label, angles in test_angles:
        x, y, z = ik_solver.fk(angles)
        q_ik = ik_solver.ik((x, y, z))
        if q_ik:
            x2, y2, z2 = ik_solver.fk(q_ik)
            err = math.sqrt((x - x2) ** 2 + (y - y2) ** 2 + (z - z2) ** 2)
            print(f"\n  [{label}]")
            print(f"    原角度 (rad): {[f'{a:.3f}' for a in angles]}")
            print(f"    FK 位置 (m): ({x:.4f}, {y:.4f}, {z:.4f})")
            print(f"    IK 解得 (rad): {[f'{a:.3f}' for a in q_ik]}")
            print(f"    验证误差: {err*1000:.3f} mm")
        else:
            print(f"\n  [{label}] IK 求解失败")

    # 2. 度输入/输出
    print("\n" + "=" * 60)
    print("度输入/输出")
    print("=" * 60)
    q_deg = ik_solver.ik((0.2, 0.0, 0.3), degrees=True)
    if q_deg:
        print(f"  IK 解得 (deg): {[f'{a:.2f}' for a in q_deg]}")
        x, y, z = ik_solver.fk(q_deg, degrees=True)
        print(f"  FK 验证位置: ({x:.4f}, {y:.4f}, {z:.4f})")


if __name__ == "__main__":
    demo()
