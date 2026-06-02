"""
URDF 解析与正运动学 (Forward Kinematics) 工具模块
==================================================
支持:
  - 解析 URDF 文件，提取运动学链信息
  - 给定关节角度，计算末端执行器的位姿 (正解)
  - 计算任意连杆的位姿
  - 雅可比矩阵计算

用法:
  from urdf_fk import URDFKinematics

  robot = URDFKinematics("genkiarm.urdf")
  x, y, z = robot.fk([0, 0.5, -0.8, 0, 0, 0])
"""

import xml.etree.ElementTree as ET
from dataclasses import dataclass
from typing import List, Dict, Optional, Tuple
import math


# ============================================================
# 数学工具函数
# ============================================================

def _rotation_x(angle: float) -> "np.ndarray":
    """绕X轴旋转 angle 弧度的 3x3 旋转矩阵"""
    import numpy as np
    c, s = math.cos(angle), math.sin(angle)
    return np.array([[1, 0, 0],
                     [0, c, -s],
                     [0, s, c]])


def _rotation_y(angle: float) -> "np.ndarray":
    """绕Y轴旋转 angle 弧度的 3x3 旋转矩阵"""
    import numpy as np
    c, s = math.cos(angle), math.sin(angle)
    return np.array([[c, 0, s],
                     [0, 1, 0],
                     [-s, 0, c]])


def _rotation_z(angle: float) -> "np.ndarray":
    """绕Z轴旋转 angle 弧度的 3x3 旋转矩阵"""
    import numpy as np
    c, s = math.cos(angle), math.sin(angle)
    return np.array([[c, -s, 0],
                     [s, c, 0],
                     [0, 0, 1]])


def rpy_to_rotation_matrix(roll: float, pitch: float, yaw: float) -> "np.ndarray":
    """
    将 RPY (roll-pitch-yaw) 欧拉角转换为 3x3 旋转矩阵。
    采用固定轴 (fixed-axis) 约定: R = Rz(yaw) * Ry(pitch) * Rx(roll)
    """
    import numpy as np
    return _rotation_z(yaw) @ _rotation_y(pitch) @ _rotation_x(roll)


def rotation_matrix_to_rpy(R: "np.ndarray") -> Tuple[float, float, float]:
    """
    将 3x3 旋转矩阵转换为 RPY (roll, pitch, yaw) 欧拉角。
    返回 (roll, pitch, yaw)，单位弧度。
    """
    import numpy as np
    # pitch
    sy = math.sqrt(R[0, 0] ** 2 + R[1, 0] ** 2)
    singular = sy < 1e-6

    if not singular:
        roll = math.atan2(R[2, 1], R[2, 2])
        pitch = math.atan2(-R[2, 0], sy)
        yaw = math.atan2(R[1, 0], R[0, 0])
    else:
        # 万向节死锁情况
        roll = math.atan2(-R[1, 2], R[1, 1])
        pitch = math.atan2(-R[2, 0], sy)
        yaw = 0.0

    return (roll, pitch, yaw)


def axis_angle_rotation(axis: Tuple[float, float, float], angle: float) -> "np.ndarray":
    """
    绕任意轴旋转 angle 弧度，返回 3x3 旋转矩阵 (Rodrigues 公式)。
    """
    import numpy as np
    axis = np.array(axis, dtype=float)
    axis = axis / np.linalg.norm(axis)  # 归一化

    c = math.cos(angle)
    s = math.sin(angle)
    v = 1 - c

    x, y, z = axis
    return np.array([
        [c + x * x * v, x * y * v - z * s, x * z * v + y * s],
        [y * x * v + z * s, c + y * y * v, y * z * v - x * s],
        [z * x * v - y * s, z * y * v + x * s, c + z * z * v],
    ])


def make_transform(R: "np.ndarray", t: "np.ndarray") -> "np.ndarray":
    """从旋转矩阵 R (3x3) 和平移向量 t (3,) 构建 4x4 齐次变换矩阵"""
    import numpy as np
    T = np.eye(4)
    T[:3, :3] = R
    T[:3, 3] = t
    return T


# ============================================================
# 数据结构
# ============================================================

@dataclass
class JointInfo:
    """关节信息"""
    name: str
    joint_type: str          # "revolute", "prismatic", "fixed" 等
    parent_link: str
    child_link: str
    origin_xyz: Tuple[float, float, float]   # 平移 (相对于父连杆)
    origin_rpy: Tuple[float, float, float]   # 旋转 RPY (相对于父连杆)
    axis_xyz: Tuple[float, float, float]     # 旋转轴 (在关节坐标系下)
    lower_limit: float        # 关节下限
    upper_limit: float        # 关节上限
    effort_limit: float       # 力矩/力限制
    velocity_limit: float     # 速度限制


@dataclass
class LinkInfo:
    """连杆信息"""
    name: str
    mesh_file: Optional[str] = None


@dataclass
class RobotModel:
    """完整的机器人模型"""
    name: str
    links: Dict[str, LinkInfo]
    joints: List[JointInfo]
    joint_map: Dict[str, JointInfo]           # name -> joint
    parent_map: Dict[str, Optional[str]]       # child_link -> parent_link
    child_map: Dict[str, List[str]]            # parent_link -> [child_link]
    joint_chain: List[JointInfo]              # 从 base 到 end-effector 的关节链


# ============================================================
# URDF 解析器
# ============================================================

class URDFParser:
    """解析 URDF 文件并构建机器人模型"""

    @staticmethod
    def parse(filepath: str) -> RobotModel:
        """
        解析 URDF 文件，返回 RobotModel。

        参数:
            filepath: URDF 文件的路径
        返回:
            RobotModel 对象，包含完整的运动学链信息
        """
        tree = ET.parse(filepath)
        root = tree.getroot()

        robot_name = root.get("name", "robot")
        links: Dict[str, LinkInfo] = {}
        joints: List[JointInfo] = []

        # ---- 解析 <link> ----
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

        # ---- 解析 <joint> ----
        for joint_elem in root.findall("joint"):
            name = joint_elem.get("name", "")
            jtype = joint_elem.get("type", "fixed")

            parent = joint_elem.find("parent").get("link") if joint_elem.find("parent") is not None else ""
            child = joint_elem.find("child").get("link") if joint_elem.find("child") is not None else ""

            # 解析 origin
            origin = joint_elem.find("origin")
            if origin is not None:
                xyz_str = origin.get("xyz", "0 0 0")
                rpy_str = origin.get("rpy", "0 0 0")
                xyz = tuple(float(v) for v in xyz_str.split())
                rpy = tuple(float(v) for v in rpy_str.split())
            else:
                xyz = (0.0, 0.0, 0.0)
                rpy = (0.0, 0.0, 0.0)

            # 解析 axis
            axis_elem = joint_elem.find("axis")
            if axis_elem is not None:
                axis_str = axis_elem.get("xyz", "0 0 1")
                axis = tuple(float(v) for v in axis_str.split())
            else:
                axis = (0.0, 0.0, 1.0)

            # 解析 limit
            limit_elem = joint_elem.find("limit")
            if limit_elem is not None:
                lower = float(limit_elem.get("lower", "0"))
                upper = float(limit_elem.get("upper", "0"))
                effort = float(limit_elem.get("effort", "0"))
                velocity = float(limit_elem.get("velocity", "0"))
            else:
                lower, upper, effort, velocity = 0.0, 0.0, 0.0, 0.0

            joint = JointInfo(
                name=name,
                joint_type=jtype,
                parent_link=parent,
                child_link=child,
                origin_xyz=xyz,
                origin_rpy=rpy,
                axis_xyz=axis,
                lower_limit=lower,
                upper_limit=upper,
                effort_limit=effort,
                velocity_limit=velocity,
            )
            joints.append(joint)

        # ---- 构建父子关系映射 ----
        parent_map: Dict[str, Optional[str]] = {name: None for name in links}
        child_map: Dict[str, List[str]] = {name: [] for name in links}
        joint_map: Dict[str, JointInfo] = {}

        for j in joints:
            joint_map[j.name] = j
            parent_map[j.child_link] = j.parent_link
            child_map[j.parent_link].append(j.child_link)

        # ---- 找出从 base 到末端执行器的关节链 ----
        # 找到 base link (没有 parent 的连杆)
        base_link = None
        for name, parent in parent_map.items():
            if parent is None:
                base_link = name
                break

        # 通过 DFS 找到从 base 到末端 (没有 child 的连杆) 的最长链
        joint_chain = URDFParser._find_kinematic_chain(
            base_link, child_map, joint_map
        )

        return RobotModel(
            name=robot_name,
            links=links,
            joints=joints,
            joint_map=joint_map,
            parent_map=parent_map,
            child_map=child_map,
            joint_chain=joint_chain,
        )

    @staticmethod
    def _find_kinematic_chain(
        base: str,
        child_map: Dict[str, List[str]],
        joint_map: Dict[str, JointInfo],
    ) -> List[JointInfo]:
        """
        通过 DFS 找到从 base 到每个末端的关节链。
        对于串联机器人，返回从 base 到唯一末端的链；
        对于树形机器人，返回最长链。
        """
        # 找到所有末端连杆 (没有子连杆的)
        end_links = [name for name, children in child_map.items() if not children]

        def get_chain_to(end_link: str) -> List[JointInfo]:
            """回溯从 base 到 end_link 的关节链"""
            chain = []
            current = end_link
            while True:
                # 找到连向 current 的关节
                found = None
                for j in joint_map.values():
                    if j.child_link == current:
                        found = j
                        break
                if found is None:
                    break
                chain.insert(0, found)
                current = found.parent_link
            return chain

        # 取最长链
        longest = []
        for end_link in end_links:
            chain = get_chain_to(end_link)
            if len(chain) > len(longest):
                longest = chain

        return longest


# ============================================================
# 正运动学引擎
# ============================================================

class URDFKinematics:
    """
    基于 URDF 模型的正运动学求解器。

    用法:
        >>> robot = URDFKinematics("genkiarm.urdf")
        >>> T = robot.forward_kinematics([0.0, 0.5, -0.8, 0.3, 0.0, 0.0])
        >>> print(f"末端位置: {T[:3, 3]}")
        >>> print(f"末端姿态 (RPY): {robot.get_end_effector_rpy([...])}")
    """

    def __init__(self, urdf_path: str):
        """
        初始化运动学求解器。

        参数:
            urdf_path: URDF 文件路径
        """
        self.model = URDFParser.parse(urdf_path)
        self._num_joints = len(self.model.joint_chain)

        # 打印加载信息
        print(f"已加载机器人模型: {self.model.name}")
        print(f"  连杆数: {len(self.model.links)}")
        print(f"  关节数: {len(self.model.joints)}")
        print(f"  运动学链 (从 base 到末端):")
        for i, j in enumerate(self.model.joint_chain):
            print(f"    J{i}: {j.name}  [{j.joint_type}]  {j.parent_link} -> {j.child_link}")
            print(f"         origin: xyz={j.origin_xyz}  rpy={j.origin_rpy}")
            print(f"         axis: {j.axis_xyz}  limit: [{j.lower_limit}, {j.upper_limit}]")

    # ----------------------------------------------------------
    # 关节变换矩阵
    # ----------------------------------------------------------

    def joint_transform(self, joint: JointInfo, angle: float) -> "np.ndarray":
        """
        计算单个关节的变换矩阵 (从父连杆到子连杆)。

        URDF 约定:
          T_parent_to_child = T_origin * R_axis(angle)

        其中:
          T_origin = translate(origin_xyz) * rotate_rpy(origin_rpy)
          R_axis = 绕关节轴旋转 angle 弧度

        参数:
            joint:  关节信息
            angle:  关节角度 (弧度)
        返回:
            4x4 齐次变换矩阵
        """
        import numpy as np

        # 1. origin 平移
        t = np.array(joint.origin_xyz)

        # 2. origin 旋转 (RPY)
        R_origin = rpy_to_rotation_matrix(*joint.origin_rpy)

        # 3. 关节轴旋转
        R_axis = axis_angle_rotation(joint.axis_xyz, angle)

        # 总变换: T = T_trans * T_rpy * T_axis
        T_origin = make_transform(R_origin, t)
        T_axis = make_transform(R_axis, np.zeros(3))

        return T_origin @ T_axis

    # ----------------------------------------------------------
    # 正运动学 (Forward Kinematics)
    # ----------------------------------------------------------

    def forward_kinematics(
        self,
        joint_angles: List[float],
        *,
        link_index: Optional[int] = None,
    ) -> "np.ndarray":
        """
        **正运动学 (FK) 主入口** — 给定关节角度，计算末端 (或指定连杆) 的位姿。

        参数:
            joint_angles: 关节角度列表 (弧度)，长度应与运动链关节数一致
            link_index:   可选，指定要计算位姿的连杆索引 (0=第1个活动连杆, None=末端)
        返回:
            4x4 齐次变换矩阵 T，表示末端/连杆在世界(Base)坐标系下的位姿

        用法:
            >>> robot = URDFKinematics("genkiarm.urdf")
            >>> T = robot.forward_kinematics([0.0, 0.5, -0.8, 0.3, 0.0, 0.0])
            >>> pos = T[:3, 3]         # 末端位置 [x, y, z]
            >>> rot = T[:3, :3]        # 末端姿态 3x3 旋转矩阵
        """
        import numpy as np

        if len(joint_angles) != self._num_joints:
            raise ValueError(
                f"期望 {self._num_joints} 个关节角度，但收到了 {len(joint_angles)} 个"
            )

        # 关节限位检查 (警告但不阻止)
        for i, (angle, joint) in enumerate(zip(joint_angles, self.model.joint_chain)):
            if angle < joint.lower_limit or angle > joint.upper_limit:
                print(
                    f"[WARN] J{i} ({joint.name}) 角度 {angle:.4f} "
                    f"超出限位 [{joint.lower_limit}, {joint.upper_limit}]"
                )

        T = np.eye(4)
        target_idx = link_index if link_index is not None else self._num_joints - 1

        for i, (angle, joint) in enumerate(zip(joint_angles, self.model.joint_chain)):
            T_joint = self.joint_transform(joint, angle)
            T = T @ T_joint

            if i == target_idx:
                return T

        return T

    def forward_kinematics_degrees(
        self,
        joint_angles_deg: List[float],
        **kwargs,
    ) -> "np.ndarray":
        """
        与 forward_kinematics 相同，但输入角度单位为度。

        参数:
            joint_angles_deg: 关节角度列表 (度)
        """
        angles_rad = [math.radians(a) for a in joint_angles_deg]
        return self.forward_kinematics(angles_rad, **kwargs)

    # ----------------------------------------------------------
    # 正解 (FK) — 关节角度 → 末端位置
    # ----------------------------------------------------------

    def fk(
        self,
        joint_angles: List[float],
        *,
        degrees: bool = False,
    ) -> Tuple[float, float, float]:
        """
        **正解方法** — 将关节角度转化为末端执行器的空间位置。

        参数:
            joint_angles: 6 个关节的角度值
            degrees:      角度单位 — False=弧度(默认), True=度
        返回:
            (x, y, z) 末端位置，单位: 米

        用法:
            >>> robot = URDFKinematics("genkiarm.urdf")

            >>> # 弧度输入 (默认)
            >>> x, y, z = robot.fk([0.0, 0.5, -0.8, 0.0, 0.0, 0.0])

            >>> # 度输入
            >>> x, y, z = robot.fk([0.0, 30.0, -45.0, 0.0, 0.0, 0.0], degrees=True)
        """
        angles = [math.radians(a) for a in joint_angles] if degrees else list(joint_angles)
        T = self.forward_kinematics(angles)
        return (float(T[0, 3]), float(T[1, 3]), float(T[2, 3]))

    # ----------------------------------------------------------
    # 便捷方法
    # ----------------------------------------------------------

    def get_end_effector_pose(
        self, joint_angles: List[float]
    ) -> Tuple[float, float, float, float, float, float]:
        """
        计算末端执行器位姿，返回 (x, y, z, roll, pitch, yaw)。

        参数:
            joint_angles: 关节角度列表 (弧度)
        返回:
            (x, y, z, roll, pitch, yaw) — 位置 (米) + 姿态 RPY (弧度)
        """
        T = self.forward_kinematics(joint_angles)
        x, y, z = T[0, 3], T[1, 3], T[2, 3]
        roll, pitch, yaw = rotation_matrix_to_rpy(T[:3, :3])
        return (x, y, z, roll, pitch, yaw)

    def get_link_pose(
        self, joint_angles: List[float], link_name: str
    ) -> Optional["np.ndarray"]:
        """
        计算指定连杆的位姿。

        参数:
            joint_angles: 关节角度列表 (弧度)
            link_name:    连杆名称 (如 "jian1", "wan2" 等)
        返回:
            4x4 齐次变换矩阵，若连杆不在运动链上则返回 None
        """
        import numpy as np

        T = np.eye(4)
        for i, (angle, joint) in enumerate(zip(joint_angles, self.model.joint_chain)):
            T_joint = self.joint_transform(joint, angle)
            T = T @ T_joint

            if joint.child_link == link_name:
                return T

        # 检查是否是 base
        if link_name == self.model.joint_chain[0].parent_link:
            return np.eye(4)

        return None

    def get_all_link_poses(
        self, joint_angles: List[float]
    ) -> Dict[str, "np.ndarray"]:
        """
        计算所有连杆在世界坐标系下的位姿。

        参数:
            joint_angles: 关节角度列表 (弧度)
        返回:
            {link_name: 4x4_transform_matrix} 字典
        """
        import numpy as np

        poses = {self.model.joint_chain[0].parent_link: np.eye(4)}

        T = np.eye(4)
        for angle, joint in zip(joint_angles, self.model.joint_chain):
            T = T @ self.joint_transform(joint, angle)
            poses[joint.child_link] = T.copy()

        return poses

    # ----------------------------------------------------------
    # 雅可比矩阵 (用于逆运动学、速度控制等)
    # ----------------------------------------------------------

    def jacobian(self, joint_angles: List[float]) -> "np.ndarray":
        """
        计算末端执行器的几何雅可比矩阵 (6 x N)。

        雅可比矩阵将关节速度映射到末端速度:
          [v; ω] = J(q) * q̇

        其中 v 是线速度 (3x1)，ω 是角速度 (3x1)。

        参数:
            joint_angles: 关节角度列表 (弧度)
        返回:
            6 x N 雅可比矩阵
        """
        import numpy as np

        N = self._num_joints
        J = np.zeros((6, N))

        # 先计算末端位姿
        T_end = self.forward_kinematics(joint_angles)
        p_end = T_end[:3, 3]

        # 计算每个关节的雅可比列
        # 关键: p_i 必须是关节旋转轴上的一点 (T_origin 之后),
        #       而不是父连杆原点, 否则末端关节的 Jv 列会错误非零
        T_cum = np.eye(4)
        for i in range(N):
            joint = self.model.joint_chain[i]

            # -- 先应用关节 origin (平移 + RPY), 到达关节轴所在位置 --
            t_origin = np.array(joint.origin_xyz)
            R_origin = rpy_to_rotation_matrix(*joint.origin_rpy)
            T_origin = make_transform(R_origin, t_origin)
            T_cum = T_cum @ T_origin

            # 此刻 T_cum 在关节轴上 — 取位置和轴方向
            p_joint = T_cum[:3, 3]
            z_axis = T_cum[:3, :3] @ np.array(joint.axis_xyz)

            # -- 再应用关节轴旋转, 到达子连杆 --
            R_axis = axis_angle_rotation(joint.axis_xyz, joint_angles[i])
            T_axis = make_transform(R_axis, np.zeros(3))
            T_cum = T_cum @ T_axis

            # 计算雅可比列
            # 线速度部分: Jv_i = z_i × (p_end - p_i)  (旋转关节)
            # 角速度部分: Jω_i = z_i                      (旋转关节)
            J[:3, i] = np.cross(z_axis, p_end - p_joint)
            J[3:, i] = z_axis

        return J

    # ----------------------------------------------------------
    # 属性
    # ----------------------------------------------------------

    @property
    def num_joints(self) -> int:
        """活动关节数量"""
        return self._num_joints

    @property
    def joint_names(self) -> List[str]:
        """关节名称列表 (按运动链顺序)"""
        return [j.name for j in self.model.joint_chain]

    @property
    def joint_limits(self) -> List[Tuple[float, float]]:
        """关节限位列表 [(lower, upper), ...]"""
        return [(j.lower_limit, j.upper_limit) for j in self.model.joint_chain]

    def print_model_info(self):
        """打印模型详细信息"""
        print(f"\n{'='*60}")
        print(f"机器人模型: {self.model.name}")
        print(f"{'='*60}")
        print(f"\n连杆列表 ({len(self.model.links)}):")
        for name, link in self.model.links.items():
            mesh = link.mesh_file or "无"
            print(f"  {name}: mesh={mesh}")

        print(f"\n关节列表 ({len(self.model.joints)}):")
        for j in self.model.joints:
            print(f"  {j.name}:")
            print(f"    类型: {j.joint_type}")
            print(f"    连接: {j.parent_link} -> {j.child_link}")
            print(f"    origin xyz: {j.origin_xyz}")
            print(f"    origin rpy: {j.origin_rpy}")
            print(f"    axis: {j.axis_xyz}")
            print(f"    限位: [{j.lower_limit}, {j.upper_limit}]")
            print(f"    力矩/速度: {j.effort_limit} / {j.velocity_limit}")

        print(f"\n运动学链 ({self._num_joints} DOF):")
        print(f"  {' -> '.join([self.model.joint_chain[0].parent_link] + [j.child_link for j in self.model.joint_chain])}")
        print(f"  {' -> '.join(self.joint_names)}")


# ============================================================
# 示例与演示
# ============================================================

def demo():
    """演示 URDF 解析和正运动学计算"""
    import os

    # 查找 URDF 文件
    script_dir = os.path.dirname(os.path.abspath(__file__))
    urdf_path = os.path.join(script_dir, "genkiarm.urdf")

    if not os.path.exists(urdf_path):
        print(f"错误: 找不到 URDF 文件 {urdf_path}")
        return

    # 1. 加载机器人模型
    print("=" * 60)
    print("步骤 1: 解析 URDF 文件")
    print("=" * 60)
    robot = URDFKinematics(urdf_path)

    # 2. 打印模型信息
    print("\n" + "=" * 60)
    print("步骤 2: 模型详细信息")
    print("=" * 60)
    robot.print_model_info()

    # 3. 正运动学测试 — 零位
    print("\n" + "=" * 60)
    print("步骤 3: 正运动学计算")
    print("=" * 60)

    zero_angles = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
    T_zero = robot.forward_kinematics(zero_angles)
    pos, rpy = T_zero[:3, 3], rotation_matrix_to_rpy(T_zero[:3, :3])
    print(f"\n零位时末端位姿:")
    print(f"  位置 (m): x={pos[0]:.4f}, y={pos[1]:.4f}, z={pos[2]:.4f}")
    print(f"  姿态 (rad): roll={rpy[0]:.4f}, pitch={rpy[1]:.4f}, yaw={rpy[2]:.4f}")
    print(f"  姿态 (deg): roll={math.degrees(rpy[0]):.2f}°, pitch={math.degrees(rpy[1]):.2f}°, yaw={math.degrees(rpy[2]):.2f}°")

    # 4. 多组关节角度测试
    print("\n--- 多组关节角度测试 ---")

    test_configs = [
        ("零位", [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]),
        ("弯腰", [0.5, 0.0, 0.0, 0.0, 0.0, 0.0]),
        ("抬肩", [0.0, 0.8, 0.0, 0.0, 0.0, 0.0]),
        ("屈肘", [0.0, 0.0, -0.8, 0.0, 0.0, 0.0]),
        ("组合1", [0.3, 0.5, -0.8, 0.4, 0.0, 0.0]),
        ("组合2", [-0.5, 0.7, -1.0, 0.3, 1.0, 0.5]),
        ("伸展位", [0.0, 1.0, -0.5, -0.5, 0.0, 0.0]),
    ]

    for label, angles in test_configs:
        T = robot.forward_kinematics(angles)
        pos = T[:3, 3]
        r, p, y = rotation_matrix_to_rpy(T[:3, :3])
        print(f"\n  [{label}]")
        print(f"    关节角 (rad): {[f'{a:.3f}' for a in angles]}")
        print(f"    关节角 (deg): {[f'{math.degrees(a):.1f}°' for a in angles]}")
        print(f"    位置 (m): x={pos[0]:.4f}, y={pos[1]:.4f}, z={pos[2]:.4f}")
        print(f"    姿态 (deg): roll={math.degrees(r):.1f}°, pitch={math.degrees(p):.1f}°, yaw={math.degrees(y):.1f}°")

    # 5. 所有连杆位姿
    print("\n" + "=" * 60)
    print("步骤 4: 所有连杆位姿 (零位)")
    print("=" * 60)
    poses = robot.get_all_link_poses(zero_angles)
    for name, T in poses.items():
        pos = T[:3, 3]
        print(f"  {name}: x={pos[0]:.4f}, y={pos[1]:.4f}, z={pos[2]:.4f}")

    # 6. 雅可比矩阵
    print("\n" + "=" * 60)
    print("步骤 5: 雅可比矩阵 (零位)")
    print("=" * 60)
    J = robot.jacobian(zero_angles)
    np = __import__("numpy")
    np.set_printoptions(precision=3, suppress=True)
    print(J)


if __name__ == "__main__":
    demo()
