"""搞清楚 URDF 坐标系 — 零位时的指向"""
import math
from urdf_fk import URDFKinematics

robot = URDFKinematics("genkiarm.urdf")

# 全部关节归零
q = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
x, y, z = robot.fk(q)

print("=== 全零位 (所有关节 = 0 rad) ===")
print("末端位置: (%.4f, %.4f, %.4f)" % (x, y, z))
print()
print("从 Base 原点看过去:")
print("  J1=0 时，大臂从 Base 往哪伸？")

# 逐关节看累积位置
from urdf_fk import URDFKinematics
T = robot.forward_kinematics(q)
print("  末端 T =")
for row in range(4):
    print("    [%+.4f  %+.4f  %+.4f  %+.4f]" % tuple(T[row]))

# 算各连杆位置
poses = robot.get_all_link_poses(q)
print()
print("=== 各连杆原点位置 (世界坐标) ===")
for name, T_link in poses.items():
    p = T_link[:3, 3]
    print("  %-10s  (%+.4f, %+.4f, %+.4f)" % (name, p[0], p[1], p[2]))

# 看 RPY=(-1.57) 的效果 — J1 轴从局部X扭到世界哪个方向
import numpy as np
Ry_neg90 = np.array([
    [math.cos(-1.57), 0, math.sin(-1.57)],
    [0, 1, 0],
    [-math.sin(-1.57), 0, math.cos(-1.57)]
])
local_x = np.array([1.0, 0.0, 0.0])
world_axis = Ry_neg90 @ local_x
print()
print("=== J1 轴方向 ===")
print("  URDF: axis=(1,0,0), origin rpy=(0, -1.57, 0)")
print("  局部X 经过 Ry(-90°) 后在世界坐标的方向:")
print("  world_axis = (%.1f, %.1f, %.1f)" % tuple(world_axis))
print("  → J1 实际绕世界 Z 轴旋转 (腰转)")
