# Arm Catch Box — ROS2 机械臂视觉抓取系统

基于 ROS2 Jazzy 的眼在手外机械臂视觉抓取系统。摄像头俯拍桌面，HSV 颜色检测识别目标，转换世界坐标后通过逆运动学解算关节角度，驱动 6 轴舵机机械臂完成抓取。以下是vibe coding的开发流程记录

---

## 目录

- [系统架构](#系统架构)
- [文件结构](#文件结构)
- [快速开始](#快速开始)
- [节点说明](#节点说明)
  - [arm_joint_node — 舵机驱动](#arm_joint_node)
  - [arm_catch_box_node — IK 解算与跟随](#arm_catch_box_node)
  - [world_joint — IK/FK 数学库](#world_joint)
  - [hsv_image_node — 目标检测](#hsv_image_node)
  - [tf_node — 坐标变换](#tf_node)
  - [camera_native_node — 画面查看](#camera_native_node)
- [数据流](#数据流)
- [标定指南](#标定指南)
- [话题参考](#话题参考)
- [命名约定](#命名约定)

---

## 系统架构

```
  Camera                    Vision Pipeline                    Arm Control
  ──────                    ──────────────                    ──────────

  /dev/video0               vision_pkg                        arm_pkg
      │                         │                                │
      ├─ usb_cam                │                                │
      │   └─ /image_raw ────────┼─→ camera_native_node (查看)     │
      │                         │                                │
      │                         ├─→ hsv_image_node               │
      │                         │     ├─ HSV 检测 + GUI          │
      │                         │     └─ /box_center (像素)      │
      │                         │           │                    │
      │                         ├─→ tf_node                      │
      │                         │     ├─ 像素→世界坐标 (mm)       │
      │                         │     └─ /world_xyz              │
      │                         │           │                    │
      │                         │           └────────────────────┼─→ arm_catch_box_node
      │                         │                                │     ├─ IK 解算 (seed 连续)
      │                         │                                │     ├─ J6 夹爪固定
      │                         │                                │     └─ cmd_angles (度)
      │                         │                                │           │
      │                         │                                ├─→ arm_joint_node
      │                         │                                │     ├─ 扭矩常开
      │                         │                                │     └─ 舵机 1~6
```

---

## 文件结构

```
dev_ws/
├── docs/
│   └── PROJECT.md
├── src/
│   ├── arm_msg/
│   │   └── msg/ArmJointAngles.msg        # float32[6] angles
│   │
│   ├── arm_pkg/
│   │   ├── arm_pkg/
│   │   │   ├── arm_joint_node.py          # 串口伺服驱动
│   │   │   ├── arm_catch_box_node.py      # IK 解算与角度发布
│   │   │   ├── world_joint.py             # IK/FK 数学库
│   │   │   └── genkiarm.urdf              # 机械臂 URDF 模型
│   │   ├── launch/
│   │   │   └── arm_catch_box.launch.py    # 一键启动
│   │   ├── setup.py
│   │   └── package.xml
│   │
│   └── vision_pkg/
│       ├── vision_pkg/
│       │   ├── camera_native_node.py       # 相机画面查看
│       │   ├── hsv_image_node.py           # HSV 检测 + 调参 GUI
│       │   └── tf_node.py                  # 像素→世界坐标
│       ├── setup.py
│       └── package.xml
```

---

## 快速开始

### 环境要求

- ROS2 Jazzy
- Python 3.12, NumPy, OpenCV, PySerial
- USB 摄像头 (`/dev/video0`)
- 6 轴舵机机械臂 (`/dev/ttyUSB0`)

### 编译

```bash
cd ~/dev_ws
colcon build --packages-select arm_pkg vision_pkg --symlink-install
source install/setup.bash
```

### 一键启动

```bash
ros2 launch arm_pkg arm_catch_box.launch.py
```

此命令按顺序启动 6 个节点：`usb_cam` → `camera_native_node` → `hsv_image_node` → `tf_node` → `arm_catch_box_node` → `arm_joint_node`。

### 分步启动（调试用）

```bash
# 1. 相机
ros2 run usb_cam usb_cam_node_exe --ros-args -p video_device:=/dev/video0

# 2. 机械臂驱动
ros2 run arm_pkg arm_joint_node --ros-args -p port:=/dev/ttyUSB0

# 3. HSV 检测
ros2 run vision_pkg hsv_image_node

# 4. 坐标转换
ros2 run vision_pkg tf_node

# 5. IK 解算
ros2 run arm_pkg arm_catch_box_node
```

### 验证话题

```bash
ros2 topic echo /box_center     # 像素坐标
ros2 topic echo /world_xyz      # 世界坐标 (mm)
ros2 topic echo cmd_angles      # 关节角度 (度)
```

> **提示**: 如果 `cmd_angles` 无输出，检查 `/world_xyz` 是否有数据；如果 `/world_xyz` 无数据，检查 `/box_center` 是否有数据；如果 `/box_center` 无数据，确认 HSV 检测窗口中能看到目标物体，并调整滑动条使检测框包围目标。

---

## 节点说明

### arm_joint_node

串口伺服驱动节点，通过 Dynamixel 协议 1.0 控制 6 个舵机。

| | |
|---|---|
| 包 | `arm_pkg` |
| 可执行文件 | `arm_joint_node` |

**话题**

| 名称 | 方向 | 类型 | 说明 |
|------|------|------|------|
| `arm_joint_states` | Pub | `sensor_msgs/JointState` | 当前角度 |
| `arm_joint_angles` | Pub | `arm_msg/ArmJointAngles` | 当前角度 (数组) |
| `cmd_angles` | Sub | `arm_msg/ArmJointAngles` | 目标角度指令 |

**服务**

| 名称 | 类型 | 说明 |
|------|------|------|
| `enable_torque` | `std_srvs/SetBool` | `true` 使能全部舵机，`false` 失能全部 |

**参数**

| 参数 | 默认值 | 说明 |
|------|--------|------|
| `port` | `/dev/ttyUSB0` | 串口设备路径 |
| `baudrate` | `1000000` | 波特率 |
| `servo_ids` | `[1,2,3,4,5,6]` | 舵机 ID |
| `read_frequency` | `50.0` | 状态回读频率 (Hz) |

**行为说明**

- 启动时读取当前角度，随即**使能全部舵机扭矩**并**持续保持**，直到节点退出时才失能释放。
- 采用扭矩常开策略，避免跟随过程中因目标短暂丢失而触发失能导致机械臂脱落。如需手动移动机械臂，调用 `enable_torque` 服务关闭扭矩即可。
- 舵机角度范围 [-90°, 90°]，对应 Dynamixel 寄存器值 1024~3072。

> **故障排查**: 如果机械臂不动，先确认串口设备存在 (`ls /dev/ttyUSB*`)。如果设备存在但仍不响应，用 `ros2 topic echo cmd_angles` 检查是否有角度指令到达。若无指令，说明上游节点异常。也可手动调用 `ros2 service call enable_torque std_srvs/SetBool "{data: true}"` 确认舵机能上电。

---

### arm_catch_box_node

接收世界坐标，通过逆运动学解算为关节角度，发送给舵机驱动。

| | |
|---|---|
| 包 | `arm_pkg` |
| 可执行文件 | `arm_catch_box_node` |

**话题**

| 名称 | 方向 | 类型 | 说明 |
|------|------|------|------|
| `/world_xyz` | Sub | `geometry_msgs/PointStamped` | 世界坐标 **(mm)** |
| `cmd_angles` | Pub | `arm_msg/ArmJointAngles` | 关节角度 **(度)** |

**参数**

| 参数 | 默认值 | 说明 |
|------|--------|------|
| `input_topic` | `/world_xyz` | 订阅话题 |
| `output_topic` | `cmd_angles` | 发布话题 |
| `z_offset` | `0.15` | Z 轴抬高量 **(m)**，默认 15 cm |
| `j6_angle` | `-90.0` | J6 夹爪固定角度 **(度)**，-90° 为闭合 |

**处理流程**

接收到 `PointStamped` 消息后：

1. **坐标转换**: 将 mm 转为 m，X 轴取负（相机坐标系 → 机械臂坐标系），Z 轴叠加 `z_offset` 抬高量
2. **IK 求解**: 调用 `world_to_joint(xyz, seed=prev_angles)`，以上一帧的角度作为搜索种子
3. **夹爪覆写**: 将第 6 个关节角度强制设为目标值
4. **发布**: 将角度发送到 `cmd_angles`

**IK 种子机制**

逆运动学求解使用 DLS（阻尼最小二乘法）配合大量随机种子。对于某些目标位置，存在多个能量相近的关节配置。如果逐帧独立求解，相邻帧可能收敛到不同的解，导致舵机振荡，表现为机械臂运动卡顿。通过在每帧求解时传入上一帧的角度作为种子（`seed`），保证解的连续性。

```
Frame₁: IK(xyz₁, seed=None)   → 全局搜索 → angles₁
Frame₂: IK(xyz₂, seed=angles₁) → 局部搜索 → angles₂ ≈ angles₁
Frame₃: IK(xyz₃, seed=angles₂) → 局部搜索 → angles₃ ≈ angles₂
```

> **故障排查**: 如果机械臂朝某一方向运动平滑但反向卡顿，检查 `world_to_joint` 调用是否传入了 `seed` 参数。种子缺失会导致 IK 在左右两侧产生不一致的多解跳变。另外，如果 Z 轴方向臂撞到了桌面，增大 `z_offset` 参数即可。

---

### world_joint

纯 Python 模块，封装机械臂的正运动学（FK）和逆运动学（IK）计算。不作为 ROS 节点运行，由 `arm_catch_box_node` 导入调用。

| | |
|---|---|
| 文件 | `src/arm_pkg/arm_pkg/world_joint.py` |
| 依赖 | NumPy, `genkiarm.urdf` |

**对外接口**

```python
from arm_pkg.world_joint import world_to_joint

# 基本调用
angles = world_to_joint((0.15, 0.20, 0.05))

# 传入种子以保证连续帧的解稳定
angles = world_to_joint((0.15, 0.20, 0.05), seed=prev_angles)
```

| 参数 | 类型 | 说明 |
|------|------|------|
| `xyz` | `tuple[float]` | 世界坐标 **(米)**，基于现实坐标系 |
| `seed` | `list[float] \| None` | 上一帧的关节角度 (度)，作为局部搜索起点 |
| 返回值 | `list[float] \| None` | 6 个关节角度 **(度)**，J6 锁定为 0°。失败返回 `None` |

**内部坐标系转换**

URDF 模型坐标系与机械臂实际安装方向存在差异。模块内部通过 `_real_to_urdf(x, y, z) = (y, x, z)` 完成转换：现实世界的 XY 在 URDF 中互换。

**求解算法**

1. **DLS（阻尼最小二乘法）**: 主迭代求解器，自适应阻尼系数
2. **零空间投影**: 利用冗余自由度将关节推向行程中心，避免限位卡死
3. **多种子搜索**: 预设姿态 + 均匀采样 + 高斯扰动，共 200+ 个种子并行搜索最优解
4. **精细化后处理**: 对最优解进行 500 次小步长精修

> **故障排查**: 如果 IK 频繁报错或返回 `None`，首先检查输入坐标是否在机械臂工作空间内。URDF 模型文件必须与 `world_joint.py` 在同一目录，缺失时模块加载即报错。

---

### hsv_image_node

HSV 颜色空间目标检测节点，提供实时调参 GUI。

| | |
|---|---|
| 包 | `vision_pkg` |
| 可执行文件 | `hsv_image_node` |

**话题**

| 名称 | 方向 | 类型 | 说明 |
|------|------|------|------|
| `/image_raw` | Sub | `sensor_msgs/Image` | 相机图像 |
| `/box_center` | Pub | `geometry_msgs/Point` | 目标中心像素坐标 |

**GUI 窗口**

| 窗口 | 尺寸 | 内容 |
|------|------|------|
| `HSV Tuner` | 960×640 | 检测画面、检测框、信息栏、Mask 小窗 |
| `Controls` | 420×320 | H/S/V 阈值滑块、腐蚀/膨胀滑块 |

**参数**

| 参数 | 默认值 | 范围 | 说明 |
|------|--------|------|------|
| `image_topic` | `/image_raw` | — | 订阅话题 |
| `publish_topic` | `/box_center` | — | 发布话题 |
| `h_min` | `110` | 0-179 | 色相下限 |
| `h_max` | `117` | 0-179 | 色相上限 |
| `s_min` | `146` | 0-255 | 饱和度下限 |
| `s_max` | `255` | 0-255 | 饱和度上限 |
| `v_min` | `73` | 0-255 | 明度下限 |
| `v_max` | `255` | 0-255 | 明度上限 |
| `erode_iter` | `3` | 0-10 | 腐蚀迭代次数 |
| `dilate_iter` | `5` | 0-10 | 膨胀迭代次数 |
| `min_area` | `500` | — | 最小轮廓面积 |

**快捷键**: `q` 退出，`s` 截图。

> **故障排查**: 如果 `HSV Tuner` 窗口未弹出，先确认是否有 X11 显示环境 (`echo $DISPLAY`)。如果窗口黑屏无图像，用 `ros2 topic hz /image_raw` 检查相机是否正常推流。如果画面中检测不到目标，用 Controls 窗口中的滑块调参直到检测框（绿色）包围目标物。如果只检测到背景噪点，增大 `min_area` 过滤面积阈值。

---

### tf_node

将 HSV 检测输出的像素坐标转换为机械臂基坐标系下的物理坐标。

| | |
|---|---|
| 包 | `vision_pkg` |
| 可执行文件 | `tf_node` |

**话题**

| 名称 | 方向 | 类型 | 说明 |
|------|------|------|------|
| `/box_center` | Sub | `geometry_msgs/Point` | 像素坐标 |
| `/world_xyz` | Pub | `geometry_msgs/PointStamped` | 世界坐标 **(mm)** |

**TF**

| 父帧 | 子帧 | 频率 |
|------|------|------|
| `Base` | `target` | 10 Hz |

**坐标模型**

采用针孔平面投影模型（默认摄像头垂直俯拍桌面）：

```
dx_px  = pixel_x - image_width / 2
dy_px  = pixel_y - image_height / 2
dx_mm  = dx_px × scale_x
dy_mm  = dy_px × scale_y
X_world = camera_x + dx_mm
Y_world = camera_y + dy_mm
Z_world = target_z
```

**参数**

| 参数 | 默认值 | 说明 |
|------|--------|------|
| `input_topic` | `/box_center` | 订阅话题 |
| `output_topic` | `/world_xyz` | 发布话题 |
| `camera_x` | `-26.0` | 相机在 Base 系 X 坐标 (mm) |
| `camera_y` | `270.0` | 相机在 Base 系 Y 坐标 (mm) |
| `camera_z` | `365.0` | 相机高度 (mm) |
| `scale_x` | `0.5` | X 方向缩放 (mm/pixel) |
| `scale_y` | `0.5` | Y 方向缩放 (mm/pixel) |
| `image_width` | `640` | 图像宽度 (px) |
| `image_height` | `480` | 图像高度 (px) |
| `target_z` | `0.0` | 目标平面高度 (mm) |
| `stale_timeout` | `0.5` | 目标丢失后停止发布的超时 (秒) |
| `arm_base_frame` | `Base` | TF 父帧 |
| `target_frame` | `target` | TF 子帧 |

**stale_timeout 机制**

HSV 检测在目标被遮挡（例如机械臂运动到相机前方）时会短暂丢失目标。如果不处理，节点会持续发布最后一次检测到的坐标，导致视觉画面出现"残留框"。`stale_timeout` 参数控制：超过设定时间未收到新检测结果时，自动清空缓冲区并停止发布。

> **故障排查**: 如果 `/world_xyz` 坐标与预期偏差较大，说明 `camera_x/y` 或 `scale_x/y` 需要重新标定，参见 [标定指南](#标定指南)。如果画面中目标可见但无坐标输出，检查 `stale_timeout` 是否过短。

---

### camera_native_node

纯查看节点，显示原始相机画面，不做任何处理。

| | |
|---|---|
| 包 | `vision_pkg` |
| 可执行文件 | `camera_native_node` |

订阅 `/image_raw`，显示 `Camera Display` 窗口。**快捷键**: `q` 退出，`s` 截图，`f` 全屏。

---

## 数据流

```
┌─────────────────────────────────────────────────────────────────────┐
│  Vision Pipeline                                     Unit: px → mm  │
│                                                                     │
│  usb_cam                                                           │
│    │  /image_raw (30 Hz)                                           │
│    ├──────────→ camera_native_node  [查看]                          │
│    │                                                                 │
│    └──────────→ hsv_image_node      [HSV 检测]                      │
│                   │  /box_center (~30 Hz, Point)                    │
│                   ▼                                                 │
│                 tf_node             [坐标变换]                       │
│                   │  /world_xyz (10 Hz, PointStamped)               │
│                   │                                                 │
├───────────────────┼─────────────────────────────────────────────────┤
│  Arm Pipeline                                       Unit: m → deg   │
│                   │                                                 │
│                   ▼                                                 │
│                 arm_catch_box_node  [IK 解算]                       │
│                   │  cmd_angles (按需, ArmJointAngles)              │
│                   ▼                                                 │
│                 arm_joint_node      [舵机驱动]                       │
│                   │  Dynamixel 协议 1.0                             │
│                   ▼                                                 │
│              舵机 1~6  (扭矩常开)                                    │
└─────────────────────────────────────────────────────────────────────┘
```

关键转换链路：

```
像素 → (camera_x/y + dx/dy × scale) → 世界坐标 mm
     → ÷1000 → m
     → X 取负 → 机械臂坐标系
     → Z + z_offset → 抬高 15cm
     → XY 互换 → URDF 坐标系
     → IK(seed=prev) → 关节角度 (度)
     → J6 覆写 → 夹爪闭合
```

---

## 标定指南

标定精度直接影响抓取成功率，以下按顺序完成。

### 1. 确定 scale（mm/pixel）

1. 将摄像头固定在工作高度
2. 在桌面检测平面放置一把尺子
3. 启动 `hsv_image_node` 观察画面
4. 记录尺子在画面中的像素长度

```
scale_x = 尺子实际长度 (mm) / 画面像素宽度 (px)
scale_y = 尺子实际长度 (mm) / 画面像素高度 (px)
```

### 2. 确定 camera 偏移

1. 将摄像头移动到机械臂基坐标系原点正上方
2. 记录此时机械臂末端在 Base 系中的 (X, Y) → 即 `camera_x`, `camera_y`

**快速校准法**: 将一个物体放在已知 Base 坐标处，查看 `/world_xyz` 输出，调整偏移直到读数匹配：

```
物体实际 X=0 但显示 +39mm   →  camera_x -= 39
物体实际 Y=0 但显示 -20mm   →  camera_y += 20
```

### 3. 确定 target_z

检测平面（桌面）在基坐标系中的 Z 值。若桌面对齐 Z=0，则 `target_z = 0`。

> **标定验证**: 启动完整流程后，移动标定物到不同位置，对比 `/world_xyz` 输出与实际位置。误差应控制在 ±5mm 以内。若系统误差持续偏大，检查摄像头是否松动。

---

## 话题参考

| 话题 | 类型 | 发布者 | 单位 | 频率 |
|------|------|--------|------|------|
| `/image_raw` | `Image` | usb_cam | — | 30 Hz |
| `/box_center` | `Point` | hsv_image_node | 像素 | ~30 Hz |
| `/world_xyz` | `PointStamped` | tf_node | mm | 10 Hz |
| `cmd_angles` | `ArmJointAngles` | arm_catch_box_node | 度 | 按需 |
| `arm_joint_states` | `JointState` | arm_joint_node | 度 | 50 Hz |
| `arm_joint_angles` | `ArmJointAngles` | arm_joint_node | 度 | 50 Hz |

---

## 命名约定

| 约定 | 说明 |
|------|------|
| TF 帧: `Base` | 机械臂基坐标系 |
| TF 帧: `zhua` | 末端执行器 |
| TF 帧: `target` | 检测目标 |
| 物理坐标 | 消息和参数使用 **mm**，TF 内部自动转 **m** |
| 关节角度 | **度**，范围 [-90°, 90°] |
| 像素坐标原点 | 图像左上角 (0, 0) |
