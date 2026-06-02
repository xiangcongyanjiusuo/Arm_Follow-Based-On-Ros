# 技术方案与设计规范

## 整体架构

```
[相机] → /image_raw
    │
    ├── box_detect_node（HSV颜色检测）→ /box_detect/center（像素坐标）
    │
    ├── camera_intrinsic_calibration_node（相机内参标定）→ camera_intrinsics.yaml
    │
    └── camera_calibration_node（手眼标定）→ hand_eye_result.yaml
            │
            └── box_tf_node（坐标转换）→ /box_arm/position（3D基座坐标）
                    │
                    └── arm_tf_node（正向运动学TF）→ Base→zhua
```

## 坐标系定义

| 坐标系 | TF帧名 | 说明 |
|--------|--------|------|
| 机械臂基座 | `Base` | arm_tf_node 发布的 TF 树根节点 |
| 末端执行器 | `zhua` | 夹爪末端，TF 树叶子节点 |
| 相机 | `camera_link` | box_tf_node 广播的虚拟帧，相对 zhua 固定 |
| 目标物体 | `target_object` | box_tf_node 广播的虚拟帧，在基坐标系下 |

## 坐标转换管线

```
像素(u,v)
  │
  ├── 畸变校正：cv2.undistortPoints()
  │
  ├── 射线方向：d_cam = K⁻¹ · [u, v, 1]ᵀ
  │
  ├── TF获取 Base→zhua 位姿：T_base_ee
  │
  ├── 加载手眼标定：T_ee_cam（相机在末端坐标系下的位姿）
  │
  ├── 合成 T_base_cam = T_base_ee · T_ee_cam
  │
  ├── 变换射线到基坐标系：d_base = R_base_cam @ d_cam
  │
  ├── 桌面平面求交：z = table_height（由于棋盘格放在桌面上，
  │     手眼标定时已自动算好桌面在基坐标系下的高度，存入hand_eye_result.yaml）
  │
  └── 输出三维坐标（基坐标系，单位米）
```

## 文件格式规范

### camera_intrinsics.yaml

```yaml
image_width: 640
image_height: 480
camera_matrix:
  data: [fx, 0, cx, 0, fy, cy, 0, 0, 1]
dist_coeffs:
  data: [k1, k2, p1, p2, k3]
reprojection_error: 0.35
```

### hand_eye_result.yaml

```yaml
R_cam_to_ee:
  data: [R11, R12, R13, R21, R22, R23, R31, R32, R33]
t_cam_to_ee:
  data: [tx_mm, ty_mm, tz_mm]
error_mm: 3.2
num_samples: 25
```

## 编码规范

1. Python 节点统一使用 ROS2 rclpy 框架
2. 节点间通过 Topic 通信（Point、PointStamped），通过文件共享标定参数
3. TF 帧名在 `Base`/`zhua` 之间保持一致
4. 数学运算使用 numpy，避免手动矩阵乘法
5. 坐标单位为米（ROS标准），参数输入可使用毫米方便配置
6. 每个节点独立可运行，有完整的错误处理和日志输出

## 开发约定

1. 逐阶段推进，每阶段独立验证通过后再进入下一阶段
2. 每日在 dev_logs/ 中记录开发日志
3. 标定文件存放在项目根目录，便于查找和版本管理
4. 修改现有节点前先理解原有逻辑，保留可用部分
