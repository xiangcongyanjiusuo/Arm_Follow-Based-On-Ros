<p align="center">
  <img src="https://img.shields.io/badge/ROS2-Jazzy-22314E?logo=ros" alt="ROS2 Jazzy">
  <img src="https://img.shields.io/badge/Python-3.12-3776AB?logo=python" alt="Python 3.12">
</p>

# Arm Catch Box

基于 ROS2 Jazzy 的眼在手外机械臂视觉抓取系统。摄像头俯拍桌面，HSV 颜色检测识别目标，转换世界坐标后通过逆运动学解算关节角度，驱动 6 轴舵机机械臂完成抓取。

核心采用五个ROS节点，相机驱动usb_camera,颜色区分与相机坐标发布hsv_image_node,相机坐标转换世界坐标tf_node,世界坐标实现ik转化为6个电机角度arm_catch_box_node节点，然后传机械臂驱动节点arm_joint_node

---

## 注意事项

- 驱动节点采用飞特舵机，详情请查看飞特舵机官网

- 颜色区分采用hsv，发布仅发布一个坐标，考虑到机械臂不能有多个目标同时去处理

- 目标丢失后 tf_node 会持续发布最后一次检测到的坐标，导致视觉画面残留检测框和中心点。修复方式是加入 stale_timeout 参数——超过 0.5 秒未收到新像素数据则自动清空缓冲区并停止发布，目标重现后立即恢复。

- 相机坐标转换世界坐标需要进行相机内参标定，手眼标定获取坐标系转化直接替换。也可以得到相机位姿后填入相应位置

- 世界坐标求IK之前应该先确定物理世界坐标系与URDF坐标系之间的相对位置关系，本项目中xy发生反转，同时x轴方向取反，可运行_check_frame.py文件来确定URDF坐标自行比对

- IK解算时，为了确保搭载灵巧手不移动，同时解算的精度高。IK 求解时用完整 6 自由度保证最高定位精度；发布时直接将 J5（腕关节）归零消除末端扭转——J5 只影响夹爪朝向不影响末端位置，所以归零不会降低精度。J6（夹爪）同理，IK 中锁定为 0°，发布时覆写为所需的夹爪开合角度。

- 流程为先根据URDF求出正解urdf_fk.py,逆解urdf_ik.py,然后进行机械臂计算与实际环境中偏差的计算ik_test,根据测试好的代码进行三者打包处理为world_joint，借助world_joint依赖写出arm_catch_box_node负责IK解算六个电机角度

- 在驱动节点arm_joint_node中的使能逻辑启动时读取当前角度后立即使能全部舵机，运行期间扭矩常开不做自动失能，仅节点退出时统一失能释放。
