#!/bin/bash

echo "=========================================="
echo "  Vision System Debug & Test Script"
echo "=========================================="
echo ""

echo "[步骤1] 检查 usb_cam 节点状态..."
source /opt/ros/jazzy/setup.bash
ros2 node list | grep -q "usb_cam" && echo "✅ usb_cam 节点正在运行" || echo "❌ usb_cam 未运行"

echo ""
echo "[步骤2] 检查图像话题发布情况..."
ros2 topic info /image_raw 2>/dev/null && {
    echo "✅ /image_raw 话题存在"
    ros2 topic hz /image_raw --window 3 &
    HZ_PID=$!
    sleep 4
    kill $HZ_PID 2>/dev/null
} || echo "❌ /image_raw 不存在"

echo ""
echo "[步骤3] 列出所有可用摄像头..."
ls -la /dev/video* 2>/dev/null || echo "未检测到摄像头设备"

echo ""
echo "[步骤4] 启动 camera_native_node (订阅 /image_raw)..."
echo "按 'q' 键退出显示窗口"
echo ""

source /home/xiangcong/dev_ws/install/setup.bash
ros2 run vision_pkg camera_native_node --ros-args -p image_topic:=/image_raw
