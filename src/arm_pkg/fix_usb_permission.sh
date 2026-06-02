#!/bin/bash
echo "正在设置USB串口权限..."
if [ -e /dev/ttyUSB0 ]; then
    sudo chmod 666 /dev/ttyUSB0
    echo "✅ 权限设置完成: /dev/ttyUSB0"
else
    echo "❌ 未找到 /dev/ttyUSB0 设备"
    echo "请检查USB设备是否已连接"
fi
