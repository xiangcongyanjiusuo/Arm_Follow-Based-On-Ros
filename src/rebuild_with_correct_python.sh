#!/bin/bash
export PATH="/usr/bin:$PATH"
unset PYTHONPATH
export PYTHON_EXECUTABLE=/usr/bin/python3

cd /home/xiangcong/dev_ws

echo "Using Python: $(which python3)"
echo "Python version: $(python3 --version)"

colcon build --packages-select arm_msg arm_pkg \
  --cmake-args \
  -DPYTHON_EXECUTABLE=/usr/bin/python3 \
  -DPython_EXECUTABLE=/usr/bin/python3
