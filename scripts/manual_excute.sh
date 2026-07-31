#!/usr/bin/env bash
# run_stack.sh
set -e
source /opt/ros/humble/setup.bash                     # 例如 humble
source /home/uni-co-jetson/ros2_ws/install/setup.bash
exec ros2 launch unico_pack manager_system_bringup.launch.py