#!/usr/bin/env bash
# manual_excute.sh
set -e
source /opt/ros/humble/setup.bash
source /home/uni-co-jetson/livox_ws/install/setup.bash
source /home/uni-co-jetson/zed_ws/install/setup.bash
source /home/uni-co-jetson/ros2_ws/install/setup.bash

exec ros2 launch unico_pack master.launch.py