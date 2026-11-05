#!/usr/bin/env bash
# RViz2 for the X2 stack from a PC on the robot's wifi (192.168.1.0/24).
# dds_pc_x2.xml pins this PC's wifi address (192.168.1.64) and peers with
# X2 .66; edit both if either changes. The robot's own DDS config is untouched.
# Namespace /qazbot_d1_map makes the Nav2 panel and goal tool use our stack.
set -e
HERE="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
source /opt/ros/humble/setup.bash
export FASTRTPS_DEFAULT_PROFILES_FILE="$HERE/dds_pc_x2.xml"
NS=/qazbot_d1_map
exec ros2 run rviz2 rviz2 -d "$HERE/x2_nav.rviz" --ros-args -r __ns:=$NS \
  -r /tf:=$NS/tf -r /tf_static:=$NS/tf_static
