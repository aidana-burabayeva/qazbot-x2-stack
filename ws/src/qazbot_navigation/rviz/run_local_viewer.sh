#!/usr/bin/env bash
# PC-only viewer for X2 on the same Wi-Fi. No robot process or motion adapter starts here.
set -e
QAZBOT_VIEW_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
source /opt/ros/humble/setup.bash
export ROS_DOMAIN_ID=0 ROS_LOCALHOST_ONLY=0 RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export FASTRTPS_DEFAULT_PROFILES_FILE="$QAZBOT_VIEW_DIR/dds_pc_x2.xml"
exec /opt/ros/humble/bin/rviz2 -d "$QAZBOT_VIEW_DIR/localization.rviz" --ros-args \
  -r __ns:=/qazbot_d1_map -r /tf:=/qazbot_d1_map/tf -r /tf_static:=/qazbot_d1_map/tf_static
