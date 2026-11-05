#!/usr/bin/env bash
# Full Nav2 (planner, RPP controller, smoother, bt_navigator) + goal coordinator; output only to the
# private /qazbot_d1_map/cmd_vel -> coordinator gate -> /qazbot_d1_map/cmd_vel_authorized (x2_motion_adapter).
# Nothing here talks to MC or publishes the global /cmd_vel. Extra args go to ros2 launch
# (e.g. allow_dry_run_driver:=true for a dry-run adapter). QAZBOT_ODOM_TOPIC: odometry for RPP / bt_navigator /
# velocity_smoother (default /qazbot_lio/odom_level). The qualified Smac overlay is the workspace nav2_smac_planner
# (ws/third_party) unless QAZBOT_SMAC_PREFIX selects another prefix.
set -e
source "$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)/env.sh"
exec ros2 launch qazbot_navigation nav_full.launch.py robot_radius:=${ROBOT_RADIUS:?set ROBOT_RADIUS (m): body envelope, no default} \
  scan_topic:=/qazbot_lio/scan odom_topic:=${QAZBOT_ODOM_TOPIC:-/qazbot_lio/odom_level} "$@"
