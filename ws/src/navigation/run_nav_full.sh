#!/usr/bin/env bash
# Full Nav2 (planner, RPP controller, smoother, bt_navigator) + goal coordinator; output only to the
# private /qazbot_d1_map/cmd_vel -> coordinator gate -> /qazbot_d1_map/cmd_vel_authorized (x2_motion_adapter).
# Nothing here talks to MC or publishes the global /cmd_vel. Extra args go to ros2 launch
# (e.g. allow_dry_run_driver:=true for a dry-run adapter). QAZBOT_ODOM_TOPIC: odometry for RPP / bt_navigator /
# velocity_smoother (default /qazbot_lio/odom_level; /qazbot_native/odom with localization odom_source:=native_imu).
set -e
QAZBOT_NAV_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
export QAZBOT_SMAC_PREFIX="${QAZBOT_SMAC_PREFIX:-$QAZBOT_NAV_DIR/../../../candidates/nav2_smac_bounds/install/nav2_smac_planner}"
source "$QAZBOT_NAV_DIR/env.sh"
exec ros2 launch "$QAZBOT_NAV_DIR/nav_full.launch.py" robot_radius:=${ROBOT_RADIUS:?set ROBOT_RADIUS (m): body envelope, no default} \
  scan_topic:=/qazbot_lio/scan odom_topic:=${QAZBOT_ODOM_TOPIC:-/qazbot_lio/odom_level} "$@"
