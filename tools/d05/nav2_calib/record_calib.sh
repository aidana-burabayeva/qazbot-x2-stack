#!/usr/bin/env bash
# P-D05 Nav2-goal calibration recorder (single ros2 bag process, no clouds, no TF, no motion).
# Commanded chain: RPP output (nav2/cmd_vel_nav) -> smoother output (cmd_vel) -> GC output (cmd_vel_authorized)
#   -> MC input (/aima/mc/locomotion/velocity, all sources). Actual: FAST-LIO, leg, native odometry. Plus speed limit,
#   GC/adapter/trust status, plan and action status (goal timing).
# Usage: record_calib.sh RUN_TAG [MAX_SECONDS=420]  -> starts in background, prints "<bag dir> pid <pid>"
#        stop early: kill -INT <pid>   (timeout forwards SIGINT to ros2 bag; bag is closed cleanly)
# Env: OUT_ROOT, ROS_DOMAIN_ID (default 0), DDS_PROFILE (default nav two-iface profile; "none" = localhost-only test),
#      NS (default /qazbot_d1_map).
set -u
TAG="${1:?usage: record_calib.sh RUN_TAG [MAX_SECONDS]}"
MAX_S="${2:-420}"
case "$MAX_S" in ''|*[!0-9]*) echo "MAX_SECONDS must be an integer"; exit 2;; esac
[ "$MAX_S" -le 900 ] || { echo "MAX_SECONDS > 900 refused"; exit 2; }
case "$TAG" in *[!A-Za-z0-9_.-]*|'') echo "RUN_TAG: only [A-Za-z0-9_.-]"; exit 2;; esac
NS="${NS:-/qazbot_d1_map}"
OUT_ROOT="${OUT_ROOT:-/home/agi/control_backend/docs/evidence/d05_nav2calib}"
DDS_PROFILE="${DDS_PROFILE:-/home/agi/control_backend/docs/evidence/nav_20261102_live/dds_nav_two_iface_log.xml}"
NAV=/home/agi/control_backend/ws/src/navigation
set +u; source "$NAV/env.sh" >/dev/null 2>&1; source "$NAV/aimdk_env.sh" >/dev/null 2>&1; set -u
if [ "$DDS_PROFILE" = none ]; then
  unset FASTRTPS_DEFAULT_PROFILES_FILE; export ROS_LOCALHOST_ONLY=1
else
  [ -f "$DDS_PROFILE" ] || { echo "DDS profile missing: $DDS_PROFILE"; exit 2; }
  export FASTRTPS_DEFAULT_PROFILES_FILE="$DDS_PROFILE" ROS_LOCALHOST_ONLY=0
fi
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-0}"
mkdir -p "$OUT_ROOT" || exit 2
out="$OUT_ROOT/calib_${TAG}_$(date +%H%M%S)"
qos="$out.qos.yaml"
{
  for t in /aima/mc/locomotion/velocity /aima/mc/common/state /aima/mc/leg_odometry /qazbot_lio/odometry /qazbot_native/odom; do
    printf '%s:\n  reliability: best_effort\n  history: keep_last\n  depth: 50\n' "$t"
  done
} > "$qos"
setsid nohup timeout -s INT "$MAX_S" nice -n 10 ros2 bag record -s mcap --include-hidden-topics \
  --qos-profile-overrides-path "$qos" -o "$out" \
  /aima/mc/locomotion/velocity /aima/mc/common/state /aima/mc/leg_odometry /qazbot_lio/odometry /qazbot_native/odom \
  "$NS/nav2/cmd_vel_nav" "$NS/cmd_vel" "$NS/cmd_vel_authorized" "$NS/nav2/speed_limit" \
  "$NS/goal_coordinator/status" "$NS/motion_adapter/status" "$NS/localization_trust/status" \
  "$NS/nav2/plan" "$NS/goal_pose" "$NS/navigate_to_pose/_action/status" "$NS/nav2/navigate_to_pose/_action/status" \
  > "$out.log" 2>&1 < /dev/null &
pid=$!
echo "$out pid $pid (hard stop ${MAX_S}s)"
