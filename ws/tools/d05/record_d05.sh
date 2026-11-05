#!/usr/bin/env bash
# P-D05 recorder (single ros2 bag process, no clouds, no motion). Topics from HW_PROTOCOLS_20261103.md:
#   MC velocity (all sources), MC state, leg odometry, FAST-LIO odometry, native odom, adapter status, adapter command input.
# Usage: record_d05.sh RUN_TAG [MAX_SECONDS=45]      -> starts in background, prints "<bag dir> pid <pid>"
#        stop early:  kill -INT "$(cat "$OUT_ROOT/record_d05.pid")"
# Env overrides: OUT_ROOT (bags dir), ROS_DOMAIN_ID (default 0 = robot), DDS_PROFILE (default = nav two-iface profile;
#   "none" = unset it and use ROS_LOCALHOST_ONLY=1, for isolated tests), NS (default /qazbot_d1_map).
# Written for operators/CLAUDE: it writes nothing but the bag, its log, the generated QoS override and the pid file.
set -u
TAG="${1:?usage: record_d05.sh RUN_TAG [MAX_SECONDS]}"
MAX_S="${2:-45}"
case "$MAX_S" in ''|*[!0-9]*) echo "MAX_SECONDS must be an integer"; exit 2;; esac
[ "$MAX_S" -le 120 ] || { echo "MAX_SECONDS > 120 refused"; exit 2; }
NS="${NS:-/qazbot_d1_map}"
OUT_ROOT="${OUT_ROOT:-/home/agi/control_backend/docs/evidence/d05_20261103}"
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
out="$OUT_ROOT/d05_${TAG}_$(date +%H%M%S)"
qos="$out.qos.yaml"
# best_effort + deep history on every high-rate vendor/odometry topic: compatible with reliable and best_effort
# publishers, and needed on the MC velocity topic where rc and sdk sources publish with different QoS.
{
  for t in /aima/mc/locomotion/velocity /aima/mc/common/state /aima/mc/leg_odometry /qazbot_lio/odometry /qazbot_native/odom; do
    printf '%s:\n  reliability: best_effort\n  history: keep_last\n  depth: 50\n' "$t"
  done
} > "$qos"
setsid nohup timeout -s INT "$MAX_S" nice -n 10 ros2 bag record -s mcap --include-hidden-topics \
  --qos-profile-overrides-path "$qos" -o "$out" \
  /aima/mc/locomotion/velocity /aima/mc/common/state /aima/mc/leg_odometry /qazbot_lio/odometry /qazbot_native/odom \
  "$NS/motion_adapter/status" "$NS/cmd_vel_authorized" \
  > "$out.log" 2>&1 < /dev/null &
echo $! > "$OUT_ROOT/record_d05.pid"
echo "$out pid $(cat "$OUT_ROOT/record_d05.pid") (hard stop ${MAX_S}s)"
