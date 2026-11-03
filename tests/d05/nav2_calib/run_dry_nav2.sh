#!/usr/bin/env bash
# Dry integration: nav2_calib.py vs real controller_server + velocity_smoother (Humble 1.1.18, runtime/qazbot_nav_deps)
# with the LIVE parameter file, isolated ROS_DOMAIN_ID=81, localhost only, vendor DDS profile unset. Robot never moves.
set -u
HERE="$(cd "$(dirname "$0")" && pwd)"
PARAMS="${1:-/home/agi/.ros/log/qazbot_ctl_t9htg14z.yaml}"
OUT="${OUT:-$(mktemp -d /tmp/d05_nav2calib_dry.XXXX)}"
set +u; source /home/agi/control_backend/ws/src/navigation/env.sh >/dev/null 2>&1; set -u
unset FASTRTPS_DEFAULT_PROFILES_FILE RMW_FASTRTPS_CONFIG_FILE
export ROS_DOMAIN_ID=81 ROS_LOCALHOST_ONLY=1 ROS_LOG_DIR="$OUT/roslog"
[ "$ROS_DOMAIN_ID" != 0 ] || exit 9
mkdir -p "$OUT"; cp "$PARAMS" "$OUT/params.yaml"
echo "OUT=$OUT"
exec nice -n 10 python3 "$HERE/dry_nav2.py" "$OUT/params.yaml" "$OUT"
