#!/usr/bin/env bash
set -u
HERE="$(cd "$(dirname "$0")" && pwd)"; OUT="$HERE/out_$(date +%H%M%S)"; mkdir -p "$OUT"
set +u; source /home/agi/control_backend/ws/src/navigation/env.sh >/dev/null 2>&1; source /home/agi/control_backend/ws/src/navigation/aimdk_env.sh; set -u
unset FASTRTPS_DEFAULT_PROFILES_FILE RMW_FASTRTPS_CONFIG_FILE
export ROS_DOMAIN_ID=84 ROS_LOCALHOST_ONLY=1 ROS_LOG_DIR="$OUT/roslog"
[ "$ROS_DOMAIN_ID" != 0 ] || exit 9
cp /home/agi/.ros/log/qazbot_ctl_t9htg14z.yaml "$OUT/params.yaml"; uptime > "$OUT/load.txt"
echo "OUT=$OUT"; ros2 pkg prefix nav2_controller; ros2 pkg prefix nav2_velocity_smoother
python3 "$HERE/harness.py" "$OUT/params.yaml" "$OUT" "$HERE/win.pkl" "${REPS:-3}"; rc=$?; uptime >> "$OUT/load.txt"; exit $rc
