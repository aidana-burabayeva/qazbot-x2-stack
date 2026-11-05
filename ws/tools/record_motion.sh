#!/usr/bin/env bash
# Passive recording for a physical motion step. Subscribes only; publishes nothing; no service calls.
# Usage: tools/record_motion.sh NAME [MAX_SECONDS=900]     (stop: tools/stop_motion.sh NAME)
# Env: WITH_JOINTS=1 adds /aima/hal/joint/leg/state to the bag (~500 Hz, large); CLOUDS=1 adds point clouds (large).
# Outputs: logs/bags/NAME/ (bag), logs/bags/NAME.joints.npz (joint_record.py, 50 Hz), logs/rec_NAME_{bag,joints}.{log,pid}
set -euo pipefail
NAME="${1:?usage: record_motion.sh NAME [MAX_SECONDS]}"; MAXS="${2:-900}"
[[ "$NAME" =~ ^[A-Za-z0-9_.-]+$ ]] || { echo "bad NAME" >&2; exit 2; }
BK="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"; NAV="$BK/ws/src/navigation"
BAG="$BK/logs/bags/$NAME"
[ -e "$BAG" ] && { echo "exists: $BAG" >&2; exit 2; }
set +u; source "$NAV/env.sh"; source "$NAV/aimdk_env.sh"; set -u
export X2_URDF_DIR="${X2_URDF_DIR:-/agibot/software/aimdk/extra/mc-rl/x2_urdf/X2_URDF-v1.3.0}"
mkdir -p "$BK/logs/bags"
FREE_MB=$(df -Pm "$BK/logs" | awk 'NR==2{print $4}')
[ "$FREE_MB" -gt 5000 ] || { echo "disk free ${FREE_MB} MB < 5000" >&2; exit 3; }

WANT=(/qazbot_lio/odometry /qazbot_lio/odom_level /qazbot_lio/scan
 /qazbot_d1_map/amcl_pose /qazbot_d1_map/particle_cloud /qazbot_d1_map/tf /qazbot_d1_map/tf_static
 /qazbot_d1_map/localization_trust/status /qazbot_d1_map/localization_trust/trusted
 /qazbot_d1_map/motion_adapter/status /qazbot_d1_map/motion_adapter/dry_run_cmd
 /qazbot_d1_map/goal_coordinator/status
 /qazbot_d1_map/nav2/cmd_vel_nav /qazbot_d1_map/cmd_vel /qazbot_d1_map/cmd_vel_authorized
 /qazbot_d1_map/nav2/plan
 /aima/mc/common/state /aima/mc/leg_odometry /aima/mc/locomotion/velocity)
[ "${WITH_JOINTS:-0}" = 1 ] && WANT+=(/aima/hal/joint/leg/state)
[ "${CLOUDS:-0}" = 1 ] && WANT+=(/qazbot_lio/cloud_registered_body)

LIST="$(timeout 30 ros2 topic list)"          # the single discovery call
TOP=()
for t in "${WANT[@]}"; do
  if grep -qxF "$t" <<<"$LIST"; then TOP+=("$t"); else echo "WARN missing topic, skipped: $t" >&2; fi
done
[ "${#TOP[@]}" -gt 0 ] || { echo "no topics" >&2; exit 4; }

python3 "$NAV/detach.py" "rec_${NAME}_joints" python3 "$BK/tests/footprint/joint_record.py" "$BK/logs/bags/$NAME.joints.npz" "$MAXS" 50
python3 "$NAV/detach.py" "rec_${NAME}_bag" ros2 bag record -o "$BAG" --max-cache-size 100000000 "${TOP[@]}"
echo "recording $NAME: ${#TOP[@]} topics; stop with tools/stop_motion.sh $NAME"
