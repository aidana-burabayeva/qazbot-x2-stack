#!/usr/bin/env bash
# Run ONE short offline collision_monitor scenario on the replayed X2 bag.
# Isolation: ROS_DOMAIN_ID=82 + ROS_LOCALHOST_ONLY=1, all topics under /cm_test, no robot topics published.
#
# usage: run_scenario.sh NAME START_OFFSET_S DURATION_S [CM_OVERRIDE] [HARNESS_OVERRIDE] [USE_GATE] [PLAY_CLOUD]
#   CM_OVERRIDE / HARNESS_OVERRIDE: file names under config/overrides (default none.yaml)
#   USE_GATE: true|false (cloud/TF fault injection through nodes/cm_topic_gate.py)
#   PLAY_CLOUD: true|false (false = replay clock+TF only: "no data" scenario)
# env: CM_OVERLAY (install/setup.bash of a nav2_collision_monitor overlay, required)
#      CM_BAG (mcap, default = d04_static_bag_183015), CM_OUT (output dir root)
set -euo pipefail
HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
NAME="$1"; OFFSET="$2"; DUR="$3"
CM_OVR="${4:-none.yaml}"; H_OVR="${5:-none.yaml}"; USE_GATE="${6:-false}"; PLAY_CLOUD="${7:-true}"
BAG="${CM_BAG:-/home/agi/control_backend/docs/evidence/nav_20261102_live/d04_static_bag_183015/d04_static_bag_183015_0.mcap}"
OUT="${CM_OUT:?set CM_OUT}/${NAME}"
: "${CM_OVERLAY:?set CM_OVERLAY to the nav2_collision_monitor overlay setup.bash}"

export ROS_DOMAIN_ID=82 ROS_LOCALHOST_ONLY=1
set +u; source /opt/ros/humble/setup.bash; source "${CM_OVERLAY}"; set -u
[ "${ROS_DOMAIN_ID}" = "82" ] || { echo "refusing: ROS_DOMAIN_ID must be 82"; exit 2; }
mkdir -p "${OUT}"
cat > "${OUT}/run_override.yaml" <<EOF
cm_probe:
  ros__parameters:
    output_csv: "${OUT}/probe.csv"
EOF

RAW=/aima/hal/sensor/lidar_chest_front/lidar_pointcloud
if [ "${USE_GATE}" = "true" ]; then CLOUD_DST=/cm_test/lidar_pointcloud_raw; TF_DST=/cm_test/tf_raw;
else CLOUD_DST=/cm_test/lidar_pointcloud; TF_DST=/cm_test/tf; fi
TOPICS=(/qazbot_d1_map/tf); [ "${PLAY_CLOUD}" = "true" ] && TOPICS+=("${RAW}")

setsid ros2 launch "${HERE}/launch/collision_monitor_offline.launch.xml" \
  cm_override:="${HERE}/config/overrides/${CM_OVR}" harness_override:="${HERE}/config/overrides/${H_OVR}" \
  run_override:="${OUT}/run_override.yaml" use_gate:="${USE_GATE}" > "${OUT}/launch.log" 2>&1 &
LPID=$!
cleanup() { kill -INT -- -"${LPID}" 2>/dev/null || true; sleep 2; kill -KILL -- -"${LPID}" 2>/dev/null || true; }
trap cleanup EXIT

# lifecycle: configure + activate (no lifecycle_manager in this isolated test)
for i in $(seq 1 30); do
  ros2 lifecycle get /collision_monitor >/dev/null 2>&1 && break; sleep 0.5
done
ros2 lifecycle set /collision_monitor configure >> "${OUT}/lifecycle.log" 2>&1
ros2 lifecycle set /collision_monitor activate >> "${OUT}/lifecycle.log" 2>&1
ros2 lifecycle get /collision_monitor >> "${OUT}/lifecycle.log" 2>&1

timeout --signal=INT "$((DUR + 8))" ros2 bag play "${BAG}" --clock 100 --start-offset "${OFFSET}" \
  --disable-keyboard-controls --topics "${TOPICS[@]}" \
  --remap "${RAW}:=${CLOUD_DST}" "/qazbot_d1_map/tf:=${TF_DST}" > "${OUT}/play.log" 2>&1 || true
echo "{\"name\":\"${NAME}\",\"offset\":${OFFSET},\"duration\":${DUR},\"cm_override\":\"${CM_OVR}\",\"harness_override\":\"${H_OVR}\",\"use_gate\":\"${USE_GATE}\",\"play_cloud\":\"${PLAY_CLOUD}\"}" > "${OUT}/run.json"
sleep 1
