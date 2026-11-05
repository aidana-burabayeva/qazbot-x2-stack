#!/usr/bin/env bash
# (container, no network) offline replay: localization + Nav2 + dry-run adapter (mode lab) fed by a recorded bag.
# The bag provides /aima/hal/sensor/lidar_chest_front/{lidar_pointcloud,imu}; nothing reaches a robot (no network).
set -euo pipefail
[ -e /bag ] || { echo "mount a bag: QAZBOT_BAG=/abs/path" >&2; exit 2; }
export QAZBOT_RUNTIME=/lab/deps QAZBOT_DDS_PROFILE=/lab/install/qazbot_bringup/share/qazbot_bringup/config/dds_loopback.xml
set +u; source /lab/install/qazbot_bringup/lib/qazbot_bringup/env.sh; set -u
ros2 launch qazbot_bringup x2_navigation.launch.xml mode:=lab start_adapter:=false "lio_cpus:=''" &
L=$!
trap 'kill -INT $L 2>/dev/null; wait $L' EXIT
ros2 bag play /bag --clock ${QAZBOT_REPLAY_ARGS:-}
