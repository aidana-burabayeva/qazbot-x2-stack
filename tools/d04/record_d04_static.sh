#!/usr/bin/env bash
# D-04 static-robot recorder (single process): raw+registered cloud, lidar IMU, TF, level_bridge outputs, local costmap_raw. Hard stop 300 s. No motion.
HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
for f in dds_nav_two_iface_log.xml qos_mc.yaml; do
  [ -f "$HERE/$f" ] || { echo "ERROR: $HERE/$f missing (ships next to this script)" >&2; exit 1; }
done
cd "$HERE"   # bag + log + pid are written next to the script; use OUT_DIR=... to redirect
[ -n "${OUT_DIR:-}" ] && { mkdir -p "$OUT_DIR" && cd "$OUT_DIR" || exit 1; }
set +u; REPO="$(cd "$HERE/../.." && pwd)"
for f in env.sh aimdk_env.sh; do [ -f "$REPO/ws/src/navigation/$f" ] || { echo "ERROR: $REPO/ws/src/navigation/$f missing" >&2; exit 1; }; done
source "$REPO/ws/src/navigation/env.sh" >/dev/null 2>&1; source "$REPO/ws/src/navigation/aimdk_env.sh" >/dev/null 2>&1
export FASTRTPS_DEFAULT_PROFILES_FILE=$HERE/dds_nav_two_iface_log.xml ROS_DOMAIN_ID=0 ROS_LOCALHOST_ONLY=0
N=/qazbot_d1_map
out=d04_static_bag_$(date +%H%M%S)
setsid nohup timeout -s INT 300 nice -n 10 ros2 bag record -s mcap --include-hidden-topics --qos-profile-overrides-path $HERE/qos_mc.yaml -o $out \
  /aima/hal/sensor/lidar_chest_front/lidar_pointcloud /aima/hal/sensor/lidar_chest_front/imu \
  /qazbot_lio/cloud_registered /qazbot_lio/cloud_registered_body /qazbot_lio/odometry /qazbot_lio/tf \
  /qazbot_lio/scan /qazbot_lio/scan_obstacles /qazbot_lio/scan_clearing /qazbot_native/odom \
  $N/tf $N/tf_static $N/amcl_pose $N/localization_trust/status \
  $N/nav2/local_costmap/costmap_raw $N/nav2/local_costmap/costmap_raw_updates $N/nav2/lookahead_collision_arc \
  > $out.log 2>&1 < /dev/null &
echo $! > record_d04.pid; echo "$out pid $(cat record_d04.pid)"
