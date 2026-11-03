#!/usr/bin/env bash
# OFFICE vs CORRIDOR bounded recorder: light topics only, mcap, hard stop after 1800 s (SIGINT). Stop early: kill -INT $(cat record_oc.pid)
cd "$(dirname "$0")"
set +u; source /home/agi/control_backend/ws/src/navigation/env.sh >/dev/null 2>&1; source /home/agi/control_backend/ws/src/navigation/aimdk_env.sh >/dev/null 2>&1
export FASTRTPS_DEFAULT_PROFILES_FILE=$PWD/dds_nav_two_iface_log.xml ROS_DOMAIN_ID=0 ROS_LOCALHOST_ONLY=0
out=oc_bag_$(date +%H%M%S)
setsid nohup timeout -s INT 1800 ros2 bag record -s mcap -o $out \
  /qazbot_lio/scan /qazbot_lio/odometry /qazbot_native/odom /qazbot_native/status \
  /qazbot_d1_map/tf /qazbot_d1_map/tf_static /qazbot_d1_map/amcl_pose /qazbot_d1_map/initialpose \
  /qazbot_d1_map/localization_trust/status /qazbot_d1_map/motion_adapter/status \
  /aima/mc/common/state /aima/mc/leg_odometry /aima/mc/locomotion/velocity > $out.log 2>&1 < /dev/null &
echo $! > record_oc.pid; echo "$out pid $(cat record_oc.pid)"
