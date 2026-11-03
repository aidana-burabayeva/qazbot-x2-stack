#!/usr/bin/env bash
# Nav goal recorder (single process): localization + full command chain + action status/feedback, no costmaps/clouds. Hard stop 1800 s.
cd "$(dirname "$0")"
set +u; source /home/agi/control_backend/ws/src/navigation/env.sh >/dev/null 2>&1; source /home/agi/control_backend/ws/src/navigation/aimdk_env.sh >/dev/null 2>&1
export FASTRTPS_DEFAULT_PROFILES_FILE=$PWD/dds_nav_two_iface_log.xml ROS_DOMAIN_ID=0 ROS_LOCALHOST_ONLY=0
N=/qazbot_d1_map
out=nav_bag_$(date +%H%M%S)
setsid nohup timeout -s INT 1800 ros2 bag record -s mcap --include-hidden-topics --qos-profile-overrides-path qos_mc.yaml -o $out \
  /qazbot_lio/scan /qazbot_lio/odometry /qazbot_native/odom /qazbot_native/status $N/tf $N/tf_static $N/amcl_pose $N/initialpose \
  $N/localization_trust/status $N/goal_pose $N/goal_coordinator/status $N/motion_adapter/status $N/motion_adapter/dry_run_cmd \
  $N/cmd_vel $N/cmd_vel_authorized $N/nav2/cmd_vel_nav $N/nav2/plan $N/nav2/lookahead_collision_arc \
  $N/navigate_to_pose/_action/status $N/navigate_to_pose/_action/feedback $N/nav2/navigate_to_pose/_action/status $N/nav2/navigate_to_pose/_action/feedback \
  /aima/mc/common/state /aima/mc/leg_odometry /aima/mc/locomotion/velocity /aima/mc/body_pose \
  > $out.log 2>&1 < /dev/null &
echo $! > record_nav.pid; echo "$out pid $(cat record_nav.pid)"
