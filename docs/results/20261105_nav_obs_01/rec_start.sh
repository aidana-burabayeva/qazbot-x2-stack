#!/usr/bin/env bash
source /home/agi/control_backend/ws/src/navigation/env.sh >/dev/null 2>&1
source /home/agi/control_backend/ws/src/navigation/aimdk_env.sh >/dev/null 2>&1
B=/home/agi/control_backend/logs/bag_e2e02_$(date +%Y%m%d_%H%M)
exec ros2 bag record --include-hidden-topics --qos-profile-overrides-path /tmp/claude-1001/-home-agi/aab0ddb0-e8ed-48c0-bb95-2e67f779791d/scratchpad/qos_override.yaml -o $B \
 /qazbot_d1_map/nav2/plan /qazbot_d1_map/nav2/received_global_plan /qazbot_d1_map/nav2/unsmoothed_plan \
 /qazbot_d1_map/nav2/cmd_vel_nav /qazbot_d1_map/cmd_vel /qazbot_d1_map/cmd_vel_authorized \
 /qazbot_d1_map/navigate_to_pose/_action/feedback /qazbot_d1_map/navigate_to_pose/_action/status \
 /qazbot_d1_map/nav2/navigate_to_pose/_action/feedback /qazbot_d1_map/nav2/navigate_to_pose/_action/status \
 /qazbot_d1_map/nav2/follow_path/_action/feedback /qazbot_d1_map/nav2/follow_path/_action/status \
 /qazbot_d1_map/nav2/compute_path_to_pose/_action/status /qazbot_d1_map/nav2/behavior_tree_log \
 /qazbot_d1_map/motion_adapter/status /qazbot_d1_map/goal_coordinator/status /qazbot_d1_map/localization_trust/status \
 /aima/mc/locomotion/velocity /aima/mc/common/state /aima/mc/leg_odometry \
 /qazbot_lio/odom_level /qazbot_lio/odometry /qazbot_d1_map/amcl_pose /qazbot_d1_map/tf /qazbot_d1_map/tf_static \
 /qazbot_d1_map/nav2/local_costmap/costmap /qazbot_d1_map/nav2/local_costmap/costmap_updates \
 /qazbot_d1_map/nav2/global_costmap/costmap_updates /qazbot_d1_map/nav2/speed_limit /rosout
