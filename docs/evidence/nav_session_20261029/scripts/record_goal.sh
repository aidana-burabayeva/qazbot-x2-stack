#!/bin/bash
# Lightweight goal recording: all odometry sources, IMU, full TF, AMCL/Trust, Nav2 (+local costmap), velocity chain.
# No PointCloud2 (cloud_registered, I1 scan_obstacles/scan_clearing, particle_cloud), no camera, no global costmap.
# P9 2026-10-28: storage mcap (sqlite3 'database is locked' crash in goal_bag_201325), aimdk_env sourced for aimdk_msgs types.
cd "$(dirname "$0")"
source /opt/ros/humble/setup.bash
set +u; source /home/agi/control_backend/ws/src/navigation/aimdk_env.sh >/dev/null 2>&1
export FASTRTPS_DEFAULT_PROFILES_FILE=$PWD/dds_nav_two_iface_log.xml ROS_DOMAIN_ID=0 ROS_LOCALHOST_ONLY=0
out=goal_bag_mcap_$(date +%H%M%S)
re='^/qazbot_[a-z0-9_]+/.*|^/cmd_vel.*|^/aima/mc/(leg_odometry|locomotion/velocity|common/state)$|^/mc/base/state/.*|^/goal_pose.*'
x='.*cloud_registered.*|.*cloud_effected.*|^/qazbot_lio/map$|^/qazbot_lio/path$|.*lidar_pointcloud.*|.*particle_cloud.*|.*scan_obstacles.*|.*scan_clearing.*|.*global_costmap/costmap.*|.*_raw.*'
nohup ros2 bag record -s mcap --regex "$re" --exclude "$x" --include-hidden-topics -o "$out" > "$out.log" 2>&1 &
echo $! > record.pid; echo "$out pid $(cat record.pid)"
