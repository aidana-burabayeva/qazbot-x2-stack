#!/bin/bash
# Exact-clearance regression for the plan-only Nav2 stack, on a PC in an isolated ROS domain.
# Usage: run_variant.sh true|false OUT.json   (true = cell_margin fix, false = baseline)
# The start TF (-4.375,-4.875) is a free cell of first_map_nav2_v2 with 1.56 m clearance (>= R + res/sqrt2) for the NavigateToPose check.
set -e
HERE="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
# control_backend keeps navigation/ and maps/ under ws/src/ (both hold what this test needs)
DEPLOY="$(cd -- "$HERE/../../ws/src" && pwd)"
LOGS="$(mktemp -d)"
source /opt/ros/humble/setup.bash
export ROS_DOMAIN_ID=77 ROS_LOCALHOST_ONLY=1; unset FASTRTPS_DEFAULT_PROFILES_FILE
NS=/qazbot_d1_map; TFR="-r /tf:=$NS/tf -r /tf_static:=$NS/tf_static"
ros2 run nav2_map_server map_server --ros-args -r __ns:=$NS $TFR -p yaml_filename:=$DEPLOY/maps/first_map_nav2_v2/map.yaml -p topic_name:=map -p frame_id:=map >$LOGS/map.log 2>&1 &
ros2 run nav2_lifecycle_manager lifecycle_manager --ros-args -r __ns:=$NS -r __node:=lifecycle_manager_localization -p autostart:=true -p "node_names:=['map_server']" >$LOGS/lm.log 2>&1 &
ros2 run tf2_ros static_transform_publisher --x -4.375 --y -4.875 --frame-id map --child-frame-id base_footprint --ros-args -r /tf_static:=$NS/tf_static >$LOGS/tf.log 2>&1 &
sleep 2
ros2 launch $DEPLOY/navigation/nav_plan.launch.py robot_radius:=0.65 map_yaml:=$DEPLOY/maps/first_map_nav2_v2/map.yaml cell_margin:=$1 >$LOGS/plan.log 2>&1 &
sleep 12
timeout 400 python3 $HERE/harness.py $DEPLOY/maps/first_map_nav2_v2/map.yaml 0.65 "$2" 2>&1 | tail -3
pkill -INT -f "nav_plan.launch.py" || true; sleep 3; pkill -f "map_server|lifecycle_manager|static_transform_publisher|planner_server|bt_navigator" || true
echo "logs: $LOGS"
