#!/usr/bin/env bash
# Healthy when every node of the stack is on the graph. Exit 0 = healthy.
set -e
source "/opt/ros/${ROS_DISTRO}/setup.bash"; source /opt/x2twin/overlay/setup.bash
nodes="$(timeout 12 ros2 node list --no-daemon 2>/dev/null)"
if [ "${TWIN_LOCALIZATION:-amcl}" = truth ]; then loc="/qazbot_d1_map/truth_localizer"; else loc="/qazbot_d1_map/amcl /qazbot_d1_map/localization_trust"; fi
for n in $loc /qazbot_d1_map/motion_adapter \
         /qazbot_d1_map/goal_coordinator /qazbot_d1_map/nav2/bt_navigator /qazbot_d1_map/nav2/controller_server; do
  grep -qx "$n" <<<"$nodes" || { echo "missing node $n" >&2; exit 1; }
done
grep -qxE '/hal_plant|/kinematic_plant' <<<"$nodes" || { echo "missing plant node" >&2; exit 1; }
