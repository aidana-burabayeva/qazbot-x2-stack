#!/bin/bash
# Closed-loop dry test of Nav2 Controller (RPP) + velocity_smoother against sim_robot.py in a PRIVATE namespace.
# No MC, no vendor topics. 6 ROS nodes, started 0.6 s apart. Usage: run_dry.sh [robot_radius=0.454]
# no set -u: ROS setup scripts use unset variables
HERE="$(cd "$(dirname "$0")" && pwd)"
NAV="$HERE/../../ws/src/navigation"
NS=qazbot_ctl_test
R="${1:-0.454}"
source "$NAV/env.sh" >/dev/null 2>&1
# Isolated model domain only (same as tests/nav_e2e/run_model.sh): never the hardware DDS domain.
export ROS_DOMAIN_ID=87 ROS_LOCALHOST_ONLY=1
export FASTRTPS_DEFAULT_PROFILES_FILE="$HERE/../nav_e2e/dds_domain87_unicast.xml"
set +e   # env.sh leaks set -e: without this the script dies when the harness returns non-zero
cd "$NAV"
python3 detach.py ctl_sim python3 "$HERE/sim_robot.py" $NS >/dev/null
sleep 1.5
python3 detach.py ctl_nav ros2 launch ./nav_control.launch.py ns:=$NS robot_radius:=$R scan_topic:=/$NS/scan odom_topic:=odom >/dev/null
sleep 12
( sleep 30; { uptime; ps -eo pcpu,rss,args --sort=-pcpu | grep -E "controller_server|velocity_smoother|sim_robot" | grep -v grep | cut -c1-90; } > "$HERE/cpu_sample.txt" ) &
python3 "$HERE/harness.py" $NS "${MAX_LIN:-0.3}" "${MAX_ANG:-0.5}"
rc=$?
for n in ctl_nav ctl_sim; do
  pid="$(cat "$HERE/../../logs/$n.pid")"
  kill -INT "$pid" 2>/dev/null
  for i in $(seq 1 30); do kill -0 "$pid" 2>/dev/null || break; sleep 0.5; done   # launch needs ~5 s to stop its nodes
done
exit $rc
