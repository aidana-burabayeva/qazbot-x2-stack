#!/usr/bin/env bash
# Stop ONLY owned P5b motion/nav processes, motion FIRST: coordinator stop (gate closed, cancel) -> adapter SIGINT
# (5 zeros if ACTIVE, then exit) -> nav_full launch SIGINT. Localization/Trust untouched. Never pkill.
EV=/home/agi/control_backend/docs/evidence/nav_20261030_1623
NAV=/home/agi/control_backend/ws/src/navigation
OWNED=$(awk '{print $1}' $EV/owned_nav_pids.txt 2>/dev/null)
[ -f $EV/adapter.pid ] && OWNED="$OWNED $(cat $EV/adapter.pid)"
[ -f $EV/nav_full.pid ] && OWNED="$OWNED $(cat $EV/nav_full.pid)"
( cd $NAV; set +u; source ./env.sh >/dev/null 2>&1; set +e
  export FASTRTPS_DEFAULT_PROFILES_FILE=$EV/dds_nav_two_iface_log.xml ROS_DOMAIN_ID=0 ROS_LOCALHOST_ONLY=0
  timeout 10 ros2 service call /qazbot_d1_map/goal_coordinator/stop std_srvs/srv/Trigger '{}' 2>&1 | tail -1
  timeout 10 ros2 service call /qazbot_d1_map/motion_adapter/stop std_srvs/srv/Trigger '{}' 2>&1 | tail -1 )
kill -INT $(cat $EV/adapter.pid) 2>/dev/null; sleep 2
kill -INT $(cat $EV/nav_full.pid) 2>/dev/null
for i in $(seq 1 60); do left=""; for p in $OWNED; do kill -0 $p 2>/dev/null && left="$left $p"; done; [ -z "$left" ] && break; sleep 0.5; done
[ -n "$left" ] && { echo "INT leftovers:$left -> TERM"; kill -TERM $left 2>/dev/null; sleep 5; }
left=""; for p in $OWNED; do kill -0 $p 2>/dev/null && left="$left $p"; done
[ -n "$left" ] && { echo "TERM ignored:$left -> KILL"; kill -KILL $left; sleep 2; }
left=""; for p in $OWNED; do kill -0 $p 2>/dev/null && left="$left $p"; done
echo "STOP_NAV done; owned nav/motion leftovers:[${left}]"
