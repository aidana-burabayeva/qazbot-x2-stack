#!/usr/bin/env bash
# P5b 2026-10-28: Nav2 (nav_full: controller/smoother/planner/bt_navigator + goal_coordinator) + LIVE motion adapter
# + ONE MC SDK input-source ADD (registration). Localization+Trust (start.sh) must already run; NOT started here.
# No goal, no cmd_vel, no initialpose, no arm_motion. MC registration only adds an input source (priority 30 < rc 80);
# it does not change stance and publishes no velocity (adapter publishes to MC only while an armed goal is ACTIVE).
# Params = last robot-run config (v13 27.10, ~/.ros/log/qazbot_ctl_ewgwlyx7.yaml): max_linear 0.3, max_angular 0.6,
# xy_tol 0.20, yaw_tol 10deg, goal_checker_stateful true, odom /qazbot_native/odom, robot_radius 0.454, map D1 v3.
# ros2 launch / python called directly (NOT run_nav_full.sh / run_x2_driver.sh: they re-source env.sh -> dds_qazbot.xml).
set -u
EV=/home/agi/control_backend/docs/evidence/nav_20261029_1425
NAV=/home/agi/control_backend/ws/src/navigation
MAP=/home/agi/control_backend/ws/src/maps/first_map_nav2_v3_despeckled/map.yaml
PROF=$EV/dds_nav_two_iface_log.xml
SRC=${QAZBOT_SOURCE_NAME:-sdk_qazbot_nav2_1029a}
REGISTER=${REGISTER:-1}
log(){ echo "[$(date +%T.%N)] $*" | tee -a $EV/start_nav_run.txt; }
PAT='goal_coordinator|x2_motion_adapter|controller_server|planner_server|bt_navigator|velocity_smoother|lifecycle_manager_navigation|nav_full.launch'
if pgrep -af "$PAT" | grep -v -E "pgrep|start_nav.sh"; then log "ABORT: nav/motion component already running"; exit 3; fi
for p in $(awk '{print $1}' $EV/owned_pids.txt); do kill -0 $p 2>/dev/null || { log "ABORT: localization/trust PID $p not running"; exit 4; }; done
cd $NAV
export QAZBOT_SMAC_PREFIX=$NAV/../../../candidates/nav2_smac_bounds/install/nav2_smac_planner
set +u; source ./env.sh >/dev/null 2>&1; source ./aimdk_env.sh; set +e
export FASTRTPS_DEFAULT_PROFILES_FILE=$PROF ROS_DOMAIN_ID=0 ROS_LOCALHOST_ONLY=0
log "profile $(sha256sum $PROF | cut -c1-16) nav_full=$(sha256sum nav_full.launch.py | cut -c1-16) nav_control=$(sha256sum nav_control.launch.py | cut -c1-16) gc=$(sha256sum goal_coordinator.py | cut -c1-16) adapter=$(sha256sum x2_motion_adapter.py | cut -c1-16) source=$SRC"
# 1) live adapter (no registration yet; equivalent of run_x2_driver.sh -p live:=true ...)
setsid nohup python3 $NAV/x2_motion_adapter.py --ros-args -r __ns:=/qazbot_d1_map -p live:=true -p priority:=30 -p source_name:=$SRC > $EV/adapter.log 2>&1 < /dev/null &
echo $! > $EV/adapter.pid
for i in $(seq 1 80); do grep -q 'x2_motion_adapter started: live=True' $EV/adapter.log && break; grep -q Traceback $EV/adapter.log && break; sleep 0.5; done
grep -q 'x2_motion_adapter started: live=True' $EV/adapter.log || { log "adapter FAILED"; exit 5; }
log "adapter started live (pid $(cat $EV/adapter.pid))"; sleep 2
# 2) nav_full (+ goal_coordinator, allow_dry_run_driver=false)
setsid nohup ros2 launch $NAV/nav_full.launch.py robot_radius:=0.454 scan_topic:=/qazbot_lio/scan odom_topic:=/qazbot_native/odom \
  map_yaml:=$MAP max_linear:=0.3 max_angular:=0.6 xy_tol:=0.20 yaw_tol:=0.35 goal_checker_stateful:=true \
  allow_dry_run_driver:=false > $EV/nav_full.log 2>&1 < /dev/null &
echo $! > $EV/nav_full.pid
for i in $(seq 1 240); do grep -q 'Managed nodes are active' $EV/nav_full.log && break; grep -qE 'exited with code|Traceback|process has died' $EV/nav_full.log && { log "nav_full FAILED"; exit 6; }; sleep 0.5; done
log "nav_full: managed_active=$(grep -c 'Managed nodes are active' $EV/nav_full.log)"
[ "$(grep -c 'Managed nodes are active' $EV/nav_full.log)" -ge 1 ] || exit 6
sleep 3
# 3) ONE MC input-source ADD (no retry, no ENABLE). Only if adapter idle, not registered, MC source rc.
if [ "$REGISTER" = 1 ]; then
  ST=$(timeout 15 ros2 topic echo --once --field data /qazbot_d1_map/motion_adapter/status 2>/dev/null | head -1)
  log "adapter status before ADD: $ST"
  if echo "$ST" | grep -q '"registration": "not_requested"' && echo "$ST" | grep -q '"state": "idle"' && echo "$ST" | grep -q '"mc_source": "rc"'; then
    timeout 20 ros2 service call /qazbot_d1_map/motion_adapter/register std_srvs/srv/Trigger '{}' 2>&1 | tail -2 | tee -a $EV/start_nav_run.txt
    sleep 3; grep -E 'registered|registration failed|SDK response' $EV/adapter.log | tail -2 | tee -a $EV/start_nav_run.txt
  else log "ADD NOT sent: precondition failed"; fi
fi
ps -eo pid,ppid,sid,etimes,args | awk -v a=$(cat $EV/adapter.pid) -v b=$(cat $EV/nav_full.pid) '$3==a||$3==b' | cut -c1-300 > $EV/owned_nav_pids.txt
log "owned nav/motion: $(awk '{print $1}' $EV/owned_nav_pids.txt | tr '\n' ' ')"
