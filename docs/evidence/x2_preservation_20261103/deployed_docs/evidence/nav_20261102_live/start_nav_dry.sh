#!/usr/bin/env bash
# 2026-11-02 live session: Nav2 nav_full + goal_coordinator + motion adapter in DRY-RUN (live=false: no MC publisher,
# no register service, no SetMcInputSource). Same params as installed 30.10 config (start_nav.sh): max_linear .3,
# max_angular .6, xy_tol .35, yaw_tol .17, stateful checker, odom /qazbot_native/odom, robot_radius .454, map D1.
set -u
EV=/home/agi/control_backend/docs/evidence/nav_20261102_live
NAV=/home/agi/control_backend/ws/src/navigation
MAP=/home/agi/control_backend/ws/src/maps/d1_qazbot_20261029/map.yaml
PROF=$EV/dds_nav_two_iface_log.xml
log(){ echo "[$(date +%T.%N)] $*" | tee -a $EV/start_nav_run.txt; }
PAT='goal_coordinator|x2_motion_adapter|controller_server|planner_server|bt_navigator|velocity_smoother|lifecycle_manager_navigation|nav_full.launch'
if pgrep -af "$PAT" | grep -v -E "pgrep|start_nav"; then log "ABORT: nav/motion component already running"; exit 3; fi
for p in $(awk '{print $1}' $EV/owned_pids.txt); do kill -0 $p 2>/dev/null || { log "ABORT: localization/trust PID $p not running"; exit 4; }; done
cd $NAV
export QAZBOT_SMAC_PREFIX=$NAV/../../../candidates/nav2_smac_bounds/install/nav2_smac_planner
set +u; source ./env.sh >/dev/null 2>&1; source ./aimdk_env.sh; set +e
export FASTRTPS_DEFAULT_PROFILES_FILE=$PROF ROS_DOMAIN_ID=0 ROS_LOCALHOST_ONLY=0
log "DRY profile $(sha256sum $PROF | cut -c1-16) nav_full=$(sha256sum nav_full.launch.py | cut -c1-16) nav_control=$(sha256sum nav_control.launch.py | cut -c1-16) gc=$(sha256sum goal_coordinator.py | cut -c1-16) adapter=$(sha256sum x2_motion_adapter.py | cut -c1-16)"
setsid nohup python3 $NAV/x2_motion_adapter.py --ros-args -r __ns:=/qazbot_d1_map -p live:=false -p priority:=30 > $EV/adapter.log 2>&1 < /dev/null &
echo $! > $EV/adapter.pid
for i in $(seq 1 80); do grep -q 'x2_motion_adapter started: live=False' $EV/adapter.log && break; grep -q Traceback $EV/adapter.log && break; sleep 0.5; done
grep -q 'x2_motion_adapter started: live=False' $EV/adapter.log || { log "adapter FAILED"; exit 5; }
log "adapter started DRY (pid $(cat $EV/adapter.pid))"; sleep 2
setsid nohup ros2 launch $NAV/nav_full.launch.py robot_radius:=0.454 scan_topic:=/qazbot_lio/scan odom_topic:=/qazbot_native/odom \
  map_yaml:=$MAP max_linear:=0.3 max_angular:=0.6 xy_tol:=0.35 yaw_tol:=0.17 goal_checker_stateful:=true \
  allow_dry_run_driver:=true > $EV/nav_full.log 2>&1 < /dev/null &
echo $! > $EV/nav_full.pid
for i in $(seq 1 240); do grep -qE 'Managed nodes are active|Timed out waiting for transform' $EV/nav_full.log && break; grep -qE 'exited with code|Traceback|process has died' $EV/nav_full.log && { log "nav_full FAILED"; exit 6; }; sleep 0.5; done
log "nav_full: managed_active=$(grep -c 'Managed nodes are active' $EV/nav_full.log) tf_wait=$(grep -c 'Timed out waiting for transform' $EV/nav_full.log)"
sleep 3
ps -eo pid,ppid,sid,etimes,args | awk -v a=$(cat $EV/adapter.pid) -v b=$(cat $EV/nav_full.pid) '$3==a||$3==b' | cut -c1-300 > $EV/owned_nav_pids.txt
log "owned nav/motion: $(awk '{print $1}' $EV/owned_nav_pids.txt | tr '\n' ' ')"
