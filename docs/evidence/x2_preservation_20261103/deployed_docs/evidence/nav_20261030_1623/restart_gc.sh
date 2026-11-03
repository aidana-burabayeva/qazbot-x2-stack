#!/usr/bin/env bash
# P10 2026-10-28 — restart procedure for goal_coordinator (GC). PREPARED, NOT RUN by the agent.
#
# WHY NOT GC-ONLY: nav_full.launch.py (lines 152-153) registers OnProcessExit -> Shutdown for EVERY node it starts,
# including goal_coordinator (python3-6, ExecuteProcess, PID 464838). Stopping GC therefore shuts down the whole Nav2
# launch (controller/planner/smoother/bt_navigator/lifecycle). Killing the launch parent first (to orphan Nav2 and keep
# it alive) would break the children's stdout pipes (output='screen') -> not acceptable on a live robot.
# => The smallest safe unit is "nav_full only": the motion adapter (PID in adapter.pid, separate process, MC source
#    sdk_qazbot_nav2_1028b already registered) and localization/trust/recorder are NOT touched, so no MC DEL/ADD needed.
#
# P10 GC change is DIAGNOSTIC ONLY (trip reason logged; no misleading "paused" after a trip). Behaviour is identical,
# so this restart is NOT required to clear the current latch — see "Clear latch without restart" at the bottom.
#
# Usage: CONFIRM=1 ./restart_gc.sh       (refuses without CONFIRM=1, refuses if GC is busy)
set -u
EV=/home/agi/control_backend/docs/evidence/nav_20261030_1623
NAV=/home/agi/control_backend/ws/src/navigation
MAP=/home/agi/control_backend/ws/src/maps/d1_qazbot_20261029/map.yaml
PROF=$EV/dds_nav_two_iface_log.xml
log(){ echo "[$(date +%T.%N)] restart_gc: $*" | tee -a $EV/start_nav_run.txt; }
[ "${CONFIRM:-0}" = 1 ] || { echo "Refusing: set CONFIRM=1 (restarts ALL of nav_full incl. GC; adapter/localization untouched)"; exit 2; }
cd $NAV
# P12 fix: SMAC prefix must be exported BEFORE env.sh (same order as start_nav.sh), else 'Qualified Smac prefix is not selected'
export QAZBOT_SMAC_PREFIX=$NAV/../../../candidates/nav2_smac_bounds/install/nav2_smac_planner
set +u; source ./env.sh >/dev/null 2>&1; source ./aimdk_env.sh; set +e
export FASTRTPS_DEFAULT_PROFILES_FILE=$PROF ROS_DOMAIN_ID=0 ROS_LOCALHOST_ONLY=0
LAUNCH=$(cat $EV/nav_full.pid)
if ! kill -0 $LAUNCH 2>/dev/null && ! ps -eo args= | grep -qE '^/[^ ]+/(controller_server|planner_server|bt_navigator|velocity_smoother) |^python3 [^ ]*goal_coordinator\.py'; then
  log "nav_full $LAUNCH already down (no Nav2/GC procs) -> skip stop phase"; SKIP_STOP=1; else SKIP_STOP=0; fi
if [ $SKIP_STOP = 0 ]; then
# 0) preflight: GC must not be busy (no active goal)
ST=$(timeout 12 python3 /home/agi/control_backend/docs/evidence/nav_20261030_1623/gc_status.py 2>/dev/null | head -1)
echo "$ST" | grep -q '"busy": false' || { log "ABORT: GC busy or status unavailable: ${ST:0:200}"; exit 3; }
python3 -m py_compile $NAV/goal_coordinator.py || { log "ABORT: goal_coordinator.py does not compile"; exit 4; }
LAUNCH=$(cat $EV/nav_full.pid)
OLD=$(pgrep -P $LAUNCH | tr '\n' ' ')
log "stopping nav_full launch $LAUNCH (children: $OLD); adapter $(cat $EV/adapter.pid) kept"
# 1) close GC gate (zero + latch) then stop the launch (SIGINT -> orderly Nav2 shutdown)
timeout 10 ros2 service call /qazbot_d1_map/goal_coordinator/stop std_srvs/srv/Trigger '{}' 2>&1 | tail -1
kill -INT $LAUNCH
for i in $(seq 1 60); do left=""; for p in $LAUNCH $OLD; do kill -0 $p 2>/dev/null && left="$left $p"; done; [ -z "$left" ] && break; sleep 0.5; done
[ -n "$left" ] && { log "INT leftovers:$left -> TERM"; kill -TERM $left 2>/dev/null; sleep 5; }
left=""; for p in $LAUNCH $OLD; do kill -0 $p 2>/dev/null && left="$left $p"; done
[ -n "$left" ] && { log "ABORT: still alive after TERM:$left (not KILLing automatically)"; exit 5; }
fi
mv $EV/nav_full.log $EV/nav_full_pre_p10_$(date +%H%M%S).log
# 2) re-launch nav_full with EXACTLY the start_nav.sh arguments (same env/profile)
log "profile $(sha256sum $PROF | cut -c1-16) gc=$(sha256sum goal_coordinator.py | cut -c1-16)"
setsid nohup ros2 launch $NAV/nav_full.launch.py robot_radius:=0.454 scan_topic:=/qazbot_lio/scan odom_topic:=/qazbot_native/odom \
  map_yaml:=$MAP max_linear:=0.3 max_angular:=0.6 xy_tol:=0.35 yaw_tol:=0.17 goal_checker_stateful:=true \
  allow_dry_run_driver:=false > $EV/nav_full.log 2>&1 < /dev/null &
echo $! > $EV/nav_full.pid
for i in $(seq 1 240); do grep -q 'Managed nodes are active' $EV/nav_full.log && break; grep -qE 'exited with code|Traceback|process has died' $EV/nav_full.log && { log "nav_full FAILED"; exit 6; }; sleep 0.5; done
grep -q 'Managed nodes are active' $EV/nav_full.log || { log "nav_full not active after 120 s"; exit 6; }
sleep 2; NP=$(cat $EV/nav_full.pid); cp $EV/owned_nav_pids.txt $EV/owned_nav_pids_pre_p10.txt
ps -o pid=,ppid=,pgid=,etimes=,args= -p "$(echo $(cat $EV/adapter.pid) $NP $(pgrep -P $NP) | tr ' ' ',')" > $EV/owned_nav_pids.txt
log "nav_full restarted pid $(cat $EV/nav_full.pid); owned_nav_pids.txt updated"
timeout 12 python3 /home/agi/control_backend/docs/evidence/nav_20261030_1623/gc_status.py | head -1
# Fresh GC starts unlatched. Adapter is still latched (localization_untrusted) -> step 3 below still needed.
#
# ---- Clear latch WITHOUT restart (recommended now) ----
# 1. RViz "2D Pose Estimate" at the true pose; wait until localization_trust/status shows trusted:true, ready:true.
# 2. ros2 service call /qazbot_d1_map/goal_coordinator/resume std_srvs/srv/Trigger '{}'   (needs not busy, source_zero, trust good)
# 3. ros2 service call /qazbot_d1_map/motion_adapter/resume   std_srvs/srv/Trigger '{}'   (adapter also latched localization_untrusted)
# 4. Send a NEW RViz goal (no goal replay; GC re-arms the adapter itself via arm_motion).
