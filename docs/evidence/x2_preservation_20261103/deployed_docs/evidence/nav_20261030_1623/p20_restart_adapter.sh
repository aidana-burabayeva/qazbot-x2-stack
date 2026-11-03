#!/usr/bin/env bash
# P20: restart ONLY the motion adapter with the yaw-rate loop + ONE MC re-registration (operator-approved procedure).
# Preconditions (checked): MC STAND_DEFAULT, GC not busy, old adapter idle & unlatched. CONFIRM=1 required.
# Order: one SetMcInputSource DELETE of $SRC (old adapter has no delete-on-exit) -> SIGINT old adapter -> start new adapter
# (same command line as start_nav.sh step 1) -> one ~/register (ADD) -> verify registered/idle/ready.
set -u
[ "${CONFIRM:-}" = 1 ] || { echo "refusing: CONFIRM=1 required"; exit 1; }
EV=/home/agi/control_backend/docs/evidence/nav_20261030_1623
NAV=/home/agi/control_backend/ws/src/navigation
PROF=$EV/dds_nav_two_iface_log.xml
SRC=${SRC:-sdk_qazbot_nav2_1030a}
log(){ echo "[$(date +%T.%N)] $*" | tee -a $EV/p20_restart_run.txt; }
cd $NAV; set +u; source ./env.sh >/dev/null 2>&1; source ./aimdk_env.sh >/dev/null 2>&1
export FASTRTPS_DEFAULT_PROFILES_FILE=$PROF ROS_DOMAIN_ID=0 ROS_LOCALHOST_ONLY=0
MC=$(timeout 8 ros2 topic echo --once /aima/mc/common/state 2>/dev/null | grep -E 'action_desc' | head -1)
GC=$(timeout 8 ros2 topic echo --once --field data /qazbot_d1_map/goal_coordinator/status 2>/dev/null | head -1)
AD=$(timeout 8 ros2 topic echo --once --field data /qazbot_d1_map/motion_adapter/status 2>/dev/null | head -1)
log "pre: mc=[$MC] adapter=${AD:0:200} gc=${GC:0:120}"
echo "$MC" | grep -q STAND_DEFAULT || { log "ABORT: MC not STAND_DEFAULT"; exit 2; }
echo "$GC" | grep -q '"busy": false' || { log "ABORT: GC busy/unknown"; exit 2; }
echo "$AD" | grep -q '"state": "idle"' && echo "$AD" | grep -q '"latch": ""' || { log "ABORT: adapter not idle/unlatched"; exit 2; }
OLD=$(cat $EV/adapter.pid); kill -0 $OLD 2>/dev/null || { log "ABORT: old adapter pid $OLD not running"; exit 3; }
log "$(python3 $EV/p20_mc_source.py DELETE $SRC 2>&1 | tail -1)"
kill -INT $OLD; for i in $(seq 1 30); do kill -0 $OLD 2>/dev/null || break; sleep 0.5; done
kill -0 $OLD 2>/dev/null && { log "ABORT: old adapter $OLD did not exit"; exit 4; }
log "old adapter $OLD stopped"
setsid nohup python3 $NAV/x2_motion_adapter.py --ros-args -r __ns:=/qazbot_d1_map -p live:=true -p priority:=30 -p source_name:=$SRC > $EV/adapter_p20.log 2>&1 < /dev/null &
echo $! > $EV/adapter.pid
for i in $(seq 1 80); do grep -q 'x2_motion_adapter started: live=True' $EV/adapter_p20.log && break; grep -q Traceback $EV/adapter_p20.log && break; sleep 0.5; done
grep -q 'x2_motion_adapter started: live=True' $EV/adapter_p20.log || { log "adapter FAILED"; exit 5; }
log "new adapter pid $(cat $EV/adapter.pid) sha $(sha256sum $NAV/x2_motion_adapter.py | cut -c1-16): $(grep 'started' $EV/adapter_p20.log | tail -1 | cut -c1-250)"
sleep 3
log "$(timeout 20 ros2 service call /qazbot_d1_map/motion_adapter/register std_srvs/srv/Trigger '{}' 2>&1 | tail -1)"
sleep 3; log "$(grep -E 'registered|registration failed|SDK response' $EV/adapter_p20.log | tail -1)"
log "post: $(timeout 8 ros2 topic echo --once --field data /qazbot_d1_map/motion_adapter/status 2>/dev/null | head -1 | cut -c1-400)"
