#!/usr/bin/env bash
# Operator OK 2026-11-02 18:4x: dry-run adapter -> live P23 adapter (prio 30 < rc 80) + ONE MC input-source ADD. No goal sent here.
set -u
[ "${CONFIRM:-}" = 1 ] || { echo "refusing: CONFIRM=1 required"; exit 1; }
EV=/home/agi/control_backend/docs/evidence/nav_20261102_live
NAV=/home/agi/control_backend/ws/src/navigation
SRC=sdk_qazbot_nav2_1102a
log(){ echo "[$(date +%T.%N)] go_live: $*" | tee -a $EV/start_nav_run.txt; }
cd $NAV; set +u; source ./env.sh >/dev/null 2>&1; source ./aimdk_env.sh >/dev/null 2>&1; set +e
export FASTRTPS_DEFAULT_PROFILES_FILE=$EV/dds_nav_two_iface_log.xml ROS_DOMAIN_ID=0 ROS_LOCALHOST_ONLY=0
MC=$(timeout 8 ros2 topic echo --once /aima/mc/common/state 2>/dev/null | grep -E 'action_desc|name:' | tr -s ' ' | tr '\n' ' ')
GC=$(timeout 8 ros2 topic echo --once --field data /qazbot_d1_map/goal_coordinator/status 2>/dev/null | head -1)
AD=$(timeout 8 ros2 topic echo --once --field data /qazbot_d1_map/motion_adapter/status 2>/dev/null | head -1)
log "pre: mc=[$MC] gc=${GC:0:160} adapter=${AD:0:160}"
echo "$MC" | grep -q STAND_DEFAULT && echo "$MC" | grep -q "name: rc" || { log "ABORT: MC not STAND_DEFAULT/rc"; exit 2; }
echo "$GC" | grep -q '"busy": false' || { log "ABORT: GC busy/unknown"; exit 2; }
echo "$AD" | grep -q '"live": false' && echo "$AD" | grep -q '"latch": ""' || { log "ABORT: adapter not dry/unlatched"; exit 2; }
OLD=$(cat $EV/adapter.pid); kill -INT $OLD; for i in $(seq 1 30); do kill -0 $OLD 2>/dev/null || break; sleep 0.5; done
kill -0 $OLD 2>/dev/null && { log "ABORT: dry adapter $OLD did not exit"; exit 4; }
log "dry adapter $OLD stopped"; mv $EV/adapter.log $EV/adapter_dry.log
setsid nohup python3 $NAV/x2_motion_adapter.py --ros-args -r __ns:=/qazbot_d1_map -p live:=true -p priority:=30 -p source_name:=$SRC > $EV/adapter.log 2>&1 < /dev/null &
echo $! > $EV/adapter.pid
for i in $(seq 1 80); do grep -q 'x2_motion_adapter started: live=True' $EV/adapter.log && break; grep -q Traceback $EV/adapter.log && break; sleep 0.5; done
grep -q 'x2_motion_adapter started: live=True' $EV/adapter.log || { log "adapter FAILED"; exit 5; }
log "live adapter pid $(cat $EV/adapter.pid) sha $(sha256sum $NAV/x2_motion_adapter.py | cut -c1-16)"; sleep 3
log "register: $(timeout 20 ros2 service call /qazbot_d1_map/motion_adapter/register std_srvs/srv/Trigger '{}' 2>&1 | tail -1)"
sleep 3; log "$(grep -E 'registered|registration failed|SDK response' $EV/adapter.log | tail -1)"
log "post: $(timeout 8 ros2 topic echo --once --field data /qazbot_d1_map/motion_adapter/status 2>/dev/null | head -1 | cut -c1-400)"
