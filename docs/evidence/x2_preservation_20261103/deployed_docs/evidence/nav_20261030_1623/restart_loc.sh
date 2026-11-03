#!/usr/bin/env bash
# P26 2026-10-30: restart ONLY localization (ros2 launch localization.launch.py: LIO, level_bridge, native_odom_bridge,
# map_server, AMCL, lifecycle) + Localization Trust + seed_loop.py. nav_full (Nav2 + GC) and the motion adapter stay.
# Stop = stop.sh pattern (trust INT, launch INT -> TERM -> KILL on owned pids); start = start_loc.sh (current file).
# Robot must be STANDING, GC not busy. Afterwards an operator initialpose is required (AMCL restarts without a pose).
# Usage: CONFIRM=1 ./restart_loc.sh        DRY=1 ./restart_loc.sh (print actions only)
set -u
EV=/home/agi/control_backend/docs/evidence/nav_20261030_1623
NAV=/home/agi/control_backend/ws/src/navigation
PROF=$EV/dds_nav_two_iface_log.xml
DRY=${DRY:-0}
log(){ echo "[$(date +%T.%N)] restart_loc: $*" | tee -a $EV/start_run.txt; }
run(){ if [ "$DRY" = 1 ]; then echo "DRY: $*"; else eval "$@"; fi; }
[ "$DRY" = 1 ] || [ "${CONFIRM:-}" = 1 ] || { echo "refusing: CONFIRM=1 (or DRY=1) required"; exit 1; }
cd $NAV; set +u; source ./env.sh >/dev/null 2>&1; set +e
export FASTRTPS_DEFAULT_PROFILES_FILE=$PROF ROS_DOMAIN_ID=0 ROS_LOCALHOST_ONLY=0
ST=$(timeout 12 python3 $EV/gc_status.py 2>/dev/null | head -1)
echo "$ST" | grep -q '"busy": false' || { log "ABORT: GC busy or status unavailable: ${ST:0:160}"; exit 3; }
for f in localization.launch.py loc_trust.py localization_trust_node.py; do python3 -m py_compile $NAV/$f || { log "ABORT: $f does not compile"; exit 4; }; done
TS=$(date +%H%M%S)
OWNED=$(awk '{print $1}' $EV/owned_pids.txt)
SEED=$(pgrep -f '^python3 -u seed_loop.py$' | tr '\n' ' ')
log "stop seed_loop [$SEED] trust $(cat $EV/trust.pid) localization $(cat $EV/localization.pid) owned [$(echo $OWNED)]"
[ -n "$SEED" ] && run "kill -INT $SEED"
run "kill -INT $(cat $EV/trust.pid) 2>/dev/null; sleep 3"
run "kill -INT $(cat $EV/localization.pid) 2>/dev/null"
if [ "$DRY" != 1 ]; then
  for i in $(seq 1 60); do left=""; for p in $OWNED $SEED; do kill -0 $p 2>/dev/null && left="$left $p"; done; [ -z "$left" ] && break; sleep 0.5; done
  [ -n "$left" ] && { log "INT leftovers:$left -> TERM"; kill -TERM $left 2>/dev/null; sleep 5; }
  left=""; for p in $OWNED $SEED; do kill -0 $p 2>/dev/null && left="$left $p"; done
  [ -n "$left" ] && { log "TERM ignored:$left -> KILL"; kill -KILL $left; sleep 2; }
  left=""; for p in $OWNED $SEED; do kill -0 $p 2>/dev/null && left="$left $p"; done
  [ -n "$left" ] && { log "ABORT: still alive:$left"; exit 5; }
fi
run "mv $EV/localization.log $EV/localization_pre_restart_$TS.log; mv $EV/trust.log $EV/trust_pre_restart_$TS.log"
run "bash $EV/start_loc.sh"
cd $EV; run "setsid nohup timeout 28800 python3 -u seed_loop.py >> $EV/seed_loop.log 2>&1 < /dev/null &"   # simple command: no subshell holding the caller's stdout
log "done: localization $(cat $EV/localization.pid) trust $(cat $EV/trust.pid) seed_loop $(pgrep -f '^python3 -u seed_loop.py$' | tr '\n' ' ') launch_sha=$(sha256sum $NAV/localization.launch.py | cut -c1-16) -> operator initialpose needed"
