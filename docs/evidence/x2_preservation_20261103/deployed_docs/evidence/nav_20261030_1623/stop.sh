#!/usr/bin/env bash
# Full stop of owned processes: (1) motion+Nav2 via stop_nav.sh (P5b), (2) Trust, (3) localization launch (P4b).
# Never pkill. Original localization-only version: stop_localization_only.sh.orig
EV=/home/agi/control_backend/docs/evidence/nav_20261030_1623
[ -f $EV/adapter.pid -o -f $EV/nav_full.pid ] && bash $EV/stop_nav.sh
OWNED=$(awk '{print $1}' $EV/owned_pids.txt)
kill -INT $(cat $EV/trust.pid) 2>/dev/null; sleep 3
kill -INT $(cat $EV/localization.pid) 2>/dev/null
for i in $(seq 1 60); do left=""; for p in $OWNED; do kill -0 $p 2>/dev/null && left="$left $p"; done; [ -z "$left" ] && break; sleep 0.5; done
[ -n "$left" ] && { echo "INT leftovers:$left -> TERM"; kill -TERM $left 2>/dev/null; sleep 5; }
left=""; for p in $OWNED; do kill -0 $p 2>/dev/null && left="$left $p"; done
[ -n "$left" ] && { echo "TERM ignored (known rclpy/FastDDS shutdown futex deadlock):$left -> KILL"; kill -KILL $left; sleep 2; }
left=""; for p in $OWNED; do kill -0 $p 2>/dev/null && left="$left $p"; done
echo "STOP done; owned leftovers:[${left}]"
