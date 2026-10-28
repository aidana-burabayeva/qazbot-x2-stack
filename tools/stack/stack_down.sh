#!/usr/bin/env bash
# Stop the dry-run stack started by stack_up.sh, reverse order, one SIGINT per launch PID (no pkill).
# Usage: tools/stack/stack_down.sh [TAG]   (default: logs/stack_last_tag)
set -u
LOGS=/home/agi/control_backend/logs
TAG="${1:-$(cat "$LOGS/stack_last_tag" 2>/dev/null)}"
[ -n "$TAG" ] || { echo "no TAG"; exit 1; }
PATTERN='fastlio_mapping|level_bridge|native_odom_bridge|nav2_amcl/amcl|map_server|planner_server|controller_server|bt_navigator|velocity_smoother|lifecycle_manager|goal_coordinator|x2_motion_adapter|localization_trust'
for s in nav_full adapter trust localization; do
  f="$LOGS/${s}_$TAG.pid"
  [ -f "$f" ] || { echo "$s: no pid file, skip"; continue; }
  p=$(cat "$f")
  if ! kill -0 "$p" 2>/dev/null; then echo "$s: pid $p not running"; continue; fi
  kill -INT "$p"
  for i in $(seq 1 60); do kill -0 "$p" 2>/dev/null || break; sleep 0.5; done
  if kill -0 "$p" 2>/dev/null; then echo "$s: pid $p still alive after 30 s (send: kill -TERM $p)"; else echo "$s: pid $p stopped"; fi
  sleep 1
done
left=$(pgrep -af "$PATTERN" | grep -v -E "pgrep|stack_down")
if [ -n "$left" ]; then echo "LEFTOVER processes (orphans; stop each with kill -TERM <pid>):"; echo "$left" | cut -c1-160; exit 2; fi
echo "STACK_DOWN OK tag=$TAG (no stack processes left)"
