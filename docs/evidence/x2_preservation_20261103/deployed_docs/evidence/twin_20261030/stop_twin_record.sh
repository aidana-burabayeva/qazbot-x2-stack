#!/bin/bash
# Cleanly stop twin recorders with SIGINT (rosbag2 flushes cache and writes metadata.yaml), then the monitor.
# Usage: ./stop_twin_record.sh [core|hf|all]   KEEP_MONITOR=1 keeps the CSV monitor running.
D=/home/agi/control_backend/docs/evidence/twin_20261030; cd "$D"
which=${1:-all}
stop_one() {
  local f=rec_$1.pid; [[ -s $f ]] || { echo "no $f"; return; }
  local pid=$(cat $f)
  if kill -0 "$pid" 2>/dev/null && tr "\0" " " < /proc/$pid/cmdline 2>/dev/null | grep -q "ros2 bag record"; then
    kill -INT "$pid"; for i in $(seq 60); do kill -0 "$pid" 2>/dev/null || break; sleep 0.5; done
    kill -0 "$pid" 2>/dev/null && { echo "pid $pid still alive after 30 s, sending SIGTERM"; kill -TERM "$pid"; }
    echo "$(date '+%F %T') stop $1 pid=$pid" | tee -a recorder_events.log
  else echo "$1 pid $pid not running"; fi
}
[[ $which == core || $which == all ]] && stop_one core
[[ $which == hf   || $which == all ]] && stop_one hf
if [[ $which == all && -z $KEEP_MONITOR && -s monitor.pid ]]; then
  kill "$(cat monitor.pid)" 2>/dev/null && echo "$(date '+%F %T') stop monitor pid=$(cat monitor.pid)" | tee -a recorder_events.log
fi
