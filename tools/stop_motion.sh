#!/usr/bin/env bash
# Stops record_motion.sh processes with SIGINT (bag finalizes metadata, joint_record saves npz).
set -uo pipefail
NAME="${1:?usage: stop_motion.sh NAME}"
BK="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
for k in bag joints; do
  f="$BK/logs/rec_${NAME}_$k.pid"
  [ -f "$f" ] || { echo "no pid file $f"; continue; }
  pid=$(cat "$f")
  if kill -0 "$pid" 2>/dev/null; then
    kill -INT "$pid"
    for _ in $(seq 1 100); do kill -0 "$pid" 2>/dev/null || break; sleep 0.2; done
    kill -0 "$pid" 2>/dev/null && echo "WARN $k pid $pid still alive after 20 s (not killed)" || echo "$k stopped"
  else echo "$k not running"; fi
  rm -f "$f"
done
du -sh "$BK/logs/bags/$NAME" "$BK/logs/bags/$NAME.joints.npz" 2>/dev/null
