#!/bin/bash
# Selective, load-guarded twin recording (TASK 20261030-twin-data-collection T1). Two rosbag2 recorders:
#   core = command chain, odom, TF, scans, MC state, Trust/AMCL/goal status (~350 msg/s)
#   hf   = raw torso IMU + lidar IMU + leg joint states (~1040 msg/s); can be dropped alone by the guard.
# Excluded: raw/downsampled 3D lidar (~0.8 MB/msg, >=1.9 MiB/s measured on down_sampling, ~8 MB/s raw), chest IMU, waist joints, costmaps, cameras.
# Usage: ./start_twin_record.sh [core|hf|all]   (default all). Stop: ./stop_twin_record.sh
set -e
D=/home/agi/control_backend/docs/evidence/twin_20261030; cd "$D"
source tools/env.sh
which=${1:-all}; ts=$(date +%H%M%S)
start_one() { # name topicfile split_seconds
  local name=$1 tf=$2 dur=$3 out=bags/twin_${1}_$ts
  setsid nohup nice -n 10 ionice -c3 ros2 bag record -s mcap -p 2000 --include-hidden-topics \
     --qos-profile-overrides-path tools/qos_overrides.yaml -d "$dur" -b 2000000000 -o "$out" \
     $(grep -v '^#' "$tf" | tr '\n' ' ') > "$out.log" 2>&1 < /dev/null &
  sleep 1
  local pid=$(pgrep -f "^/usr/bin/python3 /opt/ros/humble/bin/ros2 bag record .* -o $out " | head -1)
  echo "$pid" > "rec_$name.pid"; echo "$(date '+%F %T') start $name pid=$pid out=$out" | tee -a recorder_events.log
}
[[ $which == core || $which == all ]] && start_one core tools/topics_core.txt 600
[[ $which == hf   || $which == all ]] && start_one hf tools/topics_hf.txt 300
exit 0
