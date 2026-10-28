#!/bin/bash
# Usage (host): vel_test.sh NAME FWD LAT ANG   -> runs the OFFICIAL mc_locomotion_velocity example in the sim container
# for its fixed 5 s and records odometry with tools/twin/mc_probe.py to runtime_x86/run/NAME.csv
N=$1; F=$2; L=$3; A=$4
CB="$(cd "$(dirname "$0")/../../.." && pwd)"
docker exec -d qazbot-twin-sim bash -c "/workspace/control_backend/tools/twin/docker/ex.sh python3 /workspace/control_backend/tools/twin/mc_probe.py /workspace/control_backend/runtime_x86/run/$N.csv 14 > /workspace/control_backend/runtime_x86/run/$N.probe.log 2>&1"
sleep 1.5
printf "$F\n$L\n$A\n" | docker exec -i qazbot-twin-sim /workspace/control_backend/tools/twin/docker/ex.sh timeout 8.5 ros2 run py_examples mc_locomotion_velocity > "$CB/runtime_x86/run/$N.vel.log" 2>&1
sleep 6
