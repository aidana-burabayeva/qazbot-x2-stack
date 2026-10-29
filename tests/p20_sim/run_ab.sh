#!/usr/bin/env bash
# Isolated official-MC calibration A/B. Must be run with the existing twin's internal network.
set -euo pipefail
BACKEND=$(cd "$(dirname "$0")/../.." && pwd)
TWIN_DIR=${TWIN_DIR:-/home/adn/QazBot/digital_twin}
OUT=${P20_OUTPUT_DIR:-"$BACKEND/docs/evidence/p20_twin02_20261029"}
SUFFIX=${P20_RUN_SUFFIX:-cpu8}
cd "$TWIN_DIR"
export SIM_CPUS=8 TWIN_SIM_RES=320x180x24 TWIN_START=-7.885339914512176,12.96582446398428,0.0
compose=(docker compose -f docker/compose.yaml --env-file .env)
trap '"${compose[@]}" stop -t 20 sim' EXIT
for case in ${P20_CASES:-baseline_ccw baseline_cw p20_cw}; do
  loop=false; direction=1
  case "$case" in baseline_cw) direction=-1;; p20_cw) loop=true;direction=-1;; esac
  name="official_${case}_${SUFFIX}"
  "${compose[@]}" stop -t 20 sim
  "${compose[@]}" up -d --no-build --force-recreate sim
  docker exec x2-twin-sim-1 twin-sim-health --wait 120
  # health can become true just before the supervisor's final LD acknowledgement.
  timeout 30 bash -c 'until docker logs x2-twin-sim-1 2>&1 | grep -q "READY: standing, locomotion mode"; do sleep 1; done'
  mkdir "$OUT/${name}_environment"
  docker inspect x2-twin-sim-1 > "$OUT/${name}_environment/container.json"
  docker exec x2-twin-sim-1 cat /var/log/twin/world.json > "$OUT/${name}_environment/world.json"
  docker run --rm --name p20-official-probe --network x2-twin_twin --cap-drop ALL --security-opt no-new-privileges \
    -v "$BACKEND:/backend:ro" -v /tmp/qazbot-p20-build:/tools:ro -v "$OUT:/results" \
    x2-twin-stack:dev python3 /backend/tests/p20_sim/p20_official_mc.py \
    --loop "$loop" --direction "$direction" --out "/results/$name" || echo "probe failed: $name; evidence retained"
  docker logs x2-twin-sim-1 > "$OUT/${name}_environment/startup.log" 2>&1
  docker exec x2-twin-sim-1 cat /sys/fs/cgroup/cpu.stat > "$OUT/${name}_environment/cpu.stat"
  docker exec x2-twin-sim-1 tail -150 /var/log/twin/mc.log > "$OUT/${name}_environment/mc_tail.log"
done
