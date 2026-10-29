#!/usr/bin/env bash
# SIM ONLY. Full Nav2 + P20 adapter + official MC/MuJoCo, truth localisation, one goal with phase recorder.
#   P20_RUN=<unique name> [GOAL_ARGS="--only-ccw --record-phases"] tests/p20_sim/run_nav2_trace.sh
# Output: digital_twin/results/<P20_RUN>/ (result.json, trace.json, recorder/{motion.csv,status.jsonl}).
# Same infrastructure as the P20 A/B: SIM 8 CPU, stack 4 CPU, 320x180 Xvfb, one llvmpipe thread. Fresh containers every run.
set -euo pipefail
HERE=$(cd "$(dirname "$0")" && pwd)
BACKEND=$(cd "$HERE/../.." && pwd)
TWIN=${TWIN_DIR:-/home/adn/QazBot/digital_twin}
BUILD=${P20_BUILD:-/tmp/qazbot-p20-build}
RUN=${P20_RUN:?set P20_RUN to a new unique name}
GOAL_ARGS=${GOAL_ARGS:---only-ccw --record-phases}
[ -x "$BUILD/torso_odom" ] && [ -x "$BUILD/timing/timing_recorder" ] || { echo "build tools first: see tests/p20_sim/README.md"; exit 2; }
[ ! -e "$TWIN/results/$RUN" ] || { echo "results/$RUN exists: evidence is never overwritten"; exit 2; }

export P20_BACKEND=$BACKEND TWIN_MAP=main_office TWIN_SIM_RES=320x180x24 LP_NUM_THREADS=1 SIM_CPUS=8 STACK_CPUS=4
export TWIN_START=${TWIN_START:--7.885339914512176,12.96582446398428,0.0}
C=(docker compose -p x2-twin -f "$TWIN/docker/compose.yaml" -f "$HERE/nav2.compose.yaml" --env-file "$TWIN/.env")
cleanup() { "${C[@]}" down >/dev/null 2>&1 || true; }
trap cleanup EXIT
"${C[@]}" up -d --wait --force-recreate sim stack
docker run --rm --user 1000:1000 --network x2-twin_twin --entrypoint bash \
  -e TWIN_SANDBOX=1 -e ROS_DOMAIN_ID=87 -e TWIN_STACK_DIR=/opt/x2twin/stack -e TWIN_RESULTS_DIR=/results \
  -v "$BACKEND":/backend:ro -v "$TWIN/results":/results -v "$BUILD/timing":/timing/build:ro \
  -v /home/adn/QazBot/control_backend/ws/src/maps/main_office:/opt/x2twin/stack/maps/main_office:ro \
  x2-twin-stack:dev -lc ". /opt/ros/humble/setup.bash; . /opt/x2twin/overlay/setup.sh; python3 /backend/tests/p20_sim/nav2_goals.py --out $RUN $GOAL_ARGS"
