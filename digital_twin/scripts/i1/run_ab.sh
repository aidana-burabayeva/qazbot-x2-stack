#!/usr/bin/env bash
echo 'Historical harness: not authorized as the current Nightly baseline; use SHA-pinned campaign.py' >&2
exit 2
# I1 A/B in the twin: legacy (baseline) vs candidate (scan_format=i1 + obstacle_layer inf_is_valid), same start, same scenario.
#   scripts/i1/run_ab.sh [TAG] [MODE...]   MODE: baseline candidate (default) | abl_inf_only (legacy scan + inf_is_valid) | abl_scan_only (i1 scan, no inf_is_valid)
#   results: results/i1-TAG-MODE/ (+ stack_collision_lines.txt)
set -uo pipefail
cd "$(dirname "$0")/../.."
TAG=${1:-$(date +%H%M%S)}
shift || true
MODES=${*:-baseline candidate}
C="docker compose ${TWIN_PROJECT:+-p $TWIN_PROJECT} -f docker/compose.yaml --env-file .env"
export TWIN_LOCALIZATION=truth TWIN_START=-7.3,4.6,-0.3490658504 TWIN_SCAN_FOV_DEG=${TWIN_SCAN_FOV_DEG:-130}
for mode in $MODES; do
  case $mode in
    candidate) export TWIN_SCAN_FORMAT=i1 TWIN_I1_INF_VALID=1 TWIN_I1_MODE=candidate ;;
    abl_inf_only) export TWIN_SCAN_FORMAT=legacy TWIN_I1_INF_VALID=1 TWIN_I1_MODE=baseline ;;   # soft checks: records what happens
    abl_scan_only) export TWIN_SCAN_FORMAT=i1 TWIN_I1_INF_VALID=0 TWIN_I1_MODE=baseline ;;
    *) export TWIN_SCAN_FORMAT=legacy TWIN_I1_INF_VALID=0 TWIN_I1_MODE=baseline ;;
  esac
  export RUN_NAME=i1-$TAG-$mode
  $C --profile test down >/dev/null 2>&1
  $C up -d --wait sim stack || { echo "up failed ($mode)"; $C logs --tail 30 stack; exit 2; }
  $C --profile test run --rm scenario-i1 >"results_i1_$mode.log" 2>&1; echo "scenario $mode rc=$?"
  mkdir -p results/$RUN_NAME && mv "results_i1_$mode.log" results/$RUN_NAME/scenario.log 2>/dev/null
  $C logs stack --no-log-prefix 2>&1 | sed 's/\x1b\[[0-9;]*m//g' | grep -iE "collision ahead|Failed to make progress|Aborting|Goal failed" > results/$RUN_NAME/stack_collision_lines.txt || true
  $C --profile test down >/dev/null 2>&1
done
echo done
