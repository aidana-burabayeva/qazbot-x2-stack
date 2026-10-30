#!/usr/bin/env bash
# L2: one scenario through the real Nav2 + P20 adapter + MC/MuJoCo chain (truth localisation, robot goal tolerances 0.20/0.35).
# usage: run_l2.sh <scenario_id> <scenarios.tsv> ; one fresh sim per scenario (TWIN_START is a launch-time argument). SIM ONLY.
set -euo pipefail
cd "$(dirname "$0")/../.."
ID=${1:?id}; TSV=${2:?tsv}
LINE=$(awk -F'\t' -v id="$ID" '$1==id' "$TSV"); [ -n "$LINE" ] || { echo "no scenario $ID"; exit 2; }
read -r _ CAT SX SY SYAW GX GY GYAW EXP <<<"$(echo "$LINE" | tr '\t' ' ')"
RUN="l2_${ID}_$(date +%Y%m%d_%H%M%S)"
export TWIN_START="$SX,$SY,$SYAW" P20_RUN="$RUN"
export GOAL_ARGS="--goals=$GX,$GY,$GYAW --xy-tol 0.20 --yaw-tol 0.35 --record-phases"
rc=0; bash tests/p20_sim/run_nav2_trace.sh > "run/nightly/logs/$RUN.out" 2>&1 || rc=$?
R=/home/adn/QazBot/digital_twin/results/$RUN
CL=$(python3 tools/nightly/clearance_from_trace.py "$R/trace.json" 2>/dev/null || echo '{}')
echo "{\"scenario\":\"$ID\",\"category\":\"$CAT\",\"expected\":\"$EXP\",\"nav2_rc\":$rc,\"clearance\":$CL}" | tee "run/nightly/l2_$ID.json"
