#!/usr/bin/env bash
# L1 screening of one seed: generate -> compile -> run -> summary. SIM ONLY. Usage: run_l1.sh <seed> [per_category] [threads]
set -euo pipefail
cd "$(dirname "$0")/../.."
SEED=${1:?seed}; PER=${2:-50}; TH=${3:-4}
D=run/nightly/l1/seed$SEED; mkdir -p "$D"
python3 tools/nightly/scenario_gen.py --seed "$SEED" --per-category "$PER" --out "$D/scenarios.jsonl" | head -1
g++ -O2 -std=c++17 -pthread -o run/nightly/l1_runner tools/nightly/l1/l1_runner.cpp
run/nightly/l1_runner ws/src/maps/main_office "$D/scenarios.tsv" "$D/results.tsv" "$TH" | tee "$D/summary.txt"
sha256sum ws/src/maps/main_office/map.pgm > "$D/map.sha256"; git rev-parse HEAD > "$D/git_sha.txt"
awk -F'\t' '{k=$2"\t"$4"\t"$6; c[k]++} END{for(k in c)print c[k]"\t"k}' "$D/results.tsv" | sort -k2 > "$D/signatures.tsv"
