#!/usr/bin/env bash
# Enqueue tonight's existing `make nightly` campaign. Name carries the date: re-running the timer never duplicates or overwrites history.
set -euo pipefail
cd "$(dirname "$0")/../.."
D=$(date +%Y%m%d)
python3 tools/nightly/nq.py enqueue "make_nightly_$D" "make -C digital_twin nightly" --resource sim --priority 80 --timeout 14400 --heartbeat 1800
# L1 screening (cheap, ~1 s per 200 scenarios) on a date-derived seed; runs before the heavy tier
python3 tools/nightly/nq.py enqueue "l1_screen_$D" "tools/nightly/run_l1.sh $((10#$D % 100000)) 100 4" --priority 70 --timeout 1800
