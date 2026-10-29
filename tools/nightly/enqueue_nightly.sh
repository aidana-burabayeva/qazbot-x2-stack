#!/usr/bin/env bash
# Enqueue tonight's existing `make nightly` campaign. Name carries the date: re-running the timer never duplicates or overwrites history.
set -euo pipefail
cd "$(dirname "$0")/../.."
D=$(date +%Y%m%d)
python3 tools/nightly/nq.py enqueue "make_nightly_$D" "make -C digital_twin nightly" --resource sim --priority 80 --timeout 14400 --heartbeat 1800
