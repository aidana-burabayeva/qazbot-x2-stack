#!/usr/bin/env bash
# I1 acceptance series: 3 identical A/B repeats (plant sector 130 deg) + 1 A/B at the real ~92 deg sector. Then verdicts via summarize.py.
set -uo pipefail
cd "$(dirname "$0")/../.."
for r in r1 r2 r3; do TWIN_SCAN_FOV_DEG=130 scripts/i1/run_ab.sh $r; done
TWIN_SCAN_FOV_DEG=92 scripts/i1/run_ab.sh f92
echo series-done
