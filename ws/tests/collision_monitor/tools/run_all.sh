#!/usr/bin/env bash
# All offline scenarios (short; sim ~6 min total). Offsets are seconds from bag start 18:30:16.63 (local).
# Ground truth: .orchestrator/reports/20261103-pd04-analysis.md section 1 (person/empty timeline).
# NOTE (user correction 2026-11-03): this bag contains NO boxes, only the operator -> no low-object scenario.
set -euo pipefail
R="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)/run_scenario.sh"
#        name              offset dur cm_override               harness_override         gate  cloud
"$R" s1_empty               4     40
"$R" s2_person_front_leave  224   95
"$R" s3_operator_sides      327   75
"$R" s4_walker_blind_exit   495   20
"$R" s5_cloud_dropout       229   25  none.yaml                gate_cloud_dropout.yaml  true
"$R" s6_tf_stale_bsc        229   25  cm_base_shift_true.yaml  gate_tf_stale.yaml       true
"$R" s7_no_cloud            229   15  none.yaml                none.yaml                false false
"$R" s8_cmd_in_stops        229   20  none.yaml                cmd_stop_after_10s.yaml
"$R" s9_person_bsc_true     224   40  cm_base_shift_true.yaml
