# Reuse-first evaluation (2026-11-03)

Purpose: evaluate existing ROS 2 / Nav2 / AIMDK solutions before writing custom code for D-04/D-05/D-07. Research only; no production change.

## Reports
| File | Status |
|---|---|
| 20261103-reuse-R1-aimdk-d05.md | AIMDK MC examples vs X2, smoother vs Ruckig, D-05 criteria. Q1-Q3 done; Q4 partial (criteria defined, C1/C2 pending recomputation). |
| 20261103-reuse-R3-d07.md | D-07 low objects from raw E1R cloud. Root cause found at code/geometry level; real-box check NOT PERFORMED. |
| 20261103-reuse-R4-licenses-mppi.md | License/portability inventory of external repos + MPPI baseline. Done. |
| 20261103-reuse-R5-d05-cancel-latency.md | D-05 cancel mid-cruise (root cause from code) and T_nav (partial). Dry harness written, NOT RUN. |
| 20261103-pd04-correction.md | Correction: no boxes in bag d04_static_bag_183015. |

Related: tools/d07/ (offline bag analysis scripts), tests/d05/cancel_latency/ (dry harness, NOT YET RUN).

## Known corrections / open items
- Bag d04_static_bag_183015 contained NO boxes (operator statement; raw log pd04_steps says otherwise; the operator statement prevails). D-07 is an OPEN safety blocker; the real-box check was NOT PERFORMED.
- R1/R5 D-05 criteria C1/C2 used a minimum range of 0.695 m, which R3 found wrong (raw returns reach 0.105 m; the near limit is the lowest beam elevation, nearest floor return 0.49 m). The previous C1 range allowance is invalid; arithmetic and model limitations are documented in the V continuation report. C2 remains conditional on unmeasured obstacle-ramp coast.
- R5 T_nav is an estimate (sensing stage measured: 133/139 ms p50/p99), not measured end-to-end.
- R2/R3b reports are now preserved under [orchestrator reports](../../evidence/x2_preservation_20261103/orchestrator/reports/). Their historical results do not replace hardware verification. Fresh emulator unit tests: 11 passed. See [V continuation and C1/C2 correction](../../evidence/x2_preservation_20261103/REPORT.md).
- Licenses: aimdk_msgs/examples/py_examples have TODO license; spark-fast-lio is GPLv2 (no vendoring); byggmesterPRO has no license (ideas only).
