# PLAN

## Current task: 20261027-rpp-local-collision (see TASK.md)
Previous task archived: reports/PLAN_20261027-roadmap-v12.archived.md

## Phase
2026-10-27: D1 done: Goal01 ROOT_CAUSE = stale lethal marks (inf scan beams not clearing, inf_is_valid=false); checker NOT too strict (lethal 254 only). Trial03 NEEDS_DATA. PAUSED by user after D2; findings written to control_backend/docs/roadmap_v13/RPP-local-collision-review.md and REPORT.md. Phase 3: I1 staged implementation launched. Phase 2: D2 offline costmap rebuild + blind-zone check.
2026-10-27: phase 1 diagnosis launched (robotics-debugger).

## Task list
| # | Task | Status | Agent |
|---|------|--------|-------|
| D1 | root cause of RPP collision-ahead aborts | DONE Goal01 (b); trial03 NEEDS_DATA; reports/20261027-rpp-local-collision-debugger.md | robotics-debugger |
| D2 | trial03 offline costmap rebuild + near-field blind-zone of scan slice | DONE: both aborts = uncleared lethal marks; goal01 in-FOV inf bins, trial03 just outside FOV after real ~10° gait-start lurch; blind zone none beyond 0.45 m; FOV-masked inf clearing +guard validated offline; reports/20261027-rpp-local-collision-D2.md | robotics-debugger |
| I1 | staged fix: level_bridge NaN/inf FOV mask+guard, inf_is_valid, BT FollowPath clear-local-costmap recovery | STOPPED by user mid-way; partial unverified files in control_backend/candidates_v13/I1_rpp_collision/; no manifest/tests report; live tree untouched | general-purpose (implementer role; worktree unavailable) |
| V1 | independent verify | TODO | robotics-verifier |

## Routing decisions
| Step | Agent | Model | Effort | Reason |
|---|---|---|---|---|
| D1 | robotics-debugger | opus | high | Nav2 costmap/TF root cause, safety-relevant |
| D2 | robotics-debugger | opus | high | safety precondition for clearing change |
| I1 | general-purpose | sonnet | medium | precisely specified fix with clear acceptance |

## Continuation
Resume: "продолжи задачу 20261027-rpp-local-collision из .orchestrator/PLAN.md" — restart I1 (review/redo partial candidates_v13/I1_rpp_collision), then V1 opus/high, then ask user for live apply.
