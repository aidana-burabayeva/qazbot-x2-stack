# PLAN

## Current task: 20261027-roadmap-v12 (roadmap v1.2, see TASK.md)
Old plan archived: reports/PLAN_until_20261027.archived.md

## Phase
2026-10-27 18:5x: user: 16:16 copy is theirs (canonical); goal tol 0.20 approved. B3 impl launched.
2026-10-27 18:4x: M0b done; asked user B2 (origin of 16:16 copy) + go for restart (native_imu+unpinned) and S4 PS5 test.
2026-10-27 18:3x: P2T-b root cause = FAST-LIO CPU pinning + DDS join storms. Plan: next stack restart = native_imu odom (M0a) + unpinned FAST-LIO, then X1 join A/B; operator re-pose needed after restart. Avoid ros2 CLI during nav.
2026-10-27 18:0x: ACCELERATED TRACK started (M0a+M0b). go2 kept by user.
2026-10-27 17:5x: V2-amcl AMBIGUOUS; asked user: robot moved 17:16-17:18? tape distances right wall / wall ahead.
2026-10-27 17:4x: operator visually confirms scan on walls (user "да"). operator set approx 2D pose; V2-amcl stationary shadow launched; no anchor/goals. Earlier nav goal (-6.09,6.41) aborted 'goal occupied' (dry_run).
2026-10-27 17:2x: phase 4 P2T-b burst collection (no pose needed); asked user for operator 2D Pose Estimate.
2026-10-27 17:xx: phase 3 verifier on V0V1+V2P2T; awaiting operator 2D Pose Estimate for shadow AMCL + more burst traces.
2026-10-27 16:5x: phase 2 V2+P2T launched. User CLOSED vendor localization branch (no license request) — DECISIONS.md.
2026-10-27 16:4x: phase 1 launched — V0/V1 read-only probe + P0 inventory (parallel). P2-T live trace after V0/V1 (avoid CPU contention).
Stream A (twin) BLOCKED: laptop 192.168.1.64 unreachable from robot (no route).

## ACCELERATED TRACK (user 2026-10-27 ~18:00): goal = RViz 2D Goal Pose → X2 walks there. Trust policy go2 stays (user decision).
Motion tests allowed in principle; each physical step still gets explicit user "go" + operator with PS5.
M0 (no motion, parallel): (a) native_imu odom mode per reports/20261027-native-odom-nav2-design.md §3 (LIO drift 1.77 m on in-place turn = odom blocker); (b) live-chain readiness audit + M1 test card.
M1: live adapter, RViz goal 1 m straight in open area. M2: 90° turn goal. M3: office A→B + return. Each: bag + independent pose check.

## Task list
| # | Task | Status | Agent |
|---|------|--------|-------|
| V0V1 | host/graph/TF/MC + vendor loc/license | DONE: same unit as .66 (SN X2B0028C6Z0034); vendor Localization/Mapping license denied → vendor branch NO-GO; TF single-owner OK; AMCL no map→odom (no pose?) | robotics-debugger |
| P0 | control_backend inventory | DONE reports/20261027-roadmap-v12-P0.md (gaps: VoxelLayer, X2 profile, map registry, ingress trace tool) | Explore |
| V2+P2T | map identity + shadow + live trace | NEEDS_DATA: map v3 identical; ingress/eval on time; 1 burst (0.47 s) upstream of DDS under CPU saturation; AMCL no initial pose (NEEDS_OPERATOR); candidate fix: deploy 26.10 depth-1 level_bridge input (not applied) | robotics-debugger |
| V2-amcl | stationary AMCL shadow | AMBIGUOUS: final pose (-1.24,-0.13,0.05) stable 0 jumps, 62% beams<10cm, but σxy .4-.5 m, alt pose 1 m away fits better, ≥8 similar places on map. SAFETY FINDING: trust policy go2 → trusted+motion_ok=True despite this (adapter dry_run, no risk now). Anchor NOT justified. | robotics-debugger opus/high |
| P2T-b | burst root cause | ROOT_CAUSE_FOUND (unverified fix): 5/5 bursts at new DDS participant joins (34 participants on domain 0); FAST-LIO pinned to CPU 5,6 (stack_up.sh:42 lio_cpus) shares vendor cgroup /em/algo → 0.4-0.7 s runqueue stall; HAL on time. Fix proposal: empty lio_cpus, rerun X1 join protocol. Also HAL cloud stamp=0 once → lidar loop back. | robotics-debugger opus/high |
| M0a | native_imu odom mode | IMPL DONE (native_odom_bridge.py, level_bridge publish_tf, odom_source switch; lio 55/55, adapter 43/43; loc_trust 56 fail = pre-existing: 16:16 loc_trust.py default policy go2 vs tests strict). origin-reset dt cap 0.2 s DONE (lio 61/61). Verifier c1 RUNNING (robotics-verifier opus/high). | general-purpose (implementer role; worktree isolation unavailable, non-git) opus/high |
| M0b | live-chain readiness audit + M1 card | DONE (NEEDS_DATA live state) reports/20261027-roadmap-v12-M0b.md. Blockers B1 PS5 live takeover S4 never run; B2 ws/src/navigation trust/coordinator/launch overwritten 16:16 by external copy (likely laptop twin; candidates/ corrupted 527 empty files); B3 goal tol 0.10 < coast 0.14+drift → propose xy 0.20 + stateful; B4 fresh source name; B5 recorder QoS; B6 operator outside front ±60°; B7 loc ambiguous (M3); B8 native_imu (M2). Caps 0.3 m/s / 0.6 rad/s, ≥2 m clear. | robotics-debugger opus/high |
| B3+pin | stack_up: QAZBOT_LIO_CPUS (allow empty=unpinned) + nav extra args (xy_tol 0.20, goal_checker_stateful true) | TODO after M0a verify (same file) | general-purpose implementer sonnet/medium |
| VER | verify V0V1+V2P2T claims | PASS c1 (claims 1-5); claim-6 finding = ws/src native_odom*/rviz xml edits 16:36-16:47 belong to parallel native-odom workstream (peer session agi-98, reports/20261027-native-odom-nav2-design.md), NOT this task; not reverted | robotics-verifier |
| P2T-b | more burst traces (≥3 bursts, FAST-LIO-output observer, atop) | RUNNING | robotics-debugger |
| A | twin P2-T on laptop | BLOCKED (laptop unreachable) | — |

## Routing decisions
| Step | Agent | Model | Effort | Reason |
|------|-------|-------|--------|--------|
| V0V1 | robotics-debugger | opus | high | vendor services/licensing on live robot, safety-relevant |
| P0 | Explore | haiku | low | file inventory only |
| V2+P2T | robotics-debugger | opus | high | sensor timing/QoS/executor root cause, live robot |
| VER | robotics-verifier | sonnet | medium | routine verification of read-only diagnostic claims |
| P2T-b | robotics-debugger | opus | high | intermittent timing root cause |

## Verification cycles
0/3

## Session continuation command
```
claude "/robotics-task continue 20261027-roadmap-v12 from .orchestrator/PLAN.md"
```
- 2026-10-27: verifier cycle2 PASS (native limiter). LIO drift diagnosed (H3 degeneracy + H4 accel bias/attitude). Dry-run stack UP tag nav72_20261027 (stop: tools/stack/stack_down.sh nav72_20261027). BLOCKER: control_backend files partially overwritten 15:36–16:16 by unknown copy (candidates/ 527 empty files, tests/integration_results 944 empty) — ask user source. PC IP .64 vs .70 in RViz profile. Native-odom design: reports/20261027-native-odom-nav2-design.md (step1 native_imu, compare mode first).
