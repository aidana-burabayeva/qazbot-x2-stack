# PLAN

2026-10-23 20:xx: NEW workstream digital twin (user laptop). Handoff: reports/20261023-twin-handoff.md. PC1 MC = v1.1.0-r9-rc9, mc_param r9-rc3 snapshot in vendor_snapshots.
2026-10-23 21:xx: twin mcparam diff DONE (reports/20261023-twin-mcparam-diff.md): LD path params+onnx identical sim vs robot; mismatch = policy/physics/binary. Next: n>=5 sim step runs on laptop (needs user go), fit gain/deadband/asym from bags.

## Current phase
2026-10-23 18:2x: bringup23f UP (tools/stack/stack_up.sh; stop: stack_down.sh bringup23f). loc 43430/fastlio 43468, trust 43652 (pose_store off), adapter 43770 dry_run, nav 43879. odom p99 43 ms. T1 now only AMCL-derived (no pose yet). Operator's 18:06 pose lost by agent restart; first run AMCL diverged after poses (σxy .50→1.09, scan_score .39→.06). Quickstart doc docs/X2_QUICKSTART_BRINGUP_RU.md. Blockers summary reports/20261023-nav-blockers.md. Next: operator pose → nomotion → confirm_anchor; user decisions on calib (cap 0.6, 2 m, recording).
2026-10-23: calib-design DONE (reports/20261023-calib-design.md). Registration accuracy (3D raw cloud, stationary): ~4 mm/axis, 0.03° yaw. Existing-bag LIO error ~2–3 cm per 1–1.2 m, ≤0.2° per leg; −8% from floor marks likely mark error. Scripts copied to control_backend/tools/lio_calib/. Protocol: 3 sessions ≤10 min (24 fwd legs 1.0/0.5 m, 32 open 45° turns, 8 closed 360°). Needs user: rot cap 0.6 OK, max uninformative stretch to cover (e.g. 2 m), raw-cloud recording method; per-block go.
2026-10-23: user: robot was moved; wants orchestrator-commanded tests (operator only holds robot), no floor marks. Launched: bringup23 (general-purpose opus/high, live no-motion, trust pose_store OFF) + calib-design (robotics-debugger opus/high: closed-loop motions, reference = registration of stationary start/end lidar clouds; tools/lio_calib/). Each physical motion still needs separate user go.
2026-10-23 ~18:00 (robot +08): host up 30 min (rebooted) → whole stack DOWN, no pose file. Prior agents (locB-live, T5) died on rate limit; locB-live never restarted localization. Meanwhile Codex (from laptop) did: T5 audit (reports/20261022-codex-continuation-t5-audit.md) — current loc_trust.py sha 01a5d412 already has per-update latch (t5each); old rule = max deviation from window start, not abs-sum; replay g_motion_03/short_nav_01: 0 violations → STEP 1 DONE. AMCL live params confirmed defaults (alpha 0.2, z_hit=z_rand=0.5, sigma_hit 0.2). Physical reference (operator PS5, -physical-reference.md): 1 m mark → LIO 0.918 m; loop closure 0.061 m / 2.06°; single loop ≠ alpha calibration. Open anomaly: persistent T1_stale in trust while sources fresh. Next: user decisions (stack start; calibration protocol).
2026-10-22 22:1x: USER ORDER: (1) T5 fix if code confirms noise summation, (2) AMCL alphas from real LIO records (no arbitrary 10-200x cut), (3) persistence + verifiable restore via standard AMCL. READY must not rely on AMCL sigma alone but must not ignore global uncertainty. No custom localizer, no big model test suite; verify one change at a time on recordings then robot with operator go. locB-live told to stop at stable state (no localization restart). T5 debugger opus/high launched.
2026-10-22 22:0x: user go (no motion; pose visually correct in RViz) → locB-live (general-purpose opus/high): restart trust w/ pose_store, confirm_anchor, save; restart localization w/ seed_pose_file; measure seed→READY. Report reports/...-locB-live.md.
2026-10-22 21:5x: locB verify cycle 1 PASS (reports/...-locB-verify.md). Advisories A1-A5 (T4 rejects 0.2 m moves only 19/40 → operator visual check before confirm_anchor is the real guard). Next: user go to restart trust node (no motion) for on-robot relaunch check; then Step A (alphas) needs operator loop walk.
2026-10-22 21:4x: user OK → killed suspended param-gets 327106/327937. ros2 param dump on amcl hangs (CLI uses vendor DDS profile, stack uses dds_qazbot.xml). Live AMCL params from launch file (reports/amcl_live_launch_params_20261022.yaml): only frames/topics/max_beams 1440/OmniMotionModel → alpha1-5, z_*, laser model = Nav2 defaults CONFIRMED. locB verify resumed.
2026-10-22 21:3x: locB impl DONE (93 OK/1 skip), verify cycle 1 running (opus/high). locA-data NEEDS_DATA: candidate alphas (upper bounds) a1=.008 a2=.006 a3=.001 a4=.02 a5=.01; AMCL jitter ~0.03 m dominates, not LIO; losses 17:01/19:18 = cumulative T5 window sum; unmapped object ~1.6 m ahead flattens scan match. Live param get hung; suspended ros2 param get PIDs 327106/327937 in user's bash. Needs: operator loop walk (go) for alpha ground truth.
2026-10-22 21:0x: user 'fix it'. Parallel (no motion, no live changes): locB impl (general-purpose implementer role, opus/high: persist+reseed confirmed pose) + locA-data (robotics-debugger opus/high: live AMCL params read-once + offline bag alpha bounds). Then verifier on locB; live apply one component at a time with user go.
2026-10-22 20:5x: locpersist ROOT_CAUSE_FOUND: (1) AMCL alphas default 0.2 on LIO odom → particle spread, narrow virtual scan can't recover; (2) no pose persistence across restarts; (3) READY gated on AMCL sigma (re-pose resets 0.5). Fix plan A (alphas from measured LIO drift) → B (persist+reseed last confirmed pose via seed_pose.py) → C (READY = anchor + LIO drift bound). Agent adding .89 comparison. Next: user decisions; A needs read-only param get + drift data.
2026-10-22 20:2x: NEW subtask locpersist — user: localization keeps getting lost; need pose held through uninformative views via continuous odom + last confirmed pose. Diagnosis delegated robotics-debugger opus/high (read-only, no motion). Report reports/20261021-x2-nav2-e2e-dryrun-locpersist.md. Open: S3b cap 0.5→0.6 proposal awaits user OK.
2026-10-22 19:0x: /goal active (RViz 2D goal → Nav2 → goal → return). Owner: this session. Live: localization_v3_loopfix_01, trust_v3_07, adapter_live_01 (registered, unarmed), nav_full_dryrun_04 (RPP accel=10 live), coordinator latched (operator_stop). READY false (σ 0.48). Next: operator re-pose → READY → G-turn (yaw-only) → G-short → G-return, each with explicit go.
2026-10-22 16:4x: G1/G2 done; G3 walked ~1.2 m (overshoot); G4 no rotation. Root cause CONFIRMED (g3debug addendum): RPP rotateToHeading closed-loop on measured wz (+max_angular_accel·dt=0.05) below X2 turn threshold. Fix: RPP max_angular_accel = rotate_to_heading_angular_vel×controller_frequency; OPEN_LOOP smoother keeps ramp. Needs S3 rotation threshold (.89: 0.4 rad/s). S3 tool in prep (s3prep); auto mode now OFF; S3 user-approved.
2026-10-22 16:1x: G1/G2 blocked by session permission policy; awaiting user (permission or manual commands). Recorder ready: tools/record_motion.sh / stop_motion.sh / analyze_motion.py (self-tested synthetic). Coordinator accepts live driver without nav restart (goal_coordinator.py:80-83).
2026-10-22 15:5x: live dry-run goal with simplified trust + FAST-LIO fix: accepted, 1510 feedback, motion_ok 115/115 stable, progress-abort (expected). Next: ask user go for G1 (live adapter, no registration) + G2 (one ~/register); F1 not done → user decision on ROBOT_RADIUS for first ≤1 m goal.
2026-10-22 15:4x: simpletrust verify PASS (cycle 2); e2e nav_e2e_simpletrust_01 167/167 PASS. Next: operator relocalize → confirm_anchor → dry-run goal → physical G1 (live adapter, no registration) on explicit go.
2026-10-22 15:2x: FAST-LIO loopback fix live (sha 63903a5c, pinned 5,6, odom p50 31 ms p99 146 ms). Live stack: localization_v3_loopfix_01, trust_v3_07 (runs simpletrust cycle-2 code), adapter_dryrun_03 (live=false), nav_full_dryrun_04. Awaiting operator relocalization + simpletrust cycle-2 verify.
2026-10-22 16:xx: simpletrust verify cycle 1 FAIL (map->odom jump across stale TF gap not clearing READY) → fixes sent to implementer. lioapply (FAST-LIO loopback patch, user-approved) running; afterwards operator re-localizes; trust must be restarted again after simpletrust fix + cycle-2 verify.
2026-10-22 15:xx: parallel: liolat (opus/high: FAST-LIO one-scan wait, pin, adapter CPU) + simpletrust (opus/high: READY latch + MOTION_OK per user decision). Then verifier opus/high, restart trust/adapter live, e2e x2 with live down.
2026-10-22 14:5x: cpuprofile done (NEEDS_DATA for one-scan wait cause). Today latency odom p99 0.18-0.20 s, max 0.30-0.49; scan worst 0.40-0.46 vs T1 0.5. Main driver: FAST-LIO waits ~1 scan (~70 ms idle) + runqueue 17/44/85 ms. Awaiting user decision: mitigations (no CLI, taskset pin FAST-LIO) + real ≤1 m trial.
2026-10-22 14:3x: user chose option 1 — passive CPU/FAST-LIO latency profiling (robotics-debugger opus/high, cpuprofile) before real ≤1 m goal.
2026-10-22 14:2x: LIVE DRY-RUN on X2 PASSED functionally: CLI goal (live_dryrun_02/goal3) and operator RViz goal both accepted → plan → RPP cmds ≤0.3 m/s → adapter dry_run published → progress-checker ABORT (robot static, expected) → adapter idle. Rejections only when trust momentarily stale (FAST-LIO latency from CPU spikes, incl. ros2 CLI starts). readyrace fix applied (logging + test precondition); e2e 2-consecutive pending with live stack DOWN.
2026-10-22 13:5x: verify2 cycle 1 FAIL (verify_03 160/160 PASS; verify_04 goal-accept race at takeover_goal_A_accepted). Items 1-9 PASS. Fix+2 consecutive e2e → implementer opus/high (readyrace). Live: localization v3 (launch PID 26157) + trust (26843) running; awaiting operator pose estimate.
(prev) RESUMED 2026-10-22: VERIFY amcl fix (cycle 2/3) + VERIFY implA/2b with e2e (verify2), both robotics-verifier opus/high, parallel (unit-only vs e2e).
(prev) PAUSED by user 2026-10-21 ~22:30 (continue tomorrow). Amcl-fix verify cycle 1 FAIL; implementer applying fixes (may finish after pause — check reports/...-amclfresh.md tail).

## Task list
| # | Task | Status | Assigned subagent |
|---|------|--------|-------------------|
| 1 | Gap analysis vs 10 requirements + baseline test run + snapshot | DONE | robotics-debugger |
| 2 | Impl A: adapter dry-run shadow SM, coordinator, trust gaps, e2e with real adapter, stop test | DONE (e2e run03 158/158; run05 blocked by map replacement) |
| 2b | Re-derive nav2 map from NEW D1 first_map, rerun e2e | DONE (v2, e2e run08 160/160) | robotics-implementer |
| 3 | Live dry-run bring-up on X2 (new map) + graph/TF diff + RViz Nav2 Goal tool | DONE (running; awaiting operator pose estimate; no MC/cmd_vel publishers) |
| 3a | Phase B docs | DONE |
| 3d | Live localization + trust baseline/anchor | DONE (trusted) |
| 3e | Map v3 (erase X2 imprint) + live load_map + trust restart on v3 + new baseline/anchor | DONE (T8 correctly flagged v2 trust vs v3 map; trust restarted PID file trust_dryrun_02.pid) |
| 3f | Live dry-run goal | BLOCKED → root cause found: (1) amcl-stale-while-'moving' logic bug (LIO jitter), (2) real FAST-LIO output latency 0.24 s median / up to 2.8 s under load (gate correct) |
| 3g | Fix (1) amcl freshness = due-to-update rule + tests | DONE (trust 46→51); live AMCL update_min_d/a=0.25/0.2 confirmed by param get |
| 3h | Verify amcl fix | cycle 1 FAIL → fixes APPLIED (per-axis strict rule; rejected stamps ignored); trust 51→56, adapter 43, stop 7 OK; cycle 2 verify PASS (reports/...-verify-amclfresh-c2.md; non-blocking test gaps: pure-y, yaw=0.2 exact, cov None) | implementer role |
| 3b | F0 localization at rest (operator present) | TODO | orchestrator+operator |
| 3c | Real short goal + return-to-start — ONLY after explicit per-step user go | TODO | orchestrator+operator |
| 4 | Independent verification (code+tests, cycle 1) | INTERRUPTED by session restart; e2e verify_01 failed: bt_navigator compute_path_to_pose ack timeout (20 ms default) under load 11.8. Rerun AFTER phase C | robotics-verifier |

## Routing decisions
| Step | Agent | Model | Effort | Reason |
|------|-------|-------|--------|--------|
| 1 | robotics-debugger | opus | high | Safety-critical Nav2/TF/arbitration state establishment |
| 2b | general-purpose (implementer role) | opus | high | Map change affects fixtures/trust map identity |
| 3 | general-purpose (implementer role) | opus | high | Hardware DDS bring-up, safety-critical |
| 4 | robotics-verifier | opus | high | Safety-critical motion ownership verification |
| locB | general-purpose (implementer) | opus | high | Localization persistence, safety-relevant |
| locA-data | robotics-debugger | opus | high | AMCL/LIO noise analysis |
| locpersist | robotics-debugger | opus | high | Localization root cause (AMCL/LIO/trust), safety-relevant |
| 2 | general-purpose (robotics-implementer role; worktree isolation unavailable, non-git) | opus | high | Safety-critical motion-ownership state machine + e2e |

## Verification cycles
1/3

## Evidence / artifacts
- .orchestrator/reports/20261021-x2-nav2-e2e-dryrun-*.md

## Next action
TOMORROW (in order):
1. Read reports/20261021-x2-nav2-e2e-dryrun-amclfresh.md (cycle-1 fixes result). Run robotics-verifier opus/high (cycle 2/3) on per-axis rule + rejected-stamp fix; suites loc_trust/motion_adapter/stop.
2. Pending code verification of implA/2b (verify1 interrupted): rerun verifier incl. e2e domain 87 ONLY when live stack is down; investigate bt_navigator compute_path_to_pose ack timeout (default_server_timeout 20 ms) under load — measure, don't guess.
3. Live dry-run: start localization (map v3, portable FAST-LIO), trust (v3 map + sha f6032f75…), adapter live=false, nav_full allow_dry_run_driver=true. Operator: Pose Estimate; operator ≥2 m away; orchestrator: nomotion updates (few, not loops), baseline, anchor, dry-run goal 1 m ahead + return to anchor. Avoid ros2 CLI loops (CPU).
4. Real motion only with explicit per-step user go (MC ADD/ENABLE etc.).
Stack stopped cleanly 22:3x: all nav + localization children 'finished cleanly'; adapter, trust stopped.
(older) Wait trustflicker debugger → implementer fix (+unit test) → verifier → live re-baseline (operator ≥2 m away) → dry-run goal + return.
(older) Operator: RViz pose estimate + baseline + dry-run goal. Verifier e2e deferred until operator test done (CPU contention). Open: default_server_timeout 20 ms measure.
(older) Phase C resumed (live dry-run, leave running for operator RViz). Then verifier cycle 1 rerun (sequential to avoid CPU contention with live FAST-LIO). Then docs phase B.
(old) Impl A running. Baseline: adapter 37/37, trust 29/29, nav_e2e 73/73, launch_stagger 5/5 (env), control_geometry 1 pre-existing FAIL. Snapshot /home/agi/control_backend_backup_20261021_pre_e2e.tar.gz

## Session continuation command
```
claude "/robotics-task continue digital twin X2 from .orchestrator/reports/20261023-twin-handoff.md"
claude "/robotics-task continue 20261021-x2-nav2-e2e-dryrun from PLAN.md phase PAUSED-amclfix-verify"
```

## 2026-10-27 MC execution continuation
- Robot rebooted (uptime ~1h at 15:34); stack down; no motion sent.
- Running: general-purpose(implementer) opus/high — native-odom body limiter for step_cmd + passive state + dry-run bring-up. Report: reports/20261027-native-odom-limiter.md
- Then: robotics-verifier, then announce next trial; EVERY motion needs separate user "да/готово".
- Continuation: claude "/robotics-task continue MC execution tests from .orchestrator/PLAN.md section 2026-10-27"
- 2026-10-27 later: user approved IP .66→.72 in DDS XMLs. Running: verifier cycle2 (NaN fix), implementer (IP + dry-run stack + native-odom Nav2 design → reports/20261027-native-odom-nav2-design.md), debugger (LIO drift → reports/20261027-lio-drift-diagnosis.md). No motion.
