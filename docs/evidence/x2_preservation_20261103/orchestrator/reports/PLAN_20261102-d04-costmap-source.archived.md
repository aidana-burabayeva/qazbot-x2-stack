# PLAN — 20261102-d04-costmap-source (see TASK.md)
Previous: reports/PLAN_20261102-slam-stage1-mapping.archived.md (contains live nav session 20261102 notes, stop.sh path)

## Tasks
| # | Task | Status | Agent |
|---|---|---|---|
| P1 | Runtime costmap/voxel verification + installed hashes (read-only) | DONE (reports/20261102-d04-P1P2-debugger.md) | robotics-debugger opus/high |
| P2 | D-04 source evidence + AgiBot contract comparison + bag inventory | DONE: H1 mark/clear span mismatch (-58.8..+52 vs -46..+44.5), BT full clear = 10 s; no 11-02 cloud bags | robotics-debugger opus/high |
| P3 | Offline A/B/C(/D/E) replay harness + results | DONE: C chosen (reports/20261102-d04-P3P4-implementer.md) | robotics-implementer opus/high |
| P4 | Minimal candidate + regression suite (+ C++ scope) | DONE OFFLINE_VERIFIED: candidates/d04, tests/d04 54 pass | general-purpose as implementer opus/high |
| V  | Independent verification + replay on real static bag d04_static_bag_204153 (user-approved recording 20:41-20:46, 3.5 GB) | RUNNING cycle 1 | robotics-verifier opus/high |

## Routing
| Step | Agent | Model | Effort | Reason |
|---|---|---|---|---|
| P1+P2 | robotics-debugger | opus | high | live robot, Nav2/sensor geometry root cause |
| P3+P4 | general-purpose (implementer role; implementer needs git worktree) | opus | high | safety-relevant costmap observation model, architecture choice |

## Blockers / inputs
- Henki standard received → ~/.claude/standards/henki_ros2_best_practices.md (global mandatory rule).

## Continuation
Resume: "продолжи задачу 20261102-d04-costmap-source из .orchestrator/PLAN.md"

## Sequence (user /goal, autonomous offline)
D-04 (P1–P4 + verify) → D-05 offline model from existing LIVE-2 data → D-06 leg-odom check → S-01 trust uniqueness check → C++ level_bridge candidate → C++ motion adapter candidate (parity). Each: diagnose→fix→test→regress→verify. Hardware gates collected for user.

| V1 | robotics-verifier | opus | high | safety-critical costmap clearing change |
- Findings: low objects 0.25 m not detected by any variant (pre-existing gap → separate defect D-07 candidate). Stray ws/src/lio/__pycache__ from harness (harmless, unchanged source).
- 21:18 controlled static bag d04_static_bag_211835 (user "готово"). V1 resumed after 25-turn limit.
- 21:2x PARALLEL: D-05/D-06 offline diagnosis (robotics-debugger opus/high; safety-relevant motion model) → reports/20261102-d05-D1-debugger.md. C++ level_bridge held until V1 verdict (avoid rework + CPU contention with replay/colcon).
- D-05 D1 DONE (reports/20261102-d05-D1-debugger.md): MC nonlinear in LOW speed_mode: 1.61x @0.30, 0.93-0.99x @0.80; commands reach MC intact (H8 rejected). D-06 speed-dependent (0.885 @0.30). Option A NOT justified until calibration sweep 0.10..0.60 (~25 min, needs operator approval). Status HYPOTHESIS→root cause found, fix BLOCKED on hardware data.
- V1 PASS (candidate C, offline): A RPP-collision 87.6% vs C 1.3% on real bag 204153; C never lost persistent object. REQUIRED_FIXES (harness/docs): R1 live-matched R_lv in replay_d04.py; R2 qualify ROOT_CAUSE/SAFETY (unobserved-bearing marks still block — real stale cell +70.9°/0.37 m blocks RPP 100% in A and C → new item D-04b); R3 re-record protocol with live check (211835 had 0 pts <1 m → scenarios not covered).
- Sit/stand health 21:4x: all alive, tilt Δ0.24°, trust=false (needs initialpose), adapter LIVE idle (asked user to stop).
- NEXT (parallel): F1 fixes R1+R2+reports (general-purpose sonnet/medium, precise fixes); CPP1 C++ level_bridge candidate (general-purpose sonnet/high: standard rclcpp impl, parity-critical). Then V2 verifier opus/high.

## SESSION END 2026-11-02 ~22:0x (user: stop all, continue tomorrow)
- Stopped: F1 (docs/harness fixes) and CPP1 (C++ level_bridge) agents BEFORE they wrote reports. Partial dir ws/src/qazbot_level_bridge (36K, unverified, not built) — review or delete tomorrow.
- Robot stack stopped via docs/evidence/nav_20261102_live/stop.sh: owned leftovers [] (adapter, Nav2, trust, localization down).
- Pushed branch robot/feedback-d04-d05-20261102 (f1816fd) on origin: reports, harness tests/d04, candidate C, record script, ROBOT_TO_LAB_FEEDBACK entry. robot/stable untouched.
- TOMORROW order: (1) rerun F1 (R1 live R_lv in replay, R2 CANDIDATE.md + docs/engineering/D04_D05_CPP_REPORT.md + session doc evening section); (2) CPP1 C++ level_bridge (check partial dir first); (3) V2 verifier opus/high; (4) D-04b analysis (marks on never-observed bearings); (5) ask user: controlled re-recording with live in-FOV check; D-05 calibration sweep (motion); (6) S-01.
Resume: "продолжи задачу 20261102-d04-costmap-source из .orchestrator/PLAN.md"

## 2026-11-03 resume
- Robot stack DOWN (stopped 11-02) → CPU free for offline work. Parallel: F1 (general-purpose sonnet/medium), CPP1 (general-purpose sonnet/high; reuse/verify partial ws/src/qazbot_level_bridge), D4B D-04b diagnosis (robotics-debugger opus/high: safety, costmap/RPP semantics). Then V2 verifier opus/high on F1+CPP1.
- 14:05 robot had rebooted (~13:59). Vendor trim re-applied (user: "отключить всё что не нужно навигации"): aima em stop-app perception orbbec_camera hal_camera drp lx_webrtc_app → all success. qazbot-speech inactive. Kept MC, hal_sensor_orin, pnc, slam/map_manager/auto_integrated (D-03 list). Restart: aima em start-app <app>.
- F1 DONE (reports/20261103-d04-F1-implementer.md): R1 fit default, 59 tests pass; CANDIDATE.md, docs/engineering/D04_D05_CPP_REPORT.md, session doc evening section; pyc removed. Gap: real bags 204153/211835 not registered in BAGS → fit not exercised on real data (V2 to check).
- 14:2x External review of f1816fd (user paste): agree. Actions: rename level_bridge → qazbot_lidar_projector (C++ pkg; topics unchanged; running Python untouched); REPRO agent (sonnet/high): clean-checkout paths, real bags in BAGS + final summary/raw, self-contained recorder, REAL Nav2 ObstacleLayer test in domain 77, push to feedback branch. Remark E RESOLVED: all 73 ws/src files in running tree == origin/robot/stable (diffs were vs stale local branch). D-07 (low obstacles, band floor ~0.34 m) registered as separate safety blocker, not fixed with D-04.
- D-04b DONE (reports/20261103-d04b-debugger.md): blind cluster +62..87° 0.31-0.56 m marked during unrecorded manual relocation 19:29-20:32 (stack running), never observable from new pose (in-band max +54.4°); leg-odom standing drift ~1 cm/min moves it inward → real RPP block. Ranking: (1) operator-initiated local clear w/ visual confirmation, (2) D-06 standing drift, (3) side sensor, (4) CCW recovery rotation (motion). Proposed to user: clear local costmap tied to operator initialpose/relocalization (not per goal).
- 14:4x User: no auto clear on initialpose (operator procedure w/ visual check instead); D-04 + D-05 hardware protocols written: control_backend/docs/hardware/protocols/HW_PROTOCOLS_20261103.md. New track: controller comparison RPP/DWB/MPPI in real Nav2 Humble isolated (domain 78). Running: CPP1 (aa4b9…), REPRO (a0844…), CTRL (opus/high, a54a7…), DRIFT D-04b (opus/high, a23fa…), D05 const-cmd tool (sonnet/high, a365d…). Separate deploys: C++, D-04, controller, MC calibration never combined.
- CPP1 DONE + V2 PASS (reports/20261103-cpp-V2-verifier.md): qazbot_lidar_projector 30/30 tests, 100% core coverage, bins bit-identical, C++ 0.87 vs Py 4.72 ms p50, RSS 66 vs 102 MB. Advisories A1 (lio publish_tf yaml), A4 (assert explained-diff size), A5 (lethal replay, live R_lv, moving scenario before deploy). Commit separately to feedback branch AFTER REPRO agent pushes (shared worktree).
- DRIFT DONE (reports/20261103-d04b-drift.md): MC leg-odom constant left bias ~1 cm/min while STAND_DEFAULT (14 bags, 80 min); bridge passes through. Main blind-mark source = marked-never-cleared side sectors (5-10x vs exits). FAST-LIO as costmap odom rejected (latency p99 44-94 ms, loop-back). Operator relocation checklist in report. Launched ODOM-HOLD candidate (sonnet/high, a103e…): hold pose while standing, data-derived standing signal.
- D05 const-cmd tool DONE (reports/20261103-d05-cmd-source.md): tools/d05/const_cmd.py → /qazbot_d1_map/cmd_vel_authorized 20 Hz; 67 tests, domain-79 dry run 29/29. Needs user decision: manual arm_motion (adapter checks incl. trust, but skips GC admission) vs Nav2 goals with per-run desired_linear_vel. Step command (no ramp) → abrupt-stop distances. Pending independent verification.
- REPRO DONE 57ec656: clean-checkout paths, BAGS.md manifest, final replay 204153/211835 (first 40 s), self-contained recorder; REAL Nav2 costmap (domain 77): edge lethal A/C no-object 18/10, S9 20/10, S10 24/10, S6 present 23/15 with object cells 16/16 → matches emulator. Pushed a22e9f3 (C++ pkg, separate commit) + 408923c (protocols, d05 tool, d04b/drift reports). Waiting: CTRL compare (a54a7…), ODOM-HOLD (a103e…). User decision pending: D-05 command path A (manual arm) / B (Nav2 goals).
- ODOM-HOLD: candidate DONE (reports/20261103-odom-standing-hold.md), V PASS on code but deploy path FAIL (aimdk_msgs not importable via run_localization.sh → hold silently off). Fix cycle 1 running (sonnet/medium, ae309…): bridge-only wrapper, ERROR on import fail, offset-cap release (2×0.19 mm/s×t), post-deploy checks.
- ODOM-HOLD fix cycle 1 DONE (reports/20261103-odom-hold-F1.md): wrapper run_native_odom_bridge.sh (import checked OK by orchestrator), launch diff, ERROR on import fail, cap release 3.8e-4 m/s×t+1 mm, 16 tests. Pushed b67c1d5 (separate commit). Status OFFLINE_VERIFIED; deploy needs user OK + localization restart.
- 15:5x User: D-05 via Nav2 Goal (B); const_cmd (A) REJECTED. odom_hold not deployed until real-launch isolated test. No combined deploys. Running: CTRL finish (a54a7…), CPP integration (sonnet/high a8383…, domain 77), ODOM-HOLD launch test (sonnet/high a3937…, domain 80), D05 Nav2 calib path + minimal first pass (opus/high a7e9b…, domain 81). Protocol P-D04 updated with D-07 height ladder (0.15/0.25/0.35/0.45 m; 0.35 at 0.6/1.5 m).
- CTRL DONE (reports/20261103-controller-compare.md; tests/controllers): 24+12 runs real Nav2 1.1.18 domain 78. Nominal all reach (18-19 s), MPPI lowest jerk. Real obstacle ideal: stop dist RPP 0.38 / MPPI 0.11 / DWB 0.03 m (DWB rejected). With D-05 HYPOTHESIS MC model (1.66x, 0.4 s lag): RPP & DWB COLLIDE, MPPI 0.02 m → SAFETY FINDING: current live RPP not safe near obstacles until D-05 calibrated (model-based, unverified). Stale blind cell: all blocked (correct); with C clearing MPPI passed (1 seed). CPU/cycle RPP 2, DWB 11, MPPI 9 (29-34 near obstacles) ms. Verdict: MPPI after D-05 calibration + rerun, D-07 decision, D-04 C + D-06, tuning + dry run. Keep RPP.
- D05 NAV2-CALIB DONE (reports/20261103-d05-nav2-calib.md): speed_limit topic percentage ≤100 % only (absolute rejected: raised controller to 0.50), no GC/adapter change; tools/d05/nav2_calib 52 tests, domain-81 dry run 17/17 with real binaries; P-D05 rewritten (Run 1 = 0.30 m/s 4.4 m goal + GO/NO-GO). Points >0.30 need raising live max (user decision). Verifier running (opus/high a67da…).
- D05 calib V PASS (reports/20261103-d05-nav2-calib-V.md); A1 applied (live speed stop → operator visual, LIO >1.0 m/s post-run NO-GO).
- 18:1x rate-limit hit 17:xx; CPP integration + ODOM-HOLD launch resumed 18:11. Pushed 3ee109a (controllers eval) + 10cd0e6 (d05 nav2_calib + protocols).
- ODOM-HOLD launch test (reports/20261103-odom-hold-launch.md): launch/startup PASS, replay transitions PASS, faults PASS, SLOW DISPLACEMENT FAIL (cap resets per re-entry: 1 mm/s hides 10.5 mm/45 s, unbounded). Fix cycle 2 running (sonnet/high ab66b…): cumulative per-episode budget + latch, clean SIGINT/SIGTERM.
- CPP integration DONE (reports/20261103-cpp-integration.md): real costmap parity 107/107 static, 100/101 synthetic moving; p99 2.13 vs 7.81 ms; A1/A4 fixed; pushed 86b3ff8. OPEN: no real moving LIO-cloud bag; 3rd control costmap 0.65 agreement on moving data unexplained.
- 18:3x ODOM-HOLD F2 (reports/20261103-odom-hold-F2.md): cumulative budget implemented, 21 tests; still hides slow motion (0.5 mm/s: 35 mm before release; 1 mm/s never released, 26% hidden). Status → FAILED vs user requirement 'do not hide body motion'. Parked; redesign later (directional bias model or no hold). P-D04 LIVE: stack started 18:25 (loc+trust, nav dry, adapter dry_run mc_source rc, GC idle); waiting user initialpose for Nav2 activation; tools/d04/scan_sectors.py live aid.
- 18:30-18:37 P-D04 executed (operator self-timed; bag d04_static_bag_183015 cap 720 s; step/sector/event logs pd04_*_20261103.txt). Live monitor: person seen at R-edge 0.64@-54, L-edge 0.92-1.08@+44..47; boxes not obvious in 1 Hz monitor. Analysis agent (opus/high a4cab…). Stack still up (dry-run) for possible repeat.
- 19:15 bag 183015 closed (8.5 GB, 12 min). Boxes actual 21/45 cm. Analysis agent scope extended per user (per-position table, D-07 filter root cause). C++ vs Python on 183015: sonnet/high aecc0…, domain 76. odom_hold FAILED (no install), D-05 not run, MPPI offline.
- PD04 analysis DONE (reports/20261103-pd04-analysis.md): Person F PASS (A=C marking), person leaves: C PASS (cleared 2.4/4.9 s, 0 edge cells at end) A FAIL (39 edge cells) → D-04 reproduced live; empty PASS; no real obstacle wrongly cleared by C. L/R, S, 21 cm NOT EVALUABLE (not in raw). D-07: level_bridge.py:235 band floor 0.329+0.035 r m above floor (Z_MIN -0.65 rel IMU 0.979 m), 45 cm box at R cut by LiDAR vertical FOV beyond ±55°. Note: LiDAR raw min range 0.695 m → 0.7 m test positions at blind-range edge (likely why L/R unseen). Waiting C++ (aecc0…).
- PD04 C++ DONE (reports/20261103-pd04-cpp.md): 1199/1200 costmaps identical, p99 2.64 vs 10.5 ms, CPU 3.0 vs 15.8 %; 3/3579 scans missing in dual run (cause not isolated). Pushed f41dc85.

## Correction 2026-11-03 (operator statement; history above not rewritten)
Lines above that mention boxes in bag 183015 ("boxes not obvious", "Boxes actual 21/45 cm", "45 cm box at R", "21 cm NOT EVALUABLE") are withdrawn:
there were NO boxes during the recording (operator only stood/walked around; boxes placed later, unrecorded). Box scenarios = NOT PERFORMED.
D-07 stays OPEN safety blocker (code fact level_bridge.py:235; real-LiDAR box verification NOT PERFORMED). See reports/20261103-pd04-correction.md.
