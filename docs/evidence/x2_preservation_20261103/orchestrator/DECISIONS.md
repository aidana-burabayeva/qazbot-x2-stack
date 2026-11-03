# DECISIONS

<!-- Append one entry per decision, newest last. -->

## <YYYY-MM-DD> — <short title>
- Decision: <what was decided>
- Alternatives considered: <options and why rejected>
- Evidence: <reports, logs, docs, measurements>
- Impact: <affected packages / behavior>
- Reversibility: <how to revert, cost>

## 2026-10-21 — Keep goal coordinator as gate, not navigator
- Decision: Keep existing goal_coordinator (public NavigateToPose gate/proxy forwarding to the single internal bt_navigator). It plans/controls nothing; document explicitly as goal-ownership gate.
- Alternatives: RViz direct to bt_navigator with adapter cancelling via action client — rejected now: larger refactor, loses tested ownership/no-resume semantics.
- Evidence: reports/20261021-x2-nav2-e2e-dryrun-debugger.md (R4).
- Impact: docs must state exactly one navigator/planner/controller.
- Reversibility: remap RViz goal to internal action; low cost.

## 2026-10-21 — Dry-run adapter must run full shadow state machine
- Decision: in dry_run the adapter performs arm/latch/takeover logic identically to live except no MC publisher, no flock-on-MC, no registration; coordinator accepts explicit dry-run driver.
- Evidence: debugger G1 (dry_run rejected every goal; no latch/takeover in dry_run).

## 2026-10-21 — Switch to new D1 first_map
- Decision (user): continue on the new D1 first_map (saved 2026-10-21T18:07+05, synced 21:11); regenerate nav2-derived map and map identity.
- Impact: e2e fixtures goals, trust expected map hash, localization launch default map; old map only in snapshot tar.

## 2026-10-21 — Operator consent for real navigation trial (staged)
- User: may start trying navigation on real robot, operator holds it; task = go to a goal and return to start ("back to start"); "главное дай мне знать".
- Interpretation: consent is staged. Before EACH hardware step (live adapter start, MC ADD/ENABLE, each goal) orchestrator must notify user and wait for explicit "go". Order: 2b → live dry-run on HW DDS + RViz Nav2 Goal → F0 localization at rest → short live goal (≤1 m) → goal + return-to-start.

## 2026-10-21 — Live localization keeps the portable FAST-LIO binary
- Decision: hardware localization (phase C) runs candidates/fast_lio_portable fastlio_mapping (sha 4097314b…), not the launch default runtime/fastlio_mapping (87040c81…).
- Evidence: same binary as the previously used hardware stack (PID 284182, debugger report §1); runtime_logging.patch only; replay equality (logs/replay_portable_01/comparison.json max diff 0.0); clean SIGINT exit.
- Impact: docs (phase B) must name both binaries; launch default unchanged.
- Reversibility: relaunch localization with default lio_binary (robot standing still).

## 2026-10-21 — Operator localizes; orchestrator runs trust calibration and goals
- User must stay next to robot (fall prevention), cannot run commands. User sets 2D Pose Estimate in RViz; orchestrator then runs baseline_start/finish, confirm_anchor, and sends goals in verified free space (map matches reality per user).
- Dry-run goals: no further approval needed. Real motion (live adapter, MC ADD/ENABLE): still requires explicit per-step "go" from user.

## 2026-10-21 — Map v3: erase X2 self-imprint
- Live F0-ish: AMCL converged σxy 0.155 m, σyaw 0.043 rad, scan baseline 0.673, anchor (-7.361, 6.053, -1.294). Trust=true.
- Robot pose lies inside an occupied cluster; user confirmed X2 stood there during D1 mapping → self-imprint.
- Decision: derive first_map_nav2_v3 = v2 + documented, reproducible erase of that component (manifest records it). D1 source map untouched. Tests stay on v2.

## 2026-10-21 22:2x — TEMPORARY relaxed trust params, DRY-RUN ONLY (user request)
- User asked to temporarily bypass trust flicker to test Nav2 in dry-run. Live evidence: scan_score 0.36–0.52 vs floor 0.534 (baseline 0.668×0.8), T1 odom/scan/tf staleness bursts at load avg 25–45.
- Applied: trust node restarted with hit_baseline_frac=0.5, scan_max_age_s=1.0, odom_max_age_s=1.0, tf_max_age_s=2.0 (log trust_dryrun_03). All other checks unchanged. NOT valid for real motion; must restart with defaults before any live adapter step. Root cause analysis running (reports/...-trustflicker.md).
- 22:2x: relaxed-trust bypass blocked by the session permission policy (Security Weaken) before anchor was confirmed; trust node restored to DEFAULT params (trust_dryrun_04). No goal ran with relaxed params. Awaiting user decision.
- User decision: option 3 — wait for root-cause fix of trust flicker (agrees operator occlusion is plausible). No bypass.

## 2026-10-22 — Simplify Localization Trust (user architecture decision)
- User: operator localizes once; AMCL maintains/refines during motion; normal operation must not block navigation; only truly dangerous data loss stops the robot and requests operator.
- Decision: split into (A) READY gate (one-time, at operator localization / before accepting goals): operator pose + AMCL converged (cov, particles), scan-to-map agreement at READY, map identity. (B) Continuous MOTION SAFETY: fresh odom/scan/map->odom TF, confirmed global pose jump, map mismatch, MC/adapter faults (adapter unchanged). Continuous T2/T3/T4/T6/T7 become diagnostics (status/degraded), not motion stops. Stationary robot never loses READY due to absent amcl_pose.
- Recovery semantics: transient freshness loss → safe stop + goal aborted; new goal allowed once fresh again if READY holds; pose jump / map mismatch / prolonged loss → READY lost → operator relocalization. Old goal never resumed (unchanged).
- Rejected: keeping all T0–T8 as per-cycle stops (false refusals on real X2, uncalibrated, CPU).

## 2026-10-22 — Fewer model tests; fast path to physical trials (user)
- Model e2e: ONE real-chain run after simpletrust (not 2 consecutive), unless it fails on a safety-relevant case.
- Must-keep gates before physical motion (fall / uncontrolled motion risk): adapter live path unit tests; stop on data loss; cancel → zero; PS5 takeover latch; no resume of old goal; no foreign publishers; MC source behaviour verified ON ROBOT at first live step (registration, takeover by PS5, stop service) with operator holding robot.

## 2026-10-22 — FAST-LIO lidar loop-back time_buffer bug (pending user OK)
- Found (liolat): on lidar stamp loop-back, lidar_buffer cleared but time_buffer not → all later scans processed with previous scan's start time: odom/TF stamped ~100 ms early, IMU deskew window off by 100 ms. Live since 14:10:21. Replay: 23 ms → 126 ms after one injected back-stamped cloud; patched stays 30 ms; 609/609 identical poses in normal replay.
- Proposed: candidates/fast_lio_portable/lidar_loopback_time_buffer.patch (rebuild + restart FAST-LIO → operator relocalization). Awaiting user OK.
- Applied live: taskset FAST-LIO PID 26189 → CPUs 5,6 (odom p99 0.400→0.151 s). Opt-in launch arg lio_cpus.
- Simplified trust running live (trust_v3_06): READY+MOTION_OK true stable (66/66 samples).

## 2026-10-22 ~16:00 — User GO for G1 + G2 (no motion)
- User "да": G1 live adapter without registration; G2 one ~/register (ADD sdk_qazbot_nav2 prio 30). No goals. G3 needs separate go.
- User asks to record in parallel during motion everything needed for robot_radius (F1 dynamic envelope) and other parameters.
- G1/G2 delegation BLOCKED by session permission policy (Real-World Transactions). Not attempted via other routes. Awaiting user: add permission or run steps manually. Recorder prep (passive) proceeding.

## 2026-10-22 16:4x — G1 + G2 executed (user go; auto mode off)
- G1: adapter_live_01 live=True, registration not_requested, mc_source rc, MC publishers unchanged (4 vendor).
- G2: one ~/register → "SDK source registered", status registration=registered, motion_armed=false, published=0, mc_source rc; motion_adapter now has a PUBLISHER endpoint on /aima/mc/locomotion/velocity (expected after registration). Evidence tests/integration_results/g1g2/.
- Recorder g_motion_01 started (bag 19 topics + joints 50 Hz).

## 2026-10-22 16:47 — G3 attempt 1: ABORTED, no progress
- Goal accepted; RPP "collision ahead" ×6 within 0.65 s → "Controller patience exceeded" → ABORTED. Adapter published 33 msgs (presumably zero/near-zero), now idle no_command; MC input_source switched to sdk_qazbot_nav2 (our source) and stays; motion_armed true.
- Likely cause: operator holding the robot is inside the local costmap obstacle layer (scan, obstacle_max_range 3.0) on the forward path. To verify from bag g_motion_01 (scan, cmd_vel_authorized).

## 2026-10-22 16:3x — G3/G4 findings; S3 go
- G3: robot walked ~1.2 m (overshoot 0.19 m, yaw off 0.59 rad); progress checker abort because RPP in-place rotate stuck at 0.05 rad/s (accel-limited from measured odom ≈0) below X2 rotation threshold. Actual speed 1.6–1.9× command; coast 0.14 m/1.76 s. Attempt-1 abort: stale costmap ghost (person) not cleared (85% inf beams).
- G4 back-to-start: robot did not rotate at all (operator) → progress abort. Same root cause (pending bag confirmation).
- User GO for S3. Execution blocked by auto-mode classifier (Real-World Transactions) → tool prepared (s3prep); execution needs auto mode off.

## 2026-10-22 19:0x — Single owner: this session (agi-f9); goal set by user
- User: this session owns the robot; peer session agi-1b must not command it. agi-1b results adopted: RPP max_angular_accel = wmax×controller_hz (=10) applied live; docs/RPP_MC_ALIGNMENT_20261022.md.
- Reference code (local_control_system_py, robot_driver, azat_ws) lives on the user's PC, not reachable from .66; agi-1b reviewed it (principles only, see doc).
- Next: operator re-poses in RViz → nomotion updates → confirm_anchor (READY) → [go] Nav2 yaw-only goal (+20°) to accept RPP fix → [go] short goal ≤0.5 m → [go] return to START. Overshoot/stop latency: measure from these runs before changing approach params (one component at a time).

## 2026-10-22 19:4x — Cycle 2: forward OK, return turn fails at 0.5 rad/s
- Goal 0.5 m SUCCEEDED (overshoot 0.14 m). Return-to-START: RPP/smoother/authorized all -0.5 rad/s for 13 s (after an initial +0.5→-0.5 sign flip near 180° heading error); robot yawed only -85→-93° then stopped turning → progress abort. Evidence logs/bags/cycle2_01.
- Earlier yaw-only goal (+20°, CCW) succeeded at the same cap. Pad: -1.0 → -82° in 1.1 s; 0.3–0.72 → slow/jumpy. Hypothesis: sustained in-place turn needs |wz| above 0.5 (esp. CW); needs S3b measurement 0.6/0.8/1.0 both directions before any cap change.

## 2026-10-22 19:5x — S3b left + clock step incident
- S3b left: wz 0.6 → 45° in 2.3 s (steady ≈0.29 rad/s, onset 0.01 s). Right side not measured yet.
- 19:48:41 vendor PTP: "clock jumped backward … SYNCHRONIZATION_FAULT"; lidar/LIO stamps ran ~3 s ahead of CLOCK_REALTIME for minutes, then returned. level_bridge stopped publishing odom_level/scan (stamp-keyed caches held now-future keys) and ignored SIGINT (killed -9 at restart). Trust correctly withheld MOTION_OK.
- Fix: level_bridge clears pose cache + twist baseline when odometry stamp goes back (mirrors FAST-LIO loop-back fix). Backup level_bridge.py.bak_20261022_pre_loopback. Localization restarted (localization_v3_loopfix_02); operator re-localization needed.
- S3b right: wz -0.6 → -45.9° in 1.65 s (steady -0.92 rad/s, onset 0.35 s). Left 0.6 → 45° in 2.3 s (0.29 rad/s). At 0.5 sustained CW turn failed (cycle2 return, 8° in 13 s). Measured: 0.6 is the lowest tested magnitude that turns reliably both ways. Proposal: max_angular / rotate_to_heading_angular_vel 0.5→0.6 (needs user OK).

## 2026-10-22 20:3x — Reference code access (user)
- azat_ws / local_control_system_py / robot_driver stay on user's PC; will NOT be copied to .66. Ask user targeted questions about their logic when needed.

## 2026-10-22 22:1x — Localization fix order (user)
- Order: T5 noise-sum fix → AMCL alphas from real FAST-LIO records (no arbitrary reduction) → persistence/restore via standard AMCL initialpose (locB code already exists, live test paused).
- READY: not solely AMCL sigma, but global uncertainty must remain a factor.

## 2026-10-23 — Calibration without operator marks (user)
- Operator holds robot only; orchestrator commands motions (each with separate go) and judges accuracy.
- Proposed independent reference: registration of stationary lidar clouds before/after each leg / loop (not floor marks).

## 2026-10-27 — Vendor AGIBOT localization branch CLOSED (user decision)
- Evidence: reports/20261027-roadmap-v12-V0V1.md — Localization/Relocalization/Mapping/MapManager "license denied" every boot 20–27.10, incl. 23.10 license.
- User: license will NOT be requested. Roadmap §0.6 vendor comparison and P4.5 vendor row → NO-GO, permanently.
- Path: D1 map → FAST-LIO → level_bridge scan → AMCL → Trust → Nav2 (roadmap §0.5). No vendor map_id/PNC dependency.

## 2026-10-27 — Trust policy go2 kept (user), accelerated nav track
- User: keep go2 trust policy despite V2-amcl AMBIGUOUS finding (motion_ok with σxy 0.4-0.5 m). Accelerate to RViz-goal navigation; motion tests allowed with per-step go.

## 2026-10-27 — user confirmations
- 16:16 overwrite of ws/src/navigation (trust, coordinator, launch) + candidates/docs/tests = user's copy from laptop twin → these versions are canonical (twin-validated). loc_trust tests expecting 'strict' default are stale vs go2 default.
- M1 goal tolerance xy 0.20 m (coast 0.14 m + drift); stateful goal checker. Roadmap §P4.3 allows moderate arrival error.

## 2026-10-28 20261028-scan-latency-rviz-goal
- User priority change: working navigation over precision; localization may drift but must not jump. Asked to trust odometry more than map.
- Finding: AMCL uses Nav2 defaults (alpha1-4=0.2, likelihood_field z_hit=0.5/z_rand=0.5, update_min_d=0.25, recovery_alpha=0 i.e. no random-particle jumps) plus max_beams=1440 (default 60). No explicit "50/50" weight exists; 1440 beams makes the laser correction overconfident and CPU-heavy.
- Proposed (pending P1 result, applied after measurement so P1 stays controlled): max_beams 1440->60, alpha1-4 0.2->0.1, keep recovery_alpha=0; T1/T5/collision unchanged.
- 2026-10-28 operator answers: (1) fix freshness directly on X2 with backup, skip PC/DT first; (2) MC/Motion Adapter/Goal Coordinator + ONE short RViz goal (1-2 m) authorized AFTER fresh operator initialpose, Trust READY and operator confirms RC ready; (3) if T1 still blocks after fix: report and propose a solution even if it breaks AGENTS.md rules (do not apply without operator OK).
- 2026-10-29 operator proposals reviewed (senior view): (1) MC min rotation speed calibration + deadband compensation: APPROVED (needs live calibration run with operator OK). (2) never plan through obstacles inside footprint: already enforced by global costmap (lethal/inscribed + polygon); real gap = RPP straight-to-carrot collision projection vs curved path -> evaluate MPPI controller (trajectory sampling, obstacle critics) or shorter projection. (3) "start walking slowly into a doubtful gap while confidence evolves": REJECTED as stated (moving toward potential collision with 0.14-0.3 m coasting is unsafe); reformulated: cost-regulated speed near obstacles (RPP use_cost_regulated_linear_velocity_scaling / MPPI critics) + faster replan when blocked (failure_tolerance 10 -> ~3-4 s with 1 Hz replanning and global obstacle layer). (4) passage width: passability threshold is robot WIDTH (+margin) not radius; handled by polygon footprint + inflation in planner.

## 2026-10-30 P0 skill discovery from deployment cwd
Fresh session in /home/agi/control_backend does not list /robot-sync-validate (project skills load from cwd/.claude, checkout is a child dir). Chosen: user-level symlink ~/.claude/skills/robot-sync-validate -> <checkout>/.claude/skills/robot-sync-validate (single Git source, no copy, no write into deployment root). Rejected: copy (divergence), .claude/ inside deployment root (touches production tree), plugin marketplace (more moving parts; documented as future option). Symlink must be repointed on checkout move (MIGRATION_PLAN).
- 2026-10-30 Recovery costmap clearing: BT RecoveryNode clears LOCAL+GLOBAL after 10 s blocked. Static map (D1) is NOT erased (StaticLayer reapplies on reset); only live lidar marks. Reason: person trails out of FOV (+-45 deg, <0.45 m) are never cleared by bounded I1 clearing; global trails previously forced detours/paths behind robot. Risk: real non-map obstacles out of FOV are forgotten until re-seen; mitigated by forward-only motion. Alternative: local-only (safer, but stale global trails can force detours). Operator: test and document both; permanent fix = time decay (STVL). Pending A/B on robot.
- 2026-10-30 Rotation direction near obstacles (operator: robot turned toward chair side while other side free; proposal: probe left/right by pelvis-only turn, then choose). Senior view: pelvis "look" probe REJECTED (adds time, waist not controllable via our MC API, lidar FOV ±45 deg still narrow; side knowledge already exists in local costmap memory). APPROVED approach: choose in-place rotation direction by swept-footprint check on the local costmap (padded polygon, e.g. +5 cm), both directions; prefer collision-free/lower-cost arc even if longer (bounded extra angle); if both collide -> don't rotate, report/recover. Implement in final-alignment controller (candidates/nav2_pose_refine latch plugin) or get it natively from MPPI (trajectory sampling with obstacle critics). Note conflict: side obstacles are only known from memory -> clearing local costmap on recovery erases that memory; decay (STVL) preferable to full clear.
- 18:5x (robot clock) Operator declined external ground-truth measurements -> GROUND_TRUTH_UNAVAILABLE for GT-dependent metrics.
