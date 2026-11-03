# 20261021-x2-nav2-e2e-dryrun — S3 step-command tool (PREPARATION ONLY)

Nothing was published on domain 0, the robot was not moved, no service on domain 0 was called.
The live stack on domain 0 (adapter live=true, recording g_motion_03) was only observed with `ps` (read-only).

## 1. Injection path (what is documented and why)

Read: X2_FIRST_PHYSICAL_TEST_PLAN.md (S0/S3: "test publisher into the private topic"), X2_MOTION_ADAPTER.md,
MOTION_ADAPTER_FIRST_TRIAL_20261022.md, x2_motion_adapter.py, goal_coordinator.py, g3debug §6–8.

- `goal_coordinator` forwards `<ns>/cmd_vel` → `<ns>/cmd_vel_authorized` only while a Nav2 goal is busy and the gate is
  open, and trips `command_publisher_count` if `<ns>/cmd_vel` has more than one publisher (the smoother). So injecting
  BEFORE the coordinator is impossible without a running Nav2 goal or disabling that check → not used.
- Path chosen (the plan's "private topic"): tool → `/qazbot_d1_map/cmd_vel_authorized` → real `x2_motion_adapter`.
  All hardware-facing checks stay active: trust verdict (≤0.6 s, true), map→odom freshness, MC state/body/FSM,
  foreign MC-publisher graph scan, `pnc`/foreign SDK source, operator_takeover latch, control_not_acquired,
  motion_armed, 0.15 s watchdog, zero burst, saturation to MC bounds, lateral refusal. No adapter parameter changed.
- The coordinator's admission is reproduced per step exactly as `goal_coordinator.goal()` does for a NEW goal:
  `localization_trust/arm_check` → `motion_adapter/arm_motion` → a NEW `localization_trust/trusted` message that is true.
- Exclusion of concurrent Nav2 goals: the tool calls `goal_coordinator/stop` first (coordinator latched → every new
  goal rejected) and leaves it latched; the operator resumes it explicitly afterwards.
- Additional tool-side aborts: other publisher on `cmd_vel_authorized` (only `goal_coordinator` tolerated),
  coordinator busy, adapter status stale (>1.5 s = 3 report periods), odometry stale (>0.5 s, loc_trust odom_max_age_s)
  when a motion limit is set.
- Never publishes to `/cmd_vel` or `/aima/*` (asserted in code); never registers; never resumes after a fault.
- Known, documented caveat (MOTION_ADAPTER_FIRST_TRIAL): this path bypasses `velocity_smoother` (no accel limit);
  the adapter has no slew limiter. For S3 this is the intent (step response from rest = the G3 rotation-deadlock
  question). Magnitudes are capped at Nav2's own caps (0.3 m/s, 0.5 rad/s; forward only).

No safety check had to be disabled.

## 2. Tool: `tests/s3/step_cmd.py`

- Args: `--rot a,b,..|--lin a,b,..`, `--rot-dur` (3), `--lin-dur` (2), `--pause` (≥1 s, default 1), `--stop-after-yaw-deg`,
  `--stop-after-dist` (mandatory with `--lin`), `--expect-domain` (required, refuses on mismatch before creating a node),
  `--expect-mode live|dry_run` (required), `--ns` (/qazbot_d1_map), `--odom-topic` (/qazbot_lio/odom_level), `--csv` (new file).
- 20 Hz (controller_frequency = adapter controller_hz). Zeros while arming, `--pause` s of zeros after every step
  (= between steps and at the end).
- Abort (zeros for `--pause`, then adapter `~/stop`, adapter LEFT latched): adapter latched (operator_takeover, …) or
  hard reason, untrusted, foreign MC publisher, foreign cmd publisher, coordinator busy, disarmed, stale status/odom.
- SIGINT/SIGTERM: zeros for `--pause` s (second SIGINT cuts it short), adapter `~/stop`, exit 130.
- Limits (odometry from the pose at the first step): remaining steps skipped, zeros, exit 3.
- Normal end: adapter `~/stop` + `~/resume` = disarm (idle, motion_armed=false, no MC packets: burst only if ACTIVE).
- Outputs: CSV per published command (t_unix for bag correlation, phase, step, vx, wz, adapter state/reason/armed,
  trusted, odom x/y/yaw, odom age, dist, dyaw); `<csv>.odom.csv`; `<csv>.summary.json` (per step: onset lag, steady
  actual over second half of the step, actual/commanded ratio, coast time/dist/yaw; velocity method and thresholds
  imported from tools/analyze_motion.py).
- Exit codes: 0 done, 2 aborted, 3 limit, 4 refused, 130 SIGINT.

Constants and their source: RATE 20 Hz (nav_control controller_frequency); MAX_LIN 0.3 / MAX_ROT 0.5 (nav_full /
nav_control defaults); EPS 0.005 (adapter Config.eps); TRUST 0.6 s (adapter/coordinator); ODOM 0.5 s (loc_trust);
STATUS 1.5 s (3 × 0.5 s adapter report period); HARD reasons = goal_coordinator.adapter() list.

## 3. Model test (domain 87)

Runner: `bash tests/s3/run_model_s3.sh NAME` (domain 87, localhost, unicast profile; model + REAL trust + REAL adapter
live=false + REAL coordinator; no Nav2; children under `nice -n 19 taskset -c 0-4,7` because the live domain-0 stack
with FAST-LIO on cores 5,6 and a recording was running; load average 13–18 on 8 cores).

Runs: `tests/integration_results/s3_step_cmd_model_01` (6/10 then crash), `s3_step_cmd_model_02` (6/13 then crash).
**Result: INCOMPLETE — nominal path, latch abort, untrusted abort and SIGINT zeros were NOT demonstrated.**
- PASS: trust baseline/anchor; no publisher on /aima/mc/locomotion/velocity or /cmd_vel; C0 wrong domain refused
  (exit 4, no node); pause rows all zero; no adapter motion.
- FAIL (environment, not tool path): in both runs the first `localization_trust/arm_check` took 5–6 s (normal on
  Jetson ≈0.9 s); during it the trust verdict went stale (>0.6 s; trust node starved under nice 19 + load 18).
  Run 01: tool aborted `localization_untrusted` (fixed afterwards: trust freshness is not checked while an arm call is
  pending, mirroring goal_coordinator.goal(); a NEW true verdict is required after it). Run 02: the REAL adapter
  refused `arm_motion: localization_untrusted` → tool aborted (exit 2), 1 s zeros, adapter `~/stop` (latched
  stop_requested, published=0), coordinator latched operator_stop. I.e. the refusal chain works, but no step ran.
- Harness bug found: after an abort in C1 it waits for idle/unarmed and raises instead of continuing (cases C2–C7 not
  reached). Per coordinator instruction no further reruns.
- Remaining to show (rerun when domain 0 is idle, without contention): C1 rot path/zeros, C2 limit, C3 takeover latch,
  C4 untrusted, C5 SIGINT, C6 refuse-latched, C7 foreign cmd publisher.
- Real-robot implication: if arm_check on the Jetson is slow under load, the adapter refuses arming → tool aborts
  safely before any motion (same as a Nav2 goal would be rejected).

## 4. Operator session on domain 0 (NOT executed; each line needs an explicit "go")

Each line in its own terminal step; env.sh only inside `bash -c` (env.sh has `set -e`). Domain 0 = default.
Before: operator ready with PS5, ≥1.5 m free ahead, adapter live registered idle, trust true, no Nav2 goal active.

```bash
# 0. read-only snapshot
bash -c 'cd /home/agi/control_backend && source ws/src/navigation/env.sh && source ws/src/navigation/aimdk_env.sh && python3 ws/src/navigation/read_motion_state.py && python3 tests/nav_e2e/live_graph_snapshot.py tests/integration_results/S3_BEFORE.json'
# 1. rotation CCW
bash /home/agi/control_backend/tools/record_motion.sh s3_rot_01 600
bash -c 'cd /home/agi/control_backend && source ws/src/navigation/env.sh && exec python3 tests/s3/step_cmd.py --expect-domain 0 --expect-mode live --rot 0.1,0.2,0.3,0.4 --rot-dur 3 --stop-after-yaw-deg 30 --csv logs/s3/s3_rot_01.csv'
bash /home/agi/control_backend/tools/stop_motion.sh s3_rot_01
# 2. straight
bash /home/agi/control_backend/tools/record_motion.sh s3_lin_01 600
bash -c 'cd /home/agi/control_backend && source ws/src/navigation/env.sh && exec python3 tests/s3/step_cmd.py --expect-domain 0 --expect-mode live --lin 0.05,0.1,0.15,0.2 --lin-dur 2 --stop-after-dist 0.6 --csv logs/s3/s3_lin_01.csv'
bash /home/agi/control_backend/tools/stop_motion.sh s3_lin_01
# 3. analysis (offline, files only)
bash -c 'cd /home/agi/control_backend && source ws/src/navigation/env.sh && for b in s3_rot_01 s3_lin_01; do python3 tools/analyze_motion.py logs/bags/$b | tee logs/s3/$b.analysis.txt; cat logs/s3/$b.csv.summary.json; done'
# 4. only after review: re-enable Nav2 goals (tool leaves the coordinator latched)
bash -c 'cd /home/agi/control_backend && source ws/src/navigation/env.sh && ros2 service call /qazbot_d1_map/goal_coordinator/resume std_srvs/srv/Trigger "{}"'
```
Stop: PS5 stick first; Ctrl-C on the tool (zeros 1 s, adapter stop); `ros2 service call /qazbot_d1_map/motion_adapter/stop std_srvs/srv/Trigger '{}'`.
After an abort/SIGINT the adapter stays latched: review, then `.../motion_adapter/resume` manually before the next run.
Exit codes: 0 done, 3 limit reached (expected with yaw 30° / 0.6 m), 2 abort, 4 refused, 130 SIGINT.

What to extract:
- Rotation threshold: smallest wz whose onset in analyze_motion (`/qazbot_d1_map/cmd_vel_authorized onset k … -> motion
  after X s`, not "no motion in 5 s") and summary `steady_actual` clearly > 0.03 rad/s.
- Lag: onset lag (analyze "motion after X s"; summary `onset_lag_s`), rotation and forward separately.
- Forward threshold: same on s3_lin_01.
- actual/commanded ratio: summary `ratio` per step (odom_level, second half of each step); cross-check G3's 1.6–1.9×.
- Coast: analyze "stop k … settled after T s, travelled D m" + summary `coast_after_step` (time/dist/yaw). With 1 s pause
  and G3 coast 1.76 s, intermediate-step coasts overlap the next step: only the last executed step's coast is clean.
- Also: /aima/mc/locomotion/velocity equals the command (no saturation), MC source changes in /aima/mc/common/state.
Caveats: step input bypasses velocity_smoother by design; yaw limit 30° means only the first 1–2 rotation steps will
run (0.1×3 s = 17° nominal, ×1.6–1.9 ≈ 29–33°) — the 0.3/0.4 rad/s steps likely skipped; 0.6 m likely ends during 0.1–0.15 m/s.
