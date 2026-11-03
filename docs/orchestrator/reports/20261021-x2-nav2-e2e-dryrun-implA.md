# 20261021-x2-nav2-e2e-dryrun — Implementation phase A

Status: DONE (code + tests); final re-run BLOCKED by external deletion of ws/src/maps/first_map_nav2_v1 (see Final runs). controller_dry 19/19.
Repo: /home/agi/control_backend (not git). Snapshot: /home/agi/control_backend_backup_20261021_pre_e2e.tar.gz.
Per-file backups: *.bak_20261021_pre_shadow (adapter, coordinator, nav_full), *.bak_20261021_pre_gaps (loc_trust, trust node),
*.bak_20261021 (sim_robot, harness, run_dry.sh, run_model.sh, ros_e2e_synthetic).
Nothing ran on hardware DDS. All ROS runs used domain 87 with ROS_LOCALHOST_ONLY=1 and dds_domain87_unicast.xml. The running
localization.launch was not touched. There is no MC velocity publisher, no registration and no /cmd_vel publisher.

## Changed files
| File | Change |
|---|---|
| ws/src/navigation/x2_motion_adapter.py | G1/G2: dry run now runs the same IDLE/ACTIVE/LATCHED machine as live (a "shadow"). Faults and lateral commands latch, the zero burst is mirrored to ~/dry_run_cmd, `~/arm_motion` works through the explicit dry-run driver path (`registration=='dry_run'`), `motion_armed` is required in both modes, and status adds `mode`/`dry_run`. The only intended divergence: an unregistered source cannot be selected by MC, so in dry run there is no `control_not_acquired`. The MC source seen at activation (or our own source name) is accepted; any other source appearing while ACTIVE latches `operator_takeover`. The MC publisher is only created in `_registration_result`, which now raises in dry run. MC publishing is gated on `cfg.live and pub`. Lock and registration client stay live-only. The live path is unchanged (`test_live_still_requires_acquisition`). |
| ws/src/navigation/goal_coordinator.py | New param `allow_dry_run_driver` (default false). It accepts a driver only when live is False, mode is dry_run, registration is dry_run and the driver is ready. Status gains `driver_mode`. Live acceptance is unchanged. |
| ws/src/navigation/nav_full.launch.py | New launch arg `allow_dry_run_driver` (default false, validated true/false) passed to the coordinator. |
| ws/src/navigation/loc_trust.py | `T1_particle_cloud_stale` (while moving, like amcl_pose). `T1_tf_frozen` (odom->base TF bit-identical over `frozen_window_s` while the odom message moves; params marked EXPERIMENTAL). `T8_map_unseen`/`T8_map_mismatch` plus `map_identity()` (resolution, origin, size, occupied cells), with `require_map_match` defaulting to true. `degenerate_budget_m` is documented as EXPERIMENTAL with default 0.0 kept and `EXPERIMENTAL_DEGENERATE_BUDGET_M = 2.0` as an explicit opt-in. T4 now scores the scan at its own pose (`odom_at_scan`). The docstring notes that standing AMCL liveness is the map->odom TF. |
| ws/src/navigation/localization_trust_node.py | Subscribes to `map` (transient local) for T8. Optional `expected_map_sha256` param; the status reports `map_sha256`, `map_ok` and `experimental`. The cloud stamp is validated through SourceStamp. The odom message pose feeds the frozen-TF check. The odom TF is interpolated at the scan source stamp. Services run in a separate callback group on a MultiThreadedExecutor(2) and the node shuts down cleanly. |
| tests/motion_adapter/test_dry_run_shadow.py (new, 6) | Dry-run state sequence equals live for the 3 refusals. Takeover latch, release, resume and fresh command. Hardware-like idle source rc. Live still requires acquisition. Stop. Idle refusals. |
| tests/motion_adapter/test_adapter_core.py, test_new_faults.py | 5 old dry-run tests superseded and renamed; mapping below. Count unchanged at 37. |
| tests/loc_trust/test_trust_gaps.py (new, 17) | AMCL loss standing and moving, cloud staleness, frozen TF (4), map identity (4), experimental budget (2), anchor vs FAST-LIO scale drift, no silent recovery, scan-time alignment. |
| tests/loc_trust/test_loc_trust.py | The `make_inputs` helper passes `map_ok=''` and `cloud_stamp=t` (served map verified). The tests themselves are unchanged. |
| tests/loc_trust/ros_e2e_synthetic.py | The feeder publishes the served map so T8 can pass. Not re-run (it is domain 88). |
| tests/nav_e2e/stop_criterion.py, test_stop_criterion.py (new, 7) | New stop criterion; see section R7. |
| tests/controller_dry/harness.py | `tail_zero` (old :104-105) and the 3 "ends at zero" checks are replaced by `stop_check` with `decel=DEC`, because the smoother output is ungated there. `run()` records the terminal time, waits for the model to rest and observes a 1 s window. |
| tests/controller_dry/run_dry.sh | Now forced to domain 87, localhost and the unicast profile. Previously it used the shell's domain with the hardware profile. |
| tests/controller_dry/sim_robot.py | Hooks `map_odom`, `publish_map_odom` and `wall_half` (optional 5th reset value). The defaults keep the old behaviour. |
| tests/nav_e2e/loc_model_fixture.py (new) | Model input is `<ns>/motion_adapter/dry_run_cmd`. Provides a D1 ray-cast scan with an optional box obstacle, emulated AMCL (amcl_pose, particle_cloud, map->odom with an offset), the served map, a FAKE McCommonState in domain 87 only (refused otherwise), `<ns>/model_truth`, and toggles. |
| tests/nav_e2e/run_suite_real.py (new) | Full chain with the REAL adapter (live=false) and the REAL trust node. Cases are listed below. |
| tests/nav_e2e/run_model.sh | Runs the real-chain suite by default; `--legacy NAME` runs the old suite. Sources aimdk_env.sh, which provides only message types. |

## Superseded adapter tests (explicit mapping)
- DryRun.test_dry_run_reports_but_never_enters_live_states -> test_dry_run_runs_shadow_states
- DryRun.test_dry_run_sends_no_zero_burst -> test_dry_run_mirrors_live_zero_burst
- DryRun.test_dry_run_does_not_latch_on_faults -> test_dry_run_latches_on_faults_like_live
- TrustAndOwnership.test_dry_run_reports_but_never_latches -> test_dry_run_foreign_publisher_latches
- NewFaults.test_dry_run_reports_each_fault_without_mc_output -> test_dry_run_refuses_each_fault_like_live

## Unit test counts (log: tests/integration_results/implA_unit_20261021.log)
| Suite | Before | After |
|---|---|---|
| adapter (core, early_takeover, new_faults [+ dry_run_shadow]) | 37 OK | 37 + 6 = 43 OK |
| trust (loc_trust, source_failures [+ trust_gaps]) | 29 OK | 29 + 17 = 46 OK |
| stop_criterion (new) | — | 7 OK |
| launch_stagger (QAZBOT_SMAC_PREFIX) | 5 OK | 5 OK |
| footprint polygon + envelope_rest | 4 + 1 OK | 4 + 1 OK |

The first launch_stagger run gave an ERROR because `setup()` was called without declared args. It was fixed with `LaunchConfiguration('allow_dry_run_driver', default='false')`.

## nav_e2e (real chain), run_model.sh -> run_suite_real.py
- nav_e2e_real_dryrun_01: 92 checks passed, then the odom-loss case failed in the harness: the harness observed the model through `<ns>/odom`, which the test had cut. Fixed by observing the model via `<ns>/model_truth`.
- nav_e2e_real_dryrun_02: 101 checks passed, then amcl_loss expected an adapter latch. Actual behaviour: Nav2 loses map->odom first (its TF tolerance is shorter than the 1 s trust/adapter TF age) and aborts, the adapter ends normally (IDLE), and the coordinator does not latch. This is safe: the goal is terminal and cannot resume. The expectation was changed to record which component detected the loss and keep all no-motion and no-revival checks.
- **nav_e2e_real_dryrun_03: 158/158 passed, rc=0, shutdown 0 for all, abnormal_children 0.** The restart's first group also had 0 abnormal children this time; the defect is intermittent and recorded under known_defects.
- run 04 (final, adds the obstacle safe-termination check) and the legacy run: see the end of this report.

Cases covered (each with the stop criterion and, on success, the model pose against the goal-checker tolerances xy 0.10 / yaw 0.10):
- straight 1.5 m; rotate in place; already at goal (no motion).
- repeat goal: re-send while active is rejected; the same goal after success gives no motion.
- cancel (status 5; adapter idle, not latched).
- obstacle (unmapped 0.6 m box, local costmap only): ABORTED by Nav2, clearance > 0.464 m, adapter idle.
- unreachable, 4 out-of-map goals, occupied goal: aborted with no motion.
- RViz goal_pose.
- **PS5 takeover through the real adapter** (fake McCommonState switches to rc): Goal A ends 5/6, the internal nav2 goal is terminal, the adapter latches `operator_takeover`, and it stays latched after release. A new goal is refused while latched. Resume alone gives no motion, A is not revived, and Goal B succeeds with a pose check.
- Through the real trust node: scan loss, odom loss, AMCL loss while moving, AMCL loss while standing (T1_tf_stale, goal rejected).
- False localization: wrong initial pose (untrusted, goal rejected, no motion) and pose jump while driving (abort, latched after recovery, old goal rejected before resume, resume alone gives no motion, A not revived, Goal B succeeds).
- Operator stop.
- Driver not ready (MC fsm SAFE gives rejection) and driver loss (MC state stale gives latch and abort; recovery stays latched; explicit resume; new goal).
- Nav2 restart: controller killed mid-goal, the group exits, no motion afterwards, the adapter is not active, the restarted Nav2 has no cached goal, a new UUID goal succeeds with a pose check.
- Graph checks: no MC velocity publisher, no SetMcInputSource client, no owner lock fd in the adapter, no global /cmd_vel publisher.

Legacy -> real mapping: every legacy check has a counterpart, except that `trust_loss_*` (an injected Bool) is now covered by pose_jump / scan / odom / AMCL losses through the real trust node, and `adapter_takeover_*` (injected JSON) is now the real PS5 takeover.

## R7 Stop criterion
Implemented in tests/nav_e2e/stop_criterion.py:
- terminal or cancel status observed;
- last command value and time recorded;
- no nonzero model-input command after terminal plus one control period (0.05 s);
- model |v|,|w| < 0.01 and pose and yaw drift < 5 mm over a 1 s window;
- the same check after recovery (`no_remotion`).

Trailing zeros are not required. The controller_dry harness checks smoother output, which is not gated, so there it also allows a monotone ramp-down bounded by DEC (0.6 m/s², the launch value).

## R5 Safe termination
grep of bt/, nav_full and nav_control finds no BackUp, DriveOnHeading, Spin, RecoveryNode or behavior_server. plan_to_pose.xml is kept. A controller or progress failure leads to ABORTED (obstacle case), the adapter sends its zero burst, then goes IDLE (checked in run 04).

## Thresholds and constants (justification)
- frozen_window_s 1.0 s, frozen_motion_min 0.05 m / 0.05 rad: EXPERIMENTAL. One map cell / the T5 yaw scale, and 5 evaluations at 5 Hz. The 0.8 window-coverage factor means at least 4 of 5 periods.
- degenerate_budget_m: default 0.0 (unchanged). 2.0 is available only as an explicit EXPERIMENTAL opt-in. The D1 start pose is not ambiguous (competitor 0.25–0.37 vs nominal 1.0, measured offline), so there was no justification to change the default.
- Trust services on a second thread: arm_check measured at about 0.9 s on this Jetson, which is more than the 0.6 s trust_max_age in the adapter and coordinator.
- Test observation windows (1 s stationarity, wait deadlines) are test-only, not runtime timeouts.

## Open issues / risks
1. A dry-run takeover on real hardware is visible only if MC reports a source change. The stock idle source is rc (CURRENT_CONSTRAINTS), so touching the PS5 while rc is already selected is invisible in dry run. It is only fully observable in live mode after registration. Document in phase B.
2. T1_tf_frozen cross-checks the TF against /odom_level, but both come from the same level_bridge callback. It catches a frozen TF path or relay, not a frozen FAST-LIO. A frozen LIO is caught only by T4 or AMCL corrections once the robot has moved enough. A truly independent motion source (leg odometry or MC) is not available.
3. On AMCL loss, Nav2 aborts before trust or the adapter react (TF tolerance), so there is no latch. This is safe (the goal is terminal), but the "fault latch" semantics differ from the other losses.
4. When Nav2 dies mid-goal, the adapter ends normally (IDLE) and stays `motion_armed` until the next arm. Motion still requires a new coordinator goal and gate.
5. Lifecycle shutdown defect: intermittent, 0 abnormal children in run 03. It is recorded in known_defects and not masked.
6. ros_e2e_synthetic.py (domain 88) was updated for T8 but not re-run. controller_dry runs only if time allowed (see below).
7. The coordinator's public NavigateToPose server remains an open user question (DECISIONS).
8. Docs are not updated (phase B): X2_MOTION_ADAPTER (shadow, dry-run driver), X2_LOCALIZATION_TRUST (T8, frozen TF, cloud staleness, experimental params), README (run_model.sh default changed).

## Commands
- Unit tests: see the log above (exact commands from the debugger report, plus `python3 -m unittest test_dry_run_shadow`, `test_trust_gaps`, `test_stop_criterion`).
- e2e: `bash tests/nav_e2e/run_model.sh nav_e2e_real_dryrun_0N`; legacy: `bash tests/nav_e2e/run_model.sh --legacy nav_e2e_legacy_implA_01`.
- Artifacts: tests/integration_results/nav_e2e_real_dryrun_0{1..4}/ (results.json, nav2.log, adapter.log, trust.log, model.log).

## Final runs (appended)
- Legacy suite `run_model.sh --legacy nav_e2e_legacy_implA_01`: **73 checks, 0 failed, rc=0** (the legacy fixture fakes a live+registered adapter, so the coordinator's live path is unchanged).
- nav_e2e_real_dryrun_04: failed at bring-up before the cases started. bt_navigator logged `"compute_path_to_pose" action server not available after waiting for 1.00s`, BT load failed and the lifecycle bring-up aborted. This is a PRE-EXISTING intermittent race; the same error is in nav_e2e_driver_arm_01 and nav_restart_cancellable_01. Not masked; no timeout was changed.
- nav_e2e_real_dryrun_05: **BLOCKED by an external change**. At 21:11:48–52, outside this agent's actions, `ws/src/maps/first_map_nav2_v1/` (the D1 Nav2 map used by nav_full, the trust node and the fixtures) was DELETED and `ws/src/maps/first_map/` was rewritten (its home.yaml is now gone). No other repo file changed after 21:05. I did not restore it, because it may be a deliberate concurrent action. It can be restored from the snapshot tar (`control_backend/ws/src/maps/first_map_nav2_v1/`) if that was unintended. The obstacle safe-termination check added after run 03 has therefore not run yet.
- **Last complete real-chain run: nav_e2e_real_dryrun_03, 158/158, rc=0, clean shutdown.**
- controller_dry (now isolated to domain 87): `bash tests/controller_dry/run_dry.sh` gave all checks ok, including the new 'S1 stopped after terminal', 'S4 stopped after cancel' and 'S3 stopped after abort'. Log: tests/integration_results/implA_console_20261021/controller_dry_01.log.
- Console logs: tests/integration_results/implA_console_20261021/.
