# 20261021-x2-nav2-e2e-dryrun — Verification cycle 2 (verify2): phases A + 2b, e2e rerun

Verifier: independent robotics-verifier, fresh context. No project files edited. All ROS runs domain 87, ROS_LOCALHOST_ONLY=1 (run_model.sh). No ros2 CLI on domain 0. Live X2 stack down (only vendor camera/housekeeper/webrtc running).

## Progress log (incremental)
- 13:4x start: load avg 6.46/6.61/5.22 (8 cores). nav_e2e_verify_03 started in background. Prior: verify_01 15 checks, failed straight_succeeded (ack timeout); verify_02 67 checks, 0 failed, interrupted (no final).

## (1) Dry-run can never create MC publisher / flock / registration — PASS (code inspection, snapshot diff)
- x2_motion_adapter.py:291 registration_state='dry_run' when not live; :314-318 lock.acquire, SetMcInputSource client and ~/register service only `if self.cfg.live`.
- MC publisher created only in _registration_result (:451), reachable only via registration_client future (live only); :432-433 adds a RuntimeError guard in dry run.
- Publish gate :474 `if self.cfg.live and self.pub is not None`, else ~/dry_run_cmd Twist. destroy_node :501 zero burst live-only; lock.release on fd None is no-op.
- Live path vs snapshot: step() branches removed only where `cfg.live` was already the taken branch; _drain burst now unconditional (live identical); arm_motion/ready use _driver_path_ok() == 'registered' in live (identical); tick `not motion_armed` (live identical). Coordinator live_ok uses `d.get('live') is True` (stricter, equivalent for JSON bool). nav_full adds allow_dry_run_driver (default false, validated true/false). No unjustified live change found.
- Out-of-phase files present in diff (phase C/B, not A/2b): run_nav_full.sh, run_trust.sh, rviz/localization.rviz, tests/nav_e2e/live_graph_snapshot.py, docs/*. Not reviewed here beyond scope note.

## (7) BT / single stack / no global cmd publisher — PASS (inspection)
- bt/navigate_to_pose.xml: Sequence{ComputePathToPose, FollowPath}; no BackUp/DriveOnHeading/Spin/RecoveryNode; bt/plan_to_pose.xml retained. plugin_lib_names only compute_path + follow_path.
- nav_full.launch.py: single planner_server, bt_navigator, controller_server+velocity_smoother (nav_control), one lifecycle manager; smoother output remapped to /<outer>/cmd_vel (private); controller cmd_vel -> cmd_vel_nav.
- bt_navigator params do NOT set default_server_timeout / bt_loop_duration / wait_for_service_timeout -> Nav2 1.1.18 defaults 20 ms / 10 ms / 1000 ms (runtime/nav2_bt_deps/.../bt_action_server_impl.hpp:56-66). BtActionNode (bt_action_node.hpp:214-226) returns FAILURE when the goal ack is not received within server_timeout measured from send; the BT has no retry, so one late ack ABORTS NavigateToPose.

## (4) Trust new checks — PASS (inspection; unit results below)
- loc_trust.py: T1_particle_cloud_stale (amcl-freshness rule, not re-reviewed), T1_tf_frozen (frozen_window_s 1.0, min 0.05 m/0.05 rad, marked EXPERIMENTAL, :40-45, :142-150), T8_map_unseen/T8_map_mismatch with require_map_match=True default (:152-156), map_identity() :52.
- degenerate_budget_m default 0.0 (:70); EXPERIMENTAL_DEGENERATE_BUDGET_M = 2.0 constant only (:94), opt-in; node warns when >0 (localization_trust_node.py:58-59) and reports it under status.experimental (:209).
- Node: served map compared to trust grid (:125-133), optional expected_map_sha256 (:55). e2e passes expected_map_sha256 from manifest and checks trust_map_is_nav2_v2.

## (8) Map derivation reproducible — PASS
Re-ran tools/derive_nav2_map.py (python3 -I) into scratchpad:
- v1 from snapshot first_map: map.pgm/map.yaml/manifest.json byte-identical to snapshot first_map_nav2_v1 (7bf9bcbf/9dae2c03/119afa47).
- v2: pgm 545630c1, yaml f49dd066, manifest e8c77a18 — all EQUAL to ws/src/maps/first_map_nav2_v2.
- v3 (--erase-imprint -7.361 6.053 0.45 + reason from mapv3 report): pgm f6032f75, yaml f49dd066 EQUAL; manifest differs ONLY in recorded argv paths (relative "first_map"/"first_map_nav2_v3" vs my absolute scratchpad paths). Data reproducible.

## (9) Teleworker/Nova-Valley copy — PASS (spot-check)
grep over ws/src/navigation/*.py, tests/{nav_e2e,motion_adapter,loc_trust}, tools/derive_nav2_map.py: only hit tests/motion_adapter/test_adapter_core.py:71, a comment contrasting behaviour (pre-existing in snapshot). No Teleworker/Nova-Valley source tree on host.

## e2e nav_e2e_verify_03 — PASS
- `bash tests/nav_e2e/run_model.sh nav_e2e_verify_03` (domain 87, localhost, unicast profile), ~13:41-13:46, rc=0.
- results.json: 160 checks, failed [], shutdown {nav2_restarted 0, adapter 0, trust 0, model 0, abnormal_children 0}; known_defects nav2_group_exit_after_controller_kill 0, first_group_abnormal_children 0.
- nav2.log: 0 lines "Timed out while waiting ... acknowledge" / "not available after waiting".
- Load avg during run: 10.8-12.5 (1-min), peak 12.52 at 13:45:54 (scratchpad loadavg.log) — comparable to verify_01's 11.8.

## (2) Refusals localization_untrusted / foreign_velocity_publisher / vendor_autonomy_active — PASS
- tests/motion_adapter/test_dry_run_shadow.py:20-48 runs the REAL AdapterCore for each fault in live AND dry run, asserts identical state sequences, latch_reason == fault, no nonzero output while latched after the fault clears, IDLE after resume, and only a fresh command moves; :85-90 idle refusal in dry run. test_new_faults.py:7-36 latch/no-auto-resume, idle refusal, dry-run refusal per fault.
- e2e covers localization_untrusted end-to-end through the real trust node (scan/odom loss -> adapter latched 'localization_untrusted'; amcl standing -> goal rejected; wrong initial pose -> rejected). foreign_velocity_publisher and vendor_autonomy_active are unit-level only (core), not e2e. Acceptable for "covered by tests"; noted as residual.

## (3) PS5 takeover cancels Goal A at action level; A never revived; only B moves — PASS
run_suite_real.py:269-286, verify_03 evidence:
- takeover_goal_A_terminal: public status 6; takeover_internal_nav2_goal_terminal: internal bt_navigator goal (child_uuid from coordinator) status 5 CANCELED (action-level cancel of the internal goal).
- takeover_adapter_latched 'operator_takeover'; takeover_release_stays_latched (MC source back to sdk_qazbot_nav2, adapter+coordinator latched); takeover_release_no_remotion (late=[], drift <1e-6 m); takeover_new_goal_refused_while_latched; takeover_explicit_resume ("ready for a NEW goal; no cached command/goal replay"); takeover_resume_alone_no_remotion; takeover_goal_A_not_revived {public 6, internal 5, coord not busy}; takeover_goal_B_succeeded 4 + stop criterion + model pose xy_err 0.059.
- Same pattern for pose_jump (goal A 6, not revived, rejected before resume, B succeeds xy_err 0.059) and fault cases (`*_old_goal_not_revived`: coord busy false, child_uuid None).

## (5) Model-pose assertions, stop criterion — PASS
- pose_ok (run_suite_real.py:136-139) uses <ns>/model_truth (sim ground truth, :60) vs goal with XY_TOL=YAW_TOL=0.10 (= nav_full goal-checker defaults). verify_03: straight xy 0.058, rotate yaw 0.077, repeat 0.063, rviz 0.061, B goals 0.059-0.064.
- stop_criterion.py: terminal observed, last cmd value/time recorded, no nonzero model-input cmd after terminal+0.05 s (decel ramp allowance only when decel given; real suite passes none), model |v|,|w|<0.01 and drift <5 mm over a 1 s window, no nonzero cmd in window. No trailing-zero requirement (already_there passes with last_cmd null).
- no_remotion (:142-145) after release/resume/recovery: no nonzero cmd and drift <5 mm over 1.5 s.

## (6) reset_pose waits for trust — does not mask a defect — PASS (with note)
- reset_pose (:155-160) teleports the model, then waits for trust (40 s test deadline -> raises on failure, so a trust node that never recovers FAILS the suite).
- The wait is only a precondition BEFORE each new goal; any trust loss DURING a goal trips the coordinator (localization_untrusted) and would fail `_succeeded`/`_aborts` checks. All drive cases succeeded, so no in-goal flicker occurred.
- Teleport -> one T4 scan mismatch + 5 s 'settling' hysteresis is correct trust behaviour for an instantaneous pose discontinuity. Note: the live trust flicker (DECISIONS 22:2x) is a separate open item not exercised by this fixture (fixture scan is ideal ray-cast).

## Unit suites (re-run by verifier, 13:5x; log scratchpad v2/unit.log, controller_dry.log)
| Suite | Command | Result |
|---|---|---|
| adapter | tests/motion_adapter: unittest test_adapter_core test_early_takeover test_new_faults test_dry_run_shadow | 43/43 OK |
| trust | tests/loc_trust: unittest test_loc_trust test_source_failures test_trust_gaps | 56/56 OK |
| stop criterion | tests/nav_e2e: unittest test_stop_criterion | 7/7 OK |
| footprint | test_vendor_polygon 4/4 OK; test_envelope_rest (X2_URDF_DIR=...v1.3.0) 1/1 OK | 4+1 OK |
| launch_stagger | QAZBOT_SMAC_PREFIX set, env.sh, test_launch_stagger.py | 5/5 OK |
| controller_dry | bash tests/controller_dry/run_dry.sh (forced domain 87/localhost/unicast) | 19/19 ok, rc 0 (S1 x8, S2 x4, S4 x3, S5 x1, S3 x3) |

## e2e nav_e2e_verify_04 — FAIL (test-fixture race, not a motion-safety defect)
- `bash tests/nav_e2e/run_model.sh nav_e2e_verify_04`, 13:46-13:48, rc=1. 70 checks, failed ['takeover_goal_A_accepted'] (run_suite_real.py:270). Shutdown all 0, abnormal_children 0. nav2.log: 0 ack-timeout / 0 "not available" lines. Load avg 1-min 9.1-10.6 (before live localization started on domain 0 at ~13:49).
- nav2.log:175 `Goal rejected: busy=False latched=False trust=True zero=True nav2_ready=True` -> every LOGGED condition was satisfied, so the rejection came from an UNLOGGED term of goal_coordinator.py:112: `reserving`, `not driver_ready` or `not driver_arm.service_is_ready()`.
- trust_log: trust was False ('settling', 5 s, teleport transient) and became True at the very last entry (1812.389) immediately before the goal; reset_pose (run_suite_real.py:155-160) returns as soon as the TRUST NODE status is trusted and the next goal is sent at once.
- Most probable cause (inferred, not directly logged): coordinator driver_ready comes from the adapter status JSON (`ready` requires not _health_fault, which includes localization_untrusted), published only at 2 Hz (x2_motion_adapter.py create_timer(0.5, self._report)); so for up to ~0.5 s+ after trust returns the coordinator still sees driver_ready=False and correctly rejects. reset_pose waits for `trusted` but NOT for `S['coord']['ready']` (fault_case/resume_all do wait for coord ready). Product behaviour (reject while driver not ready) is safe; the suite is flaky. Same race applies to every reset_pose -> new_goal pair; implA run07 failed at the same check.
- Diagnostic gap: the coordinator rejection log (goal_coordinator.py:113) omits driver_ready / reserving / driver_arm readiness, so the cause can't be proven from logs.

## (10) bt_navigator compute_path_to_pose ack timeout — NOT REPRODUCED
- Mechanism confirmed from installed headers: bt_navigator default_server_timeout 20 ms, bt_loop_duration 10 ms, wait_for_service_timeout 1000 ms (runtime/nav2_bt_deps/.../bt_action_server_impl.hpp:56-66); BtActionNode returns FAILURE if the goal response is not received within server_timeout from send (bt_action_node.hpp:214-226, 400-413). bt/navigate_to_pose.xml has no retry, so ONE late ack aborts NavigateToPose. nav_full.launch.py sets none of these.
- Occurrences: verify_01 1 (load 11.8, live localization + vendor running); verify_02 0 in 67 checks; verify_03 0 in 160 checks (load 1-min 10.8-12.5); verify_04 0 in 70 checks (load 9.1-10.6). Approx 20+ ComputePathToPose acks per full run -> intermittent, low rate at ~10-12 load.
- No ack-latency distribution was measured (coordinator narrowed scope; a measurement-only probe is prepared at scratchpad v2/probe/ack_probe.py: separate nav_plan planner_server in ns qazbot_d1_map, domain 87, not run). Therefore NO value for default_server_timeout is recommended here. Note for the follow-up: the measurement must be done under live-like load (live localization running), and a sharing probe against the e2e planner_server must not be used (nav2_util SimpleActionServer would preempt the BT's goal).

## Scope note
Files outside A/2b present vs snapshot (phase B/C, not reviewed): docs/*, run_nav_full.sh, run_trust.sh, rviz/localization.rviz, tests/nav_e2e/live_graph_snapshot.py. loc_trust.py/localization_trust_node.py amcl-freshness logic excluded per instruction.

## VERDICT: FAIL
Failed: required "real-chain e2e twice" — verify_03 160/160 PASS, verify_04 FAIL at takeover_goal_A_accepted (fixture race).
Required fixes:
1. tests/nav_e2e/run_suite_real.py reset_pose (:155-160): after `wait(trusted, 40)` also wait for the coordinator's composite readiness (`S['coord'].get('ready')`, as resume_all does) before the caller sends a goal. Test-only; no runtime timeout change.
2. goal_coordinator.py:113: include driver_ready, reserving, driver_arm/service readiness in the "Goal rejected" warning (diagnostics only, no behaviour change).
3. Then re-run real-chain e2e twice (domain 87) and require 2 consecutive full passes.
Non-blocking: measure compute_path_to_pose ack latency (probe prepared) before choosing default_server_timeout; e2e lacks foreign_velocity_publisher / vendor_autonomy_active cases (unit-covered).
