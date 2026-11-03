# 20261021-x2-nav2-e2e-dryrun — readyrace (verify2 REQUIRED_FIXES, cycle 1/3)

Implementer: robotics-implementer. Edit in place (/home/agi/control_backend, not git).
Backups: ws/src/navigation/goal_coordinator.py.bak_20261022_pre_readyrace,
tests/nav_e2e/run_suite_real.py.bak_20261022_pre_readyrace.

## Progress log (incremental)
- start: read verify2 report, goal_coordinator.py, x2_motion_adapter.py, run_suite_real.py, verify_04 logs. Backups taken.
- edits done: goal_coordinator.py reject() logging; run_suite_real.py reset_pose composite wait; docs/RVIZ_LOCAL.md:36 log text (backup .bak_20261022_pre_readyrace). Next: unit suites, then e2e ready_01/_02.

## Root cause of nav_e2e_verify_04 takeover_goal_A_accepted (evidence + elimination)
Evidence (tests/integration_results/nav_e2e_verify_04):
- results.json trust_log: reset_pose teleport -> trust False at 1807.187 (T4_scan_drop), then 'settling' (stable_s hysteresis) until 1812.389 True; the goal was sent immediately after (reset_pose returned on the first trusted status).
- nav2.log:175 (wall 1792648086.041): `Goal rejected: busy=False latched=False trust=True zero=True nav2_ready=True`; previous goal (rviz goal_pose) finished 7 s earlier (nav2.log:174 'Goal succeeded' at ...079.158).
- Unlogged terms of goal_coordinator.py:112 (pre-fix): reserving, driver_ready, driver_arm.service_is_ready().
Elimination:
- reserving: set only inside goal() between pre-check and return (finally resets it). No goal was in flight (previous one terminal 7 s earlier, harness sends one goal at a time) -> False.
- driver_arm.service_is_ready(): adapter process alive the whole run (adapter.log, shutdown adapter=0), service created in __init__, discovered since bring-up check coordinator_accepts_dry_run_driver -> True.
- driver_ready: the ONLY remaining term. Mechanism (code, deterministic):
  * adapter x2_motion_adapter.py _report() (create_timer(0.5)) status.ready = driver_path_ok and state in (idle,active) and not _health_fault(); _health_fault returns 'localization_untrusted' while its own trust Bool is False -> ready False during the 5 s settling.
  * coordinator.adapter() sets driver_ready ONLY on receipt of that 2 Hz status. The coordinator receives the trust Bool directly (trust node 5 Hz, same tick as the trust status String the harness waits on), so good() becomes True at once, while driver_ready stays False until the adapter's next periodic report AFTER it processed the same trust Bool: a window of 0..500 ms (uniform phase).
  * The harness sends the goal within milliseconds of seeing trust status True -> lands inside that window -> rejected by `not self.driver_ready`.
- Consistent with frequency: reset_pose only occasionally drops trust (trust_log keeps the last 400 samples only: verify_03 shows recoveries at 1607.794 and 1621.193 and the run passed, i.e. the goal landed after the adapter report; verify_04 shows one drop/recovery -> rejected). Cases without a trust drop never hit it because driver_ready never went False. implA run07 failed at the same check (same reset_pose -> new_goal pattern).
- Direct log proof was impossible pre-fix (diagnostic gap); the new rejection log prints driver_ready, driver_status (state/reason/ready/...) and driver_status_age_s, so any recurrence is now attributable from nav2.log alone.

## Decision: test precondition race, NOT a product defect -> no behaviour change
- The coordinator publishes its own composite admission flag goal_coordinator/status.ready, computed from the SAME cached driver_ready plus all other pre-check terms (it is a superset of the goal() pre-check at :112). In the race window status.ready is False, so a client that sends goals when the coordinator says ready is never refused. resume_all() and fault_case() in the suite already used that contract; reset_pose used the trust node's verdict as a proxy, which is the wrong precondition.
- Operator impact: a goal sent < 0.5 s after a trust recovery or after an adapter resume (before coordinator status shows ready) is refused, fail-closed, now with an explicit log line naming driver_ready=False and the stale adapter status age. No motion, no goal revival, no safety relevance. Human RViz clicks after a resume are practically always > 0.5 s later.
- Why no "event-driven" product fix: making the adapter publish status on change only shrinks the window (~0.5 s -> ~1 tick 20 ms + DDS); it cannot remove it, because coordinator and adapter are two independent subscribers of the trust topic and DDS gives no ordering between the trust topic, the adapter status topic and the action goal request. A strictly race-free admission would need goal() to rely on the synchronous driver_arm call instead of the cached flag, which also requires reworking the post-arm re-check (:138, cached driver_ready with no ordering vs the arm response) — a change to the safety gate, not minimal, and not required by the evidence. Recorded as an optional follow-up: adapter publish-on-change would also cut adapter-latch -> coordinator-cancel latency from <=500 ms to ~20 ms (motion is already stopped by the adapter itself, so it is not a safety gap).

## Changes
1. ws/src/navigation/goal_coordinator.py (diagnostics only): new reject(why) logs every admission term: busy, reserving, latched, fault, trust (good()), trusted_raw, trust_age_s, zero, driver_ready, driver_mode, driver_status {state,reason,latch,ready,registration,motion_armed} + driver_status_age_s, driver_arm_svc, trust_arm_svc, lifecycle_svc, nav_active, nav2_ready. Used for all REJECT paths (precondition, frame_id, non-finite, quaternion, custom BT, arm/lifecycle service, bt_navigator state, trust arm check, driver arm, post-arm). Every return value and condition is unchanged (each branch still returns GoalResponse.REJECT under exactly the same condition); the two previous separate warnings ('Goal arm check rejected', 'Driver arm rejected') are folded into 'Goal rejected (trust_arm_check: ...)' / '(driver_arm: ...)'. New fields driver_status/driver_status_at are written in adapter() and read only by reject().
2. tests/nav_e2e/run_suite_real.py reset_pose: `wait(trusted, 40)` -> `wait(lambda: trusted() and S['coord'].get('ready', False), 40)`: explicit coordinator-admission precondition under the SAME single 40 s observation deadline (no new timeout, sleep or retry). All reset_pose callers send a goal next or are already followed by resume_all; obstacle case leaves fault '' (verify_03), so coordinator is not latched at the :254 reset.
3. docs/RVIZ_LOCAL.md:36: log text updated to the new format + note that a goal sent before coordinator status ready is refused (safe).

## Unit suites (after edits)
| Suite | Command (cwd tests/<dir>, env.sh sourced) | Result |
|---|---|---|
| motion_adapter | python3 -m unittest test_adapter_core test_early_takeover test_new_faults test_dry_run_shadow | 43/43 OK |
| loc_trust | python3 -m unittest test_loc_trust test_source_failures test_trust_gaps | 56/56 OK |
| stop criterion | python3 -m unittest test_stop_criterion | 7/7 OK |

## e2e nav_e2e_ready_01 — FAIL (environment: CPU starvation; NOT the readiness race)
- `bash tests/nav_e2e/run_model.sh nav_e2e_ready_01`, 13:55-13:58, rc=1. 50 checks, failed ['unreachable_accepted_for_evaluation'] (run_suite_real.py:257). Shutdown all 0, abnormal_children 0; 0 ack-timeout lines.
- Load avg 1-min 16-29.6 during the run (scratchpad loadavg_ready01.log); the operator's live stack on domain 0 started meanwhile (fastlio_mapping, localization_trust_node v3, x2_motion_adapter live=false ns qazbot_d1_map, ros2 topic echo) on top of vendor camera/perception (~40% CPU each). Not touched.
- New rejection log (nav2.log:142) attributes it unambiguously: `Goal rejected (driver_arm: localization_untrusted): ... trust=False trusted_raw=False ... driver_status={'state':'idle','ready':False,...}` — trust dropped AFTER reset_pose+resume_all saw coordinator ready: trust_log 2386.973 T1_odom_stale, then T1_tf_stale / T1_scan_stale / T1_odom_message_stale flapping until the end: the python kinematic fixture's odom/scan publications were late under starvation. Trust node and coordinator behaved correctly (stale sources -> untrusted -> refuse).
- nav2.log:114 rejection is the expected repeat_while_active_rejected check (busy=True) — new log format verified in both the pre-check and the driver_arm branch.
- The fixed reset_pose precondition held in every reset in this run (straight .. obstacle all passed).

## e2e nav_e2e_ready_02 — FAIL (environment/fixture: post-teleport trust drop AFTER the precondition; NOT the readiness race)
- 13:59:40-14:03, rc=1, 85 checks, failed ['scan_loss_goal_accepted'] (fault_case first goal, run_suite_real.py:174). Shutdown all 0, abnormal_children 0, 0 ack-timeout lines. Load 1-min 11.2-15.9 (scratchpad loadavg_ready02.log); operator's domain-0 live stack still running.
- The whole takeover block now PASSES (takeover_goal_A_accepted .. takeover_goal_B_model_pose).
- nav2.log:194 `Goal rejected (driver_arm: localization_untrusted): ... trust=False ... driver_status={'state':'idle','ready':False,...}`; trust_log: True through 2657-2665, then 2665.112 T1_tf_frozen, then 'settling'. reset_pose's precondition (trusted AND coord ready) was satisfied by verdicts that predate the trust node's evaluation of the teleport; the teleport (NEAR -> START, 0.44 m, robot stationary) then produced a T1_tf_frozen evaluation: the frozen-TF check (loc_trust.py:342-350) fires as soon as ONE evaluation sees the new odom message pose while the TF odom->base sample is still the pre-teleport one (TF constant for the preceding >=0.8 s because the robot stood still) — i.e. TF processing lagging odom in the trust node, plausible under CPU contention. Correct trust behaviour for the inputs it saw; the transient is a property of the non-physical teleport fixture.
- nav2.log:114 and :189 rejections are expected checks (repeat_while_active busy=True; takeover_new_goal_refused_while_latched latched=True fault='operator_takeover').

## e2e nav_e2e_ready_03 — FAIL (known bt_navigator ack timeout under load; NOT the readiness race)
- 14:04-14:09, rc=1, 101 checks, failed [] (aborted by an observation deadline, not a check): fault_case('scan_loss') goal ACCEPTED, then nav2.log:212 `Timed out while waiting for action server to acknowledge goal request for compute_path_to_pose` -> Goal failed before motion -> wait(moving) deadline. This is the open item (10) of verify2 (bt_navigator default_server_timeout 20 ms), 1 occurrence, load 1-min 11.3-19.3. Shutdown all 0.
- Readiness precondition evidence: trust dropped twice under load in this run (2938.514 T1_scan_stale -> True 2945.701; 2954.113 T1_odom_message_stale -> True 2961.108) and every following goal acceptance check passed — the composite wait handled the recoveries. Takeover block passed again (2nd time).
- ready_04 not run (loop stops on first failure; two consecutive passes no longer reachable in this cycle's run budget under the current load).

## Verdict for this cycle
- Readiness race: FIXED in the test (explicit coordinator-ready precondition); diagnosed with evidence; takeover_goal_A_accepted passed in ready_02 and ready_03; no recurrence of a rejection with all conditions true across 3 runs (every rejection is now fully attributed in nav2.log).
- Required "2 consecutive real-chain passes": NOT achieved. All 3 failures are environmental under 1-min load 11-29 (8 cores): the operator's live domain-0 stack (fastlio_mapping, trust v3, adapter live=false, ros2 topic echo) started after verify2 (verify_03/04 ran with it down, load 9-12.5) on top of vendor camera/perception. Failure modes: fixture odom/scan staleness (ready_01), post-teleport T1_tf_frozen transient after the precondition (ready_02), known compute_path_to_pose ack timeout (ready_03). None is a product regression from this change (coordinator change is logging only).
- Next action: rerun `bash tests/nav_e2e/run_model.sh nav_e2e_ready_05` and `_06` when the operator's domain-0 live stack is stopped (verify_03/04 conditions). Optional hardening (not done, needs decision): (a) expose the trust node's evaluated odom/TF input pose (diagnostic field) so reset_pose can wait for a verdict computed from post-teleport inputs instead of relying on spin(.5); (b) measure compute_path_to_pose ack latency (verify2 probe) before setting default_server_timeout.
- Out-of-scope note: T1_tf_frozen (EXPERIMENTAL) fires on a single evaluation where the odom message has moved but the TF sample has not yet been processed after >=0.8 s standstill — on hardware a start-from-standstill with TF lag >= the time to move 5 cm would do the same; worth a look in the trust-flicker item.
