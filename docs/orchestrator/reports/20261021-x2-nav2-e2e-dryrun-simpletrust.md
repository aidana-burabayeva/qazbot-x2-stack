# 20261021-x2-nav2-e2e-dryrun — Simplify Localization Trust (READY / MOTION_OK)
Status: DONE (unit suites pass). Backups: *.bak_20261022_pre_simpletrust next to each changed file.

## State logic (ws/src/navigation/loc_trust.py, raw checks T0..T8 unchanged, new `_verdict`)
- READY latched. confirm_anchor() -> pending; READY set at the first evaluation whose gate is clean:
  T0/T1, T2, T3 (incl. no_particles), T4 score/drop/empty, T5, T6 anchor agreement, T8. Skipped in gate: T4_no_baseline, T6_no_anchor, T7.
  Pending has no timeout (no new duration); reasons show why.
- READY cleared only by: T5_map_odom_jump with fresh map->odom (= existing T5 logic, "confirmed" = observed in the
  published, fresh, T0-valid transform; stale TF jump not counted), any T8, relocalize() (node: <ns>/initialpose),
  new confirm_anchor (re-gate), goal_request() while T0/T1 loss present ("prolonged loss" = still lost at a new goal request).
- MOTION_OK = READY and no T0/T1 reason (AMCL freshness only via the existing per-axis due rule). ~/trusted = MOTION_OK.
- Diagnostics after READY: T2, T3, T4, T6, T7 -> status.diagnostics. Never clear READY/MOTION_OK.
- Baseline: optional (justification: T4 now gates only once at READY; the absolute floor hit_min_abs still applies;
  with a baseline the stricter 0.8*baseline floor applies). Folding would need an implicit sample-collection window.
- Settling hysteresis (stable_s 5 s) removed; field kept unused for parameter compatibility. Justification: after any
  MOTION_OK loss while busy the coordinator trips (latched, goal aborted) and the adapter latches; the old goal is never
  resumed; motion needs operator resume + NEW goal + arm_check on current data + a post-arm fresh trust sample. A flapping
  input causes another stop, not motion. No safety function depended on the 5 s.

## Node (localization_trust_node.py)
- status JSON adds ready, motion_ok, ready_cause, diagnostics (+ existing fields). ~/trusted contract unchanged.
- Subscribes initialpose (relative -> /qazbot_d1_map/initialpose, same as seed_pose.py / AMCL) -> relocalize().
- arm_check: goal_request() first; success = READY and MOTION_OK (+fresh inputs); ambiguity scan runs as T7 diagnostic only.
- threading.Lock around evaluate / confirm / relocalize / goal_request (tick vs service thread); ambiguity scan outside lock.

## goal_coordinator.py (3 lines)
- goal(): when trust is not good, fire-and-forget arm_check (not awaited) so the trust node can apply the "loss at goal
  request" rule; goal still rejected as before. Needed because the coordinator rejected before calling arm_check.
## x2_motion_adapter.py: NOT changed (consumes ~/trusted = MOTION_OK as before).

## Tests (python3 -W ignore -m unittest ..., with /opt/ros/humble sourced)
- tests/loc_trust: 56 -> 65 (test_loc_trust 16, test_trust_gaps 27, test_source_failures 13, NEW test_ready_motion 9). OK.
- tests/motion_adapter (test_adapter_core, test_dry_run_shadow, test_early_takeover, test_new_faults): 43 OK, unchanged.
- tests/nav_e2e/test_stop_criterion: 7 OK.
Old -> new mapping:
- test_trust_after_settling -> test_ready_at_first_clean_gate_after_confirm
- test_no_anchor_no_trust: reason T6_no_anchor -> not_ready:no_anchor (fresh evaluator, never confirmed)
- test_false_global_lock_jump: T5 -> not_ready:T5_map_odom_jump, stays not READY, T6 in diagnostics
- slow slide along wall: T6 now diagnostic, robot stays MOTION_OK (accepted consequence of the decision)
- wrong-place scan / covariance / two hypotheses / T7 budget / anchor LIO drift: assertions moved to diagnostics, trusted stays True
- map identity: unseen blocks READY gate; mismatch after READY clears READY; needs confirm_anchor after fix
- NoSilentRecovery (settling) -> transient loss recovers immediately; loss at goal_request clears READY until confirm
- source_failures: missing baseline / ambiguous -> diagnostic only; empty scan / negative cov -> READY gate refusal
- NEW test_ready_motion: stationary 9-min-old AMCL stays READY+MOTION_OK; partial lidar occlusion while moving ->
  diagnostic; covariance growth while moving -> diagnostic; pose jump -> READY lost until operator; jump on stale TF not
  confirmed; odom/odom msg/scan/TF stale -> MOTION_OK false, READY kept, recovers on fresh data; map mismatch -> READY lost;
  initialpose -> READY lost until confirm; READY gate refuses unconverged AMCL.

## e2e (tests/nav_e2e/run_suite_real.py) — NOT run (unit suites only per task; domain 0 untouched)
- new helper reconfirm(jump): waits for the restoring map->odom change to leave the T5 window, checks READY false,
  calls confirm_anchor, waits trusted.
- amcl_loss standing: goal during loss now also clears READY (check amcl_loss_at_goal_request_clears_ready); recovery via reconfirm.
- wrong_initial_pose: expects ready=false + not_ready:T5_map_odom_jump; recovery via reconfirm (was: automatic).
- pose_jump fault_case: restore = zero offset + reconfirm (was: automatic recovery after settling).
- scan/odom/amcl loss while driving, takeover, operator stop, no old-goal revival: unchanged expectations.
- reset_pose comment updated (teleport -> T4/T6 diagnostics only, no T5).

## Risks / notes
- T5 now latches READY loss: a legitimate large AMCL correction (>0.10 m within 1 s) during motion requires operator
  confirm. Thresholds still candidates; observe on X2.
- Slow false slides (single wall) are no longer stopped (T6 diagnostic) — per decision; visible in status.diagnostics.
- tests/loc_trust/ros_e2e_synthetic.py (domain 88) not updated/re-run; may assume settling.
- docs/X2_LOCALIZATION_TRUST.md: new section "2026-10-22: READY / MOTION_OK" added on top; older table kept for thresholds.

## Cycle 2 (verifier FAIL fixes, 20261021-x2-nav2-e2e-dryrun-verify-simpletrust.md)
- loc_trust.py: mo_ref = last map->odom of a fresh valid tick; any T1_tf_stale or T0 (NaN) tick sets mo_gap (T0 early
  return keeps mo_ref). First fresh tick after a gap: differs from mo_ref by > jump_xy_max / jump_yaw_max ->
  'T5_map_odom_jump_across_gap' -> READY cleared (existing thresholds, none new). T1_tf_stale exemption removed: any T5* clears READY.
- test_ready_motion: test_jump_on_stale_tf_is_not_confirmed replaced by: stale-TF gap (1.6 s > window) with jump ->
  READY lost at first fresh tick; NaN-TF gap with jump -> 'not_ready:T5_map_odom_jump_across_gap'; gap without jump keeps READY (stale and NaN).
- Suites: loc_trust 67 OK; motion_adapter 43 OK; stop_criterion 7 OK. Live processes untouched.
