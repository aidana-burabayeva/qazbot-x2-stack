# Verifier report — 20261021-x2-nav2-e2e-dryrun — simplified Localization Trust (READY latch + MOTION_OK)
Date: 2026-10-22. Scope: fall / uncontrolled-motion risks only. No project file edited; no e2e, no domain-0 CLI, no live processes touched.

## VERDICT: FAIL (one safety hole in the READY clear on a confirmed jump; everything else passes)

## Diff reviewed (against *.bak_20261022_pre_simpletrust)
- ws/src/navigation/loc_trust.py: new `_verdict`, `relocalize`, `goal_request`; settling (stable_s) removed; arm_check ambiguity no longer refuses.
- ws/src/navigation/localization_trust_node.py: threading.Lock, `initialpose` sub -> relocalize, `_arm` = goal_request + READY gate, ambiguity outside lock.
- ws/src/navigation/goal_coordinator.py:125-128: fire-and-forget arm_check when `not good()`; goal still rejected.
- tests/nav_e2e/run_suite_real.py: `reconfirm()` helper and updated expectations (not run, per instruction).
- x2_motion_adapter.py unchanged (mtime Oct 21 20:39).
- Out of scope but noted: localization.launch.py modified 15:04 (TasksetPrefix / lio_cpus). That belongs to the parallel liolat task (PLAN.md line 4), not simpletrust. Default '' gives no behaviour change.

## Commands run
- `cd tests/loc_trust && python3 -W ignore -m unittest test_loc_trust test_trust_gaps test_source_failures test_ready_motion`: Ran 65, OK
- `cd tests/motion_adapter && python3 -W ignore -m unittest test_adapter_core test_dry_run_shadow test_early_takeover test_new_faults`: Ran 43, OK
- `cd tests/nav_e2e && python3 -W ignore -m unittest test_stop_criterion`: Ran 7, OK
- Probe (scratchpad/probe_stale_jump.py, pure evaluator): jump masked, see F1.
- Mutation run (scratchpad/mutate.py on a scratch copy of loc_trust.py; 12 mutants): all killed. See the table under check (7).

## Per-check results
(1) MOTION_OK false right away on loss: PASS.
- loc_trust.py `_verdict`: `motion` = every T0*/T1* reason; `motion_ok = self.ready and not motion` is computed on the same tick. That covers T1_odom_stale, T1_odom_message_stale, T1_tf_stale (map->odom), T1_scan_stale, T1_tf_frozen, T1_amcl_pose_stale / T1_particle_cloud_stale (due rule), T0_invalid_pose and T0_invalid_scan.
- The coordinator latches: goal_coordinator.py:76 trips on any False while busy, and :183 check() trips at 20 Hz on `not good()`. `latched` is cleared only by resume(), which requires `not busy`, i.e. the old action is terminal. New-goal requirement: goal() admits only via awaited arm_check success + driver arm + a NEW trust sample == True (:142-152).
- The adapter latches 'localization_untrusted' while active (x2_motion_adapter.py step(): fault and was_active -> _latch). Resume ignores commands stamped earlier.

(2) READY transitions: PARTIAL. FAIL on the T5 path (F1).
- READY becomes True only in the `elif self.pending` branch. `pending` is set only by confirm_anchor() and is cleared by READY set and by relocalize(). Result: READY cannot be set without confirm_anchor. Mutant M8 (READY on clean eval without pending) was killed.
- T8 unseen/mismatch clears READY (M3 killed). initialpose -> relocalize() clears READY and the anchor (M2 killed). The topic resolves to /qazbot_d1_map/initialpose, the same as seed_pose.py and the rviz config. QoS reliable/depth 1 matches the reliable publishers.
- New confirm_anchor clears READY and re-gates (M6 killed). goal_request with T0/T1 loss clears READY (M7 killed).
- T5: cleared only if `'T1_tf_stale' not in why`. This exemption plus the 1 s sliding window lets a real jump be masked permanently. See F1.

(3) Diagnostics cannot re-enable motion or mask T1/T5/T8: PASS.
- `diag` only goes into the status. READY can only be set from pending. `motion` and `clear` are computed independently of `diag`.
- The T0 early return skips T5/T8 for that tick, but motion is blocked on that tick and map_ok persists, so T8 clears on the next valid tick. The T5 part of this is in F1(b).

(4) Race safety: PASS.
- The tick and the initialpose sub are in the default MutuallyExclusive group. Services are in a separate MutuallyExclusive group on a 2-thread executor.
- The lock covers evaluate / confirm_anchor / relocalize / goal_request.
- arm_check runs outside the lock but only mutates degenerate_since_len (T7 diagnostic). set_baseline outside the lock only affects the READY gate floor; that race is benign.
- The arm response can be up to one tick old, but the coordinator still requires a NEW trusted==True sample after both arms (:150-152), and good() before forwarding any command (:113).
- The coordinator's non-awaited arm_check (:125-128) runs only when `not good()`, and the same callback then rejects at :129 ('precondition'). It can only clear READY (safe direction). It cannot admit a goal.

(5) /localization_trust/trusted semantics: PASS.
- The node publishes `r['trusted']`, and `trusted == motion_ok` in `_verdict`.
- The adapter subscribes to /qazbot_d1_map/localization_trust/trusted (depth 10) and gets loc_reason 'localization_untrusted'. The coordinator hard list includes it (goal_coordinator.py:90).
- goal_request's update of `last` is not published, but the next tick keeps READY False, so the next published value is False.

(6) Flapping MOTION_OK with settling removed: PASS.
- A single False tick (held ≥200 ms) trips the coordinator latch for the whole goal.
- The adapter samples at 50 Hz with depth-10 reliable history, so a one-tick False cannot be missed. It latches, and the coordinator then trips via adapter status.
- Nothing un-latches during an ACTIVE goal. Resume needs a terminal old action, a source zero and current trust, plus a NEW goal.
- Residual (not new): the coordinator's own trust sub is depth 1, so it could miss a single False if it stalled ≥200 ms. The adapter latch covers this.

(7) Tests meaningful: PASS, with one exception.
- Mutants killed:
  - M1 (motion ignores T1): 5 failures
  - M2, M3, M4 (T5 never clears)
  - M5 (gate ignored): 4 failures
  - M6, M7, M8, M10 (T1_tf_frozen not blocking), M11 (T1_scan_stale not blocking), M12 (T0 -> ok)
- Exception: test_ready_motion.py:57 `test_jump_on_stale_tf_is_not_confirmed` asserts the unsafe behaviour (READY kept after a jump on stale TF). It is the only test that kills M9, the safer variant. No test covers "stale/invalid period > jump_window_s, then fresh jumped map->odom".
- No unit test for the node's `_arm` gating, the initialpose wiring or the lock. Accepted, because coordinator admission additionally requires a fresh True sample.

## Findings
F1 (SAFETY, must fix): a confirmed global pose jump can be masked, and READY stays latched with a wrong pose.
- loc_trust.py `_verdict`: `clear = [... (r == 'T5_map_odom_jump' and 'T1_tf_stale' not in why)]`. T5 itself uses a 1 s sliding window (`mo_hist`, jump_window_s).
- (a) Case 1: map->odom content changes while its stamp is classified stale for ≥ jump_window_s. Examples are AMCL lag > transform_tolerance + tf_max_age (≈2 s; AMCL lag "0.36–2.0 s under load" per loc_trust.py comment), or a back-stamped TF that SourceStamp rejects. The node still assigns `self.mo` (localization_trust_node.py:119 assigns content regardless of stamp acceptance). Each stale tick sees T5+T1_tf_stale, so nothing clears. After 1 s the window holds only jumped values, so T5 disappears. When TF is fresh again READY=True and MOTION_OK=True. T6_anchor_disagreement is only a diagnostic.
- (b) Case 2: same effect via T0: invalid map->odom or odom (NaN) for > 1 s. The early return skips the mo_hist append, and the pre-gap entries are pruned on the first valid tick.
- Evidence (probe, scratchpad/probe_stale_jump.py):
  - 0.5 m jump, TF stamp 1.5 s old for 1.4 s: every tick `ready=True motion_ok=False ['T1_tf_stale']`.
  - Then fresh: `ready=True motion_ok=True [] diag ['T4_no_baseline','T6_anchor_disagreement']`.
  - `goal_request()` returns [] with READY True, so the arm_check succeeds and the next goal is accepted on a pose 0.5 m off.
  - The NaN-gap variant gives the same result.
- Before this change T6 blocked continuously, so this is a regression on the "confirmed global pose jump -> stop + operator" requirement (DECISIONS "Simplify Localization Trust").
- Required fix (either option):
  - Clear READY on any T5 regardless of TF freshness. A jump in AMCL's published content is real whatever its stamp age.
  - Or keep a reference = last map->odom seen on a fresh, T0-valid tick, and clear READY when the first fresh valid map->odom after a stale/invalid gap differs from it by > jump_xy_max / jump_yaw_max.
  - Also cover T0 gaps: do not let the jump reference age out while T0/T1_tf_stale persist.
- Replace test_ready_motion.py:57 with a test that a jump on a stale TF lasting > jump_window_s clears READY by the first fresh tick at the latest, plus a NaN-gap variant.

## Notes (no fix required)
- N1: Slow false slides (single wall) are no longer stopped (T6 diagnostic). This follows the user decision. Visible in status.diagnostics.
- N2: Re-localization through a path other than /qazbot_d1_map/initialpose (e.g. an AMCL service) does not call relocalize(). A resulting map->odom change >0.1 m still clears READY via T5 once F1 is fixed.
- N3: tests/loc_trust/ros_e2e_synthetic.py was not updated (per the implementer). It is not a required gate.

---
# Cycle 2 re-verification (2026-10-22) — VERDICT: PASS

## What changed
- Only loc_trust.py changed (mtime 15:17), plus tests/loc_trust/test_ready_motion.py.
- localization_trust_node.py: byte-identical to the cycle-1 copy (diff against the scratchpad copy is empty).
- goal_coordinator.py: unchanged (mtime 15:01).
- Fix, in loc_trust.py:
  - `mo_ref` holds the last fresh, valid map->odom. `mo_gap` is set on any T1_tf_stale tick and on both T0 early returns (loc_trust.py:347, :350, :422).
  - On the first fresh tick after a gap, a difference from mo_ref greater than jump_xy_max / jump_yaw_max adds 'T5_map_odom_jump_across_gap' (:419-430).
  - `clear` now covers any T8* or T5* reason; the T1_tf_stale exemption is removed (:471).
  - map_odom None always yields T1_tf_stale, so the else branch never sees None.

## Commands and results
- loc_trust: 67 OK (test_loc_trust, test_trust_gaps, test_source_failures, test_ready_motion).
- motion_adapter: 43 OK.
- test_stop_criterion: 7 OK.
- Probe (scratchpad/probe2.py). Gap ticks 1.4-2.8 s; every gap tick had motion_ok False; then a fresh tick at 3.0 s:

| Case | Final state |
|---|---|
| 0.5 m jump, stale TF (stamp 1.5 s old) | ready=False, not_ready:T5_map_odom_jump (cleared at the first stale tick, exemption gone); goal_request leaves READY False |
| 0.3 rad yaw jump, stale TF | ready=False |
| NaN TF gap, then 0.5 m jump | ready=False, not_ready:T5_map_odom_jump_across_gap |
| map_odom None gap, then 0.5 m jump | ready=False, across_gap |
| NaN scan gap (T0_invalid_scan) with 0.5 m jump | ready=False, across_gap |
| No-jump gaps (stale, NaN TF, None TF) and a 5 cm / 0.05 rad sub-threshold change | ready=True, motion_ok=True; no false READY loss |

- Mutation tests on the new code (scratch copy):

| Mutant | Result |
|---|---|
| N1: across_gap check removed | killed (test_jump_during_nan_tf_gap...) |
| N2: T0 early return does not set the gap flag | killed |
| N3: stale exemption restored | survives alone: the across-gap check also catches it, so the two guards overlap |
| N4: stale tick does not set the gap flag | survives alone: the window check also catches it |
| N3+N4 together | killed (test_jump_during_stale_tf_gap_clears_ready_at_first_fresh_tick) |

## Re-check of (1)-(6)
- No regression. `motion` and `motion_ok` are unchanged, so T0/T1 still drop MOTION_OK on the same tick.
- READY is still set only from pending (confirm_anchor).
- Diagnostics are unchanged and never clear or set READY.
- Lock, arm gating and the coordinator path are unchanged.
- ~/trusted is still motion_ok.
- Coordinator and adapter latches are unchanged.
- Note: a legitimate AMCL correction above 0.10 m / 0.10 rad across a TF gap now requires operator re-confirm. This is the conservative direction and matches "prolonged loss -> READY lost". It affects availability only.

## Remaining notes (no fix required)
N1-N3 from cycle 1 still apply.
