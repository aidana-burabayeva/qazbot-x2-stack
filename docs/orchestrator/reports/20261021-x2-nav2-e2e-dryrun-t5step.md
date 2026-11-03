# T5 step change verifier: PASS
Diff: loc_trust.py only T5 block, docstring, jump_window_s comment (now unused). Tests: +2 (gradual keeps READY, single 0.15 m step clears); gap test comment only. No other thresholds changed (diff of config dataclass).
1. Single 0.5m/0.3rad step: jx>0.10 -> T5_map_odom_jump (unit test with 0.15 m passes; logic identical for larger). Clears READY.
2. Across gap: gap block unchanged; T5_map_odom_jump suppressed only when across_gap already added (still not ready). Existing gap tests pass.
3. Residual risk: any drift made of per-sample steps <=0.10 m / 0.10 rad is NOT tripped, with no cumulative bound. Slow convergence to a wrong mode (e.g. 0.05 m/update over several updates = metres) was caught before only if >0.10 within 1 s; now relies on T4 scan score/drop, T6 anchor (anchor_xy_base 0.15 + LIO displacement) and T2/T3. T6 is the real backstop; if no anchor confirmed READY is already denied (T6_no_anchor). Evidence for the change is one short session (replay: only 8 distinct samples, max step 0.066 m).
   Also: comparison is per evaluator tick on held values; a continuous ramp republished at tick rate would be compared per tick (smaller steps, even less detectable).
4. Duplicates/same-stamp/high-rate republish: identical value -> step 0 (no trip, no masking since a real step still shows on the first changed tick). mo_hist keeps only last sample, updated every tick incl. after gap (gap path covers the gap).  Minor: no stamp is used; equal values, not stamps, give 0.
5. No other thresholds changed.
Runs: tests/loc_trust 69 passed; tests/motion_adapter 43 passed; replay_t5_bag.py: 8 samples, max step 0.066, OLD trips 2, NEW 0.

## per-update latch (2026-10-22)
- loc_trust.py: new `TrustEvaluator.observe_map_odom(value)`: compares each accepted map->odom value with the previous accepted one (same jump_xy_max/jump_yaw_max), LATCHES a violation (`step_latch`) until the next `evaluate()` consumes it -> T5_map_odom_jump (clears READY). Identical republish = step 0; non-finite ignored. evaluate() also feeds its tick value (no-op if already seen) so tick-only callers behave as before; gap block unchanged; mo_hist removed.
- localization_trust_node.py `_tf`: calls observe_map_odom for each stamp-accepted map->odom under the existing `self.lock` (same lock as `_tick` evaluate), so the 2-thread executor is safe. Backups: *.bak_20261022_pre_t5each.
- Tests (test_ready_motion.py, class PerUpdateLatch, 9 new): smooth 6x0.06, single 0.15 step, 0->0.5->0 between ticks, yaw out-and-back, identical republish, several <=0.10 updates, latch consumed once, gap+jump clears, gap w/o jump keeps READY. loc_trust 78 OK, motion_adapter 43 OK, nav_e2e stop_criterion OK, replay_t5_bag: NEW trips 0 (OLD 2, max per-update 0.066 m).
