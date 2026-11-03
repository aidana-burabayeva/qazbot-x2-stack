# P23 20261030-yawloop-fix: yaw-rate loop integral/small-target fix (candidate, NOT deployed)

Evidence: EV=/home/agi/control_backend/docs/evidence/nav_20261030_1623 (adapter.log, goal_bag_mcap_162723 [unfinalized mcap, read with StreamReader], nav_full.log, goal_watch.log).
Running file untouched: x2_motion_adapter.py sha256 9c05bb34... Candidate: `x2_motion_adapter.py.p23_candidate` sha256 c76ac1d1..., diff `20261030-p23-yawloop-fix.diff` (114 lines). Replay scripts: `reports/p23_replay/` (extract.py, tl.py, tf.py, replay.py, synth.py; window pickle in the session scratchpad).
Not restarted. No unit-test suites. py_compile OK.

## Root causes (evidence)
1. **I=+0.00 is not a broken integrator: no pure-rotation run lasted past `yaw_settle_s`=0.6 s.** Before P23, every exit called `reset()`. Replaying the bag (cmd_vel_authorized at receive time + /qazbot_native/odom) through the OLD AdapterCore reproduces all 5 live log lines (target/meas/u/I match to 0.01). It shows 7 loop segments of 0.34/0.16/0.34/0.42/0.12/0.22/0.68 s. Integration ran only in the last 0.08 s, so max |I| was 0.004. The runs were cut by:
   - RPP vx ramps (|vx| ≥ 0.02, at 87.18).
   - A cmd gap of 0.46 s (94.29→94.75; cmd_timeout is 0.15 s).
   - Trust-gate zero pauses. nav_full: "trust lost while busy" at 94.886, 96.035 and 97.44; zero commands at 94.89, 96.04 and 97.44.
   - Native-odom gaps of up to 0.69 s.
2. **Small-target inflation.** With `yaw_w_min`=0.05, any |w| > 0.05 was lifted to u_min. At 87.84 RPP reversed through -0.003→+0.072→+0.122. The loop output +0.60 (CCW u_min) while the robot was turning CW at -0.35 rad/s. That matches the live log line `target=+0.07 meas=-0.35 u=+0.60`. The old replay has 9 ticks with |w| < 0.15 lifted to |u| ≥ 0.4, 3 of them against the measured motion.
3. **CW is not weak; the log samples the dead time.** The 1 Hz log fires on the first tick of a segment, so its meas values (-0.07, -0.23, +0.04, -0.13) are inside the MC dead time. The replay measured -0.58…-0.72 rad/s at 94.16–94.40 and -0.50…-0.53 at 97.20–97.38, with u ≈ -0.62 for a target of 0.6. That fits the YAW_FF CW curve (0.54→0.6). `yaw_u_min_cw` only matters for CW targets below about 0.46 rad/s, and this run had none in the loop. The data does not support raising it to 0.6, which would overspeed 0.15–0.45 targets from stand. **Kept at 0.45.** P20's after-walk CW case (-0.58 with u 0.77–0.86) shows the integral working when the run is long enough.

## Changes (candidate)
- `yaw_w_min` 0.05 → **0.15**. Below this the command passes through raw, with no u_min lift, and the loop pauses. Why 0.15: RPP ramps wz by about 0.05–0.075 per 20 Hz cycle (1.0–1.5 rad/s²; see 87.81→88.13 and 93.53→94.08), so |w| < 0.15 lasts at most 2–3 cycles during a ramp or reversal. The MC cannot render it anyway (deadband: u 0.4 CW / 0.6 CCW gives 0.27/0.45 rad/s). The end of the turn is left to RPP and the goal checker.
- New `YawRateLoop.pause(now, stopped)`. An exit now pauses the loop instead of resetting it: the output goes back to the caller, and I is kept and frozen.
  - **I is reset only on** a sign change of the TARGET, a pause longer than `yaw_hold_s`=1.0 s (new), or a latch / `yaw_loop`=false.
  - After a stop-type pause (zero/drain, stale measurement, |w| ≤ w_min), the MC dead time applies again: I stays frozen for `yaw_settle_s`, so no windup during dead time.
  - After a vx-ramp exit (robot still turning), the loop resumes without a new settle.
- `_drain` pauses the loop (reset only if LATCHED). Zero input still gives an immediate zero output. `step()` stores `_t_step` for this.
- Existing anti-windup is unchanged: conditional integration, the clamp to [u_min-ff, u_max-ff], and the overspeed bleed.

## Offline verification
- Real trace replay, old vs new (same inputs):
  - Small-target lifts: 9 → **0**.
  - Amplified against the measured motion: 3 → **0**.
  - The 87.84 inversion is gone: +0.07/+0.12 pass through, and the loop enters at +0.17.
  - I is still about 0 on this trace for both versions. Every pause in this run was a stop-type pause (trust gate, cmd gap), and each segment after it was ≤ 0.68 s. That is correct behaviour (no integrating through the dead time), and the measured rate reached the target anyway (cause 3).
- Synthetic plant: after-walk gain 0.6, dead time 0.4 s, lag 0.2 s. Pattern: CW 0.6 with a vx blip (0.15 s), a 0.7 s trust zero and a 0.05 s small-w dip, as in this run.
  - OLD: I resets at every break, max |I| = 0.09, total turn -118°.
  - NEW: I grows 0 → 0.15 and holds across breaks (u → -0.77, meas -0.59…-0.65), total turn -163° against an ideal of -175°.
- Unit checks: I is kept after a non-stop pause, reset after a pause > hold (1.2 s), and reset on a target sign change.

## Goal checker: why ~30° short was "Reached"
The launch overrides `yaw_tol:=0.35` (= 20.05°; the default is 0.10, and the start_nav.sh comment says 10°). goal_checker_stateful is true.
- Controller view: the localizer post-dates map->odom by about +0.95 s (stamp - recv = 0.86–0.97 s). So tf2, looking up at the odom->base_footprint stamp, uses a map->odom that is about 0.9 s old: 172.9° instead of 175.3°, 2.4° behind.
- With that, the TF yaw error crossed the tolerance at 97.359 (-19.69°). Nav2 logged "Reached the goal" at 97.393. The robot was still turning at about 0.5 rad/s with cmd -0.6.
- Latest-TF error at that moment was -21.7°. /amcl_pose said -22.9° (stamp 0.09 s old). The controller's view was therefore about 3.5° more optimistic than AMCL.
- After the stop, the yaw rolled back about +11° in odom (21.8° → 33.1° by 99.8), and map->odom moved another +0.3°. Final: TF -32.3°, AMCL -30.0° (goal_watch -30.0°).
- So the 30° error = about 20° accepted at the tolerance edge + about 3° from map->odom post-dating + about 10° post-stop roll-back. The yaw loop is not the cause, and the adapter fix cannot remove it.
- Recommendations (not done, outside scope):
  - Set `yaw_tol` back to 0.10–0.17.
  - Give RPP a slower final rotate-to-heading, so it is not at full -0.6 when the checker latches.
  - Review the localizer's map->odom future-dating of about 0.95 s (the transform_tolerance-style offset).
  - The trust-gate pauses during fast in-place rotation (3 in 2.5 s) also break the turn.

## Deploy note (for orchestrator)
To apply: `cp x2_motion_adapter.py.p23_candidate x2_motion_adapter.py` and restart the adapter. Also sync the qazbot-x2-stack candidate copy. Live check: the yaw_loop log should show `I` non-zero on CW/CCW turns longer than 1 s, and there should be no `u=±0.60` with |target| < 0.15. Optional log improvement (not done): log at segment end rather than on the first tick, so dead-time meas values are not mistaken for weak response.
