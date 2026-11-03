# 20261103-odom-standing-hold -- independent verification (V)
VERDICT: PASS (candidate code) -- with deploy-blocking condition F1 (hold would be silently OFF if deployed as described).
Standard: ~/.claude/standards/henki_ros2_best_practices.md checked.

## Scope / integrity
- Running bridge untouched: ws/src/lio/native_odom_bridge.py mtime 2026-10-27 17:32; ws/src/lio/odom_hold.py absent. No ws/src/lio file modified.
- native_odom_bridge.diff == fresh `diff -u running candidate` (DIFF_MATCHES); 36 +/- lines: docstring, import, 2 params
  (standing_hold default true, state_topic), lazy McCommonState subscription (ImportError -> WARN, hold=None), on_state,
  clear_window on stamp_back, hold.update after guards, twist zeroed while held, status JSON `hold` field. No scope creep.
- Tests ran on ROS_DOMAIN_ID=88, ROS_LOCALHOST_ONLY=1 (fixture refuses domain 0). Replays are offline (no ROS).

## Commands run
1. pytest test_odom_hold.py + test_odom_hold_bridge.py (env.sh + aimdk_env.sh, domain 88): `15 passed in 1.73s`, rc=0.
2. Own replay checker (scratchpad/v/check.py, uses candidate replay.replay = real native_guard + StandingHold, bag receive order):
   - nav_bag_183504 (standing + 25 mm shove): 35323 samples, 11 entries/10 exits; non-held step diff (pub-raw) max 0.0;
     transitions 21, max published step at transition 0.172 mm == raw step (no jump); held while state not standing 0;
     offset changes outside hold 0; stamps strictly increasing; longest held run 155.7 s, raw drift absorbed 26 mm
     at 0.19 mm/s (= vendor drift ~1.1 cm/min); shove window (160-170 s) published 9.4 mm vs raw 8.1 mm (passed through,
     drift component removed).
   - nav_bag_183911 (walking): 8872 samples, 7 entries/6 exits; non-held step diff 0.0; transition step 0.081 mm == raw;
     held while not standing 0; offset changes outside hold 0; max raw excursion in last 0.5 s before any exit 0.445 mm
     (<= D_POS); the 1 state exit came 4 ms after the last held sample (state leads motion at walk start, released in time).
3. `source env.sh; import aimdk_msgs` -> ModuleNotFoundError (see F1).

## Per-criterion
(1) Diff minimal; identical outside hold -- PASS. Published = raw - offset, offset constant outside hold (0 changes in
    both bags), increments identical (diff 0.0). Bit-identical absolute x,y only until the first hold; afterwards a
    constant offset (odom frame origin shift, REP 105-legal, continuous). Claim "bit-identical walking" holds for increments.
(2) Gate -- PASS (data-justified), residual risk documented below. is_standing exactly fsm 4 & STAND_DEFAULT & 100 & body 1
    (odom_hold.py is_standing); state age <= 1.0 s (2x observed max gap 0.50 s). Fail-open verified by code + unit tests:
    no state (state_t=-inf), stale state (>1 s), other state, no aimdk_msgs -> no hold = today's pass-through. D_POS 0.45 mm /
    D_YAW 0.2 mrad = measured maxima of 0.5 s window excursion over 1.09 M quiet standing samples; exceedance only releases.
    Worst-case hidden motion: a steady ramp < D_POS/WINDOW = 0.9 mm/s (5.4 cm/min) stays held INDEFINITELY while MC says
    standing -- bounded in rate, NOT in total (implementer's "bounded" applies to rate only). Measured real held rates
    max 0.19 mm/s, so the gate has ~4.7x headroom over the drift. Pushes (25 mm/1 s), walk starts (>=0.17 m/s), turns
    (fsm 5) release within samples. True foot slip is invisible to leg odometry regardless of the hold; AMCL corrects
    map->odom. Stale/missing MC -> fails open after <=1 s. Frozen-but-alive MC stream reporting standing cannot be
    detected (pose gate still releases on motion > 0.9 mm/s).
(3) No jump / yaw / twist / stamps -- PASS. Entry hx = x - ox (current published), exit keeps offset -> continuous
    (transition step == raw step). Yaw always raw (test_yaw_is_never_modified). Twist set (0,0,0) only when held and
    tw not None (no new Odometry before twist baseline). TF and Odometry stamps = source header stamp, never re-stamped ->
    keep advancing while held (Trust T1/T5 freshness, AMCL/costmap TF lookup unaffected). Frames/covariances unchanged.
    angular.z zeroed while held is consistent: held requires yaw within 0.2 mrad over 0.5 s (<0.4 mrad/s).
(4) pytest re-run 15/15 PASS; two replay bags independently checked (above).

## Findings
F1 (deploy-blocking, reproducibility) ws/src/navigation/localization.launch.py:132-139 starts the bridge via
   run_localization.sh which sources only env.sh (run_localization.sh:4); aimdk_msgs is NOT importable there
   (verified ModuleNotFoundError). Deployed as described ("copy 2 files + restart"), the bridge logs a WARN and runs
   pass-through: the fix silently does nothing. stack_up.sh:59 wait pattern still matches, so nothing would catch it.
F2 (advisory) odom_hold.py quiet(): only a 0.5 s window -> unbounded cumulative hidden displacement at <0.9 mm/s.
   Optional hardening: release when absorbed offset since hold entry exceeds e.g. 0.4 mm/s * held_time + 1 mm
   (2x measured drift rate), bounding hidden motion to ~drift rate.
F3 (standard deviation, accepted as legacy) params declared in code / no YAML, no README entry for new params,
   std_msgs/String status -- legacy package style kept; new logic correctly separated into pure odom_hold.py with tests.

## Required fixes (before deploy)
1. Make aimdk_msgs importable for the bridge process ONLY (e.g. ExecuteProcess additional_env / wrapper sourcing
   aimdk_env.sh for native_odom_bridge), not for the whole localization launch (avoid prefix shadowing for FAST-LIO/AMCL).
2. Post-deploy acceptance: startup log contains `standing_hold=True` and /qazbot_native/status `hold` != null with
   standing=true/held=true while robot stands; no `standing hold disabled` WARN.
