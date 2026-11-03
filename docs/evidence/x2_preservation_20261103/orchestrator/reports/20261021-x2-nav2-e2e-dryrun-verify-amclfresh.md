# 20261021-x2-nav2-e2e-dryrun — verifier: trust amcl-freshness fix

VERDICT: FAIL (two small required fixes; the core change is sound and detection of real AMCL loss is not weakened)

Scope reviewed: ws/src/navigation/loc_trust.py, localization_trust_node.py, tests/loc_trust/test_trust_gaps.py,
each diffed against *.bak_20261021_pre_amclfresh. Root cause: ...-trustflicker.md; implementation: ...-amclfresh.md.
Nothing was run on ROS domain 0; no processes were touched; no project files were edited.

## Commands run
- `diff -u <f>.bak_20261021_pre_amclfresh <f>` for all three files. Only the amcl/cloud freshness rule, two new
  TrustConfig fields, two new Inputs fields, and node plumbing changed.
- `cd tests/loc_trust && python3 -m unittest test_loc_trust test_source_failures test_trust_gaps` -> Ran 51, OK
- `cd tests/motion_adapter && python3 -m unittest discover -p 'test_*.py'` -> Ran 43, OK
- `cd tests/nav_e2e && python3 -m unittest test_stop_criterion` -> Ran 7, OK
- Mutation runs on a scratch copy of loc_trust.py (scratchpad/mut/run.sh), against the same 51 tests:
  - M1, revert the pose rule to `moving and age>3`: 5 tests fail. Killed.
  - M2, revert the cloud rule: 6 tests fail. Killed.
  - M3, overdue never true: 3 tests fail. Killed.
  - M4, overdue true as soon as due (no lag): 3 tests fail. Killed.
  - M5, drop the yaw criterion: 1 test fails (rotation test). Killed.
  - M6, no timer reset on a new stamp: 6 tests error. Killed.
  - M7, timer measured from the amcl stamp instead of the due time: 3 tests fail. Killed.
  - M8, odom_at None counts as due: 20 tests fail. Killed.
  - M10, age-only rule without moving: 7 tests fail. Killed.
  - M9, drop `amcl_cov is None` from T1_amcl_pose_stale: 51 OK. Survived. This gap existed before the change. In
    practice it is backstopped by T3_no_particles and T1_tf_stale, but no test pins "amcl never published ->
    T1_amcl_pose_stale".
- Offline node-logic exercise (scratchpad/mut/node_logic.py: imports the node module and calls the methods on a stub
  self, with no rclpy.init and no DDS):
  - odom_src span is 4.0 s, 41 entries at 10 Hz.
  - Interpolation at an amcl stamp 2.05 s old is exact (0.795 m).
  - Wrap-around yaw interpolation is correct.
  - When the stamp is outside the history, the fallback uses the latest odom (0.45 m lenient at 0.1 m/s with a 4.5 s
    old stamp).
  - A re-delivered or rejected amcl stamp still moves odom_at_amcl to the current odom (see F2).
- Behaviour sim (scratchpad/mut/diag.py): the trust rule is run against the nav2 AMCL update rule, copied from source.
- Nav2 source checked: candidates/nav2_smac_bounds/src/navigation2-1.1.18/nav2_amcl/src/amcl_node.cpp.
  - Defaults are update_min_a 0.2 (:201), update_min_d 0.25 (:205), resample_interval 1, tf_broadcast true and
    transform_tolerance 1.0.
  - shouldUpdateFilter is at :768-780; publish logic at :683-731.

## Per-criterion results
(a) Does the fix match the root cause without weakening real-loss detection? PARTIAL.
  - The semantics match H1: freshness applies only once AMCL is due, measured from the odom pose at AMCL's last
    output. The 3.0 s lag budget is unchanged, and `moving` no longer gates safety.
  - Real loss is still detected. Odom moves 0.30 m or turns 0.25 rad with no new stamp -> T1_amcl_pose_stale and
    T1_particle_cloud_stale at due+3.1 s, and not at due+2.9 s (tests confirm). The trust rule is never more
    lenient than AMCL, because hypot >= max(|dx|,|dy|).
  - BUT the due rule does not equal AMCL's (F1). The result is residual false withdrawals during slow diagonal
    motion, which is the same flicker class the fix targets.
(b) Can "odom_at None -> not due" hide a real failure? NO (PASS).
  - In the node, odom_at_amcl is None only when cov is None, which is already T1_amcl_pose_stale. Otherwise it is None
    only when odom was never seen, which is already T1_odom_stale. _inputs pins the reference as soon as odom exists
    (localization_trust_node.py:180-185).
  - AMCL never published: amcl_cov None -> T1_amcl_pose_stale. Also cloud None -> T3_no_particles, and map_odom None
    -> T1_tf_stale.
  - AMCL dies, walking or standing: map->odom (only AMCL publishes map->odom on /qazbot_d1_map/tf; local_tf_relay
    forwards only odom->base_footprint) stops -> T1_tf_stale. That fires after tf_max_age 1.0 s plus AMCL's 1.0 s
    postdating, so about 2 s, independent of motion. An existing test covers it
    (test_amcl_death_while_standing_is_seen_through_map_odom_tf).
  - Large AMCL lag (> about 2 s) is also caught by T1_tf_stale, because the map->odom stamp is scan stamp + 1.0 s.
  - Exception: F2, which is a narrow masking path that T1_tf_stale backstops.
(c) Interpolation, history and thread-safety: PASS, with notes.
  - The interpolation is correct (verified offline). The 4 s history is sound for AMCL latency up to 4 s, and
    T1_tf_stale catches anything beyond about 2 s.
  - Subscriptions and the timer share the default MutuallyExclusive group, so _tf, _amcl, _cloud and _tick are
    serialized. Only _tick mutates evaluator._due.
  - The service thread (_ready -> _inputs) can only perform an idempotent pin of odom_at_* to self.odom. Lists are
    replaced by rebinding, never mutated in place. I found no race that matters.
(d) No other thresholds changed: PASS.
  - The diff adds amcl_update_min_d 0.25 and amcl_update_min_a 0.2. pose_max_age_moving_s stays 3.0. All other
    TrustConfig values are unchanged.
  - The new fields automatically become ROS params (TrustConfig -> declare_parameter). run_trust.sh does not override
    them.
(e) Are the tests meaningful? PASS, with gaps.
  - Reverting the rule (M1/M2) kills 5-6 tests. Each component of the rule is killed by at least one test.
  - Gaps:
    - No test exercises the node wrapper: interpolation at the amcl stamp, the fallback, the pin, or rejected stamps.
    - No diagonal-motion test.
    - No `amcl_cov None` test (M9 survives).

## Findings
F1 (correctness, must fix): the due rule is Euclidean, but AMCL's is per-axis. Location: loc_trust.py:298-299
    `due = hypot(dx, dy) >= amcl_update_min_d or |dyaw| >= amcl_update_min_a`.
  - nav2 amcl_node.cpp:775-777 uses `fabs(dx) > d_thresh || fabs(dy) > d_thresh || fabs(dyaw) > a_thresh`, per axis
    in the odom frame.
  - For motion at 45 deg in odom, trust becomes due at 0.25 m but AMCL only at 0.354 m. At speed v, AMCL is silent
    for 0.104/v s longer than the trust node expects.
  - diag.py (AMCL modelled with the per-axis rule, 5 Hz ticks) shows false T1_amcl_pose_stale:
    - v=0.03 m/s diagonal: 16-50 stale ticks at every AMCL latency tested (0.4/1.0/2.0 s).
    - v=0.05 m/s diagonal with 2.0 s latency (the measured worst case): 36 stale ticks.
    - Axis-aligned motion: 0 stale ticks.
  - This is fail-safe in direction, but it is a false withdrawal during slow approach. It contradicts the
    TrustConfig comment that the values "MUST equal" AMCL, and the root cause's goal of removing slow-motion false
    withdrawals.
  - Fix: mirror shouldUpdateFilter, i.e.
    `abs(dx) > min_d or abs(dy) > min_d or abs(wrap(dyaw)) > min_a`. Detection of real loss is unchanged, because
    AMCL cannot publish before this condition holds, and T1_tf_stale still covers AMCL death.
F2 (safety, small, must fix): a rejected amcl_pose stamp still refreshes the reference.
  - Location: localization_trust_node.py:138-141. `_amcl` sets `odom_at_amcl = _odom_at(amcl_src) or self.odom` even
    when SourceStamp rejects the stamp (amcl_t = -inf: duplicate/older, future, or zero).
  - For an old stamp outside the 4 s history this falls back to the current odom. The stamp key also changes, so the
    due timer resets.
  - Reproduced offline: AMCL output at 307.9 with odom at 0.79 m; the robot moves to 1.40 m; the same message is
    re-delivered -> odom_at_amcl = 1.40 m and amcl_t = -inf. The gate treats this as "AMCL just updated".
  - The old code treated a rejected stamp as stale while moving.
  - The scenario is rare (needs a TRANSIENT_LOCAL re-delivery or clock skew) and T1_tf_stale backstops AMCL death.
    It is still a silent reset of a safety timer by invalid input.
  - Fix: update amcl_src and odom_at_amcl only when amcl_t is finite. Apply the same guard in `_cloud`
    (cloud_t finite).
F3 (non-blocking): the particle_cloud reference is the odom at receipt (localization_trust_node.py:149).
  - This is later than AMCL's scan-time reference, so the cloud check is lenient by about v x AMCL latency (up to
    0.6 m at 0.3 m/s with 2 s latency).
  - Cloud and pose are published in the same AMCL callback (amcl_node.cpp:700/703), and the pose check uses the
    correct reference, so detection is not lost.
  - Optional: use odom_at_amcl from the paired amcl_pose.
F4 (non-blocking, coupling): AMCL's update_min_d/update_min_a are not set explicitly in localization.launch.py:49-54.
  - The coupling exists only by comment plus a constant test.
  - If AMCL thresholds were ever lowered, trust would become lenient (delayed detection; T1_tf_stale still holds).
  - Recommended in the root-cause report and not done: set both explicitly in the launch file and pass the same
    values to the trust node.
F5 (non-blocking, coverage): add these tests:
  - diagonal motion with the per-axis rule;
  - a rejected/duplicate amcl stamp must not reset the reference (node-level, stub as in node_logic.py);
  - amcl_cov None -> T1_amcl_pose_stale.

## Required fixes
1. loc_trust.py:298-299: use AMCL's per-axis due rule
   `abs(dx) > d or abs(dy) > d or abs(wrap(dyaw)) > a`, and add a 45-deg slow-motion test (0.30 m diagonal -> not
   due; 0.26 m along one axis -> due).
2. localization_trust_node.py:138-141 (and :147-149): do not update amcl_src / odom_at_amcl (odom_at_cloud) when the
   source stamp is rejected (amcl_t / cloud_t == -inf). Add a test that a duplicate stamp does not move the
   reference.
Then re-run the 51 / 43 / 7 suites.
