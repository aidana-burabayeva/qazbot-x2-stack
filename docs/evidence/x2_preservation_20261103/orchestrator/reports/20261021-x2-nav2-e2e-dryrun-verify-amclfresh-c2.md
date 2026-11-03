# 20261021-x2-nav2-e2e-dryrun — verifier, cycle 2/3: trust amcl-freshness fix

VERDICT: PASS. Both cycle-1 required fixes are correctly implemented. The remaining items are non-blocking test gaps.

## Scope and constraints
- Diffed against *.bak_20261021_pre_amclfresh:
  - ws/src/navigation/loc_trust.py
  - ws/src/navigation/localization_trust_node.py
  - tests/loc_trust/test_trust_gaps.py
- No ROS processes were started, no domain-0 CLI was used, and no project file was edited.
- Mutation testing ran on a scratch copy: /tmp/claude-1001/-home-agi/c9b41d51-5b1b-455f-88cf-eb93053f257e/scratchpad/cb, driven by mut.py.
- Files modified since the backup are limited to the three above (plus runtime logs). No scope creep.

## Commands run
- `diff -u <f>.bak_20261021_pre_amclfresh <f>` for the 3 files.
- `cd tests/loc_trust && python3 -m unittest -v test_loc_trust test_source_failures test_trust_gaps` -> Ran 56, OK.
  - The 2 NodeStampReject tests actually ran. They were not skipped.
- `cd tests/motion_adapter && python3 -m unittest discover -p 'test_*.py'` -> Ran 43, OK.
- `cd tests/nav_e2e && python3 -m unittest test_stop_criterion` -> Ran 7, OK.
- Checked the nav2 source: /home/agi/control_backend/candidates/nav2_smac_bounds/src/navigation2-1.1.18/nav2_amcl/src/amcl_node.cpp
  - :200/:204 give the defaults update_min_a 0.2 and update_min_d 0.25.
  - :768-780 is shouldUpdateFilter: per-axis deltas of the odom-frame pose at the scan stamp, strict '>', angle_diff for yaw.
  - :650-653 is getOdomPose at laser_scan->header.stamp.
- Mutation runs against the 56 tests:
  - E1 Euclidean hypot>= rule: 2 fail. Killed.
  - E2 '>=' on the x axis: 1 fail. Killed.
  - E3 '>=' on yaw: OK. Survived (the test uses 0.199, not exactly 0.2).
  - E4 dy axis dropped: OK. Survived (no pure-y or y-dominant test).
  - E5 wrap() dropped: OK. Survived (no ±pi crossing test; the mutation is fail-safe, so it is stricter).
  - E6 _amcl reject guard removed: 1 fail. Killed.
  - E7 _cloud reject guard removed: 1 fail. Killed.
  - E8 `amcl_cov is None` dropped: OK. Survived (pre-existing M9 from cycle 1, F5 item not addressed).
  - E9 x check replaced by y: 2 fail. Killed.

## Per-criterion
1. Fix 1 (per-axis due rule): PASS.
   - loc_trust.py:300-302 is `abs(dx)>d or abs(dy)>d or abs(wrap(dyaw))>a`. This is the same as amcl_node.cpp:775-777, including strictness.
   - The deltas are odom-frame and not rotated, which matches AMCL: pf_odom_pose_ comes from getOdomPose in odom_frame_id.
   - The reference is right: node odom_at_amcl is interpolated at the amcl_pose header stamp, which is the scan stamp. AMCL's pf_odom_pose_ is the pose at that same scan stamp.
   - The required diagonal tests exist: 0.30 m at 45 deg is not due, 0.36 m at 45 deg is due, and exactly 0.25 m is not due.
2. Fix 2 (rejected stamp must not refresh the reference): PASS.
   - _amcl (node:138-144) and _cloud (node:148-155) return before touching cov, amcl_t, amcl_src, odom_at_amcl, cloud, cloud_t or odom_at_cloud when SourceStamp returns -inf.
   - Node-level tests cover duplicate, older and newer stamps for amcl, and duplicate and newer stamps for cloud. Mutations E6 and E7 are killed.
3. The widened fix 2 (ignore the whole message) does not hide a real AMCL failure: PASS.
   - Stale latched or republished pose (same or older stamp): ignored. amcl_t and the _due key are unchanged, so the due timer keeps running. Once odom moves past AMCL's own update threshold, T1_amcl_pose_stale and T1_particle_cloud_stale fire after 3.0 s. That is the same detection as "AMCL silent", and it is stricter than the pre-cycle-1 code, where the rejected stamp reset the reference.
   - Backward clock jump: SourceStamp.last is never reset, so every later stamp is rejected.
     - Old code: cov updated, amcl_t=-inf, stale only if `moving`.
     - New code: cov and stamp are frozen and become stale once due + 3 s.
     - The same jump also rejects odom TF stamps (T1_odom_stale after 0.5 s), map->odom stamps (T1_tf_stale after 1 s) and scan stamps (T1_scan_stale). Trust is withdrawn within about 1 s regardless.
   - Future stamps (skew): amcl_pose carries the scan stamp, so scan (future=0) and map->odom (stamp+1.0 tolerance against the 1.0 future allowance) are rejected too. T1_scan_stale and T1_tf_stale fire.
   - Frozen cov: only messages with invalid stamps are dropped. Any genuinely new AMCL output has a newer stamp and is still evaluated by the cov checks (T2), and the particle cloud by T3.
   - AMCL never published: cov None -> T1_amcl_pose_stale, and cloud None -> T3_no_particles. Both are unchanged.
   - AMCL death while standing: test_amcl_death_while_standing_is_seen_through_map_odom_tf still passes.
   - T1/T2/T3 behaviour: all pre-existing T1/T2/T3 tests in test_loc_trust and test_source_failures pass unchanged (only test_trust_gaps changed).
4. No other thresholds changed: PASS.
   - The only config additions are amcl_update_min_d 0.25 and amcl_update_min_a 0.2. pose_max_age_moving_s stays 3.0. No other TrustConfig value differs from the backup.
   - No launch or script file overrides the new params.
5. Required verification (56/43/7): PASS, re-run as listed above.

## Status of cycle-1 non-blocking notes
- F3 (cloud reference is odom at receipt, lenient by v x latency): OPEN, unchanged (node:155). The pose check uses the correct scan-stamp reference and detects the same failure. Non-blocking.
- F4 (AMCL update_min_* not set explicitly in localization.launch.py): OPEN. The launch file is unchanged and the coupling is still held only by the comment plus test_amcl_thresholds_match_nav2_defaults. Non-blocking. Detection would be delayed, not lost, because T1_tf_stale still holds.
- F5 (coverage):
  - Diagonal test: DONE.
  - Node-level rejected-stamp test: DONE.
  - `amcl_cov None -> T1_amcl_pose_stale` test: NOT DONE (E8 survives).
- New observations, non-blocking:
  - N1: there is no pure-y-motion test. E4 (dropping the dy term) survives, and that mutation would make trust lenient for y-dominant motion. Suggest adding `due_after((0.0, 0.26, 0.0))` -> True.
  - N2: the exact-threshold yaw test uses 0.199 rather than 0.2, so yaw strictness is not pinned (E3). This has negligible practical impact.
  - N3: a late-joining trust node accepts AMCL's TRANSIENT_LOCAL latched pose once (SourceStamp.last None). Its stamp is outside odom history, so the reference falls back to the current odom, which is lenient. This is pre-existing, accepted in cycle 1, and backstopped by T1_tf_stale.

## Required fixes
None.

## Not verified
- Runtime behaviour on the live stack (no restart of the trust node, no e2e).
- The actual running AMCL update_min_d/update_min_a values (param get is forbidden).
