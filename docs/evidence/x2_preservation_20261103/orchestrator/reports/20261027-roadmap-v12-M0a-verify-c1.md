# 20261027-roadmap-v12 M0a verify (cycle 1/3): independent verifier

VERDICT: PASS (with non-blocking notes). Live stack untouched: PIDs 86415/86488/86545/86641/86794/92536 were alive before and after. All tests ran with ROS_DOMAIN_ID=88, ROS_LOCALHOST_ONLY=1 and FASTRTPS_DEFAULT_PROFILES_FILE unset. No project files were edited.
Method: diffed each changed file against its *.bak_20261027_pre_m0a (and native_odom.py against *_pre_m0a_gap). native_odom_bridge.py is byte-identical to its _gap backup (cmp).

## Per-check results
1. Single TF owner / invalid source refused: **PASS**
   - localization.launch.py:88 has DeclareLaunchArgument odom_source with choices=['lio','native_imu'].
   - LevelBridgePublishTf (:50-62) raises on anything else, as a second guard.
   - level_bridge receives publish_tf:=true only for lio (:103). The native bridge is started only on LaunchConfigurationEquals native_imu (:127-134).
   - level_bridge.py:82-83,142: with publish_tf false no TransformBroadcaster is created at all.
   - No other node publishes odom->base_footprint. local_tf_relay.py can relay it but is not referenced by any launch or script (grep).
   - FAST-LIO's /tf goes to /qazbot_lio/tf.
   - The native node itself refuses mode != native_imu (bridge :68).
   - stack_up exits 2 on a bad QAZBOT_ODOM_SOURCE.
   - Note: the native process also gets a shutdown guard, so if it crashes the whole localization launch goes down (fail-closed).
2. base_footprint definition identical: **PASS**
   - LIDAR_FWD_IN_IMU is identical; the test asserts equality.
   - quat_to_rot and the atan2(f_y, f_x) yaw match level_bridge on_odom with R_lv = I.
   - Effect of the 2.2° tilt:
     - planar scale error 1 − cos 2.2° = 0.07%, i.e. 0.7 mm per metre;
     - vertical-to-horizontal leakage sin 2.2° = 3.8% of z motion, i.e. ~2 mm for a 5 cm bob;
     - yaw error is second order (~0.04°) plus a first-order term from the LiDAR's ~5° down-pitch, ≤ 0.0384·tan5° ≈ 0.19°. It varies with heading. That is ≤ 2.8 cm at the 8 m scan range_max, and AMCL absorbs it.
     - Negligible for x, y over metres.
   - MIN_FWD_XY 'tilt' drop exists only in native, on the fail-closed side.
3. Stamps and guards: **PASS**
   - TF and Odometry both use m.header.stamp (bridge :123, :133). The only now() use is in the guards and status.
   - Every non-ok path returns before both TF and Odometry (bridge :106-118):
     - non_finite: latch;
     - origin_reset: latch (native_odom.py :151-166);
     - stale: withheld;
     - future > 0.2 s: dropped;
     - foreign frame / tilt / duplicate: dropped.
   - The dt cap is verified: min(max(0, gap), 0.2) gives a bound ≤ 0.35 m (native_odom.py :119).
   - `latched` is never reset anywhere and the guard object is never replaced.
   - Notes (non-blocking):
     - (a) future stamps ≤ 0.2 s are published. Trust treats negative age as stale (`not 0<=now-odom_t`), so this fails closed.
     - (b) the yaw jump bound of 0.3 rad is not dt-scaled. A stall > ~0.4 s during a turn latches. This is an availability cost only.
     - (c) an origin reset < 0.35 m / 0.3 rad is undetectable. That residual is left to T5.
     - (d) Odometry covariance is copied without a finite check. No consumer reads it.
4. lio-mode semantics and trust: **PASS**
   - The level_bridge diff is only the publish_tf param, the TF-send guard and a kwargs ctor. odom_level and scan code paths are byte-unchanged.
   - Trust diff: odom_topic param (default = the old hard-coded topic), odom_source (informational), a log line, a mismatch warn and status fields.
   - No threshold or policy change.
5. Scripts with env unset: **PASS**
   - stack_up defaults to lio with ODOM_TOPIC=/qazbot_lio/odom_level. The values passed explicitly equal the previous defaults.
   - run_nav_full uses ${QAZBOT_ODOM_TOPIC:-/qazbot_lio/odom_level}.
   - stack_down only adds native_odom_bridge to the pattern.
   - check.sh is additive and read-only: the owner probe, plus /qazbot_native/odom in age_probe.
   - The PATTERN addition in stack_up is stricter. That is fine.
6. Test reruns (domain 88): **PASS**
   - tests/lio: 61/61 OK.
   - tests/motion_adapter: 43/43 OK.
   - tests/footprint: 5 OK (1 skipped).
   - tests/launch_stagger: 6/6 OK **only** when QAZBOT_SMAC_PREFIX is exported BEFORE `source env.sh`. With the order in the implementer's documented command (source first, then export), test_full_nav_launch_is_paced ERRORs with "Qualified Smac prefix is not selected in the ament index". This is a documentation/reproducibility error in the report, not an M0a defect.
   - tests/loc_trust:
     - real tree: 107 run, 56 F / 5 E (the 3 new test_odom_topic_param tests pass);
     - scratch copy with the pre-M0a trust node (and without the new test file): 104 run, 56 F / 5 E;
     - the FAIL/ERROR name sets are IDENTICAL (diff empty), so the failures are pre-existing and not caused by M0a.
7. Twist for RPP: **PASS**
   - Twist0p3 gives (vx, vy) in base_footprint axes (world displacement rotated by −yaw), in m/s, and wz = wrap(Δyaw)/dt in rad/s. Same as level_bridge, with a 0.3 s baseline and ~0.15 s lag.
   - The publisher is default reliable (depth 20), compatible with the Humble controller_server odom subscriber.
   - nav_control passes odom_topic to controller_server and velocity_smoother (OPEN_LOOP).
   - RPP max_angular_accel = wmax·20, so the measured wz clamp is effectively non-binding.

## Commands run
- `diff -u <file>.bak_20261027_pre_m0a <file>` for all 8 changed files, plus `diff native_odom.py.bak_..._gap` and `cmp` on the bridge.
- Test suites, each with `env -u FASTRTPS_DEFAULT_PROFILES_FILE ROS_DOMAIN_ID=88 ROS_LOCALHOST_ONLY=1 python3 -m unittest discover -p "test_*.py"`: lio, motion_adapter, launch_stagger, footprint.
  - launch_stagger was rerun with `export QAZBOT_SMAC_PREFIX` before `source env.sh`.
- loc_trust: real tree, plus a scratch copy (scratchpad/cbB) with the pre-M0a trust node. FAIL/ERROR sets were compared with sort and diff.
- Logs: /tmp/claude-1001/-home-agi/7c4b68ea-a3ae-4491-bba6-a5f62ecb9b4c/scratchpad/{tests1.txt, lt_control_backend.txt, lt_cbB.txt}.

## Required fixes
None blocking. Recommended:
- (R1) Correct the test command in the M0a report and test docstrings: export QAZBOT_SMAC_PREFIX before sourcing env.sh.
- (R2) Live bring-up must still confirm, with check.sh, that the probe reports owner_graph ["native_imu"] and that trust shows no T1_odom_stale. This was not verifiable offline.
- (R3) Track notes 3(b) and 3(c) (yaw-bound availability; sub-threshold origin reset relies on T5) as open risks.
