# 20261021-x2-nav2-e2e-dryrun-locB — independent verification (cycle 1/3)
Date 2026-10-22 ~20:45, host .66. Verifier: read-only, except that tests ran under nice -n 19 on isolated DDS domains 86/87/89 with ROS_LOCALHOST_ONLY=1 and FASTRTPS profile unset. No live node was touched: trust PID 312093 and localization launch PID 314304 run with LOCALHOST_ONLY=0 on the default domain. nav_e2e was not run. No motion.

**VERDICT: PASS** (advisories only, no required fixes)

## Diff reviewed (against *.bak_20261022_pre_locB)
- pose_store.py (new)
- seed_pose.py
- localization_trust_node.py
- localization.launch.py
- run_trust.sh
- docs/X2_LOCALIZATION_TRUST.md
- tests/loc_trust/test_pose_persist.py and test_seed_ros.py (new)

loc_trust.py has mtime 19:34, which is before the locB work and equal to the pre-locB backup time of the trust node. It is not in the diff. lio/x2.yaml has mtime 10-20. In the AMCL Node() params of the launch, the only change is the appended seed process.

## Per-criterion results
1. **The seed never grants READY and never becomes the anchor: PASS.**
   - seed_pose only publishes /qazbot_d1_map/initialpose.
   - The trust node `_initialpose` calls `ev.relocalize()`, which sets anchor=None, ready=False, pending=False, cause=initialpose (loc_trust.py:317-319).
   - A freshly started trust node has anchor None and cause no_anchor.
   - READY is set only in `_verdict` while `pending`, and `pending` is set only by `confirm_anchor` (loc_trust.py:310-314, 499-504).
   - Test SeededPoseIsNotReady: 30 clean ticks give no READY, then READY after confirm.
2. **Save only at the READY rising edge, or at clean stop when READY+MOTION_OK: PASS.**
   - `ConfirmedPoseSaver.on_verdict` uses a rising-edge latch; `on_shutdown` requires ready, motion_ok and no diagnostic except T4_no_baseline.
   - It is called after every `_tick` evaluation (node:222) and in `main()` finally (node:303).
   - I verified the shutdown path myself with the real `TrustNode` + `main()` + SIGINT (domain 87):
     - frozen READY/MOTION_OK verdict: file written, source=shutdown, pose = compose(map_odom, odom), saved covariance;
     - not READY: "not refreshed", no file.
3. **Atomic write: PASS.** The write goes to `path.tmp`, then fsync, then `os.replace` (pose_store.py:48-55). The test with a failing replace keeps the old file. Minor: there is no directory fsync. After a power loss the rename may be lost, but the result is still the old or the new file, never a partial one.
4. **Map sha mismatch, corrupt or missing file gives no publish: PASS.**
   - `load()` never raises and returns a reason.
   - seed_pose exits 0 before importing rclpy (test SeedRefusals: no_file, map_sha256_mismatch, unreadable; argparse misuse exits 2).
   - The sha is the image sha256 of the launch's map_yaml, so a trust node and launch on different maps (v2 vs v3) fall into "mismatch → no seed". That is correct.
5. **Exactly one publish with the saved covariance, only after the AMCL subscriber is matched, with no sleeps or magic timeouts: PASS.**
   - The wait condition is that the graph has node `amcl` in /qazbot_d1_map, every discovered subscriber is matched, and an odom->base_footprint TF has been seen. The spin uses a 0.1 s poll and has no deadline.
   - AMCL creates the initialpose subscriber in `on_configure` (amcl_node.cpp:237/247, SystemDefaults QoS, compatible with a reliable depth-1 publisher). An initialpose received while inactive is stored and applied in `on_activate` (amcl_node.cpp:280-281, 542-547). `handleInitialPose` copies cov xy 2x2 + [35] (amcl_node.cpp:600-608).
   - Re-ran test_seed_ros on domain 89: 1 OK in 10.9 s. The seed waits while only a trust-like subscriber exists, then sends exactly 1 msg to each subscriber with frame map, the pose, and the covariance equal to the saved one.
   - SIGINT and SIGTERM while waiting: I ran both; each gives exit 0 with "nothing seeded; shut down while waiting".
   - The only fixed constant is the existing 5 s `wait_for_all_acked` from the manual path. On failure it raises, gives a non-zero exit and is not repeated. Acceptable.
6. **Seed failure does not kill the launch: PASS.** The guards list is built before `actions.append(seed)` (launch.py:93-94). The test checks that every other ExecuteProcess is still guarded and the seed is not. The seed is staggered as the 7th start (+3.6 s, under the vendor 2 starts/s limit).
7. **Trust criteria, AMCL params and FAST-LIO unchanged: PASS.** Covered by the diff review above. TrustConfig and the GATE_SKIP set are unchanged.
8. **pose_store default off: PASS.**
   - The node param default is `''`. run_suite_real.py, model_test_step_cmd.py and run_ros_synthetic.py start the node directly without it, so they write nothing.
   - The later `-p "pose_store:=''"` override in run_trust.sh resolves to `''` (verified with an rclpy param parse).
   - Note: the launch-side `seed_pose_file` defaults to ON. It is harmless because the file is created only by run_trust.sh trust nodes after an operator-confirmed READY. No file exists yet at ~/.ros/qazbot_x2_last_confirmed_pose.json.
9. **No regression: PASS.**
   - `python3 -m unittest test_loc_trust test_ready_motion test_source_failures test_trust_gaps test_pose_persist test_seed_ros` (domain 87, isolated) gave **Ran 93, OK (skipped=1)**.
   - tests/launch_stagger gave 5 OK and 1 ERROR. The error is `test_full_nav_launch_is_paced`, which needs `QAZBOT_SMAC_PREFIX`. It is pre-existing and unrelated; `test_localization_launch_is_paced` passes.
   - py_compile passes on all 4 modules; `bash -n run_trust.sh` passes.
10. **Other trust-node behaviour unchanged: PASS.**
    - The changes are: the saver call in `_tick` (its exceptions are confined: `save` errors are caught as OSError/ValueError), `_pose_cov`/`save_on_shutdown`, the map sha via the same computation, and the removal of the unused hashlib import.
    - Status JSON fields, subscriptions, QoS, services and locks are unchanged.
11. **Docs match code: PASS (minor).** The docs reason list omits `pose` (pose_store._check can return 'pose'). Everything else matches: paths, ROS_HOME, SIGINT-only shutdown save, T4_no_baseline exemption, launch arg, no timeout, test counts 93.
12. **Stale file after the robot was moved: PARTIAL guard, as designed; advisory.** I ran an offline T4 experiment on the real v2/v3 maps: 40 random free poses, simulated ±52° scan, current `score_sigma=0.05`, `hit_min_abs=0.35`. These are the scan_score >= floor pass counts at the displaced pose:

    | displacement | v2 pass | v3 pass |
    |---|---|---|
    | 0 | 40/40 | 40/40 |
    | 0.2 m | 21/40 | 21/40 |
    | 0.5 m | 4/40 | 4/40 |
    | 1.0 m | 2/40 | 2/40 |
    | 10° | 11/40 | 11/40 |
    | 20° | 3/40 | 3/40 |

    - So T4 rejects most moves of 0.5 m or more and 20° or more, but not small moves, and not every large one.
    - The seeded covariance is AMCL's tight confirm-time covariance. T2 therefore passes immediately, and AMCL may stay at, or converge locally near, the stale seed.
    - The remaining guards are: the operator's visual check before `confirm_anchor` (docs step 4/5), T5/T6 after confirm, and T8 for a map change. This matches spec §5/§8 ("T4 ... must pass; otherwise operator re-poses"), so it is not a FAIL. It must stay in the on-robot verification and operator briefing.

## Advisories (no fix required for PASS)
- A1: Add 'pose' to the docs reason list.
- A2: The READY-edge save can happen with `T7_*` present, because GATE_SKIP includes T7_. The shutdown save excludes T7. These are inconsistent but operator-confirmed. Consider documenting this, or skipping the save when T7 is present.
- A3: The stale-pose risk above. Optionally seed with an inflated covariance (e.g. max(saved, manual-seed std)) so that T2/T3 must re-converge with nomotion updates, instead of passing T2 at once. That would be a design change and needs the coordinator's decision.
- A4: There is no "same LIO run" map<-odom branch (§8). The implementer declared it out of scope (no LIO run id).
- A5: On-robot check still open: relaunch while standing, then measure the time and the nomotion count to READY with no RViz initialpose. The live trust node still runs the old code.

## Commands run (all nice -n 19)
- diff -u of each *.bak_20261022_pre_locB against the current file (5 code files + docs)
- unittest of the 6 loc_trust modules, domain 87, LOCALHOST_ONLY=1: 93 OK, 1 skip
- test_seed_ros, domain 89: 1 OK
- tests/launch_stagger: 5 OK + 1 pre-existing env error
- py_compile of the 4 modules; bash -n run_trust.sh
- scratchpad shut.py (real TrustNode.main + SIGINT, ready / notready)
- scratchpad seed_pose SIGINT / SIGTERM during the wait (domain 86)
- scratchpad t4exp.py (offline T4 displacement sweep, v2 and v3 maps)
- rclpy check of the param override (`pose_store:=''`)
- AMCL 1.1.18 source review (amcl_node.cpp:237-281, 520-611, 1521-1540)
