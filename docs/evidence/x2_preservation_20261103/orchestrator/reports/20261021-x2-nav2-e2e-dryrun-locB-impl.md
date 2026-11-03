# 20261021-x2-nav2-e2e-dryrun-locB — Step B implementation: persist + re-seed the CONFIRMED localization pose
Date 2026-10-22 ~20:40, host .66. Role: robotics-implementer. Source design: reports/20261021-x2-nav2-e2e-dryrun-locpersist.md §5 Step B, §8.
No live node was restarted, killed or reconfigured. No robot motion, no MC calls. Logs and bags were not touched.
FAST-LIO, AMCL params and the trust READY criteria are unchanged. The running trust node (PID 312093) still runs the OLD code; the change takes effect only after a restart (not done).

## Changes (control_backend is not git; backups `<file>.bak_20261022_pre_locB`)
- NEW `ws/src/navigation/pose_store.py` (pure Python, no ROS):
  - `DEFAULT_PATH` = `$ROS_HOME|~/.ros/qazbot_x2_last_confirmed_pose.json`.
  - `map_image_sha256(yaml)`: the same identity as the trust node's `map_sha256`; it equals manifest `pgm_sha256` (tested).
  - `save()`: atomic write (`.tmp`, fsync, `os.replace`). Refuses a non-finite pose or a covariance that is non-PD in xy or has yaw<=0.
  - `load()` returns `(rec, '')` or `(None, reason)` and never raises. Reasons: `no_file`, `unreadable`, `format`, `map_sha256_mismatch`, `pose`, `covariance`.
  - `ConfirmedPoseSaver`:
    - `on_verdict()` saves ONCE per READY rising edge, only when map_ok == ''.
    - `on_shutdown()` saves only if the last verdict is READY and MOTION_OK and the only diagnostic, if any, is `T4_no_baseline`.
- `localization_trust_node.py`:
  - New param `pose_store` (default `''` = off, so nav_e2e and the synthetic tests never write the real file).
  - `_tick` calls `saver.on_verdict(r, compose(map_odom, odom_base), amcl_cov, map_ok)`.
  - `main()` finally calls `save_on_shutdown()`.
  - The map sha now comes from `pose_store.map_image_sha256` (the same computation, de-duplicated). Unused `hashlib` import removed.
- `run_trust.sh`: passes `-p pose_store:=<DEFAULT_PATH>`. A later `-p "pose_store:=''"` disables it.
- `seed_pose.py`:
  - New mode `--from-file PATH --map-yaml YAML`. With a missing, corrupt or other-map file it prints the reason and exits 0 BEFORE importing rclpy.
  - Otherwise it publishes ONE PoseWithCovarianceStamped with the saved 36-element covariance, using the existing odom-TF stamp and the ack path.
  - The wait has no deadline, by design. It checks two things:
    - the graph has subscriber node `amcl` in `/qazbot_d1_map`, and matched count >= discovered subscribers (so the trust-node subscriber alone is not enough);
    - an odom->base_footprint TF has been seen.
  - SIGINT or ExternalShutdown while waiting means nothing is published, exit 0.
  - AMCL humble keeps an initialpose received while inactive and applies it on activation (amcl_node.cpp:282, 542).
  - Manual mode is unchanged: same args, checks, covariance indices, 10 s bounded discovery and JSON output.
- `localization.launch.py`:
  - New arg `seed_pose_file` (default `pose_store.DEFAULT_PATH`; `''` = off, via `LaunchConfigurationNotEquals`).
  - New one-shot ExecuteProcess `seed_pose.py --from-file ... --map-yaml <map_yaml>`. It is appended after the shutdown guards are built, so its exit does not stop the launch. It is staggered as the 7th start.
- Default ON is justified because the trust gate stays mandatory:
  - the seed is handled by the trust node `_initialpose` -> `relocalize()` (anchor None, `ready_cause=initialpose`);
  - READY needs operator `confirm_anchor` plus a clean T2/T3/T4/T5/T8 gate.
- `docs/X2_LOCALIZATION_TRUST.md`: new section "сохранение подтверждённой позы" with the operator procedure (nomotion updates via `request_nomotion_update`, confirm_anchor, manual fallback). Test counts updated.
- NEW tests:
  - `tests/loc_trust/test_pose_persist.py` (14);
  - `tests/loc_trust/test_seed_ros.py` (1; skipped unless `ROS_DOMAIN_ID=89 ROS_LOCALHOST_ONLY=1`).

## Verification (all under nice -n 19; nav_e2e NOT run)
- Baseline before the changes: `cd tests/loc_trust && python3 -m unittest test_loc_trust test_ready_motion test_source_failures test_trust_gaps` gave 78 OK.
- After the changes: the same 4 modules plus `test_pose_persist` and `test_seed_ros` gave 93 run, OK, 1 skipped. That is 78 unchanged + 14 new + 1 skipped ROS test.
- `env -u FASTRTPS_DEFAULT_PROFILES_FILE ROS_DOMAIN_ID=89 ROS_LOCALHOST_ONLY=1 python3 -m unittest test_seed_ros` gave 1 OK (10.8 s):
  - with only a trust-like subscriber + TF for 8 s, the seed keeps waiting and publishes nothing;
  - after the fake `amcl` appears: exactly 1 msg to amcl and 1 to the trust subscriber; frame map; pose (1.25, -3.5, 0.7); covariance equal to the saved one.
- Coverage of the acceptance criteria by `test_pose_persist`:
  - atomic write: when `os.replace` fails, the old file is intact and no partial file is left;
  - saves only on the confirmed READY rising edge: 0 saves before confirm, 1 after, none while READY holds, 1 more after relocalize + reconfirm;
  - a failed gate after confirm does not save; map_ok mismatch or unseen does not save; an empty path disables saving;
  - shutdown save rules (6 cases);
  - sha mismatch, missing, truncated, empty, wrong-format and bad-covariance files are rejected;
  - a seeded initialpose (relocalize) does not give READY on 30 clean ticks, only after confirm_anchor;
  - seed_pose refusals: exit 0 with reasons no_file / map_sha256_mismatch / unreadable; argparse errors (exit 2) for mixed or missing args;
  - launch: one seed process, default path, condition off for `''`, cmd tail, not shutdown-guarded, all other processes still guarded.
- Isolated smoke test of the trust node (domain 89): `TrustNode()` constructs with `pose_store`, ticks, and `save_on_shutdown()` logs "not refreshed" (not READY); no file is written.
- `python3 -m py_compile` passed on pose_store.py, seed_pose.py, localization_trust_node.py, localization.launch.py and both new tests. `bash -n run_trust.sh` passed.
- `ros2 launch ./localization.launch.py --show-args` lists `seed_pose_file` (default `/home/agi/.ros/qazbot_x2_last_confirmed_pose.json`).
- `tests/launch_stagger`: 5/6 OK. `test_localization_launch_is_paced` passes with the extra start. `test_full_nav_launch_is_paced` errors exactly as before the change: nav_full needs `QAZBOT_SMAC_PREFIX` from env.sh, which is unrelated and pre-existing.

## Limits / not done
- FAST-LIO run id is not available. The §8 "same LIO run -> republish map<-odom (+) current odom" branch is NOT implemented; the seed always uses the saved map pose. The record has no map->odom field.
- The seed covariance is the AMCL covariance at confirm, which is tight (sigma_xy <= 0.20). If the robot was moved while the stack was down, AMCL may converge near the wrong seed. T4 (scan score) and the operator check before confirm_anchor are the guards, as the design specifies.
- Shutdown save works only on SIGINT/KeyboardInterrupt or ExternalShutdown, through `finally`. SIGKILL or a crash keeps the last READY-edge record.
- Not verified on the robot. The report's §5 check is still open: restart localization with the robot standing, then measure the time and the number of nomotion updates until the gate is clean and READY after confirm_anchor, with no RViz initialpose. To take effect, the trust node must be restarted via run_trust.sh, and a READY must happen once to create the file.
