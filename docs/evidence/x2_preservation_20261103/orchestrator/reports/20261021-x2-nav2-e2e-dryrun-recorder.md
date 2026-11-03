# recorder prep (passive, nothing started)
Scripts: control_backend/tools/record_motion.sh NAME [MAX_S], tools/stop_motion.sh NAME, tools/analyze_motion.py BAGDIR [JOINTS.npz].
Edit: tests/footprint/joint_record.py now saves on SIGINT (backup .bak_20261022_sigint); needed so stop_motion.sh does not lose the npz.
Bag topics (verified once via ros2 topic list, missing ones are warned/skipped): lio odometry/odom_level/scan, amcl_pose, particle_cloud, d1_map tf/tf_static, trust status+trusted, adapter status+dry_run_cmd, goal_coordinator/status, nav2/cmd_vel_nav, cmd_vel, cmd_vel_authorized (adapter input), nav2/plan, /aima/mc/common/state, /aima/mc/leg_odometry, /aima/mc/locomotion/velocity.
Opt-in: WITH_JOINTS=1 (/aima/hal/joint/leg/state ~500 Hz, est. ~45 MB/min), CLOUDS=1. Joints are otherwise covered by joint_record.py 50 Hz npz (logs/bags/NAME.joints.npz).
Size: estimate only (not measured; no extra subscribe), ~10-40 MB/min (particle_cloud dominates); clouds excluded: existing liolat bag was 487 MB/61 s. Disk free 380 GB; script refuses below 5 GB.
Self-test: synthetic bag (domain 88, localhost) - AMCL windows, onset lag 0.46 s (true 0.4), stop settle 0.60 s/0.047 m, latency 0.050 s (true 0.05), trust transitions all correct; envelope on rest npz via sec_env ran (see run). Not tested: record_motion.sh itself (not run), particle_cloud/MC types, real joint record.
