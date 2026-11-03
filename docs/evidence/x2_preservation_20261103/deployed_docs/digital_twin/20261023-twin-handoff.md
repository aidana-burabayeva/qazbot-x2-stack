# 20261023-twin-handoff: digital twin of X2 (.66) — continue on user's laptop

## Goal
Local digital twin of X2 locomotion response (cmd -> actual body motion) for Nav2/RPP tuning, AMCL alpha, and adapter dry-run model.

## Data to copy from .66 (/home/agi/control_backend/logs), total ~4.7 GB
Priority A (raw cloud + LIO + odom_level + IMU + /aima/mc/leg_odometry):
  manual_amcl_walk_01 (178M), manual_amcl_walk_02 (839M), manual_amcl_turn_01 (665M),
  manual_amcl_reverse_01 (511M), raw_comparison_input_01 (124M)
Priority B (LIO + leg_odometry, no raw cloud; cmd/response):
  bags/g_motion_01..04, bags/short_nav_01, bags/turn_nav_01, bags/cycle2_01,
  operator_turn_01, operator_full_turn_dense_01/02
Joint states (npz: t, names[31], q[N,31], imu_xyzw[N,4]) — CORRECTION: joints DO exist, as npz extracts:
  bags/{g_motion_02,g_motion_03,g_motion_04,cycle2_01,cycle2_02,short_nav_01,turn_nav_01,manual_ready_turn_02}.joints.npz
Config/model: control_backend/ws/src/lio/x2.yaml (LiDAR-IMU extrinsic), ws/src/navigation (Nav2 params, footprint),
  tests/s3/step_cmd.py, tests/motion_adapter (kinematic model), docs/X2_AIMDK_VERSIONS.md, docs/MC_BALANCE_NAV2_CONTRACT.md.
Offline calib tools: see reports/20261023-calib-design.md §0 (scratchpad lio_calib/*.py; install to tools/lio_calib).

## Known facts
- Firmware v1.1.5, aimdk_msgs v1.1.0.4, MC on PC1 10.0.1.40 (soc0). MC version UNKNOWN: ssh agi@10.0.1.40 publickey/password denied; .66 metadata covers soc1 only.
- Turn asymmetry at |wz|=0.6: left 0.29 rad/s (0.49), right 0.92 rad/s (1.53).
- Doc start thresholds: fwd ~0.1, lateral ~0.5-0.7, yaw ~0.03-0.05; zeroing 200 ms, stand 2000 ms.
- Laptop has no ROS needed if using rosbags (pip) for db3 reading.

## Open data needs (robot, each motion needs explicit user go)
1. MC version on PC1 — needs credentials or user-run read command.
2. Step responses x5: fwd 0.1/0.2/0.3; yaw ±0.4/0.5/0.6.
3. Start thresholds lin/ang both directions. 4. Coast-down after command.
5. Tape/markers ground truth for final pose (separate FAST-LIO vs AMCL error).
6. Simultaneous leg_odometry + IMU + /joint states + LIO (+ raw cloud: record_motion.sh lacks it).

## Next steps on laptop
1. Copy data above; verify md5 of bags.
2. Build twin v0: per-axis first-order + deadband + asymmetric gain fit from Priority B bags (cmd vs LIO body velocity).
3. Validate on held-out bags; report residuals; list which robot trials (2-6) still reduce error.

## Laptop transfer (2026-10-23)
- Laptop adn@192.168.1.64 (Ubuntu, kernel 6.8, ROS humble, claude at ~/.local/bin). Key from .66 installed.
- Copied to ~/x2_twin: CLAUDE.md, .orchestrator, control_backend (no runtime), Priority A+B bags, all *.joints.npz.
- Laptop disk was 95% full (13 GB free before copy) — keep outputs small.
- lio_calib scripts from calib-design §0 are LOST (old scratchpad cleaned); must be rewritten from the report design.
- Start: cd ~/x2_twin && claude, then the continuation command from PLAN.md.

## PC1 MC version (read-only, ssh agi@10.0.1.40, user-provided creds; nothing run/changed on PC1)
- mc: v1.1.0-r9-rc9 (project 2049, commit e3c738dc, image soc0:v0.4.60); mc_param: v1.1.0-r9-rc3; platform v1.1.5.
- DCU firmware: dcu1/dcu2 V2.0.8-RC.0, dcu3 V2.0.2-RC.5.
- Snapshot copied to control_backend/vendor_snapshots/: pc1_mc_param_r9-rc3/ (URDF x2_31dof_hand, mc.yaml, mc_sim.yaml,
  speed_state.yaml, trajectory_generator.yaml, rl_models/*.onnx locomotion policies) + pc1_metadata.yaml.
- Twin implication: sim2sim possible (URDF + ONNX policy in MuJoCo/Isaac); which policy is active for LOW walk — TBD from mc.yaml/motion_list.yaml.
