# Physical robot state check — 2026-10-27 20:49 (Asia/Almaty)

Read-only check of AGIBOT X2 Ultra at `192.168.1.72`, continued after RViz retry 1257. No ROS process was restarted, no service/control request was sent, and no motion goal was issued.

## Observed state

Passive monitor `evidence/Static_map_retry_monitor_1258b/latest.json` at robot time `1793105357.18` (~20:49 local) reported:

- MC: source `rc`, body state `1`, FSM `4`, action `STAND_DEFAULT`.
- Trust: `trusted=false`, `ready=false`, `motion_ok=false`, `initialized=false`; reason `not_ready:awaiting_initialpose_and_amcl`. Diagnostics: `T2_covariance`, `T3_particles`, `T4_no_baseline`, `T4_scan_score`, `T6_no_anchor`.
- Inputs at capture were fresh: scan age 0.205 s, odometry age 0.037 s, map→odom age 0.148 s. Thus the current block is not a live stale-scan verdict; it is the uninitialized/relocalization state and associated diagnostics after the prior T5 reset.
- Native odometry owner remains `native_imu`, publishing 30 Hz; age 0.012 s, no fault and no latch reported.
- Motion adapter remains registered/live but latched `localization_untrusted`, `motion_armed=false`, `ready=false`.
- Goal coordinator is idle, latched `localization_untrusted`, gate closed, `source_zero=true`.
- Last AMCL pose is from the prior attempt (`1793104975.91`); no new valid post-seed AMCL output was observed by this monitor.

The launched stack processes were present (localization, Nav2, Trust, adapter, coordinator). Passive readiness monitor PID 313985 was still running. No rosbag recording process was found; the previous retry bag is closed at `evidence/Static_map_retry_1257/bag/`.

## Assessment and next safe step

Robot is standing and the navigation output remains gated. Current odometry and scan input ages look fresh, but localization has not recovered. Operator should send a new RViz 2D Pose Estimate and verify that Trust becomes initialized/ready and AMCL publishes a fresh pose before any new goal. Keep both adapter and coordinator latches, the `T1` freshness check, and `T5` jump handling intact. No movement trial is authorized by this state inspection.

## Evidence / limitations

This is a monitor snapshot, not ground truth of physical pose. It confirms process and gate states only at the capture time. It does not prove loaded code hashes or that the robot is localized correctly. Source: `evidence/Static_map_retry_monitor_1258b/latest.json`; previous movement evidence and command analysis: `RViz-goal03-retry-1257-result.md`.
