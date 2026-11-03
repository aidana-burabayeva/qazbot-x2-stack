# RViz goal retry 1257 — pure rotation, Trust revoked before translation

Date: 2026-10-27. Physical X2 Ultra, static-map navigation retry. Evidence: `evidence/Static_map_retry_1257/` and passive state capture `evidence/Static_map_retry_monitor_1258b/`. No stack/config changes were made during this trial.

## Outcome

Operator reported that the robot did not translate and only turned its body slightly. The capture agrees: Nav2 issued `vx=0`, `wz=-0.6 rad/s` from approximately robot time 1793104974.45 to 1793104975.76 (about 1.3 s), then the coordinator cancelled. There was no linear command in the captured motion window. Native odometry changed yaw by about -21.9 degrees and XY by about 4.8 cm over the sample; this is not independent ground truth and may include residual/coasting motion. The first pure rotation is consistent with the path's southbound heading alignment, but the requested route was not physically traversed.

The global `ComputePathToPose` query for the seeded start (-5.8378, 4.5599) and goal (-4.806, -4.460) succeeded with 184 poses. Its bounds were X[-6.715, -4.825], Y[-4.475, 4.575], passing west of the unknown table quadrilateral. This confirms a static global route was available; it does not confirm local controller execution or safe physical clearance.

## Stop cause and evidence

During the initial turn Trust reported `T1_scan_stale`; adapter latched `localization_untrusted`, and coordinator cancelled. Around 1793104976.532 Trust then reported `T5_map_odom_jump` and `not_ready:awaiting_initialpose_and_amcl`, with `initialized=false`. Do not bypass either gate.

The recorded scan stream had 41 messages between robot times 1793104970 and 1793104978; maximum receive gap was about 0.365 s (p95 0.355 s). Maximum source-stamp age in this interval was about 0.756 s, exceeding configured `scan_max_age_s=0.5 s`. Native odometry max receive gap was about 0.132 s; FAST-LIO odometry max receive gap about 0.315 s. Therefore stale scan/source-time delivery is a confirmed trigger, but its underlying producer/DDS/CPU cause is not established. The scan receive gap alone does not explain the stale verdict.

## Current gate state (read-only monitor at 1793105259)

Trust is false and uninitialized, reason `not_ready:awaiting_initialpose_and_amcl`; diagnostics include covariance/particle/baseline/anchor checks. Adapter remains registered/live but latched `localization_untrusted`, `motion_armed=false`; coordinator is latched with `gate=false`. MC is RC / body=1 / FSM4 / STAND_DEFAULT. Native odometry is publishing 30 Hz with no reported fault. No recorder process is running; the passive monitor was active at capture time.

## Next action

Operator must establish a fresh RViz 2D Pose Estimate and wait for Trust READY before any new goal. Keep scan freshness and T5 safeguards unchanged. Before another movement attempt, preserve passive capture and inspect scan source/receive timing around Trust evaluation. The global route is verified; physical translation and table bypass remain unverified. No speed, MC mode, local-costmap footprint, collision, or Trust thresholds were changed.
