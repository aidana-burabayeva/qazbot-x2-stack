# Runtime diagnostic after RViz retry 1257 — 2026-10-27 20:53

Physical X2 Ultra only. Read-only inspection; no ROS stack, MC, TF owner, or navigation parameters were changed, and no goal or motion command was sent.

## Localization and motion gate

The passive readiness monitor was live through 20:52:31 local and reported Trust still false/uninitialized (`awaiting_initialpose_and_amcl`), with no fresh AMCL pose since the previous attempt. The adapter remained registered/live but latched `localization_untrusted`, unarmed; coordinator was idle and latched with gate closed and source zero. MC remained RC / body 1 / FSM 4 / STAND_DEFAULT. Native odometry remained `native_imu`, 30 Hz, no fault. At the latest sample, scan/odom/map→odom ages were 0.208/0.037/0.148 s, so those individual inputs were fresh at that instant.

The monitor recorded transient `T1_scan_stale` state changes at robot times 1793105369.48 and 1793105373.53 while the robot was standing. They cleared back to the existing not-initialized state about 2.5 s later. This confirms intermittent freshness excursions continue, even though the latest snapshot itself had a fresh scan. The monitor was stopped by SIGTERM at 20:53:59 after collecting the state evidence; it was a passive helper, not part of the ROS stack.

## Retry 1257 stale-scan timing reconstruction

The closed bag summary provides a direct timing match for the Trust event at 1793104975.768:

- Last preceding scan was received at 1793104975.404548 with source stamp 1793104975.113346.
- At the Trust stale verdict, that source stamp was about 0.655 s old, above the configured 0.5 s maximum.
- The receive gap from the prior scan (received 1793104975.049477) to this scan was 0.355 s, and this scan was already about 0.291 s old on arrival.

Thus the T1 verdict is explained by accumulated scan-source lag plus the inter-message gap. This does not yet identify why the scan pipeline lags (FAST-LIO processing, upstream sensor scheduling, DDS delivery, or combined system load remain candidates). Keep the freshness limit unchanged.

## Compute load snapshot while standing

A separate 5 s `pidstat` sample and 2 s `tegrastats` sample around 20:53:15–20:53:22 showed the 8-core Jetson CPU cores around 70–89% utilization, load average around 18–22, and no memory pressure (about 6.5 GiB available). Representative process CPU: FAST-LIO mapping ~39% of one core; `level_bridge` ~10%; native odometry bridge ~33% in the accompanying `ps` snapshot; Localization Trust ~18%; motion adapter ~20%; coordinator ~10%; individual Nav2 processes mostly ~8–10%. Vendor camera/perception processes also appeared among the higher consumers.

This is a confirmed high-load condition during a stationary robot state and a plausible contributor to scan latency, but it was sampled several minutes after the retry, not synchronized with the exact T1 event. It is therefore not proof of causation. Do not stop vendor services or alter CPU affinity/FAST-LIO settings as part of this observation.

## Next step and limits

The robot remains gated pending a new operator RViz 2D Pose Estimate and fresh valid AMCL output. Before another movement attempt, start a bounded passive recorder and monitor after that seed; record scan source age, receive interval, Trust verdict timestamps, and per-process CPU in the same time window. Current evidence confirms the global route existed and the prior attempt turned but did not translate. Physical traversal remains unverified.

Sources: `evidence/Static_map_retry_1257/scan_gap_summary.json`, `evidence/Static_map_retry_1257/static_trial_summary.json`, `evidence/Static_map_retry_monitor_1258b/latest.json`, `evidence/Static_map_retry_monitor_1258b/events.jsonl`, and live `pidstat`/`tegrastats`/`ps` snapshots from this check.
