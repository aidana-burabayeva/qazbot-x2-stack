# TASK 20261028-scan-latency-rviz-goal
Repo: /home/agi/control_backend/qazbot-x2-stack (branch handoff/first-rviz-goal-20261028). Deployed runtime: /home/agi/control_backend (no .git). Running ON X2.
Rules: AGENTS.md, CLAUDE.md, docs/HANDOFF.md, docs/TEST_STATUS.md of repo.

## Objective
1. Measure where latency arises raw cloud/IMU -> /qazbot_lio/cloud_registered -> scan -> native odom/Trust (T1 stale: scan max 1.207 s).
2. Fix measured root cause (review + operator permission for production change).
3. Repeat passive I1 check (needs explicit operator permission for I1 install; sequential operator steps, wait for confirmation each).
4. Prepare short RViz 2D Goal Pose run (MC/Motion Adapter/Goal Coordinator/goal need separate permission).

## Constraints / non-goals
- No motion, cmd_vel, goals, MC/Motion Adapter/Goal Coordinator without explicit permission.
- No duplicate FAST-LIO/AMCL/Trust/level_bridge; do not stop foreign/vendor processes (HAL PID204720, camera).
- Do not weaken T1/T5 thresholds, freshness, collision detection, footprint. No magic timeouts/retries.
- Minimize DDS observers. Stop only owned processes and verify release.
- No architecture change (AMCL, FAST-LIO stay).

## Acceptance
- Latency breakdown per stage with p50/p95/max, attributable stage identified with evidence.
- Blockers recorded in docs/HANDOFF.md + docs/TEST_STATUS.md, committed on separate branch.

## Operator spec update 2026-10-28 "Root Cause Fix — X2 T1 Data Freshness" (supersedes objectives 1-2)
- Read only: HANDOFF, TEST_STATUS, docs/evidence/i1_deploy_2d240dbd/freshness_same_stamp_analysis.json, docs/digital_twin/default_localization_and_freshness_20261026.md. No re-audit of architecture/Nav2/AMCL/TF/T5/I1.
- One targeted experiment with existing tools/stack/age_probe.py: Raw LiDAR/IMU -> FAST-LIO -> cloud_registered/odometry -> level_bridge -> LaserScan -> Trust; compare existing UDP DDS profiles, same process set; p50/p95/p99/max, callback gaps, source-stamp age, T1 count; identify FIRST stage where delay appears.
- Known suspect: ws/src/lio/level_bridge.py Odometry depth=100, PointCloud2 depth=20 (DT shows stale accumulation under overload). If confirmed, A/B KEEP_LAST depth=1 BEST_EFFORT vs current on identical input; keep correct cloud/odom stamp matching.
- Trust changed only on proven freshness-detection bug. Forbidden: change T1 thresholds (scan .5, odom .5, TF 1.0), rewrite stamps, retries/watchdogs, disabling checks.
- Regression before/after: no stale accumulation; stamps = sensor time; LaserScan+TF correct; no AMCL/T5/I1 regression; sensor loss blocks motion; recovery does not auto-resume old Nav2 goal. Scenarios: nominal, high CPU/DDS load, scan loss, odom loss, recovery.
- Order: PC/Digital Twin first, then separately agreed passive X2. No MC/cmd_vel/goals/motion. No T5/I1 reinstall.
- Deliverables: proven root cause with before/after numbers, minimal diff, regression results, GitHub PR updating HANDOFF/TEST_STATUS.
- Note: session runs ON X2 (no PC access); P1 passive X2 measurement already started under earlier operator request. AMCL P2a change (max_beams 60, alpha .1) is a confound for AMCL regression — record separately.

## Operator authorizations 2026-10-28 (explicit, in chat)
- Restart of all OUR processes (localization, Trust, Nav2, GC, motion adapter, recorder); install I1 2d240dbd (r2 backup); despeckled map; T1 pause-resume code; stop vendor camera/perception/drp/webrtc (done). MC/MA/GC + operator RViz goals authorized after initialpose+READY. HAL vendor-restart deferred.
