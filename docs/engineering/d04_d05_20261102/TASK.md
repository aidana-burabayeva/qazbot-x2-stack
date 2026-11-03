# TASK 20261102-d04-costmap-source — QazBot X2 D-04 / D-05 / C++ modernization
Source: user message 2026-11-02 (full brief, Russian). Faithful structured copy below.

## Context
- Project: /home/agi/control_backend (NOT a git repo locally); qazbot-x2-stack clone inside. GitHub aidana-burabayeva/qazbot-x2-stack branch robot/stable (prod), lab/stable (lab).
- Reference: AgiBot-Community/agibot-x2-navigation main @ 9ad1ad8af1b47b1d8fda42c3ca9c828bc5625629
  (src/my_nav2_pointcloud_pkg/config/nav2_params.yaml, .../cmd_to_aimavel.py, src/py_examples/py_examples/pcl_rectifier.py, src/spark-fast-lio/spark_fast_lio/launch/octomap.launch.xml).
- Standard: henki_ros2_best_practices.md — NOT FOUND locally (2026-11-02); request copy from user, do not invent.
- Host = X2 PC (agi), ROS 2 Humble, live graph. Published GitHub may differ from running files: record installed SHAs/hashes + effective params before changes. Do not overwrite local changes.
- Keep: D1 2D map, AMCL, FAST-LIO, odometry/TF, Nav2, Goal Coordinator, P20/P23, RViz goals, MC arbitration/RC priority, T1/T5, safety admission. No global 3D map.

## D-04 problem
Nav2 delayed start ~10 s "collision ahead" with no visible obstacle; cleared after full costmap clear. Marking FOV ≈ ±52°, clearing FOV −46…+44.5°; lethal cells reappear at +46…+49° on stationary X2. Candidate causes: floor reflections, glass door, real points beyond clearing FOV, 3D misclassification, clearing geometry, source errors.
FORBIDDEN as final fix: clipping marking to clearing FOV (−46..+44.5); disabling collision detection; arbitrary obstacle forget timeout; auto full clear per goal.

## Required work (this cycle = Phases 1–4, offline/read-only only)
1. Phase 1 (read-only): effective local/global costmap plugins, observation_sources, marking/clearing, topic types, voxel_layer / STVL installed & active, /voxel_grid. Output CURRENT_LOCAL_COSTMAP_LAYER, CURRENT_GLOBAL_COSTMAP_LAYER, VOXEL_LAYER_INSTALLED, VOXEL_LAYER_ACTIVE, STVL_INSTALLED, ACTUAL_SENSOR_SOURCES.
2. Phase 2: source evidence — raw 3D points behind +46..+49° lethal cells, clearing geometry, why scan_obstacles/scan_clearing split was introduced and what it guarantees.
3. Phase 3: offline replay on real X2 bags, same Nav2 version/map/TF/footprint/inflation/ranges: A baseline (split, ObstacleLayer), B single PointCloud2 marking+clearing (validate frame/origin/deskew; check clearing under low obstacles / over-the-top rays), C bounded clearing extension (no artificial FOV, no fabricated endpoints), D VoxelLayer with real 3D (not z=0), E STVL only if Humble-compatible and justified. Metrics: false collision, real obstacle detection, clearing after removal, low-obstacle safety, blind-FOV safety, CPU, RAM, latency p95/p99, bandwidth, regression.
   Regression scenarios: obstacle ahead; obstacle at +49°; side chair; person leaves (cleared on new evidence); person into blind zone (absence ≠ free); floor reflection; glass; low object; tall object within envelope; LiDAR loss (T1); TF jump; AMCL correction; repeated goal. Not PASS unless present in a recorded bag or actually tested.
4. Phase 4: choose simplest correct architecture; minimal candidate + regression suite. If processing path changes, C++17/rclcpp/ament_cmake candidate for level_bridge (pure lib: geometry, classification, rays; separate ROS node). Python↔C++ parity on real messages with tolerances derived from numerics. Never run both publishers in production namespace.
Later phases (NOT this cycle): 5 hardware qual (needs user OK), 6–7 D-05 velocity calibration/compensation (LIVE-2: MC integral 1.90 m, FAST-LIO 3.11 m, tape ≈3.10 m, leg odom 2.87 m; no copied 0.5 scale; no PID default; P23 untouched), 8 C++ Motion Adapter migration (behavior parity first), 9 full regression.

## Safety (untouchable without approval)
T1, T5, Trust admission, MC priorities, P20/P23, E-stop, collision detection, footprint, GC stop/latch. S-01 open. No goals sent by agent (user sets in RViz). No live adapter activation, no robot install, no combined deploys, no new motions without separate permission. No heavy parallel experiments on robot (CPU history).

## Reporting
Update docs/hardware/PHYSICAL_NAV_SESSION_20261102.md; add engineering report docs/engineering/D04_D05_CPP_REPORT.md. Each candidate: CANDIDATE, SOURCE_SHA, BASELINE_SHA, ROOT_CAUSE, CHANGED_FILES, TEST_EVIDENCE, SAFETY_IMPACT, PERFORMANCE_IMPACT, ROLLBACK, STATUS ∈ {HYPOTHESIS, OFFLINE_VERIFIED, SIM_VERIFIED, HARDWARE_VERIFIED, FAILED, BLOCKED}.
Reply format: CURRENT COSTMAP / VOXEL STATUS / D-04 ROOT CAUSE / AGIBOT COMPARISON / A/B/C RESULTS / RECOMMENDED ARCHITECTURE / WHY IT IS SAFE / CHANGED FILES / C++ MIGRATION SCOPE / CPU / LATENCY BEFORE-AFTER / REGRESSION / HARDWARE DEPLOY: NOT APPROVED / NEXT REQUIRED HARDWARE TEST.

## Update 2026-11-02 (user /goal)
- Execute FULL plan autonomously: D-04, D-05, plus check S-01 and D-06; ObstacleLayer/VoxelLayer/STVL vs AgiBot; C++ migration of high-frequency critical nodes.
- Strict sequence per fix: diagnosis → fix → tests → regression → next fix. Do not repeat completed research.
- Autonomous: analysis, development, offline/replay/sim tests, docs. Needs user permission: physical deploy, critical service restarts, any X2 motion. Goals only by user in RViz.
- Mandatory standard: ~/.claude/standards/henki_ros2_best_practices.md (global rule in ~/.claude/CLAUDE.md). Pass path to every implementer/verifier.
