# 20261103-reuse-R4 — license/portability inventory + MPPI baseline (read-only)
Date 2026-11-03. No code changed, nothing installed, no ROS nodes started. Remote repos shallow-cloned into scratchpad only
(/tmp/claude-1001/-home-agi/b5c1d480-a1af-4b3a-828c-b93cffe75cd0/scratchpad/{nav,slam}/r). gh CLI absent -> git ls-remote + git clone.
NOTE: shallow clone checkout hit an LFS smudge error (src/FAST_LIO_ROS2/doc/Fast_LIO_2.pdf); inspected via `git show/ls-tree HEAD:`; unaffected.

## 1. Remote repos
| repo | HEAD (main) | license |
|---|---|---|
| AgiBot-Community/agibot-x2-navigation | 9ad1ad8af1b47b1d8fda42c3ca9c828bc5625629 (2026-07-24 "Update README.md") | root LICENSE = MIT, "Copyright (c) 2026 Simon525" (blob 42be95b9) |
| Nova-Valley/agibot-x2-navigation | 9ad1ad8af1b47b1d8fda42c3ca9c828bc5625629 | identical (same commit SHA => same tree) |
| byggmesterPRO/agibot-x2-slam | 58e623a10cf73f517080644d4567f1fae7d100ee (2026-07-20) | NO LICENSE file (26 files; none matches LICENSE/COPYING; no package.xml; README/RESEARCH/LIDAR-STRATEGY never mention license) => all rights reserved; idea-only reuse, no code copy |
Correction: "9ad1ad8af1b4" is the COMMIT sha, not the tree. Root tree = 154eea81fd6b91d9e8914f92f82a9c4071644301. Nova-Valley branches: only main, same SHA.
agibot-x2-navigation is a 1-branch snapshot; no vendored Nav2 copies (git ls-tree: no nav2_*, mppi, velocity_smoother, collision_monitor dirs).

### agibot-x2-navigation src/ packages (package.xml <license> | nested LICENSE)
| path | tree/blob | package.xml license | nested LICENSE |
|---|---|---|---|
| src/agibot_nav | tree 941feb5c | Apache-2.0 | none (RPP + SmacPlanner2D params, bringup launch) |
| src/my_nav2_pointcloud_pkg | tree 939243a8 | Apache-2.0 | none (cmd_to_aimavel.py, send_goal.py, nav2_params.yaml; RPP+NavFn, no smoother/collision_monitor/MPPI) |
| src/aimdk_msgs | tree 268ccc36 | "TODO: License declaration" | none |
| src/examples | tree 35b1f72b | "TODO: License declaration" (package.xml blob 6434b4a3) = EMPTY | none |
| src/py_examples | tree 359706ee | "TODO: License declaration" (blob 99209c65) | none; contains pcl_rectifier.py, imu_rectifier.py |
| src/ruckig | tree f23380d1 | MIT (v0.9.2) | LICENSE MIT (c) 2021 Lars Berscheid |
| src/spark-fast-lio/spark_fast_lio | tree 6c164127 | GPL (package.xml blob fe26708c) | LICENSE = GPLv2 (blob d159169d) -> copyleft; MIT-SPARK, Jazzy-oriented README |
| src/FAST_LIO_ROS2 | tree 307fc120 | BSD (declared) | LICENSE = GPLv2 text (MISMATCH declared vs file) |
| src/lidar_localization_ros2 | tree c5df5765 | BSD2.0 | LICENSE BSD-2 (c) 2020 Ryohei Sasaki |
| src/ndt_omp_ros2 | tree dfd28ba5 | BSD | LICENSE BSD-2 (c) 2019 k.koide |
| src/livox_ros_driver2 | tree 125df213 | MIT | LICENSE.txt + 3rdparty/rapidjson/license.txt |
| loose scripts: src/compressed_to_raw.py, src/object_detection.py, merge_pcd.py, pt_to_onnx.py | - | none | covered only by root MIT (no headers checked) |
Implication: root MIT does not clearly cover aimdk_msgs/examples/py_examples (derived from AgiBot SDK, empty license decl). Treat those as "license unknown, SDK-derived": use only as reference/API facts; code we need must be re-authored or obtain written confirmation. pcl_rectifier.py (blob 1d52d38e) lives in py_examples => same uncertainty. Do not vendor spark-fast-lio or FAST_LIO_ROS2 (GPL).

### Local /home/agi/aimdk/src vs upstream (git hash-object compare, diff -rq)
- Local SDK = "aimdk v0.0.0-g14fe2e8" (not a git repo; no LICENSE anywhere in /home/agi/aimdk, README has no license text). It is a NEWER/different SDK drop than the upstream snapshot.
- ruckig: IDENTICAL (diff 0 files). MIT.
- examples: 38 differences (local has extra autonomous/, data/, echo_gnss_data, upper_body_control...; mc_locomotion_velocity.cpp differs: local blob vs upstream d5ca7f3b). package.xml identical (blob 6434b4a3, empty license).
- aimdk_msgs: 60 differences; local layout has app_proxy/hds/mm/sm/task_engine/mc/{motion,openapi,data}, includes McLocomotionSpeedBounds/Mode/Status + GetMcLocomotionSpeedStatus (not upstream); upstream has interface/robot/aima/ and mc/motion/McLocomotionVelocity.msg. package.xml differs (local faa6c6f1 vs upstream 3cb61564), both license TODO.
- py_examples: 50 differences; local lacks pcl_rectifier/imu_rectifier/drift_detect; package.xml identical (99209c65).
- robotop: local only (license "TODO"); not in upstream.
=> Upstream repo is NOT a copy of local SDK; local SDK is the authority for on-robot interfaces (QazBot already builds against it).

## 2. Installed Nav2 on host (Jetson, aarch64)
- NO system Nav2: dpkg has no ros-humble-navigation2 / nav2-mppi / nav2-velocity-smoother / nav2-collision-monitor; `ros2 pkg prefix` not found; /opt/ros/humble/share has only nav2_common, nav2_costmap_2d, nav2_msgs, nav2_util, nav2_voxel_grid.
- Nav2 comes as extracted debs (ROS snapshot 2025-08-15, all 1.1.18), none built from source:
  - /home/agi/control_backend/runtime/qazbot_nav_deps/root/opt/ros/humble: nav2_controller 1.1.18 (Apache-2.0), nav2_regulated_pure_pursuit_controller 1.1.18, nav2_velocity_smoother 1.1.18 (deb ...20250729.010012, sha256 5155aaac..., package.xml Apache-2.0), amcl, planner, navfn, smac, lifecycle, core.
  - /home/agi/control_backend/runtime/nav_ext_ws/root/opt/ros/humble: nav2_mppi_controller 1.1.18-1jammy.20250729.010514 (deb sha256 803b6426ceeb56e789372556b64caf9d6ee8804035f417e08a8b0bf4d3f632e9; package.xml <license>MIT</license> — official Nav2 package license). Same dir: STVL 2.3.4 BUILT FROM SOURCE (nav_ext_ws/src/stvl_2.3.4, git tag 2.3.4, colcon install) + openvdb_vendor deb. Overlay NOT sourced globally; added only via additional_env of *.p25_candidate launchers.
  - runtime/nav2_bt_deps: bt_navigator, behavior_tree stack.
- navigation2 meta: not installed (individual packages only). nav2_collision_monitor: NOT present anywhere on host. (Robot PC1 native stack has its own /collision_monitor node, see qazbot docs/evidence/d1_baseline.txt — separate onboard system.)
- Docker daemon not running (not used).

## 3. MPPI baseline (FROZEN) — /home/agi/.orchestrator/reports/20261103-controller-compare.md
Source commit: qazbot-x2-stack (/home/agi/control_backend/qazbot-x2-stack) branch robot/feedback-d04-d05-20261102, commit 3ee109abd011b61cdde81bf38653ebca7b6fd44b "eval(controllers): RPP vs DWB vs MPPI on real Nav2 Humble 1.1.18 (isolated domain 78), no deploy"; tests/controllers tree f68aaef1d0e07224e85988363c2470b0d78b8927. (Also materialised at /home/agi/control_backend/tests/controllers/, results/summary.json + 24 JSON.) Verified commit exists and is on that branch.
Official Nav2 confirmed: MPPI = ros-humble-nav2-mppi-controller 1.1.18 deb from snapshots.ros.org humble/2025-08-15 (sha256 above), libmppi_controller.so/libmppi_critics.so from that deb; no third-party copy. Params: DiffDrive, vx 0..0.3, wz ±0.6, batch 1000, 56x0.05 s, default Humble critics, ObstaclesCritic consider_footprint true (inflation_radius 0.5/cost_scale 3.0); accel limiting by live velocity_smoother (1.1.18 MPPI has no ax_max).
Setup: 36 runs (24 matrix + 12 extra), seed 1, offline sim, one 5.31 m plan, models ideal / mc_hyp (D-05 HYPOTHESIS: dead 0.4 s, v=0.34+0.53*cmd, tau_v 0.25, tau_w 0.15) / mc_lin (const gain 1.61).
Frozen MPPI numbers (RPP / DWB in brackets):
| scenario/model | MPPI result | key metrics |
|---|---|---|
| nominal ideal | SUCCEEDED 18.38 s | dev 2.2/5.6 cm, min clr 0.492, jerk 2.6 (RPP 5.2), CPU 8.8 ms/cycle (RPP 2.1, DWB 11.2) |
| nominal mc_hyp | SUCCEEDED 13.09 s | dev max 29 cm (RPP 17, DWB 18), min clr 0.425, jerk 0.65 (11.6/11.7), pure-rot switches 2 (RPP 5) |
| static_block ideal | ABORTED (progress) | stops 0.11 m before box (RPP 0.38, DWB 0.03), CPU 29 ms |
| static_block mc_hyp | ABORTED (optimizer fail) | clr +0.022 (RPP -0.236 contact, DWB -0.236 contact) |
| transient ideal | SUCCEEDED 22.4 s | clr 0.305, resume 0.36 s |
| transient mc_hyp | ABORTED (optimizer fail) | clr 0.002 (RPP -0.095, DWB -0.236) |
| stale_cell (A clearing) | ABORTED 15 s, blocked | CPU 34 ms/cycle (~68-69 % core); with D-04 candidate C clearing: SUCCEEDED 20.1 s (RPP/DWB unchanged) |
| mc_lin static_block / transient | stop 0.085 m abort / stop clr 0.114 OK | RPP 0.034 / 0.002; DWB contact |
Other MPPI: period p99 <=55.1 ms, RSS 73-80 MB, 0 cmd gaps >150 ms, low-vx (0.02..0.15) ticks up to 31 % near obstacles, sub-deadband wz up to 17.5 % when blocked.
Report verdict: MPPI CONDITIONAL YES (DWB NO); keep RPP live. Controller does not fix stale blind marks (costmap/odom domain).
Caveats: single seed; mc_hyp is a hypothesis, not a model; D-06 native-odom under-read and D-07 low obstacles not modelled.

Preconditions before ANY X2 install/switch (from report section 5 + task):
1. D-05 calibration: fixed-cmd straight walks 0.05..0.30 + stops -> MC speed map (low-speed region), dead time, coast; then inverse map in adapter or limited cmd range; re-run harness with measured RobotModel.
2. D-07: obstacles below ~0.34 m invisible (sensor) -> operator rule or 2nd sensor (cf. camera hazard idea) before unattended MPPI.
3. D-04/D-04b clearing fix (candidate C or equivalent) + D-06 odom creep.
4. MPPI tuning in harness: wz_std/PathAlign for sub-deadband wz, VelocityDeadbandCritic with MC deadbands, batch 1000 vs 2000 under FAST-LIO CPU load; then controller_dry before LIVE.
Install note: only overlay under runtime/nav_ext_ws; do not touch /opt/ros; remove = rm -rf the dir.

## 4. Candidate solutions
Legend: QazBot analogue = under /home/agi/control_backend/qazbot-x2-stack (HEAD 982f6180 nav/first-goals-20261028; note working tree has uncommitted mods to ws/src/navigation/bt/navigate_to_pose.xml, localization.launch.py).
Humble: all targets are Humble-era (Nav2 1.1.18 installed).

| # | candidate | exact file + SHA | license (repo / component) | Humble | QazBot analogue | functions to preserve | minimal regression checks |
|---|---|---|---|---|---|---|---|
| 1 | AIMDK mc examples (mc_locomotion_velocity.cpp, set_mc_input_source.cpp) | upstream src/examples/src/mc/mc_locomotion_velocity.cpp blob d5ca7f3b (agibot-x2-navigation@9ad1ad8a); local /home/agi/aimdk/src/examples/src/mc/ differs (newer SDK, v0.0.0-g14fe2e8) | repo MIT but package examples = EMPTY (TODO) => unknown; local SDK no LICENSE | Yes (rclcpp) | ws/src/navigation/x2_motion_adapter.py (blob 726dbeb6): SetMcInputSource, priority, timeout 1000 lease, yaw loop P20 | example facts only: deadband <0.005->0, min fwd/lat 0.2, min ang 0.1, ranges 0.2..1.0; input-source lease; do NOT copy code | adapter unit tests tests/motion_adapter; walk_check; input-source registration test (tests/motion_adapter/ros_registration_fake.py) |
| 2 | aimdk_msgs | upstream src/aimdk_msgs package.xml 3cb61564; local package.xml faa6c6f1 | license TODO (both) | Yes (rosidl) | local /home/agi/aimdk/install/aimdk_msgs (builds for adapter); tests/aimdk/check_aimdk_interfaces.sh | use LOCAL SDK msgs only (has SpeedBounds/Status that upstream lacks); never mix upstream copy | tests/aimdk/check_aimdk_interfaces.sh; ros2 interface show of McLocomotionVelocity before/after |
| 3 | Ruckig | src/ruckig (identical to local /home/agi/aimdk/src/ruckig), package.xml aefb8144, LICENSE f550ecf9 | MIT (v0.9.2; community edition) | Yes (plain CMake, no ROS dep) | none (no accel/jerk limiter besides velocity_smoother; review 20261027 notes ruckig not a MC coast fix) | jerk-limited stop trajectories; must not alter MC deadband/min-speed behaviour | offline step/stop profile test; stop distance vs commanded; no added latency (>0.4 s dead time already) |
| 4 | nav2_velocity_smoother | runtime/qazbot_nav_deps/.../share/nav2_velocity_smoother/package.xml sha256 f53708be0b0a; deb sha256 5155aaac...1.1.18 | Apache-2.0 (repo+pkg) | Yes, 1.1.18 | ALREADY LIVE: ws/src/navigation/nav_control.launch.py (velocity_smoother, OPEN_LOOP, cmd_vel_nav->cmd_vel) | accel/decel limits (0.4/1.0; -0.6/-1.5), OPEN_LOOP mode, 20 Hz, zero-on-timeout | existing tests/controllers harness; check /cmd_vel gaps <150 ms, ticks, adapter timeout |
| 5 | nav2_collision_monitor | not installed on host (no deb in runtime/*/debs); upstream pkg in Nav2 Humble (not verified locally) | Apache-2.0 (Nav2; unverified locally) | Available in Humble (ros-humble-nav2-collision-monitor); needs deb + dds overlay under runtime/ | none in QazBot; robot native /collision_monitor exists on PC1 (d1_baseline.txt) = separate; planner_clearance/footprint tests partial | velocity-limit/stop polygon on /cmd_vel with footprint from control_geometry.py; blind-range 0.45 m + D-04b FOV; must be below MC arbitration | footprint polygon vs control_geometry.py; stop-polygon fires on box; false-stop on stale cells (D-04); chain order with smoother; CPU |
| 6 | nav2_mppi_controller | runtime/nav_ext_ws/root/opt/ros/humble/share/nav2_mppi_controller/package.xml sha256 70f5b265a6fd; deb sha256 803b6426...ced | MIT (package.xml) / Nav2 Apache-2.0 | Yes 1.1.18 (no ax_max, ObstaclesCritic own inflation params) | *.p25_candidate launchers (nav_control/nav_full), tests/controllers/config/mppi.yaml, tests/controller_dry; live = RPP | smoothness, pure-rot adapter compat (P23), no ‘collision ahead’ spam | re-run 36-run matrix after any change vs frozen table; CPU <=35 ms/cycle; controller_dry before LIVE; D-05/D-07/D-04C/D-06 preconditions |
| 7 | agibot nav2_params.yaml | upstream src/my_nav2_pointcloud_pkg/config/nav2_params.yaml blob 7cb826e0; src/agibot_nav/config/param_nav2.yaml blob 06580788 | Apache-2.0 (pkg) / repo MIT | Yes (RPP, NavFn, Smac2D, obstacle layer on /lidar/points_rectified) | QazBot params in nav_control.launch.py (generated YAML, live footprint/padding/inflation) | reference only: no smoother/collision monitor/MPPI inside; RPP+NavFn equals ours | n/a (reference); diff of RPP params vs live (20/20 already verified in report) |
| 8 | cmd_to_aimavel.py | upstream src/my_nav2_pointcloud_pkg/my_nav2_pointcloud_pkg/cmd_to_aimavel.py blob 7639394d | Apache-2.0 (pkg), repo MIT | Yes | x2_motion_adapter.py (superset: yaw loop, timeouts, state read) | watchdog 0.5 s zero, deadband 0.005, min-speed bump (0.2/0.2/0.1), clamp 1.0, SetMcInputSource priority 40 timeout 1000, publish 50 Hz | compare against adapter: cmd timeout, deadband, source registration; no min-speed bump conflicts with D-05 map |
| 9 | pcl_rectifier | upstream src/py_examples/py_examples/pcl_rectifier.py blob 1d52d38e (imu_rectifier.py bbd3a056) | py_examples license TODO (empty) => unknown | Yes (rclpy, numpy) | ws/src/lio/level_bridge.py (level/clearing), raw_scan_bridge.py, x2.yaml | RPY extrinsic rotation of cloud to level frame, PointField layout, sensor QoS | numeric: rotate known points, compare with level_bridge output; ring/time fields preserved (FAST-LIO) |
| 10 | spark-fast-lio | src/spark-fast-lio/spark_fast_lio package.xml fe26708c, LICENSE d159169d | GPLv2 (declared "GPL") — copyleft; repo root MIT does not override | README says Jazzy; Humble untested | live FAST-LIO (runtime/fast_lio, fastlio_mapping, ws/src/lio) | none to copy | do not integrate; license incompatibility; reference only |
| 11 | byggmesterPRO camera hazard idea | js/camera.js blob ac331b06, LIDAR-STRATEGY.md blob 7bc805f6 (agibot-x2-slam@58e623a1) | NONE (no license) => all rights reserved | N/A (JS simulation; params ASSUMED: cliff 0.9, low 0.8/0.55, CLIFF 0.30..1.9 m) | none; D-07 low-obstacle gap, tests/planner_clearance partial | concept only: depth-based cliff/low-obstacle camera layer fused with lidar | re-implement from scratch (idea not copyrightable); validate with real depth data; false-positive rate on stairs/floors |

## 5. Risks / open
- License-unknown (TODO) files: aimdk_msgs, examples, py_examples (incl. pcl_rectifier). Ask AgiBot-Community (issue) for clarification before copying code.
- Collision monitor and Ruckig would add a second layer to cmd path; validate order smoother -> collision_monitor -> adapter and that MC input-source lease is not starved.
- "Tree 9ad1ad8af1b4" in task is a commit; tree is 154eea81.
