# level_bridge C++ migration — scope note (NOT implemented; D-04 P4)
Standard: ~/.claude/standards/henki_ros2_best_practices.md. Behaviour reference: candidates/d04/level_bridge.py
(sha256 in the P3P4 report) + tests/d04 suite. No C++ is written in this cycle.

## Why
level_bridge is the high-bandwidth path (dense /cloud_registered ~8.4 k pts x 48 B at 10 Hz in, 3 outputs at 5 Hz)
in rclpy + numpy (~8 % CPU live, scan_every 2 exists only to save Python CPU). Henki: C++ for high-bandwidth nodes;
composable node + intra-process for point clouds.

## Package layout (ament_cmake, C++17)
- `qazbot_level_bridge` (new package)
  - `include/qazbot_level_bridge/level_core.hpp`, `src/level_core.cpp` — PURE library, no rclcpp:
    `levelRotation(up)`, `quatToRot`, pose levelling (p_lv, yaw from LIDAR_FWD_IN_IMU),
    `inBand(q, z_min, z_max, floor_margin_per_m)`, `obstaclePoints(Q, yaw, range_min, range_max, n_bins)`
    (nearest in-band return per bin), `scanRanges(...)`, `clearingEndpoints(Q, yaw, range_min, range_max, n_bins)`
    (D-04 candidate semantics: no bearing window, band-exit truncation, per-bin longest ray capped by nearest hit).
    Inputs/outputs as std::vector<Eigen::Vector3d>/float spans; no allocation in the hot loop after warm-up.
  - `src/level_bridge_node.cpp` — rclcpp::Node, composable (`rclcpp_components_register_node`), single responsibility:
    subscriptions (Odometry, PointCloud2, lidar IMU for the first LEVEL_S only), stamp-keyed pose cache (50),
    clock-step-back handling, publishers /scan, scan_obstacles, scan_clearing, /odom_level, optional TF.
    SingleThreadedExecutor; RCLCPP_*_THROTTLE for per-cloud warnings.
  - `config/level_bridge.yaml` — range_max, scan_every, publish_tf, Z_MIN/Z_MAX/FLOOR_MARGIN_PER_M, n_bins,
    range_min, level_s, twist_baseline_s (today module constants -> declared params, defaults = Python values).
  - `launch/level_bridge.launch.xml` (XML front-end, params from config, no hard-coded values).
  - `test/test_level_core.cpp` (gtest, >= 90 % line coverage of the library) — ports tests/d04/test_clearing_geometry.py
    cases 1:1; `test/test_level_bridge_node.py` (launch_testing integration: publish recorded clouds in an isolated
    ROS_DOMAIN_ID, wait on messages with timeouts, no sleeps).
  - `README.md` per Henki (topics, params with type/default, usage).
  - package.xml deps: rclcpp, rclcpp_components, sensor_msgs, nav_msgs, geometry_msgs, tf2_ros, eigen, ament_cmake_gtest.

## Parity plan (gate before any deploy)
1. Recorded clouds (tests/d04 bags) -> Python candidate and C++ library offline; compare per-cloud outputs.
2. Tolerances derived from numerics, not chosen: inputs are float32 (eps 6e-8 rel, ~1e-6 m at 8 m) and the Python
   path computes in float64 then publishes float32. Expect bit-identical bin indices except points within one float32
   ulp of a bin edge (count them and require each mismatch to be such a point); endpoint coordinates equal after the
   float32 cast (|d| <= 1 float32 ulp of the value).
3. Re-run tests/d04/replay_d04.py with a C++-output adapter: identical lethal-cell sets per update for A/C scenarios.
4. Never run Python and C++ publishers in the production namespace at once (isolated domain or remapped outputs).

## Out of scope here
Motion adapter C++ migration (separate task), VoxelLayer/STVL adoption, any param/launch change on the robot.
