# 20261103-cpp-level-bridge — implementer report — COMPLETE
STATUS: OFFLINE_VERIFIED (candidate; not deployed, no restarts, no edits to running files; ws/src/lio/level_bridge.py sha c817a5ed... unchanged)
RENAME applied per coordinator: package `qazbot_lidar_projector` (ws/src/qazbot_lidar_projector), lib `lidar_projection` (ns qazbot_lidar_projector),
node/component `lidar_projector` (LidarProjectorNode), config/lidar_projector.yaml, launch/lidar_projector.launch.xml. Old qazbot_level_bridge skeleton
(only empty dirs, never had content) removed (moved/renamed). Topics/types/QoS/frames identical to Python (/qazbot_lio/scan, scan_obstacles, scan_clearing, odom_level).

## Built (colcon --packages-select, nice 19, 1 worker, -j2, bases /home/agi/control_backend/ws_cpp_build/{build,install}); no warnings, no missing deps (Eigen not needed)
Files: include/qazbot_lidar_projector/{lidar_projection.hpp,lidar_projector_node.hpp}, src/{lidar_projection.cpp,lidar_projector_node.cpp,lidar_projector_main.cpp},
config/lidar_projector.yaml (running values: range_max 8.0, scan_every 2, publish_tf false; every param typed/unit/commented), launch/lidar_projector.launch.xml
(params via file, remap args, node_name arg for drop-in `qazbot_level_bridge`), test/*, README.md (overview, usage, API, params, TF ownership, deploy, rollback, limitations, results), package.xml.
Core is ROS-free and compiled -ffp-contract=off (no FMA vs numpy). Node: thin, SingleThreadedExecutor, rclcpp_components registered, zero-copy cloud sub, unique_ptr publish, IMU sub dropped after levelling, throttled WARN only.

## Tests (colcon test, ROS_DOMAIN_ID=77): 3/3 ctest suites, xunit 30 tests 0 failures
- gtest test_lidar_projection: 23 pass; gcov line coverage lidar_projection.cpp 100 % (223/223), branches executed 100 %, taken 83 % (separate build ws_cpp_build/build_cov).
- test_parity.py: 2 cases (range_max 8.0 and 25.0), 39 processed clouds, 330 062 points, 981 IMU msgs, Python candidate on_imu/on_odom/on_cloud (stub self) vs C++ lib.
  scan ranges: 112 320/112 320 bins bit-identical; obstacles: all rows within 1 ulp32 + derived error; R_lv, /odom_level pose/twist/covariance within derived bounds;
  clearing: all within tol except 660/17 272 rows (8.0 case only) = ties among rays that all end at nextafter(range_max) (either impl may keep another ray of the bin; max diff 1.05 cm).
  Derivation (eps=2^-53, gamma_n bounds, bin-edge/threshold ambiguity, ties) is in the test module docstring; 0 unexplained.
- test_integration.py (launch_test via XML launch, domain 77, no sleeps; parameter-service barrier + graph/message events; canonical replay order to remove cross-topic races): PASS, 15/15 consecutive runs.
  Checks QoS RELIABLE/VOLATILE, exact output set (no extra topics, no TF publisher with publish_tf false), frames/stamps/fields, data == library dump, IMU sub dropped. Real 48-byte FAST-LIO layout fed.
- Fixture: test/data/static_204153_w50.bin (5.1 MB, 50 clouds, bag d04_static_bag_204153 t0+60 s). R_lv from fixture's own first IMU second (live R_lv fit script was not available).

## Benchmark (X2 with live stack running, nice 19; test/tools/bench_compare.py; same 39 clouds x 20 passes)
| | Python level_bridge | C++ lidar_projector |
| per-cloud compute p50/p95/p99 (ms) | 4.72 / 5.02 / 5.17 (on_cloud incl. msg build) | 0.87 / 0.93 / 1.03 (core process, no msg build) |
| whole node CPU per processed cloud | 13.6 ms | 2.3 ms |
| peak RSS | 102 MB | 66 MB |

## Deviations / notes
- Params static (read at start; none expected to change at runtime) -> no parameter callbacks. scan_every<1 / n_bins<1 clamped with WARN (Python would raise).
- Launch derives nothing from odom_source: with odom_source lio use a YAML copy with publish_tf true. Default node name lidar_projector; graph probes (check.sh, odom_owner_probe.py) expect /qazbot_level_bridge -> pass node_name:=qazbot_level_bridge.
- parity_dump has an unused optional 5th arg (first_cloud_key). Package licence "Proprietary" placeholder.
- Housekeeping: bench/manual runs briefly left own orphan nodes in domain 77 (killed by PID); none remain. A pkill was denied by the permission classifier (not retried).
- Limits: parity = one static 5 s window; no live R_lv; deploy/replay_d04 lethal-cell parity (scope step 3) not done.
