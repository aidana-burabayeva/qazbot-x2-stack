# 20261103-cpp-lidar-projector: independent verification V2

VERDICT: PASS (no blocking findings; advisories below)
Scope: /home/agi/control_backend/ws/src/qazbot_lidar_projector (C++ port of /home/agi/control_backend/candidates/d04/level_bridge.py, D-04 variant C).
Build/install: /home/agi/control_backend/ws_cpp_build/. Everything ran in ROS_DOMAIN_ID=77 under nice 19. No project files were edited. Scratch scripts are in /tmp/claude-1001/-home-agi/875283d9-732e-4c1e-ba4d-0d8188cfcd4d/scratchpad/chk/.

## Commands run
1. `colcon build --packages-select qazbot_lidar_projector` (bases ws_cpp_build/{build,install}, Release, 1 worker). Result: finished in 2.5 s. The only stderr output was a CMake developer warning from find_package(launch_testing_ament_cmake). No compiler warnings.
2. `colcon test` with the same bases and domain 77, then `colcon test-result --verbose`. Result: **30 tests, 0 errors, 0 failures, 0 skipped**. Breakdown: gtest test_lidar_projection 23, test_parity 2, launch test test_integration 5 (test_end_to_end + test_exit_codes).
3. Re-ran test_parity.py directly with PARITY_REPORT set.
   - range_max 8.0: 39 clouds, 330 062 points. range_equal 56160/56160 (bit-identical). obs_within_tol 10066 (0 explained, 0 unexplained). clr_within_tol 16612, clr_explained 660, max distance 0.010543 m, 0 unexplained.
   - range_max 25.0: range_equal 56160/56160. obs 11272 within tolerance. clr 17272/17272 within tolerance.
   - Note: 112 320 bins = 2 cases x 39 x 1440. This matches the implementer's claim.
4. Wrote my own script, chk/tie_check.py, which compares C++ and Python clearing directly.
   - Row counts and bin order are identical in every cloud.
   - Of the 660 differing rows (all at range_max 8.0), the radial difference is at most 5.8e-7 m. The largest lateral difference is 1.05 cm.
   - max(r_cpp - r_py) over all 34 544 rows is 5.8e-7 m.
   - No C++ clearing ray goes past the nearest in-band return (scan range) of its bin: 0 rows.
5. Wrote chk/tie_cause.py to check where the 660 rows come from. In all 660 cases the bin's Python maximum ray end is nextafter(range_max) within ±4 ulp. The ray C++ picks has a Python-computed end within at most 1.0 ulp of that maximum. So the cause is an ulp-level tie in arg-max, not a logic difference.
6. gcov on the existing build_cov data (the .gcda file is newer than the source): src/lidar_projection.cpp **100.00 % of 223 lines**. This matches the claim.

## (1) Behavioural equivalence: side-by-side code read (Python candidate vs lidar_projection.cpp / lidar_projector_node.cpp)
| Item | Python | C++ | Result |
|---|---|---|---|
| IMU gate | `R_lv is not None or R_wb is None` -> return | `ready() \|\| !hasOrientation()` | equal |
| t_acc0 `or` semantics (0.0 counts as unset) | `t_acc0 or t` | `!has_t0_ \|\| t0_==0.0` | equal |
| acc averaging / level window | mean(R_wb·a); `t-t0 >= LEVEL_S` | running sum / n; same test | equal (pairwise-sum rounding only; R_lv within 4nε, asserted) |
| level_rotation | z=up/‖up‖, a=x or y if \|z.x\|≥0.9, Gram-Schmidt, [x; z×x; z] | same expressions | equal |
| IMU unsubscribe after levelling | destroy_subscription | sub_imu_.reset() (integration test checks it) | equal |
| on_odom: R_wb set before R_lv check; yaw = atan2 of R_lv R_wb fwd | yes | yes | equal |
| Pose cache: clock step-back clears poses+track; keep newest 50 | sorted()[:-50] | std::map erase(begin) while >50 | equal |
| Twist: baseline pop rule, no Odometry before baseline, body-frame v, wrapped wz | yes | identical loop and formulas | equal |
| TF odom->base_footprint z=0, yaw quaternion; only if publish_tf | yes | yes | equal |
| /odom_level header (stamp, 'odom'), child base_footprint, z=p_lv.z, cov = B Σ Bᵀ, B=kron(I2,R_lv) | yes | rotateCovariance | equal |
| scan_every: increment, then skip if n%every | yes | yes | equal |
| pose lookup by exact stamp key, R_lv required | yes | yes | equal |
| NaN skipping: read_points_numpy skip_nans only when !is_dense; empty -> return | yes | skip_nans=!is_dense; read==0 -> return | equal |
| Q = R_lv P - p_lv; body rotation by -yaw; r=hypot, a=atan2 | yes | yes (-ffp-contract=off) | equal |
| Band: z > Z_MIN + 0.035·flat(level frame) and z < Z_MAX; range_min < r < range_max | yes | inBand(z, flat) + range test | equal; non-finite points are excluded on both sides |
| Bin index: trunc((a+π)/2π·N) % N | astype(int) % N | (long long) % N; operand always ≥0 | equal |
| ranges: float32 minimum per bin, inf if empty | minimum.at float32 | min(float) | equal (bit-identical in parity run) |
| Obstacle point: smallest r per bin, first on tie (stable lexsort); (r cos a, r sin a, 0) as float32 | yes | strict `<` | equal |
| Clearing a: all finite points with r>0, t=min(1, range_max/r) | yes | yes | equal |
| Clearing b: floor/Z_MAX exits (slope<0 -> Z_MIN/slope; z>0 -> Z_MAX/z); end=nextafter(r t,0) | yes | yes | equal |
| Clearing c: obstacle[bin] = min r of hits (r-based band, range_min<r<range_max) | yes | yes | equal |
| Clearing d: longest end per bin (first on tie), bounded=nextafter(min(end, obstacle)), drop if ≤ range_min, scale bx,by | yes | strict `>` first-wins, same | equal |
| No bearing window (variant C) | valid = r>0 | none | equal |
| LaserScan: stamp=cloud stamp, frame base_footprint, angle_min -π, inc 2π/N, angle_max, range_min/max, no intensities, time_increment/scan_time 0 | yes | yes | equal |
| Output clouds: create_cloud_xyz32 layout (x,y,z FLOAT32 at 0/4/8, step 12, height 1, is_dense False, LE), header = scan header; publish order scan, obstacles, clearing | yes | makeXyzCloud identical | equal |
| range_max param used for scan, hit, clearing | global RANGE_MAX | core_.range_max | equal |
Result: no logic divergence. The only differences are floating-point rounding at the ulp level and the hardening listed under advisories.

## (2) range_max tie explanation
Verdict: confirmed and safe. All 660 differing rows are bins where several rays tie, at the 1-ulp level, for the longest end at nextafter(range_max). Python and C++ then keep a different ray of the same bin. The ray length is the same within 5.8e-7 m, because it is set by the same min(end, obstacle[bin]). Only the bearing inside the 0.25° bin changes, by at most 1.05 cm. C++ never clears further than Python (max excess 5.8e-7 m) and never past the bin's nearest in-band return (0 rows). An observed obstacle can only be cleared if its bin's nearest return lies beyond the cap, and the cap is the same bin-wide value as in Python. This is the same within-bin bearing freedom Python already has.

## (3) QoS, topics, types against the running node (P1P2 report + ws/src/navigation/localization.launch.py:94-104)
- Publishers:
  - /scan LaserScan, depth 20
  - <resolved scan>_clearing PointCloud2, depth 20
  - <resolved scan>_obstacles PointCloud2, depth 20
  - /odom_level Odometry, depth 50
  - All RELIABLE/VOLATILE, which matches P1P2 line 18 (RELIABLE/VOLATILE publisher, BEST_EFFORT subscribers are compatible).
- Subscriptions:
  - IMU /aima/hal/sensor/lidar_chest_front/imu: BEST_EFFORT, KEEP_LAST 400
  - /Odometry: depth 100
  - /cloud_registered: depth 20
  - All identical to Python.
- Launch remaps match the running ExecuteProcess: /Odometry, /cloud_registered, /scan, /odom_level -> /qazbot_lio/*, and /tf -> /qazbot_d1_map/tf. There is no /tf_static remap, but the node uses no static broadcaster or listener, so none is needed.
- The parameter names the running launch passes (range_max, scan_every, publish_tf) exist with the same types. The package YAML holds the running values: 8.0, 2, false.
- Node name defaults to lidar_projector. A drop-in needs `node_name:=qazbot_level_bridge`, because check.sh and odom_owner_probe.py expect that name. This is documented.

## (4) Tests
Re-run once: 30/30 pass (see commands 2-3). The 23/2/5 split is consistent with the report. The integration test uses no sleeps; it uses spin_until with timeouts (test_integration.py:10).

## (5) Henki standard compliance
| Rule | Status |
|---|---|
| Single responsibility; logic in a ROS-free library (lidar_projection) and a thin node | compliant |
| launch/ folder, XML launch | compliant |
| Parameters in config/ YAML, nothing hard-coded in launch | compliant; only use_sim_time is passed from a launch arg, which is common practice (minor) |
| Package YAML treated as defaults; users override with a copy via params_file | compliant (header comment, README) |
| Dynamic parameters | none, all read at start-up; justified because none change at runtime (README) |
| ROS logger only, no print/cout; throttled logs on the hot path | compliant; the unthrottled WARN/INFO calls are start-up or one-shot only |
| Standard interfaces (sensor_msgs, nav_msgs, geometry_msgs), no std_msgs primitives | compliant |
| SingleThreadedExecutor | compliant (lidar_projector_main.cpp) |
| C++ composable node | compliant (rclcpp_components_register_node; zero-copy const subscription, unique_ptr publish) |
| package.xml dependencies | compliant |
| README: overview, usage, API, parameters with type/default | compliant |
| Unit tests with high coverage | 100 % lines of the core; parity and integration tests also present |

## Advisories (non-blocking)
- A1 config/lidar_projector.yaml:8 sets `publish_tf: false`, while the code default is true. This mirrors the running odom_source native_imu. With odom_source lio, a YAML copy with publish_tf true is required; otherwise there is no odom->base_footprint owner. That fails closed (Nav2 sees no TF), it is not unsafe. Documented in the README and the implementer report.
- A2 lidar_projector_node.cpp:45-55,173: clouds whose x/y/z are not little-endian FLOAT32 are dropped with a throttled WARN. Python would also accept other dtypes. FAST-LIO publishes float32 little-endian, so there is no practical effect.
- A3 lidar_projector_node.cpp:38-41,69-72: n_bins<1 and scan_every<1 are clamped with a WARN. Python would raise or divide by zero. This is hardening, not a behaviour change for valid configs.
- A4 test_parity.py:232-235,246: rows classified as "explained" have no magnitude assertion; the distance is only reported. My independent checks (commands 4-5) bound them, but adding an assertion on |dr| and the lateral distance would make the test self-contained. Recommended, not required.
- A5 Evidence covers one static 5 s fixture window. Live R_lv, a moving scenario, and the replay_d04 lethal-cell parity (scope step 3) were not done. Deployment was not done or verified. These must be covered before the deploy gate.

## REQUIRED_FIXES
None blocking.
