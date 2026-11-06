# qazbot_lidar_projector

C++17 port of the D-04 variant C python bridge (`test/reference/level_bridge_d04.py`, kept only as the parity-test
oracle). **Production** since 2026-11-04 (live build of 9417bed; started by qazbot_localization as `qazbot_level_bridge`);
the old python `ws/src/lio/level_bridge.py` was removed in the 2026-11-05 consolidation (tag `pre-consolidation-fb18dc1`).
Future role: qazbot_lidar_processing (raw-LiDAR mode, P4). Pure port: same algorithm, same topics, types, QoS, frame ids and stamps.

FAST-LIO's world is its first IMU frame (not level). The node levels it from the LiDAR IMU gravity, publishes the
gravity-levelled planar pose (`/odom_level`, optional TF `odom -> base_footprint`) and projects the deskewed
`/cloud_registered` into the base frame as a 2D `LaserScan`, an obstacle cloud and a clearing-ray cloud.

Layout: `include/qazbot_lidar_projector/lidar_projection.hpp` + `src/lidar_projection.cpp` (library `lidar_projection`,
no ROS types: levelling, pose, twist, band, bins, ranges, obstacle points, clearing endpoints);
`src/lidar_projector_node.cpp` (thin node, composable component `qazbot_lidar_projector::LidarProjectorNode`);
`src/lidar_projector_main.cpp` (executable, `SingleThreadedExecutor`).

## Node `lidar_projector`
### Usage
```
ros2 launch qazbot_lidar_projector lidar_projector.launch.xml            # production remaps, package YAML
ros2 launch qazbot_lidar_projector lidar_projector.launch.xml params_file:=/path/my_copy.yaml node_name:=qazbot_level_bridge
ros2 run qazbot_lidar_projector lidar_projector                          # default (unremapped) topic names
ros2 component load /ComponentManager qazbot_lidar_projector qazbot_lidar_projector::LidarProjectorNode
```
Launch arguments (no parameter values in the launch file): `params_file`, `node_name` (default `lidar_projector`; use
`qazbot_level_bridge` to keep the old graph name that `tools/stack/check.sh` / `odom_owner_probe.py` look for),
`odometry_topic`, `cloud_topic`, `scan_topic`, `odom_level_topic`, `tf_topic`, `use_sim_time`.
Defaults of the remaps equal `localization.launch.py`: `/Odometry -> /qazbot_lio/odometry`, `/cloud_registered ->
/qazbot_lio/cloud_registered`, `/scan -> /qazbot_lio/scan`, `/odom_level -> /qazbot_lio/odom_level`,
`/tf -> /qazbot_d1_map/tf`.

### API (names before remapping; QoS identical to the Python node)
| Direction | Topic | Type | QoS |
|---|---|---|---|
| sub | `/Odometry` | `nav_msgs/Odometry` | RELIABLE, KEEP_LAST 100 |
| sub | `/cloud_registered` | `sensor_msgs/PointCloud2` (float32 x/y/z, any layout; zero-copy ConstSharedPtr) | RELIABLE, KEEP_LAST 20 |
| sub | `imu_topic` (`/aima/hal/sensor/lidar_chest_front/imu`) | `sensor_msgs/Imu` | BEST_EFFORT, KEEP_LAST 400; **dropped after levelling** |
| pub | `/scan` | `sensor_msgs/LaserScan`, frame `base_footprint`, stamp = cloud stamp | RELIABLE, VOLATILE, 20 |
| pub | `<scan>_obstacles` (`/scan_obstacles`) | `sensor_msgs/PointCloud2` xyz32, `base_footprint`, nearest in-slice return per bin (marking) | RELIABLE, VOLATILE, 20 |
| pub | `<scan>_clearing` (`/scan_clearing`) | `sensor_msgs/PointCloud2` xyz32, `base_footprint`, conservative ray endpoints (clearing only) | RELIABLE, VOLATILE, 20 |
| pub | `/odom_level` | `nav_msgs/Odometry` (`odom` -> `base_footprint`, twist over a 0.3 s baseline, covariance rotated into the level frame) | RELIABLE, VOLATILE, 50 |
| pub (only `publish_tf: true`) | `/tf` | `tf2_msgs/TFMessage` `odom -> base_footprint` | tf2 default |
No services or actions. Exactly these outputs (no extra topics). `<scan>_*` follows the resolved/remapped `/scan` name.

### Parameters (all read at start-up; not changed at runtime, so no parameter callbacks)
Package defaults are in `config/lidar_projector.yaml` (= values of the running node: `range_max 8.0`, `scan_every 2`,
`publish_tf false`). Do not edit it in place; pass a copy with `params_file`. Code defaults (when a parameter is absent) are the
Python node's: `publish_tf true`, `range_max 25.0`, `scan_every 1`.
| Name | Type | Unit | YAML / running | Description |
|---|---|---|---|---|
| `publish_tf` | bool | - | false | Broadcast TF `odom -> base_footprint`. false when `native_odom_bridge.py` owns that TF |
| `range_max` | double | m | 8.0 | Scan / obstacle / clearing range limit |
| `scan_every` | int | clouds | 2 | Process every Nth cloud (counted from the first cloud received, including clouds before levelling). `<1` is clamped to 1 with a warning (Python would raise ZeroDivisionError) |
| `range_min` | double | m | 0.45 | Nearer returns ignored |
| `z_min` / `z_max` | double | m | -0.65 / 0.80 | Slice bounds relative to the LiDAR IMU |
| `floor_margin_per_m` | double | m/m | 0.035 | Lower bound rises with range (sway margin) |
| `n_bins` | int | bins | 1440 | Bearing bins over [-pi, pi); `<1` falls back to 1440 |
| `level_s` | double | s | 1.0 | Gravity averaging window |
| `twist_baseline_s` | double | s | 0.3 | Pose baseline for the twist |
| `lidar_fwd_in_imu` | double[3] | unit vec | [-0.0038440, 0.00088109, -0.99999222] | LiDAR forward axis in the LiDAR IMU frame |
| `imu_topic` | string | - | `/aima/hal/sensor/lidar_chest_front/imu` | IMU used for levelling |
Logging: INFO once on levelling, WARN (throttled 5 s) on odometry clock step-back, on malformed clouds and on a cloud dropped
without a matching odometry stamp (with totals); INFO once at shutdown with the cloud/pose sync totals; nothing per cloud.

### TF ownership
With `publish_tf: false` (running configuration, `odom_source:=native_imu`) the node creates **no** TF publisher; the single
`odom -> base_footprint` owner is `native_odom_bridge.py`. With `odom_source:=lio` use the preset
`config/lidar_projector_lio.yaml` (identical values, `publish_tf: true`): `params_file:=$(ros2 pkg prefix qazbot_lidar_projector)/share/qazbot_lidar_projector/config/lidar_projector_lio.yaml`
(the Python launch derived this from `odom_source`; this package does not). Never have two owners.

## Behaviour notes (port fidelity)
`-ffp-contract=off` is set for the library so that no fused multiply-add changes a rounding relative to numpy. Reproduced
Python details: `t_acc0 or t` (stamp 0.0 re-arms), unnormalised quaternion, pose cache of newest 50 keyed by integer ns,
cache + twist baseline cleared when the odometry stamp goes back, `is_dense == false` => NaN points skipped, non-finite points
dropped, first point wins ties (stable lexsort), `a == pi` wraps to bin 0, `nextafter` caps in the clearing endpoints,
`/odom_level` only once a full baseline exists.

## Build, test
```
# from the repository root, build/install OUTSIDE the tree (e.g. /tmp/qazbot_build); never ROS_DOMAIN_ID 0
nice -n 19 colcon build --base-paths ws/src --packages-select qazbot_lidar_projector \
  --build-base /tmp/qazbot_build/build --install-base /tmp/qazbot_build/install --parallel-workers 2
ROS_DOMAIN_ID=88 ROS_LOCALHOST_ONLY=1 colcon test ... same bases ...   # launch tests use ROS_DOMAIN_ID from the environment (default 88 if unset/0), ROS_LOCALHOST_ONLY=1
# coverage: add --cmake-args -DENABLE_COVERAGE=ON -DCMAKE_BUILD_TYPE=Debug with separate bases, then gcov
```
Tests: `test_lidar_projection` (gtest, 31 cases incl. 8 `Sync.*`, 100 % line coverage of `lidar_projection.cpp`); `test_parity.py` (C++ library vs
the Python candidate on `test/data/static_204153_w50.bin`: 50 clouds, 981 IMU, bag `d04_static_bag_204153`, t0+60 s; R_lv from the
fixture's own first IMU second, not the live R_lv; tolerances derived in the module docstring);
`test_cloud_before_odom.py` (regression, launch_testing, isolated domain from env: same fixture, clouds k % 10 in {0, 1} published
before their odometry; expects every in-order scan, bit-identical); `test_integration.py` (launch_testing via the XML launch, isolated domain from env, event-based waits only; compares the node output
with the library dump, checks topics/types/frames/QoS/no TF publisher/IMU subscription dropped). `test/tools/` holds the
fixture extractor and `bench_compare.py`.

### Results (offline, 2026-11-03)
Parity: scan ranges bit-identical in 112 320 of 112 320 bins (both range_max 8.0 and 25.0); obstacle points within 1 float32
ulp + derived error in all rows; /odom_level pose, twist and covariance within the derived bounds; clearing points: all rows within
tolerance except 660 of 17 272 rows (3.8 %, range_max 8.0 only) where either implementation may keep a different one of several rays
of one bin that tie at `nextafter(range_max)` within rounding (max endpoint difference 1.05 cm, less than a 5 cm costmap cell).
Benchmark (X2, live stack running, nice 19, same 39 clouds x 20 passes): per-cloud compute Python `on_cloud` p50/p95/p99
4.72/5.02/5.17 ms vs C++ `CloudProcessor::process` 0.87/0.93/1.03 ms; whole process CPU per processed cloud 13.6 vs 2.3 ms; peak RSS
102 MB vs 66 MB.

### Cloud / odometry delivery order (T1 2026-11-04)
FAST-LIO publishes `/Odometry` before `/cloud_registered` with the same stamp, but the two topics are separate DDS readers and a
cloud can be taken before its odometry (measured 0.2-0.7 % of frames). The old node looked up the exact stamp, missed and
returned silently (sporadic 0.4 s gaps on `/scan`, `_obstacles`, `_clearing`). Now `CloudPoseSync` (pure core, unit-tested)
holds at most ONE cloud until the next odometry decides it: same stamp -> processed with that exact pose; newer stamp -> dropped
(its odometry was skipped). Exact stamp matching is kept (no nearest-pose substitution); there is no timer, timeout or retry.
Forced reproduction on bag `d04_static_bag_183015` (clouds k % 10 in {0, 1} before their odometry): old 116/145 scans, new
145/145, common stamps bit-identical. Regression test: `test/test_cloud_before_odom.py` (old 4/19 lost, new 0/19).

## Deploy / rollback
Built in place by `ws/tools/build.sh`; production = a tagged release commit (root README, docs/RELEASE.md). Rollback =
previous release tag (`git checkout x2-release-…`, rebuild, restart); the pre-consolidation python bridge is recoverable only from tag `pre-consolidation-fb18dc1`.

## Limitations
* Reads only little-endian FLOAT32 x/y/z (as FAST-LIO publishes); other clouds are dropped with a throttled WARN.
* Levelling uses the first IMU second after start; parity used the fixture's levelling, not the live R_lv of the running process.
* Parameters are start-up only. Intra-process composition is possible (component), but the consumers (costmaps) are other processes.
* Parity evidence is one static 5 s window of one bag; the live-matched R_lv (V1 report) was not available to this task.
* Package licence in `package.xml` is a placeholder (`Proprietary`).
