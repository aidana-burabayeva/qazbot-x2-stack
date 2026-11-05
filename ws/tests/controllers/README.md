# Offline Nav2 controller comparison (RPP / DWB / MPPI) — task 20261103-controller-compare

Closed-loop, offline comparison of the Nav2 Humble **1.1.18** controller plugins, using the real installed binaries.
Nothing is installed system-wide, nothing is deployed. Graph is isolated: `ROS_DOMAIN_ID=78`, localhost-only unicast
DDS profile (`dds_domain78_unicast.xml`), `nice -n 19 ionice -c3`. One sim at a time.

## Usage
```bash
./fetch_dwb.sh <scratch>/dwb_prefix          # once: DWB debs (download only, sha256-checked), dpkg-deb -x
export DWB_PREFIX=<scratch>/dwb_prefix/opt/ros/humble
./run.sh matrix                               # 3 controllers x 4 scenarios x {ideal, mc_hyp}, seed 1 -> results/
./run.sh matrix --controllers mppi --scenarios transient --models mc_hyp --seed 2
./run.sh single --controller rpp --scenario nominal --model ideal --out $PWD/results/x.json
./run.sh table                                # markdown table of results/summary.json
CS_LOG_LEVEL=debug ./run.sh single ...       # controller_server log level
python3 -m pytest -q test_sim_model.py       # unit tests of the pure logic (no ROS)
CMP_CLEAR_FOV=-54,54 ./run.sh matrix --results $PWD/results_C   # D-04 candidate C clearing band
python3 summarize.py results/                # compact table
```
Binary sources (overlaid by `run.sh` in this order): `/opt/ros/humble` (costmap_2d, nav2_util, msgs),
`runtime/qazbot_nav_deps/root` (controller_server, RPP, velocity_smoother, lifecycle_manager, nav_2d_utils),
`runtime/nav_ext_ws/root` (MPPI), `$DWB_PREFIX` (dwb_core/plugins/critics/msgs, costmap_queue).
RMW is `rmw_fastrtps_dynamic_cpp` for every run: the dwb_msgs/nav_2d_msgs fastrtps typesupport libraries link
`libfastcdr.so.1`, the host has Fast-CDR 2.x only (same issue as the rejected STVL deb, P25).

## Components
| Process | Role |
|---|---|
| `controller_server` (+ `local_costmap`) | plugin under test; FollowPath action `/follow_path`; publishes `/cmd_vel_nav` |
| `velocity_smoother` | live config, `/cmd_vel_nav` -> `/cmd_vel` |
| `lifecycle_manager_cmp` | autostart of the two nodes above |
| `run_compare.py single` (`cmp_sim` node) | robot sim + synthetic LiDAR + FollowPath client + metrics |

`launch/controller_harness.launch.xml` (args `common_yaml`, `controller_yaml`, `cs_log_level`) starts the Nav2 nodes;
all parameters are in `config/` (`common.yaml` = live costmap/checkers/smoother, `rpp|dwb|mppi.yaml` = FollowPath).

### cmp_sim node API
| Topic | Type | Dir | Notes |
|---|---|---|---|
| `/cmd_vel` | geometry_msgs/Twist | sub | smoothed command = robot input (what the adapter sees) |
| `/cmd_vel_nav` | geometry_msgs/Twist | sub | raw controller output (oscillation, period metrics) |
| `/odom` | nav_msgs/Odometry | pub 50 Hz | ground truth (D-06 leg-odom under-read NOT modelled) |
| `/tf`, `/tf_static` | tf2_msgs/TFMessage | pub | `odom->base_footprint` 50 Hz, `map->odom` identity |
| `/scan_obstacles` | sensor_msgs/PointCloud2 | pub 10 Hz | marking points, z=0, frame base_footprint |
| `/scan_clearing` | sensor_msgs/PointCloud2 | pub 10 Hz | clearing ray endpoints (hit or 3.5 m) |
| `/follow_path` | nav2_msgs/action/FollowPath | client | fixed LIVE-2 global path |

No ROS parameters; scenario/model/seed are CLI arguments (`single --controller --scenario --model --seed --out`).

## Model
- Path: first `/qazbot_d1_map/nav2/plan` of LIVE-2 `nav_bag_183029` (`data/live2_plan_183029.json`, map frame,
  5.31 m, 101 poses). Same path, start pose and seed for every controller.
- World: D1 map `ws/src/maps/d1_qazbot_20261029` (walls) + 0.4 x 0.4 m tall boxes.
- Synthetic LiDAR (from D-04b): FOV +-54 deg, 1 deg rays, marking only 0.45..3.0 m, clearing to 3.5 m, 1 cm noise.
  Points at z=0 (Humble ObstacleLayer per-source `max_obstacle_height` defaults to 0.0; level_bridge output is 2D).
- Robot: `ideal` (cmd == motion) or `mc_hyp` = **HYPOTHESIS** MC response from D-05: dead time 0.4 s,
  v = 0.34 + 0.53*cmd for |cmd| >= 0.02 (2-point line, report: "NOT a model"), tau_v 0.25 s, wz 1:1, tau_w 0.15 s.
  `mc_lin` = alternative hypothesis: constant gain 1.61 + same lag. All: commands older than 0.15 s -> zero (adapter cmd_timeout).

## Scenarios
| name | content |
|---|---|
| `nominal` | walls only |
| `static_block` | box centred on the path at s = 3.0 m from the start |
| `transient` | box at s = 2.4 m appears when the robot reaches s = 1.0 m, removed 6 s later |
| `stale_cell` | D-04b-like LETHAL cluster (start-pose body x 0.02..0.26, y 0.31..0.50) marked once, never re-observed (outside FOV / min range); NOT a real obstacle |

FollowPath only (no BT): an ABORTED result is where the live BT would replan / run recoveries — "replans/recoveries"
are therefore reported as the abort cause, not executed.

## Metrics (per run JSON in `results/`, aggregated in `summary.json`)
status/time, progress (arc length), deviation from the path (mean/max), min footprint clearance (walls + boxes) and
contact, box clearance, stop/resume (transient), `wz` sign flips/s and `vx` jerk RMS of `/cmd_vel_nav`,
P23 adapter view of `/cmd_vel` (pure-rotation fraction |vx|<0.02 & |wz|>0.15, low-vx fraction 0.02..0.15,
mode switches, gaps > 150 ms), `/cmd_vel_nav` period p50/p95/p99 (compute stretch), controller_server CPU %,
CPU ms per control cycle, RSS, and log counts (collision ahead, failed progress, MPPI optimizer fail, DWB no valid
trajectories, missed control rate).
