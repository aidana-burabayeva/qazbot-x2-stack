# 20261103-cpp-projector-integration (robotics-implementer/tester)

STATUS: DONE. All runs in ROS_DOMAIN_ID=77, ROS_LOCALHOST_ONLY=1, nice 19. Domain 0 was never used.
Scripts and raw results are in /tmp/claude-1001/-home-agi/875283d9-732e-4c1e-ba4d-0d8188cfcd4d/scratchpad/ci/.
- Harness: run3.py (start-paused bag play, resume after all nodes have matched). Analysis: an_costmap.py, an_out.py, an_edge.py. Moving-bag generator: synth_moving.py.
- Outputs per run: out_<run>/stats.json and data.pkl.

## Setup
- C++ node: `ros2 launch qazbot_lidar_projector lidar_projector.launch.xml node_name:=qazbot_level_bridge use_sim_time:=true`, with params_file set to the package YAML. Python: candidates/d04/level_bridge.py (candidate C), range_max 8, scan_every 2, use_sim_time.
- Both nodes ran at the same time on the same replay. Outputs were remapped to /cpp/* and /py/*. Each fed its own real Humble nav2_costmap_2d standalone instance with the live local_costmap params (4x4 m, 0.05 m, obstacle_layer scan marking and bounded_rays clearing, taken from qazbot_ctl_t9htg14z.yaml).
- Costmap lifecycle was configure then activate. Two or three instances share the node name /costmap/costmap, so each one had costmap_raw and change_state remapped.
- Bag replay used `ros2 bag play --clock 100`.
- Static data: d04_static_bag_204153, first 60 s. Topics: cloud_registered, lio odometry, lidar IMU, and the bag's native-odom TF /qazbot_d1_map/tf remapped to /tf. The node had publish_tf false (the package YAML), as in the running configuration.
- Phase fix: the first harness version let the two nodes start with a different `scan_every` phase, so they processed different clouds. start-paused plus resume fixes it. Every result below uses the fixed harness (run3).

## Moving data (limitation, read first)
- There is NO bag with a moving robot AND a LIO cloud_registered. All cloud_registered bags are static: udp_20261028 moves 0.16 m, replay_* 0.2 m, d04_static 2 m of LIO jitter. In the walk and turn bags LIO itself stayed put (LIO path 2 m, extent 0.12 m) while leg_odometry shows 1 m of walking and 450 to 1090 deg of yaw.
- I therefore SYNTHESISED a moving cloud_registered bag from manual_amcl_walk_02 (60 s). The raw lidar cloud was registered with a pose built from leg_odometry (x, y, yaw delta) on top of the LIO initial orientation, using the real lidar-to-IMU extrinsics. Every third point was kept, giving 8.4k points per cloud like the real cloud. 610 clouds, 12k IMU messages, 1.35 m path, 454 deg yaw.
- The pose is held still for the first 2.5 s so that the 1 s gravity levelling is deterministic.
- The geometry is approximate, so use the data for rate, latency, CPU, parity and edge behaviour, not for map quality.
- In this mode the node publishes the TF itself (publish_tf true, via the NEW config/lidar_projector_lio.yaml, which exercises A1).
- The levelling `up` vector is [0.9973 -0.0164 0.0714] in both nodes.

## 1. Costmap parity (real Nav2, costmap_raw lethal cells = 254)
| data | publications matched | lethal cells summed, C++ / Py | identical cells | publications with an identical lethal set |
|---|---|---|---|---|
| static 204153, first 60 s | 107 | 16233 / 16233 | 16233 (Jaccard 1.0000, 0 only-C++, 0 only-Py) | 107/107 (cell-level 100 %) |
| synthetic moving | 101 | 9240 / 9240 | 9239 (Jaccard 0.9998, 1 only-C++ and 1 only-Py) | 100/101 |

- The final costmap of both runs is identical (static 137 cells; moving 281 cells).
- Output clouds on the 150 stored scans, static: the obstacle clouds are bit-identical on 150/150. The clearing clouds have the same bin set on 150/150 and the same row count. Only the known within-bin tie rows differ (clearing bearing within a bin, at most 1.0 cm; moving data 1.3 cm; radial 6e-7 m). That is the 660-row ulp tie explained by V2.
- Output clouds, moving: same result, obstacle clouds 150/150 bit-identical.
- Control: a third costmap fed with the C++ outputs gave 100 % agreement on static. On moving data it gave only 0.65 Jaccard against both of the others. The cause is not found. Suspects are a third same-named costmap instance and timing under a rotating TF. It did not affect the C++-vs-Python comparison, which is 0.9998.

## 2. Moving data: rate, drops, latency, CPU, RSS
Single-node runs, one node at a time, same synthetic moving replay, 59 s (579 clouds in, 10 Hz):
| | scans | rate | latency p50 / p95 / p99 (ms) | CPU (% of one core) | RSS mean / peak (MB) | threads |
|---|---|---|---|---|---|---|
| C++ | 295 | 5.00 Hz | 0.91 / 1.46 / 2.13 | 2.5 | 32.7 / 33.2 | 13 |
| Python C | 295 | 5.00 Hz | 5.46 / 6.88 / 7.81 | 13.4 | 70.4 / 71.9 | 19 |

- Scan rate is 5.00 Hz for both, equal to clouds/2.
- Drops: 0 for both. The 295 vs 305 expected gap is the 1 s levelling (about 10 clouds), identical in both nodes. No cloud was lost after levelling.
- Latency is the wall-clock difference between the cloud arriving at the probe and the scan arriving at the probe, matched by header stamp. It includes DDS delivery.
- Static 204153 single-node runs (59 s, 292 expected scans): C++ 287 scans, 0.98 / 1.25 / 1.51 ms, 2.4 % CPU, 30 MB. Python 287 scans, 5.52 / 6.32 / 6.86 ms, 13.1 % CPU, 71 MB.
- Ratio: C++ is about 5.5x lower in latency, 5.3x lower in CPU and 2.2x lower in RSS than Python.
- In the dual runs (both nodes plus costmaps running) the figures are close: C++ 2.6 % and 1.1 ms; Python 13.9 % and 5.5 ms.

## 3. FOV edges (150 scans each; sectors by bearing in base_footprint)
| | C++ | Python |
|---|---|---|
| static: obstacle marks in 44..60 deg | 3549 | 3549 |
| static: clearing rays in 44..60 deg | 13648 | 13648 |
| static: marks / rays beyond 60 deg | 0 / 0 | 0 / 0 |
| moving: obstacle marks in 44..60 deg | 1385 | 1385 |
| moving: clearing rays in 44..60 deg | 13807 | 13807 |
| moving: marks / rays beyond 60 deg | 0 / 0 | 0 / 0 |

- Nav2 lethal cells (|bearing| 44..60 deg, r<3.5 m, summed over publications): static 1054 C++ and 1054 Python; moving 3308 and 3306. Beyond 60 deg: 0 and 0 on static.
- Input geometry: the lidar sees at most 57.3 deg (static) and 57.2 deg (moving). The clearing rays span -57.3..+55.8 deg in both nodes.
- No clearing in unobserved bearings. I recomputed the bearing of every input point with the candidate's own level_rotation and yaw. Every C++ and Python clearing ray falls in a bearing bin that holds an input point. 2 or 3 rays out of 66k sit one bin off exactly on a bin boundary (0 violations with a +-1 bin tolerance), identically in both nodes. Obstacle marks in unobserved bins: 0.
- Rays or marks beyond 60 deg: 0.

## 4. Advisories
- A1 fixed: new /home/agi/control_backend/ws/src/qazbot_lidar_projector/config/lidar_projector_lio.yaml, identical to the default except publish_tf true. It is installed through the existing directory install of config/. The README "TF ownership" section now points to it. It was exercised in the moving runs, where the node's own TF fed the costmaps.
- A4 fixed: test_parity.py now asserts the explained rows. The bounds, with the measured value in brackets:
  - the explained distance is at most one bin arc at range_max, 3.5 cm at 8 m (measured 1.05 cm);
  - the clearing radial difference is at most 1e-5 m (5.8e-7);
  - explained rows are at most 5 % of all rows (660 of 27888, 2.4 %);
  - range_explained is at most 0.1 % of bins (0).
  The dumped stats gained `*_explained_max_dr_m` and `obs_explained_max_dist_m`.
- A5: items 1 to 3 above, within the limitation that no real moving LIO bag exists. Deployment was not done.

## Build and tests
- Rebuilt only qazbot_lidar_projector with nice 19 (bases ws_cpp_build/build and install, Release, one worker, 2 s). Ran colcon test once in domain 77.
- Result: 30 tests, 0 errors, 0 failures, 0 skipped.
- The parity report is /tmp/claude-1001/-home-agi/875283d9-732e-4c1e-ba4d-0d8188cfcd4d/scratchpad/ci/parity_a4.txt. range_max 8: clr_explained 660, max distance 0.01054 m, max dr 5.8e-7 m. range_max 25: 0 explained.

## Henki standard
- No new nodes. The launch, config and README changes follow it (the parameter YAML lives in config/, the launch carries no parameters).
- The harness scripts use print in tooling only. No deviation.
