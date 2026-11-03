# tests/d04 — D-04 costmap clearing (offline only)

Offline A/B/C/D replay of the X2 local-costmap ObstacleLayer on recorded
`/cloud_registered` bags, plus the regression suite for the D-04 candidate
`docs/engineering/d04_d05_20261102/candidate/level_bridge.py`. Nothing here publishes, subscribes or plays
bags into a ROS graph: bags are read from sqlite, and the bridge methods are
called with a stub `self`.

## One command (replay, ~25 min at nice 19 on the X2, single process)
    source /opt/ros/humble/setup.bash
    export QAZBOT_D04_BAGS=/path/to/bags   # see BAGS.md (default: robot's evidence dir)
    nice -n 19 python3 <repo>/tests/d04/replay_d04.py --level fit --out <repo>/tests/d04/results
Output: `results/summary.json` (metrics per bag/scenario/variant + coverage +
LiDAR attitude + file SHA-256s), `results/raw.json` (per-update series).
Default bags: the two registered live static bags (BAGS.md). Subset: `--bags static_204153 --scenarios S9_D04_right_-52deg_0.5m_transient`.

## Tests (~2 s + one real-bag integration test)
    source /opt/ros/humble/setup.bash
    nice -n 19 python3 -m pytest -q -p no:cacheprovider <repo>/tests/d04

## Files
| file | role |
|---|---|
| costmap_emul.py | Humble 1.1.18 ObstacleLayer emulation (Bresenham transcribed from the installed header) + RPP isCollisionImminent |
| replay_d04.py | harness: levelling/poses via the RUNNING node's on_imu/on_odom, A = running on_cloud, C = candidate on_cloud, B = single PointCloud2 source, D = approximate VoxelLayer |
| test_costmap_emul.py | emulator fidelity (header transcription parity, clear-before-mark, height filter, ranges, footprint, RPP) |
| test_clearing_geometry.py | candidate clearing geometry: edge sectors, blind bins, floor-band rule, Z_MAX, range cap, hit cap, node path, scope guard |
| test_replay_d04.py | real-bag integration: D-04 edge transient (A keeps / C clears) and blind-sector seeds kept |

## Variants
A running split (`scan_obstacles` mark-only + `scan_clearing` bounded rays in fixed -46..+44.5 deg);
B one 3D cloud, marking+clearing (AgiBot style), fixed height band;
C candidate: bounded rays from every actual point (no bearing window);
D approximate VoxelLayer (3D marks, 3D ray clearing, sampled rays);
E STVL: not installed, not replayed.
SYNTHETIC scenarios re-range recorded beams that hit an injected box; no beam is invented.

## Real Nav2 check (isolated domain 77)
    nice -n 19 python3 <repo>/tests/d04/nav2_real_costmap.py     # -> results/nav2_real.json
Runs the installed Humble `nav2_costmap_2d` standalone node (live local_costmap params, use_sim_time) fed with
A/C `scan_obstacles`/`scan_clearing` built from bag d04_static_bag_204153 + the bag's odom->base_footprint TF;
compares `costmap_raw` lethal cells with the emulator. Never domain 0, localhost-only DDS.
