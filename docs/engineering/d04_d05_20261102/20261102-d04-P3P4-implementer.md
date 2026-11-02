# 20261102-d04-costmap-source — P3+P4 implementer report
STATUS: DONE (offline only). Candidate C = OFFLINE_VERIFIED. HARDWARE DEPLOY: NOT APPROVED / not done.
Host X2 (agi), live stack running. All compute nice -n 19, one process at a time, bags read from sqlite (no `ros2 bag play`, no ROS graph traffic, no domain use). No edits to running files, no params, no restarts, no installs, no goals, no motion.
Standard applied: ~/.claude/standards/henki_ros2_best_practices.md (logic/node split, tests, C++ scope note).

## 0. Deliverables
| path | what |
|---|---|
| /home/agi/control_backend/candidates/d04/level_bridge.py | CANDIDATE (sha256 c880d3c8f66eed3c58eecb1db23024a37fd6d877704d6d779cac61906f75abef) |
| /home/agi/control_backend/candidates/d04/level_bridge.diff | diff vs running ws/src/lio/level_bridge.py (sha256 c817a5ed503386be..., unchanged) |
| /home/agi/control_backend/candidates/d04/CPP_MIGRATION_SCOPE.md | C++ migration scope (not implemented) |
| /home/agi/control_backend/tests/d04/costmap_emul.py | Humble 1.1.18 ObstacleLayer + RPP isCollisionImminent emulation |
| /home/agi/control_backend/tests/d04/replay_d04.py | replay harness A/B/C/D (one command) |
| /home/agi/control_backend/tests/d04/table_d04.py | renders results/table.md from summary.json |
| /home/agi/control_backend/tests/d04/test_*.py, conftest.py, README.md | regression suite + docs |
| /home/agi/control_backend/tests/d04/results/{summary.json,raw.json,table.md,run.log} | full replay results (4 bags x 11 scenarios x 4 variants) |
Side effect (disclosed): first harness run wrote ws/src/lio/__pycache__/level_bridge.cpython-310.pyc (20:44, bytecode cache of the UNCHANGED source; the live node runs the .py as __main__ and does not use it). Harness now sets sys.dont_write_bytecode; verified no further writes. Not deleted (it may have pre-existed; harmless).

## 1. Commands (reproducible)
```
source /opt/ros/humble/setup.bash
nice -n 19 python3 /home/agi/control_backend/tests/d04/replay_d04.py --out /home/agi/control_backend/tests/d04/results   # ~25 min
python3 /home/agi/control_backend/tests/d04/table_d04.py /home/agi/control_backend/tests/d04/results/summary.json
nice -n 19 python3 -m pytest -q -p no:cacheprovider /home/agi/control_backend/tests/d04                              # 54 passed in 8.1 s
```
Determinism: subset re-run (udp_20261028, S9) gave identical non-timing metrics (True).

## 2. Inputs — CORRECTION to P1/P2 bag inventory
- i1_cloud_registered_20261028_084925Z_r2 and _084925Z contain ZERO clouds (metadata message_count 0; capture_summary matched_clouds 0, r2 "interrupted"). Not usable.
- Used (all robot STATIC, open scene, no people): 
  | bag | clouds used (scan_every 2, after levelling) | levelling |
  |---|---|---|
  | docs/evidence/i1_cloud_registered_udp_20261028_092700Z | 37 (100 clouds, 9.9 s) | running on_imu (lidar IMU) |
  | logs/replay_dense_01/bag | 53 | floor-plane fit (no IMU in bag) |
  | logs/replay_portable_01/bag | 68 | floor fit |
  | logs/replay_dense_controlled_01/bag | 68 | floor fit — gives results identical to portable_01 (same source data) -> effectively 2 independent scenes |
  Floor-fit check: LiDAR pitch 4.24-4.25° (floor fit) vs 4.18° (IMU levelling) — agree within 0.07°.
- Not used: raw-lidar bags (manual_amcl_*, mc_execution23/*) need a FAST-LIO re-run (CPU on live robot, excluded); native_qualification27_cloud01 has 3 clouds.
- No bag has people/chairs/glass/low objects -> all obstacle scenarios are SYNTHETIC (below).

## 3. P3 fidelity of the emulation (costmap_emul.py)
- raytraceLine/bresenham2D/MarkCell: TRANSCRIBED from installed header /opt/ros/humble/include/nav2_costmap_2d/nav2_costmap_2d/costmap_2d.hpp (end cell acted on; max_length/min_length scaling; sign(0) = -1). Test: vectorised version == scalar header transcription on 1600 random lines x 4 (max,min) settings.
- From 1.1.x .cpp (NOT on disk -> recalled, same caveat as D2): updateBounds clear-all-then-mark-all; raytraceFreespace map-edge clipping; cellDistance = ceil; marking uses 3D squared distance vs obstacle_max/min_range and layer max_obstacle_height; ObservationBuffer height filter min<=z<=max (0/0 keeps only z==0); rolling updateOrigin with int truncation; footprint clearing = setConvexPolygonCost(FREE) on the padded footprint (padFootprint sign rule); persistence 0 = one obs per source per update. RPP isCollisionImminent: projection_time res/|v| (or 2 sin(res/2/r_max)/|w| when rotating), < 1.5 s, carrot 0.5, footprint OUTLINE cells >= 254.
- The D2 replay.py was NOT reused: it samples rays every 0.5 % (not Bresenham), uses a circle footprint (old radius config) and works on LaserScan, not the two PointCloud2 sources.
- A and C are not re-implementations: the harness calls LevelBridge.on_imu/on_odom/on_cloud of the RUNNING file (A) and of the CANDIDATE (C) on recorded messages with a stub self, and feeds the actually published scan_obstacles/scan_clearing clouds to the emulated layer (sources exactly as live: mark-only obstacle_max 3.0/raytrace 3.5, clearing-only raytrace 3.5, heights 0/0, origin base_footprint z=0).
- Same for all variants: 4x4 m @0.05 rolling, 8-pt footprint + padding 0.01, ranges 3.0/3.5, bridge range_max 8.0, scan_every 2, pose = LIO pose at the cloud stamp.

## 4. Why -46..+44.5 and why marks reach -58.8° (C prerequisite)
History: D1 (27.10) measured "only 180-210 of 1440 bins finite, all within roughly -46..+44 deg" on the in-band LaserScan (walls at 2-8 m). D2 used it as a fixed mask for LaserScan **inf** bins (bins with NO point need a mask, otherwise the rear is "cleared"). I1 (commit 5b17bd9, 28.10) carried the same constant into clearing_endpoints, which only emits rays for actual points — there the mask has no protective role. No body self-hit, IMU offset or occlusion reason exists in code/docs: D2 measured 0 non-floor points within 0.6 m; LiDAR-IMU offset 0.0198 m (logs/x2_extrinsics.json).
Measured observed bearing span (all points / in-band points), body frame, per flat range:
| range | udp_20261028 all pts min/max (p0.5/p99.5) | replay_dense_01 all pts | in-band (udp) |
|---|---|---|---|
| 0.45-1.0 m | -54.45 / +54.46 (-51.8/+51.6) | -54.49 / +54.32 | none in scene |
| 1.0-2.0 m | -57.07 / +55.55 (-54.5/+52.6) | -57.73 / +55.75 | none |
| 2.0-3.5 m | -50.83 / +51.07 | -50.42 / +50.88 | -45.8 / -32.5 |
| 3.5-8.0 m | -47.86 / +47.25 (-45.6/+45.2) | -47.92 / +48.13 | -44.2 / +44.3 |
LiDAR attitude in the levelled frame: pitch 4.18-4.25° down, roll -90.4° (mount roll -90° + residual -0.37..-0.46°).
=> The bearing span is NOT constant: steep-depression beams (near ground / near range) project to wider bearings (±54-58°) than near-horizontal ones (±45-48°). -46..+44.5 is the far/horizontal span; near people at 0.45-0.9 m are seen out to -58.8/+52 (P2), and the residual roll gives ~1.5° extra on the right. A fixed window cannot match a range/elevation-dependent span; per-ray clearing from the actual points matches it by construction. Nothing in the data indicates a real reason to forbid clearing outside -46..+44.5.

## 5. Candidate C (P4) — minimal change
Only clearing_endpoints changed (diff: remove `fov_deg=(-46.0, 44.5)` parameter and its mask; `valid = r > 0`; docstring). AST scope test proves every other top-level node is identical; positional call used by ws/src/lio/raw_scan_bridge.py `clearing_endpoints(Q, 0.0, range_max)` still works. Rules a-d of the split are unchanged and tested: (a) rays only from actual points of the current cloud; (b) capped at nearest in-band return of the bin; (c) floor/upward rays end at band exit (Z_MIN+0.035 r / Z_MAX) and range_max; (d) bins with no point emit nothing. Inside the old window output is bit-identical to running (5000 random points x 3 yaws). Marking is untouched (no clipping of marking, no decay/timeout, no auto clear, collision detection untouched).

## 6. A/B/C/D/E results (measured, tests/d04/results)
Recorded-only (no injection, 4 bags): final lethal cells A=18/28/27/27; C identical cell sets in all 4 bags (0 cells only-in-A, 0 only-in-C) -> no real obstacle lost on recorded data. B +8..+35 cells and D +13..+42 cells vs A (all in-band points marked, not only the nearest per bin). No edge marks, no collision flags in any variant (scene has no near edge objects).

SYNTHETIC edge transients (box present 10 updates = 2 s, then removed; re-ranged recorded beams). Same pattern in every bag; numbers udp / dense_01 / portable_01 (controlled_01 = portable_01):
| scenario | metric | A baseline | B single cloud | C candidate | D voxel (approx) |
|---|---|---|---|---|---|
| S9 right -52° ~0.5 m (D-04 geometry) | lethal while present min | 6/7/6 | 6/7/6 | 6/6/6 | 6/-/- |
| | object cells left after leave (final) | 2/4/2 | 2/4/2 | 0/0/0 | 0 |
| | updates to clear | never | never | 0 (first update) | 0 |
| | RPP collision flagged after leave | 22/38/53 of 22/38/53 updates (all, to end of bag) | same as A | 0 | 0 |
| S10 left +48° ~0.55 m | cells left / coll after leave | 3/3/3; 22/38/53 (all) | 3; all | 0; 0 | 0; 0 |
| S3 right -55° 0.65 m | cells left / coll after leave | 2/2/2; all | 2; all | 0; 0 | 0; 0 |
| S2 left +49° 0.8 m | cells left / coll after leave | 4/4/4; 0 (outside RPP sweep) | 4; 0 | 0; 0 | 0 after 4/12 updates |
| S7 person +30°->+75° (into blind) | cells left after leave | 72/86/86 | 44 (udp) | 0/0/0 (0-2 updates) | 0 |
| | coll after leave | 22/38/53 (all) | 0 | 0 | 0 |
-> A reproduces D-04 exactly: an edge transient at 0.5-0.65 m leaves lethal cells that keep RPP "collision ahead" for the whole remaining bag (only BT ClearEntireCostmap would remove them = the observed ~10 s). C clears them on the first update after the object leaves. B does NOT fix it: open edge bearings return only floor points, which B's height filter drops, so there is no ray to clear with (no-return bearings are never cleared in B; S7 leaves 44 cells).

Retention / safety (SYNTHETIC, all bags):
| scenario | A | B | C | D |
|---|---|---|---|---|
| S1 ahead 0° 0.75 m: updates marked / RPP flagged while present | 10/10, 10/10 | same | same | same |
| S6 static chair +49° 0.8 m (32/48/63 updates) | marked every update, min 10-11 cells | min 20 | every update, min 10-11 cells | min 19 |
| S5 overhang 0°, 1.0-1.7 m above floor, 0.6 m | 32/48/63 marked, RPP flagged every update | same | same | same |
| S4 low box 0.25 m high, 0.8 m | NOT marked (0 updates) | 0 | 0 | 0 |
| S8 seeds at +65,+75,-68,-80,180,120° (blind, absence != free) | 6/6 kept | 6/6 | 6/6 | 6/6 |
S4: pre-existing perception gap (band floor 0.34 m + 0.035 r, D2 §1), identical in all variants -> not a regression, not a fix.

Cost (recorded_only, X2 under live load, nice 19; bridge time = full on_cloud incl. message build):
| variant | bridge ms p50/p95/p99 | layer update (Python emul, not C++) | wire bytes/update | clearing rays |
|---|---|---|---|---|
| A | 4.99/5.32/7.46 (udp 4.99/5.43/6.16) | 4.4 ms | 6.8 kB | 362 |
| B | 0.75/0.84/0.91 (+ cloud publish not measured) | 5.2 ms | 93.7 kB (all in-range pts, AgiBot style) | 826 (all marking pts) |
| C | 5.24/5.49/5.58 (udp 5.29/5.69/6.81) | 4.5 ms | 7.8 kB (+15 %) | 444 |
| D | (3D clouds) | 147-166 ms Python sampled 3D rays | 103 kB (band + all pts, 2 clouds) | ~8.4 k 3D rays |
C vs A: +0.25-0.3 ms per processed cloud (+5 %) at 5 Hz = +0.15 % of one core; bandwidth +1.0 kB x 5 Hz x 2 costmap subscribers = +10 kB/s. Memory: identical costmap (80x80 uint8 local); bridge arrays scale with cloud (~8.4 k pts), unchanged order. Latency p95/p99 of the bridge in the live graph: NOT measured (offline only).

D (VoxelLayer) feasibility: installed (liblayers.so 1.1.18), feasible but not cheap: needs 3D clouds (103 kB/update ≈ 1 MB/s to two costmaps vs 78 kB/s for C), a sensor origin with real z (TF base_footprint->level LiDAR origin z≈0.97 m) instead of z=0, explicit per-source min/max_obstacle_height (defaults 0/0 drop everything), a fixed height band (the range-dependent floor margin must stay in the bridge as a separate marking cloud), origin_z/z_resolution/z_voxels(<=16)/mark_threshold/unknown_threshold tuning, and a Python bridge serialising 8.4 k-point clouds. Grid RAM small (local 80x80x uint32 = 25.6 kB). Clearing work ≈ 8.4 k rays x <=70 voxels/update (C++ estimate a few ms, NOT measured). The approximate replay (sampled rays, labelled APPROX) shows D also clears edge transients and keeps all objects, with the physically better per-voxel clearing. Excluded for D-04 because C removes the root cause with no new params/TF and ~1/13 of the bandwidth; revisit with the C++ bridge if over-the-top clearing becomes a measured problem.
E (STVL): not installed (P1); excluded without install (install forbidden). The 20261030-p25 STVL plan used decay (voxel_decay 8 s) = obstacle timeout, a FORBIDDEN fix class for D-04, and a frustum model with a fixed hFOV ±44° which reproduces the same edge mismatch.

## 7. Chosen variant: C
Evidence: only C (and approx D) removes edge marks on new evidence (0 residual cells, 0 RPP flags after leave in S2/S3/S7/S9/S10, all bags) while keeping every present object (S1/S5/S6, same counts as A), never clearing unobserved bearings (S8 6/6), and producing identical lethal sets to A on all recorded data. Smallest change (one function, 3 lines of logic), no param/launch/TF changes, cost +5 % bridge time.

## 8. Regression scenarios (TASK.md)
| scenario | status | evidence |
|---|---|---|
| obstacle ahead | PASS (SYNTHETIC on recorded beams) | S1, all bags, A=C |
| obstacle at +49° | PASS (SYNTHETIC) | S2 (transient), S6 (static) |
| side chair | PASS (SYNTHETIC) | S6 |
| person leaves (cleared on new evidence) | PASS for C / FAIL for A,B (SYNTHETIC) | S2,S3,S7,S9,S10 |
| person into blind zone (absence != free) | PASS (SYNTHETIC) | S8 seeds 6/6 kept; S7: blind position never observed in any variant |
| floor reflection | NOT_COVERED | no recorded reflection; band rule unit-tested only |
| glass | NOT_COVERED | no recording |
| low object | NOT DETECTED in any variant (pre-existing, unchanged) | S4 |
| tall object within envelope | PASS (SYNTHETIC) | S5 overhang 1.0-1.7 m |
| LiDAR loss (T1) | NOT_COVERED | no cloud -> no layer update in any variant (emulator semantics); T1 path not replayed |
| TF jump | NOT_COVERED | |
| AMCL correction | NOT_COVERED | local costmap in odom; not replayed |
| repeated goal | NOT_COVERED | BT not replayed |
| recorded static scene | PASS (RECORDED) | A and C lethal sets identical, 4 bags |

## 9. Tests
`nice -n 19 python3 -m pytest -q -p no:cacheprovider /home/agi/control_backend/tests/d04` -> **54 passed** in 8.1 s:
test_costmap_emul.py 12 (header parity x4, clear-then-mark, end-cell clearing, unobserved untouched, 0/0 height filter, 3D marking range, raytrace max, footprint + rolling, D-04 RPP geometry); test_clearing_geometry.py 36 (edge bearings emit rays x7, running drops them x4, bit-identical inside old window, yaw removal, no ray without points, NaN/inf/r=0, rear bins, floor band exit x12, < RANGE_MIN dropped, Z_MAX, range_max, hit cap, random cap/planarity, node on_cloud span, AST scope guard, positional signature); test_replay_d04.py 6 (real udp bag: edge transients S9/S10/S2/S3 kept by A cleared by C on first update; blind seeds kept; recorded A==C).
Note: the clearing endpoint can equal the hit range to 1 ulp after planar rescaling (same in the running file); harmless because marking runs after clearing in the same update; tests assert with float-precision bounds.

## 10. Risks / limits
- Emulator .cpp semantics recalled (header parts transcribed); recorded scenes are static and empty near the robot -> fix evidence is SYNTHETIC (re-ranged real beams, no invented beams) on 2 independent scenes.
- C clears edge cells using steep floor rays (passes through the band near the robot). Same mechanism as inside the window today; an object missed by one cloud (sampling gap) can be cleared for one update — unchanged class of risk (D2 synth: 47/48 at extreme sparsity), clear-before-mark keeps every current hit.
- With A, stale edge marks "accidentally" protected a person who walked from the edge into the blind sector; with C those cells clear once beams pass them. Neither variant ever sees a person who is only in the blind sector (S8/S7). Not a regression of observed safety, but the operator should know.
- Origin = LiDAR-IMU (2 cm from the LiDAR), native base_footprint offset unmodelled (same for all variants).
- Bridge latency p95/p99 and live CPU not measured (no live run allowed).
- BT ClearEntireCostmap remains as is; C removes the reason it fires for observed-edge marks.

## 11. NOT covered -> minimal static hardware recording (needs operator approval, robot standing, no goals, no motion)
Record (bag, ~5 min, live stack unchanged): /qazbot_lio/cloud_registered, /qazbot_lio/odometry, /aima/hal/sensor/lidar_chest_front/imu (start >=2 s before bridge levelling), /qazbot_d1_map/tf, /qazbot_d1_map/tf_static, /qazbot_lio/scan, scan_obstacles, scan_clearing, local costmap costmap_raw (+updates) with header stamps, controller log.
Protocol: 60 s empty; person stops 3 s at right-front 0.5 m (-50..-58°) then walks away to rear-right (blind); same at left +48° 0.6 m; chair at +49° 0.8 m 20 s then removed; 0.25 m box at 0.8 m ahead 10 s; glass/specular panel at 1-2 m if available; optional floor-reflective sheet. Then replay offline: `replay_d04.py` with the new bag added to BAGS -> PASS/FAIL for person-leaves, blind, glass, floor reflection on RECORDED data.

## 12. Candidate record
CANDIDATE: candidates/d04/level_bridge.py (bounded clearing over the actually observed span)
SOURCE_SHA: c880d3c8f66eed3c58eecb1db23024a37fd6d877704d6d779cac61906f75abef
BASELINE_SHA: c817a5ed503386be742fa7b6fdab2b3209bdefaf42be799d2a130cd198f9369b (ws/src/lio/level_bridge.py, live)
ROOT_CAUSE: fixed clearing window -46..+44.5° narrower than the range-dependent observed/marking span (±54-58° near range) -> mark-only edge sectors, marks never cleared, BT full clear after failure_tolerance 10 s.
CHANGED_FILES: none in the running path; new files listed in §0.
TEST_EVIDENCE: 54 passed; replay summary.json/table.md.
SAFETY_IMPACT: marking/collision/footprint/inflation/T1/T5 unchanged; clearing only along real rays; unobserved bearings never cleared (tested).
PERFORMANCE_IMPACT: +0.25-0.3 ms per processed cloud (+5 %), +1 kB/update.
ROLLBACK: restore baseline file (sha c817a5ed...) and restart level_bridge only.
STATUS: OFFLINE_VERIFIED. HARDWARE DEPLOY: NOT APPROVED.
C++ MIGRATION SCOPE: candidates/d04/CPP_MIGRATION_SCOPE.md (pure lib level_core + composable rclcpp node, config yaml, XML launch, gtest port of tests/d04, parity tolerances from float32 numerics).
