# 20261027-rpp-local-collision — D2 report (offline, read-only)

Status: COMPLETE (see end)

## Inputs
- Prior: reports/20261027-rpp-local-collision-debugger.md (D1), checker reports/20261027-rpp-local-collision-rpp_offline.py
- Bags (docs/roadmap_v13/evidence/): RViz_goal2_1216 (1793103352.3-3565.4), RViz_goal3_1220 (3749.2-3855.9),
  Static_map_trial_1230 (4200.8-4393.1), Static_map_retry_1257 (4895.9-5011.3). /tf_static count 0 in all four.
  -> Recording gaps 3565-3749 and 3856-4200: a continuous replay from the 3310 clear to trial03 (4301.6) is NOT possible.
- Snapshots: Static_map_trial_1230/local_costmap_before_clear.npy (stamp 1793104349.006, origin (-5.0,-2.0), 80x80@0.05, 121x254),
  Static_map_trial_1230/post_seed_costmap.npy (stamp 1793104881.97, origin (-4.0,-3.7), 42x254) — this one is for the retry_1257 session.
- level_bridge.py (ws/src/lio): slice Z in (-0.65 + 0.035*r_flat, +0.80) m relative to the chest-LiDAR IMU, RANGE_MIN 0.45,
  bins with no point = +inf (line 193), frame base_footprint, pose = LIO pose at the cloud stamp (NOT the native_imu odom TF).
- x2.yaml: chest LiDAR 96 rings, ~103 deg horizontal wedge, FAST-LIO blind 0.5 m.

## 1. Blind zone of the floor-level scan slice (measured)
Data: logs/raw_comparison_input_01 (raw /aima/hal/sensor/lidar_chest_front/lidar_pointcloud, 40 clouds, robot standing, 1.01 M points)
+ lidar IMU gravity (|a| 9.806, std 3 mm/s^2) rotated by /tf_static R_imu_lidar (logs/x2_extrinsics.json). Script: scratchpad d2/blind.py.
- LiDAR x axis pitched 4.4 deg down. Horizontal azimuth span ±51.5 deg. 96 rings.
- Elevation coverage (level frame) from -62.7 deg (straight ahead; -55 deg at az ±50) up to +54 deg.
  Ahead (|az|<20 deg) there are 400-800 points per cloud in every 8-deg elevation band from -64 to +16 deg (11-15 rings per band, about 0.6 deg ring spacing).
- Sensor height above floor H = 0.99 m (floor histogram peak; 431k floor points).
  The nearest floor return is 0.48-0.50 m ahead (0.57 m at ±40 deg, 0.70 m at ±50 deg).
- Height of the lowest beam above the floor at horizontal range r (front): 0.13 m at 0.45, 0.03 m at 0.50, below the floor from 0.55 m on.
- Slice lower bound (level_bridge Z_MIN -0.65 + 0.035*r relative to the IMU, IMU ~ LiDAR height) = 0.34 + 0.035*r above the floor:
  0.36 m at 0.5 m, 0.37 at 1 m, 0.39 at 1.5 m, 0.44 at 3 m. Upper bound 1.79 m.
- No self points: 0 non-floor points within 0.6 m per cloud.
CONCLUSIONS
- Vertical-FOV blind zone ahead: none beyond RANGE_MIN 0.45 m (|az| <= 40 deg); about 0.5-0.6 m at |az| 45-50 deg.
  The lowest beam reaches below the slice band (0.36 m) from r >= 0.33 m. So an obstacle whose height lies in the band stays in the slice all the way in to 0.45 m.
  0.45 m equals the robot radius (0.454), so it cannot "leave the slice" before it is inside the body polygon.
- Low obstacles below the band (top < 0.34 + 0.035*r, e.g. 0.1-0.3 m boxes or steps) are NEVER in the scan at any range.
  They are never marked today, so changing the inf handling neither creates nor removes protection for them. This is a pre-existing perception gap, independent of the fix.
  A 0.40 m object is in the slice only within r < 1.7 m. A 0.36 m object only within r < 0.6 m.
- Caveat (hypothesis, not measured on these bags): dark or specular surfaces that return no points would also give inf.
  Today they stay marked once seen; after the fix they would be cleared. Measured on one standing robot pose only, with no walking sway (sway ~1-2 deg moves the lowest-beam reach by ~2 cm).
- IMPORTANT sampling finding: the raw horizontal spacing ahead is ~0.75 deg per ring, and FAST-LIO point_filter_num=3 thins /cloud_registered further.
  The scan bin is 0.25 deg (1440 bins), so many in-FOV bins are inf only because of SAMPLING GAPS between beams, not because the space is open.
  A further cause of inf in the slice is level_bridge `r < RANGE_MAX` with launch range_max 8.0: walls beyond 8 m become inf (Goal01's neighbouring band had a median of 7.2-7.8 m).

## 2. Offline validation of the inf -> clearing fix
Tools (scratchpad d2/): replay.py emulates the Humble ObstacleLayer, using the D1 checker for RPP.
- Rolling 80x80 grid at 0.05 m, re-origined to the robot like Costmap2D::updateOrigin.
- Per scan: clear first (raytrace to min(range, 3.5)), then mark finite returns < 3.0 m, then footprint clearing (r 0.464). Pose comes from odom->base_footprint on /qazbot_d1_map/tf, interpolated at the scan stamp.
- inf_is_valid=true is modelled as in Nav2 laserScanValidInfCallback: inf -> range_max - 1e-4, a clearing-only ray.
  This is recalled from the Nav2 source, which is not on disk; the 3.0 m marking limit means these rays never mark.
- Approximation: ray cells are sampled every 0.5 % of the length (supercover-ish), not exact Bresenham.
Modes:
- baseline: today's behaviour, inf ignored.
- inf_all: every inf bin clears (naive inf_is_valid=true).
- inf_fov: inf clears only for bearings in -46..+44.5 deg, the observed FOV.
- inf_guardK: like inf_fov, but excluding inf bins within ±K bins of a finite return.

### Goal01
Start state: lethal cells of First_RViz_goal_C3_1205/local_costmap_before_clear.npy (246 cells; robot stationary from goal to snapshot).
Then the bag scans 1793103176.0-181.43 are applied, and RPP is checked at the goal pose (-1.884, -1.623, 2.061) with carrot 0.5.
| mode | lethal total / within 1.6 m | v=0.3 w=0,±0.05,±0.1 | rotate ±0.6 |
|---|---|---|---|
| snapshot as-is | 246 / 121 | COLLISION | - |
| baseline | 239 / 116 | COLLISION at projection steps 7-8 (reproduces the live abort) | clear |
| inf_fov | 156 / 83 | clear (all 5) | clear |
| inf_guard2 / inf_guard4 | 160 / 83 | clear (all 5) | clear |
| inf_all | 15 / 0 | clear | clear (UNSAFE: also wipes the unobserved rear/sides) |
=> The FOV-limited fix removes the Goal01 phantom within 5 s of scans (in practice the first update that covers the -14..-2 deg band). The baseline replay reproduces the bug.

### Synthetic real obstacle (safety)
A 0.4 m wide face perpendicular to heading, at d ahead, is injected into the same 50 scans (10 s). It is present in each of its bins with probability p per scan, to emulate sampling gaps.
The neighbouring bins keep the real data (inf in that band). Measured: the cells of the face that stay lethal, and RPP v=0.3 collision per update (48 updates).
Real sampling density measured on bags: near (<3 m) surfaces are finite in 0.89-0.99 of their bins. Over the whole FOV, 0.50-0.57 of bins are finite (gaps occur mostly at long range).
| d | p | baseline lethal min/mean, RPP | inf_fov | inf_guard2 |
|---|---|---|---|---|
| 0.6 | 0.5 | 10/10.9, 48/48 | 10/11.0, 48/48 | 10/11.0, 48/48 |
| 0.6 | 0.05 | 4/5.9, 48/48 | 2/5.0, 48/48 | 2/5.0, 48/48 |
| 0.8 | 0.5 | 11/11.9, 48/48 | 10/11.8, 48/48 | 11/11.9, 48/48 |
| 0.8 | 0.2 | 8/10.2, 48/48 | 8/10.0, 48/48 | 8/9.9, 48/48 |
| 0.8 | 0.05 | 4/6.1, 48/48 | 0/4.2, **47/48** | 2/4.4, 48/48 |
| 1.0 | 0.5 / 0.05 | 10/11.8 ; 3/6.2 | 10/11.5 ; 0/3.7 | 10/11.6 ; 1/3.8 |
| 1.5 | 0.5 / 0.05 | 8/9.7 ; 2/4.7 | 7/9.6 ; 0/2.4 | 7/9.6 ; 0/2.6 |
(d >= 1.0 m never triggers RPP in any mode, because the 1.5 s / carrot 0.5 projection reaches only ~0.87 m. At d = 2.5 m the face lies outside the 4x4 window for this heading.)
=> At realistic density (p >= 0.2) a real obstacle in the slice stays lethal and still stops RPP 100 % of the time with inf_fov.
   Only at extreme sparsity (p = 0.05, i.e. 1 bin of ~9 per scan) does inf_fov lose an update (47/48). Guarding inf bins next to finite returns (inf_guard2) restores 48/48.
   Mechanism: Nav2 clears before it marks in each update, so a surface hit in the current scan is always re-marked. Only cells missed by the current scan can be erased by an adjacent inf ray.

## 3. Trial03 (goal 1793104301.58; first collision 301.840, then 7 cycles to patience at 302.138)
Replay: Static_map_trial_1230, all scans 4204.0 -> 4349.0 from an empty layer, grid phase aligned to the snapshot (scripts t03*.py).
The history before 4204 is NOT recorded (gaps 3565-3749 and 3856-4200), so the replay is a lower bound of the live layer.
Validation vs the 4349 snapshot: replay has 79 lethal cells, snapshot 121, 76 common.
All 45 snapshot-only cells are more than 1.2 m from the robot (at 4301.8 and at 4349). The near field (<1.2 m) matches exactly. The extra snapshot cells are older far marks from before 4204.
Findings:
- Near returns (<1.2 m) in this bag occur only in scans at 4215.0-4216.2 and 4290.2-4293.0: bearings -15..+54 deg, 0.56-1.2 m.
  This is something in front of the robot right after trust came up, most likely the operator; that is a hypothesis.
  These marks survive only at the left FOV edge: the replay's nearest lethal cells are at 0.68-0.75 m, bearing +47..+55 deg.
  The same cells appear in the 4349 snapshot.
- In the replay state at 301.60-302.14, RPP (v=0.3, w=-0.129, carrot 0.5) is CLEAR in all 12 cycles in baseline mode and in inf_fov mode. The recorded scans alone therefore do not reproduce the live collision.
- Yaw at walk start is REAL body rotation, not an odom artifact.
  native_imu odom and the independent LIO /qazbot_lio/odom_level agree to within 0.4 deg: -1.1/-2.0 at 301.85, -8.2/-8.4 at 302.0, -11.6/-11.6 at 302.2, settling at -9.3 deg after STAND (304.5).
  The commanded w (-0.129 rad/s) explains only about 3 deg. This is an MC gait-start lurch to the right (STAND->LOCOMOTION at 301.825), consistent with the mc/common/state transitions.
  (/aima/mc/leg_odometry yaw is unusable: it jumps by tens of degrees.)
- Candidate-cell analysis: some cells enter the RPP sweep at 301.84 that were in none of the 5 earlier sweeps (301.60-301.79). Of these, the ones not inside the body and not provably free afterwards are cells (51,36), (51,37), (52,34) on the snapshot grid.
  These are right-front, bearing -50..-62 deg, 0.56-0.67 m, i.e. just OUTSIDE the right FOV edge (-46 deg). No clearing ray crossed them between 4204 and 301.84.
  After the lurch they entered the FOV and were cleared 117 times, which explains why they are absent from the 4349 snapshot.
  The other candidates are provably not lethal: never cleared afterwards, still in the window, yet 0 in the snapshot.
- Injection test: a single lethal cell at (51,36) added to the replayed layer reproduces the live pattern. Replay gives `.....C.CCCCC` vs live `.....CCCCCCC` over cycles 301.60..302.14.
  The 301.89 miss coincides with a TF yaw rebound sample (2.0087). The pattern is the same in inf_fov mode.
VERDICT trial03: (b)+(c). The proximate trigger is the real ~10 deg right lurch at locomotion start, which swings the RPP-projected footprint into a lethal mark ~0.6 m right-front that sits just outside the scan FOV and so is never cleared.
The root cause is the same family as Goal01: marks in unobserved bearings persist indefinitely.
The mark's origin is NOT provable: it predates 4204 and falls in the recording gap. Most likely it is a person or object near the robot at the FOV edge, like the 4290 event on the left side (hypothesis).
(c) is not an odom fault: no odom/IMU fix is indicated. MC gait-start yaw lurch is a separate locomotion observation; magnitude ~10 deg / 0.4 s.
IMPORTANT: the FOV-limited inf fix would NOT have prevented trial03, because the mark is outside the FOV.

## 4. Recommendation
### 4a. level_bridge.py (primary, fixes Goal01 class)
Make "in-FOV, no slice return" an explicit clearing ray, and leave unobserved bearings untouched:
```python
# after Q = P @ R_lv.T - p_lv  (BEFORE the slice filter), bearing coverage of the whole cloud:
flat_all = np.hypot(Q[:, 0], Q[:, 1])
c, s = math.cos(-yaw), math.sin(-yaw)
a_all = np.arctan2(s * Q[:, 0] + c * Q[:, 1], c * Q[:, 0] - s * Q[:, 1])
seen = np.zeros(N_BINS, bool)
seen[((a_all[flat_all > RANGE_MIN] + math.pi) / (2 * math.pi) * N_BINS).astype(int) % N_BINS] = True
... existing slice filter -> r, a, idx ...
ranges = np.full(N_BINS, np.nan, dtype=np.float32)      # was np.inf: unobserved bearing -> NaN (dropped, never clears)
ranges[seen] = np.inf                                    # observed, nothing in slice up to RANGE_MAX -> clearing ray
np.fmin.at(ranges, idx, r.astype(np.float32))            # fmin, NOT minimum (NaN-safe)
fin = np.isfinite(ranges)                                # guard: no inf ray within ±2 bins of a hit (inf_guard2)
ranges[np.isinf(ranges) & (np.convolve(fin, np.ones(5), 'same') > 0)] = np.nan
```
The 'seen' mask could alternatively be a fixed FOV mask of -46..+44.5 deg, which is what the replay validated.
The data-driven mask is preferable because it also handles self-occlusion, but its coverage on /cloud_registered is NOT yet verified (open item).
### 4b. local costmap param (nav_control params, obstacle_layer.scan)
`inf_is_valid: true` (default false). Keep raytrace_max_range 3.5, obstacle_max_range 3.0 and range_max 8.0 unchanged.
Nav2 maps inf to range_max - 1e-4 = 7.9999, so these rays clear only to 3.5 m and never mark. NaN stays dropped.
### 4c. BT recovery (needed for the trial03 class; the inf fix cannot clear out-of-FOV marks)
Wrap FollowPath in bt/navigate_to_pose.xml in a RecoveryNode(number_of_retries=1-2) with ClearCostmapAroundRobot(local, reset_distance ~1.0-1.5 m) or ClearEntireCostmap(local) + Wait 0.5 s.
The recovery re-marks real obstacles in the FOV from the next scan. Residual risk: a real obstacle at the side or rear (outside FOV) is forgotten after a clear. That is acceptable for this front-only sensor because the robot cannot see it anyway, and RPP only sweeps ~0.4 m forward.
Optional: clear the local costmap once at goal acceptance (the operator already does this manually).
### Do NOT
Do not use inf_is_valid=true with today's level_bridge, i.e. inf in all 1440 bins. That is the inf_all mode: it wipes 239 -> 15 lethal cells, including the unobserved rear and sides.
Do not shrink the radius. Do not raise failure_tolerance.
observation_persistence > 0 does not help: it keeps marks longer and does nothing for clearing. Humble's ObstacleLayer has no decay.
### Residual risk of 4a/4b
- Obstacles below the slice (top < 0.34 + 0.035 r m above the floor) remain invisible. This is unchanged and pre-existing.
- Surfaces that return no LiDAR points (black, glass, mirrors) in the slice, if previously marked, are now cleared within one update. Today they would stay as stale marks.
- With extreme sampling sparsity (1 of ~9 bins hit), an obstacle at 0.8 m can lose its marks for one update (47/48 in the synthetic test). The ±2-bin guard removes this (48/48).
- Measured near-surface sampling density is 0.89-0.99, so in practice no loss is expected.
- Vertical blind zone ahead: none beyond 0.45 m. A slice-height obstacle is visible down to RANGE_MIN 0.45 m (|az| <= 40 deg) and 0.5-0.7 m at |az| 45-50 deg. It cannot drop out of the slice before it is within the body radius.

## Open items
1. Verify the coverage of the data-driven 'seen' mask on a /cloud_registered bag (quick attempt on native_qualification27_cloud01 inconclusive; needs the level_bridge R_lv/pose).
   If it is unreliable, fall back to the fixed FOV mask -46..+44.5 deg.
2. Trial03 mark origin is unprovable (recording gap 3856-4200). Future bags should start before the stack/costmap starts, or periodically snapshot the costmap.
3. MC gait-start yaw lurch (~10 deg right in 0.4 s at STAND->LOCOMOTION) is a locomotion behaviour worth a separate look. It also causes AMCL/plan transients.
4. The replay uses sampled rays (not exact Bresenham) and recalled Nav2 semantics (clear-before-mark, inf -> range_max - eps); the Nav2 source is not on disk.
   Live A/B after implementation (I1/V1) is needed. Verify with the checker against costmap snapshots before and after.
5. Blind zone was measured on a standing robot (raw_comparison_input_01); walking sway of ~1-2 deg was not measured.

Scripts (scratchpad /tmp/claude-1001/-home-agi/85355f79-5e00-4825-8c75-b2bc85299e5c/scratchpad/d2/): bagio.py, blind.py, replay.py, g01.py, synth.py, t03.py, t03b-e.py.
Status: ROOT_CAUSE_FOUND (Goal01 proven; trial03 strongly supported, mark origin unprovable).
