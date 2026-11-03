# 20261027-roadmap-v12 — V2-amcl: stationary shadow assessment of X2 AMCL on D1 map v3 after operator 2D Pose Estimate

Agent: robotics-debugger (read-only). Started 17:17 CST 2026-10-27. Written incrementally.
Constraints: no /initialpose, no goals, no confirm_anchor/baseline, no param set, no restarts, no /aima/*, no ws/src edits.
request_nomotion_update budget: max 3 calls (count recorded below).

## 1. Log evidence — operator initial poses (localization_nav72_20261027.log, AMCL)
Operator sent SIX 2D Pose Estimates within 88 s (all `Setting pose`, each preceded by a harmless
"Failed to transform initial pose in time (extrapolation into the future ~0.1 s)" warning — stationary robot, no effect):
| wall | AMCL Setting pose x y yaw | trust log |
|---|---|---|
| 17:14:35 | -5.897 5.496 -1.371 | initialpose received: READY cleared |
| 17:14:56 | -6.383 6.065 -1.169 | READY cleared; nomotion attempt 1 |
| 17:15:04 | -6.981 6.647 -1.196 | READY cleared; nomotion attempt 1 |
| 17:15:40 | -5.716 4.045 -1.063 | READY cleared; nomotion attempt 1 |
| 17:15:56 | -6.776 5.529 -0.935 | READY cleared |
| 17:16:03 | **-6.364 5.090 -1.123** (final) | READY cleared; nomotion attempt 1 |
Spread of operator guesses: x -6.98..-5.72 (1.27 m), y 4.05..6.65 (2.60 m), yaw -1.37..-0.94 (25 deg) → operator was itself uncertain of the
true pose, mostly ALONG one axis (y), i.e. along a corridor direction.

**UPDATE 17:19 — task premise superseded during this assessment.** The operator continued seeding AFTER the pose named in the task:
| wall | AMCL Setting pose x y yaw |
|---|---|
| 17:17:50 | -2.154 0.344 0.051 |
| 17:18:00 | -1.113 0.107 0.139 |
| 17:18:06 | -1.344 -0.429 0.217 |
| 17:18:12 | -1.574 -0.379 -0.021 |
| 17:18:29 | **-1.183 -0.102 0.057** (latest at 17:19) |
This is ~6.6 m and ~67 deg away from -6.364 5.090 -1.123 → the operator (RViz) has switched the hypothesis to a different place on
the map (or the robot was relocated — checked below via LIO odom continuity). trust_nav72 log: 'initialpose received: READY cleared'
for each; NO 'requested AMCL update without motion' line for these five (vs. attempt 1 for 4 of the first 6) — see §4.

## 2. Observation run1 (passive, 60 s, 17:19:19–17:20:19, nice 10; tool: scratchpad amcl_shadow_obs.py, subscriptions only)
Raw data: /tmp/claude-1001/-home-agi/7c4b68ea-a3ae-4491-bba6-a5f62ecb9b4c/scratchpad/v2amcl/run1.jsonl, run1_scans.npz (297 scans, 592 odom TF).
request_nomotion_update calls by this agent: **0 / 3** (not used — turn limit; covariance-convergence trend therefore NEEDS_DATA).
| item | result |
|---|---|
| amcl_pose (latched, stamp 17:18:29 = last seed) | x -1.235, y -0.126, yaw 0.049; **σx 0.403 m, σy 0.481 m, σyaw 0.264 rad (15°)**; 0 new amcl_pose in 60 s (stationary → AMCL silent by design, update_min_d 0.25 / min_a 0.2) |
| covariance trend | flat (single post-seed update); no trend measurable without nomotion → NEEDS_DATA |
| map→odom (/qazbot_d1_map/tf) | 299 samples, 5.04 Hz, ONE unique value (-6.80057, 5.72002, 0.61199); max step 0.000 m / 0.000 rad; stamp age −0.97 s (transform_tolerance, normal) → **stable, 0 jumps** |
| LIO odom→base_footprint (60 s, stationary) | (1.20, -7.98, -0.57) in odom; peak-to-peak x 0.042 m, y 0.069 m, yaw 0.021 rad → continuous, small stationary noise |
| particle_cloud | none received (no AMCL update during window) |
| trust (298 ticks) | reasons [] on all ticks; **trusted=True, ready=True, motion_ok=True**, initialized, moving=False, map_ok ''; diagnostics on all ticks: T2_covariance, T3_particles, T4_no_baseline, T6_no_anchor |
| trust metrics | scan_score min/p5/p50/p95/max 0.363/0.372/0.406/0.426/0.436 (hit_min_abs 0.35); sigma_xy 0.485, sigma_yaw 0.264, cloud_std 0.276, **second_mode_weight 0.903**; jump 0/0 |
| input ages | scan 0.19, odom 0.09, map 0.15 s (healthy) |

## 3. Scan-to-map agreement (offline, scan_fit.py / near.py, loc_trust.OccGrid on map v3, 30 scans, beams 0.5–8 m, 367 valid beams, bearing −46.3°..+45.5°)
Sectors = thirds of observed span (right <−15°, center, left >+15°). hit10 = fraction of endpoints within 0.10 m of an occupied cell.
| pose | soft score (σ 0.05) | hit10 | hit15 | >0.5 m from any wall | median dist | L / C / R hit10 (median dist) |
|---|---|---|---|---|---|---|
| **B = current AMCL (-1.235,-0.126,0.049)** | 0.388 (p5/p95 0.374/0.398) | **0.622** | 0.698 | **1.1 %** | **0.071 m** | 0.62 (0.071) / 0.80 (0.050) / **0.44 (0.169)** |
| A = task seed (-6.364,5.090,-1.123) | 0.011 | 0.020 | 0.033 | 65.3 % | 0.64 m | 0.00 / 0.06 / 0.00 |
| best within ±2.5 m/±20° of B: (-0.99,-1.13,-0.039) | **0.629** | 0.762 | — | 0.0 % | 0.050 | 0.65 / 0.86 / 0.77 |
| other near-B fits | (-3.44,2.37,0.38) 0.563; (-1.44,2.17,-0.27) 0.540; (-1.44,0.22,0.10) 0.525 | 0.71–0.78 | | | | |
| global search (whole map, 0.2 m/5°, refined) | ≥8 distinct poses 0.58–0.68, e.g. (-9.82,-0.27,-0.30) 0.678, (-6.28,4.53,3.02) 0.625, (-5.48,-1.32,0.00) 0.618, (-8.43,6.58,1.36) 0.613 | 0.77–0.80 | | | | |
Interpretation:
- Pose A (named in the task) is **wrong for the current scan** (65 % of beams >0.5 m from any wall). Whether A was a bad guess or the robot
  was relocated before 17:17:50 is UNVERIFIED (no LIO pose history before 17:19 was recorded) — NEEDS_DATA (ask operator).
- At pose B the scan IS near the walls (median 7 cm, 99 % within 0.5 m) — consistent with the operator's visual RViz confirmation
  (coordinator message, 17:2x: "operator visually confirmed in RViz that the scan lies on the map walls"; recorded as operator evidence).
- BUT B is **not at the scan-fit optimum**: a pose 1.03 m away (−0.99,−1.13, −5°) fits clearly better (0.63 vs 0.39; right sector 0.77
  vs 0.44). The right sector at B is systematically off (median 0.17 m) → B is likely laterally offset ~0.5–1 m and/or a few degrees
  in yaw, or the right sector sees a non-mapped object (occlusion) — undecided.
- Single-scan matching is **globally non-discriminative**: with the ±46° / 8 m projected scan, ≥8 unrelated map poses score higher
  than B. Scan score alone cannot certify the pose (same conclusion as roadmap P4.3 / quickstart "вдоль одной прямой стены положение не
  определено"). Only operator ground truth (known floor mark / tape distances) can disambiguate.
- Projection mismatch: no evidence of a systematic height/projection mismatch (near-optimal poses reach hit10 0.76–0.80 in all three
  sectors, far50 0 %), so the 3D→2D slice is compatible with map v3; the deficit at B is pose error, not projection.

## 4. navigate_to_pose (-6.09, 6.41) provenance (nav_full_nav72_20261027.log l.5952–5958)
- 17:14:42.97 `goal_coordinator: Goal Pose accepted` → this path is goal_coordinator.pose_goal(), i.e. a PoseStamped on
  **/qazbot_d1_map/goal_pose**, the topic of the RViz `SetGoal` (2D Goal Pose) tool in ws/src/navigation/rviz/localization.rviz.
  Forwarded to nav2 bt_navigator: "Begin navigating from current location (-5.85, 5.53) to (-6.09, 6.41)"; planner SmacPlanner2D:
  "goal is occupied with no tolerance" → abort → "Goal failed". Adapter dry_run → no motion.
- Timing: 7 s after the operator's first 2D Pose Estimate (17:14:35) → most likely the operator's RViz 2D Goal Pose tool (possibly
  mis-selected instead of 2D Pose Estimate). Publisher identity is NOT in the logs (HYPOTHESIS; confirm with operator).
- Safety-relevant: the goal was ACCEPTED by goal_coordinator + trust arm_check because under admission_policy=go2 READY = initialpose +
  any post-seed AMCL output; T2/T3/T4 are diagnostics only. Currently trust reports motion_ok=True with σxy 0.49 m, second-mode weight
  0.90 and scan_score 0.41 at a pose that is not the scan-fit optimum. With a live adapter this would have allowed motion on an
  unverified pose (only the occupied goal stopped it).
- Trust issued nomotion refresh ("attempt 1") for 4 of the first 6 seeds and for none of the last 5 (17:17:50–17:18:29); cause not
  determined (candidates in _update_seed_without_motion: seed_stamp from RViz PC clock vs robot scan stamps, 6 s expiry, _moving()).

## 5. Verdict (roadmap §0.5 initial-localization acceptance, §P4.3, §P9a)
- Pose named in task (-6.364, 5.090, -1.123): **WRONG** (for the current scan; superseded by operator at 17:17:50+).
- Current pose B (-1.235, -0.126, 0.049): **AMBIGUOUS** — stationary-stable (map→odom 0 jumps, odom continuous, inputs fresh), scan
  roughly on walls (median 7 cm, 62 % beams ≤10 cm; operator visual OK), but σ 0.40–0.48 m / 15°, particle second-mode weight 0.90,
  better-fitting pose 1.03 m away, right sector misaligned, global aliasing. Not acceptable as "plausible & stable" until operator
  ground truth confirms it.
- confirm_anchor: **NOT justified** (and under go2 it is a no-op: returns `initialized`, does not set the T6 anchor). Do not call.
## 6. What the operator must check visually / physically
1. Where the robot physically stands vs B (-1.24,-0.13) and vs (-0.99,-1.13): measure tape distance to the nearest wall on the right
   and ahead; compare with the map at both poses.
2. RViz zoom on the RIGHT part of the scan (−46°..−15°): does it lie on the wall or ~0.15–0.2 m off / on a non-mapped object (door,
   furniture, person)?
3. Was the robot moved between 17:16 and 17:18 (explains A→B switch) or was A a wrong guess?
4. Never use the 2D Goal Pose tool while seeding (the 17:14:42 goal).
## 7. NEEDS_DATA / next commands (not run)
- Up to 3 × `ros2 service call /qazbot_d1_map/request_nomotion_update std_srvs/srv/Empty {}` (spaced ≥10 s) with run of
  amcl_shadow_obs.py (--nomotion 20,40,60) → covariance/particle convergence and whether AMCL drifts toward (-0.99,-1.13).
- LIO odom history 17:14–17:18 (bag or P2T-b data with poses) to decide "A wrong" vs "robot moved".
- Proposal (design, not applied): in go2, gate READY/arm on T2/T3 (e.g. second_mode_weight) or require operator floor-mark confirmation.
