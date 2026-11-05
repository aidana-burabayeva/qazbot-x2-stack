# NAV-OBS-01: safe detour preparation — global inflation 0.60 -> 0.80 (one parameter), offline only

Decision (operator): variant B. Baseline `48697e7` kept. Nothing installed on the robot, no Goal sent. Branch `fix/global-inflation-080` (one line in `nav_full.launch.py`).

## 1. Box geometry (operator): 23.5 cm wide x 12 cm deep x 40 cm high, the wide face towards the robot
Matches the marked face (0.20 m wide, 5-6 cells) at along 2.04 m, lateral -0.09 m. Modelled as 0.235 x 0.12 m, along [2.02, 2.16], lateral [-0.2075, +0.0275] (start frame).

## 2. Why Smac keeps only 0.22-0.25 m of footprint clearance
Cost model verified on the live costmap: `cost = 252*exp(-1.5*(d-0.215))` for 0.215 < d <= inflation_radius, 0 beyond (inscribed 0.17 + padding 0.045); it reproduces the recorded values (mean abs diff 0.65 cost units;
141 on the recorded path at d = 0.60 m). With `inflation_radius = 0.60` the cost is exactly zero beyond 0.60 m from a lethal cell, so the planner (cost_travel_multiplier 4) has no reason to go farther than that
boundary and takes the shortest zero-cost path: centre clearance 0.60 m, i.e. 0.60 - 0.372 (right side of the polygon, box side) - padding ~ 0.22 m, although the free space is >= 3 m each side.

## 3. Isolated environment (no robot, domain 90, loopback)
Real `planner_server` (Smac2D, qualified overlay) + global costmap with the production parameters, map = occupied cells of the saved costmap (walls + box). Validation: R = 0.60 reproduces the recorded
production plan (81 poses; mean lateral difference 0.009 m, max 0.044 m). One parameter changed at a time (dynamic `inflation_layer.inflation_radius`):
| R [m] | length | max lateral | swept-footprint clearance to the real box | nearest other lethal | max heading change / 0.5 m | simulated contact* (drift 0.2 / 0.3) |
|---|---|---|---|---|---|---|
| 0.60 (now) | 4.33 | +0.65 | 0.22 | 0.69 | 0.65 rad | 0/300 / 14/300 |
| 0.70 | 4.36 | +0.71 | 0.29 | 0.67 | 0.65 | 0/300 / 11/300 |
| **0.80** | 4.45 | +0.86 | **0.42** | 0.67 | 0.64 | 0/300 / 3/300 |
| 0.90 | 4.47 | +0.89 | 0.47 | 0.67 | 0.61 | 0/300 / 3/300 |
| 1.00 | 4.64 | +1.06 | 0.53 | 0.66 | 0.65 | 0/300 / 3/300 |
*Indicative closed-loop simulation (`trk2.py`): pure pursuit, D-05 plant (cmd 0.30 -> 0.50 m/s, 0.4 s dead time, realised curvature ~0.6), heading drift calibrated so that the straight-line cross-track matches the robot
(drift 0.2: mean max 0.15 / p90 0.24 / max 0.33 m; measured 0.10-0.36 m), drift 0.3 is a pessimistic case (mean max 0.32 m). RPP's own collision stop is not modelled. Not a measurement.

## 4. Choice: 0.80 m
Geometry: required footprint clearance >= the largest measured tracking excursion 0.36 m + one costmap cell (0.05 m) = 0.41 m; 0.80 gives 0.42 m (0.70 gives 0.29 m, below the measured excursion).
Checks: plan exists, +0.12 m longer; nearest non-box obstacle stays 0.67 m from the footprint (no closer to walls); heading change per 0.5 m unchanged (0.64 vs 0.65 rad, no sharper turn);
the empty-scene 4 m route is unchanged (straight, 4.02 m, lateral 0.07 in both); inscribed radius and obstacle marking unchanged, so reachability is not reduced (only costs rise within 0.8 m of obstacles).
Not changed: Motion Adapter, RPP, smoother, AMCL, Trust, MC, goal checker, projector, local inflation (0.50), cost scaling 1.5, travel multiplier 4.0, tolerance 0.3.
Offline test: `tests/controllers/test_global_inflation.py` 2 PASS (robot env) + the goal-checker profile tests 6 PASS: global radius 0.80, scaling 1.5, multiplier 4.0, tolerance 0.3, local 0.50, checker 0.30/0.35/stateful.

## 5. Rollback and deployment (not done)
Deploy = copy `nav_full.launch.py` from this branch to the robot and a controlled Nav2 restart (parameters are read at start; the same restart procedure as for the goal-checker profile).
Rollback = restore `nav_full.launch.py` from `fix/goal-checker-profile` (52cd81e) and restart. No backup files on the robot.

## 6. Proposed hardware run (needs a separate permission)
One Goal 4.0 m ahead (-8.833,-2.238) with the box placed at along 2.0 m (face centre (-8.62,-4.19)), operator with RC and checked stop; record Nav2, Smac path, local/global costmap, RPP, smoother, GC, adapter, MC, FAST-LIO, TF;
stop immediately on loss of reliable localisation, dangerous approach or unexpected motion. PASS = safe detour + physical stop at the goal (0.30 m / 0.35 rad checked separately from SUCCEEDED).

## 7. Deployment and live verification (2026-11-05, operator on site, robot standing, no Goal)
Deployed `63749f2` (robot `nav_full.launch.py` sha256 1fc9583d... = Git; diff vs the previous robot file: the single inflation line; no `.bak`). `nav_full` (Nav2 + GC) restarted via `detach.py`
(log `logs/nav_full_20261105c.log`); localisation, Trust, adapter, MC, projector untouched. REAL parameters read once after the restart: global `inflation_radius` **0.8**, scaling 1.5, local
inflation 0.5, Smac multiplier 4.0 / tolerance 0.3, goal checker 0.3 / 0.35 / stateful true. GC ready and not latched, Trust READY.

Live Smac plan on the real costmap (box in place, start (-8.385,-6.168,1.640), Goal point 4.0 m ahead (-8.660,-2.178)): 81 poses, **4.46 m**, max lateral detour 0.84 m (to the left),
max path cost 104, no lethal/inscribed cell on the path (`boxcheck_live080.json`).
- Marking: GLOBAL costmap 7 lethal cells for the box (along 2.08-2.18 m, lateral -0.20..+0.01 m: the wide face, as expected). LOCAL costmap: 0 cells — expected, the rolling window is 4 x 4 m
  (+-2.0 m) and the box is 2.1 m ahead; it enters the window at <= 2.0 m (before the restart it was marked at 2.03 m). Local obstacle layer after restart not yet verified with the box inside the window.
- Footprint: swept X2 polygon along the whole path vs the physical box (23.5 x 12 cm placed at the marked face): **minimum clearance 0.43 m** (at along 2.49 m, lateral +0.77), required >= 0.41 m
  (largest measured tracking excursion 0.36 m + one cell). Vertex metric against all lethal cells: 0.46 m; path centre to the box 0.80 m.
- New obstacles on the path: none; nearest other lethal cell is 1.28 m from the path centre.
- Note: the robot pose in the map moved ~0.2 m since the earlier check (-8.574,-6.230 -> -8.385,-6.168) while the box is marked at the same place relative to the robot (2.10 m ahead, lateral -0.10 m); consistent with
  the open FAST-LIO standstill-drift issue, not with a moved box. No Goal sent.
Decision pending: one physical Goal 4 m with the box in place (separate permission).
