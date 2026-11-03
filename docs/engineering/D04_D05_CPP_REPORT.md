# D-04 / D-05 / D-06 and C++ migration: engineering report (2026-11-02, offline)

Standard: ~/.claude/standards/henki_ros2_best_practices.md. Nothing deployed on the robot.

## D-04 stale costmap marks (RPP "collision ahead" on a clear path)
Root cause: marking spans about +-55 deg, clearing rays only -46..+44.5 deg; edge marks of a transient object never clear.

| Variant | Idea | Edge transient cleared | Present objects kept | Cost / remark |
|---|---|---|---|---|
| A | running baseline | no (residual 2-8 cells, RPP flagged to end of bag) | yes | baseline |
| B | single cloud, height filter | no (open bearings return only floor, dropped) | yes | +8..35 cells, 94 kB/update |
| C | clearing without FOV window (CHOSEN) | yes, first update (observed bearings) | yes (cell flicker at edge) | +1 kB/update, no measurable CPU |
| D | VoxelLayer | yes (approx replay) | yes | 3D clouds ~100 kB/update, z origin, tuning; not chosen |
| E | STVL | not installed; uses decay = forbidden fix class | - | excluded |

Measured (real bag 204153, V1): A RPP flag 87.6 % vs C 1.3 %; A residual up to 8 cells vs C 0.
V1 verdict: PASS offline (candidate correct, safe on real data); hardware deploy not approved.
Harness note (F1): replay now fits the live R_lv from recorded scan_obstacles by default (`--level auto|fit|bag|file.json`); earlier 4 bags have no scan_obstacles and are not live-matched.

Not covered / residuals
| Item | Status |
|---|---|
| Person leaving into blind zone, chair, box +49 deg, glass, floor reflection | SYNTHETIC or NOT_COVERED (bag 211835 had 0 returns < 1 m) |
| LiDAR loss, TF jump, AMCL correction, repeated goal, RPP arc | NOT_COVERED / emulated |
| Edge-cell flicker in C (70.6 % vs 96.4 % persistence) | accepted LOW; 0 on RPP outline, object never lost |
| D-04b: marks in never-observed bearings (+70.9 deg/0.37 m in 204153) block RPP in A and C | open, needs separate fix (investigation 20261103-d04b) |
| D-07: low objects below the band floor (0.34 m + 0.035 r) not detected in any variant | open, pre-existing perception gap |

## D-05 / D-06 speed characteristic (D1 debugger, bags only)
| Finding | Evidence |
|---|---|
| D-05: MC forward speed is nonlinear, not unity gain | cmd 0.30 -> 0.48 m/s (R 1.61 +- 0.04, 16 segments, 3 days); cmd 0.80 -> 0.74-0.79 (R 0.93-0.99); reverse -0.78 -> 0.42 |
| Command path intact | MC input integral 1.898 m = authorized 1.899 m; LIVE-2 tape 3.105 m vs FAST-LIO 3.109 m |
| D-06: leg odometry under-reads | leg/LIO 0.885 +- 0.035 at 0.30; 0.93-1.00 at 0.80; worse in turns; not a single scale |
| Stopping | 0.10 m (ramped) to 0.20-0.30 m (abrupt) |
| Data gap | no steady data between 0.30-0.80 and below 0.28 |

Calibration proposal (operator approval needed, not executed): straight corridor >= 8 m, constant-cmd walks 0.10, 0.15, 0.20, 0.25, 0.30 (anchor), 0.40, 0.50, 0.60 m/s, >= 6 s steady, 2 repeats (16 runs, ~25 min), FAST-LIO primary + tape on 3 runs, log leg odom; optional wz 0.2/0.4 and stop type. Option A (inverse map in adapter) only if >= 3 monotonic bracketing points with spread <= 5 %; if a speed floor exists, set Nav2 limits and collision lookahead to real speeds. No PID default.
Status D-05: ROOT_CAUSE_FOUND, NEEDS_DATA; D-06: SUPPORTED, fix direction open (no single scale on native odom).

## C++ scope
candidates/d04/CPP_MIGRATION_SCOPE.md: pure library (levelling, band, obstacle points, clearing endpoints) + composable node, intra-process, config YAML, XML launch, SingleThreadedExecutor, gtest >= 90 %, parity test against tests/d04 and candidate C. Status: in progress (scope only, no C++ in this cycle; new package ws/src/qazbot_level_bridge is being worked on by another agent).

## 2026-11-03 reuse-first + corrections
Details: [reuse_first_20261103/README.md](reuse_first_20261103/README.md).
- D-04 person results unchanged (person F PASS A=C; person leaves C PASS / A FAIL).
- P-D04 box claims WITHDRAWN: bag d04_static_bag_183015 contained no boxes (operator statement). Box checks are NOT PERFORMED.
- D-07 (low objects) OPEN. Root cause: code height band (0.329 + 0.035*r m) plus FAST-LIO point_filter_num 3. LiDAR-only is partially solvable (theory). Candidate: floor-relative 0.07 m threshold with >=3 points per 5 cm cell (analysis only, not implemented, not verified on real boxes).
- C++ projector: 3 dropped scans = exact-stamp odom lookup before odom arrival (fix direction only, not implemented).
- D-05: AIMDK 0.2 m/s minimum is demo-only (not an MC limit); the smoother is kept; Ruckig rejected; GoalCoordinator cancel = abrupt step stop; acceptance criteria pending (C1/C2 recomputation, T_nav end-to-end not measured).
