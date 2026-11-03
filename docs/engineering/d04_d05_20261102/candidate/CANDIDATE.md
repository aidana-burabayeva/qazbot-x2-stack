# CANDIDATE C — level_bridge clearing without the fixed FOV window (D-04)

- CANDIDATE: C (remove `fov_deg=(-46.0, 44.5)` mask from `clearing_endpoints`; clearing rays follow the observed points)
- SOURCE_SHA: candidates/d04/level_bridge.py sha256 c880d3c8f66eed3c58eecb1db23024a37fd6d877704d6d779cac61906f75abef (feedback branch robot/feedback-d04-d05-20261102, f1816fd)
- BASELINE_SHA: running ws/src/lio/level_bridge.py sha256 c817a5ed503386be742fa7b6fdab2b3209bdefaf42be799d2a130cd198f9369b (process started 16:55:09, unchanged)
- ROOT_CAUSE: marking covers all observed bearings (about +-55 deg, range/elevation dependent) but clearing rays were limited to -46..+44.5 deg. A transient object (person) at an edge bearing leaves lethal cells that nothing can clear; they keep RPP isCollisionImminent true until BT ClearEntireCostmap. Qualification: C removes this only for bearings that are OBSERVED (a return exists in the bin). Marks in never-observed bearings are not cleared (absence != free) and still cause the symptom, see Residuals.
- CHANGED_FILES: candidates/d04/level_bridge.py (+ level_bridge.diff): `clearing_endpoints` only (param removed, `valid = r > 0`). Marking path byte-identical (AST test). Harness: tests/d04/*. Running file NOT modified.
- TEST_EVIDENCE:
  - pytest tests/d04: 59 passed (54 before + 5 live-matched levelling tests); no sleeps.
  - Real bag 204153 (cleaning, uncontrolled), 10 edge events 0.35-1.59 m: A residual up to 8 cells, RPP collision flagged 87.6 % of updates; C 0 residual cells, 1.3 % (only while object present). Independent verifier V1: PASS.
  - Synthetic (S2/S3/S7/S9/S10): A never clears, C clears on first update; S1/S5/S6/S8 (ahead, overhang, chair, blind seeds) A == C. Recorded-only bags: identical lethal sets A and C.
  - Harness fidelity: with fitted live R_lv, 99.997 % of recorded scan_obstacles points within 1 mm, costmap median mismatch 1 cell/publish. The 4 earlier bags (no scan_obstacles) use bag levelling and are NOT live-matched; A-vs-C conclusions unaffected (same R_lv both).
  - Bag 211835 (controlled protocol): no returns < 1 m, no events -> chair/box/glass/blind-exit stay SYNTHETIC / NOT_COVERED.
- SAFETY_IMPACT: no new marks, marking untouched, no decay/timeout/auto clear, collision detection untouched. Clears only cells on rays to observed points, capped at the nearest in-band hit of the bin. Object level: no present object lost (0 losses A and C on the persistent object in 204153; 0 flicker events on the RPP outline). Cell-level edge flicker: 5 most persistent edge cells lethal 96.4 % of updates (A) vs 70.6 % (C), min 42 %; 231 present-cell clear events per 300 s, re-marked within 1 s; same mechanism and rate class already present inside the old window in A. Residuals: (1) marks at never-observed bearings stay (real example +70.9 deg / 0.37 m in bag 204153, blocks RPP 100 % in A and C) -> follow-up D-04b; (2) low objects < band floor not detected (D-07, pre-existing, all variants); (3) glass, reflections, LiDAR loss, TF jump, AMCL correction, repeated goal NOT_COVERED.
- PERFORMANCE_IMPACT: bridge time C vs A on real data p50/p95/p99 5.24/7.08/9.59 vs 5.43/7.34/10.09 ms (204153), i.e. no measurable overhead (synthetic run showed +5 %, not reproduced); wire +1.0 kB per update (+15 %, ~+10 kB/s); costmap layer unchanged.
- ROLLBACK: nothing deployed. If deployed later: keep copy of the baseline (sha above), restore it and restart level_bridge; no params, launch or TF changes involved.
- STATUS: OFFLINE_VERIFIED (hardware deploy NOT approved; needs operator approval and re-recording of the controlled protocol with confirmed LiDAR returns < 1.5 m)

## Reproducibility and real-Nav2 check (2026-11-03)
- Harness resolves the repo root from `__file__`; bags via `QAZBOT_D04_BAGS` (manifest + sha256: `tests/d04/BAGS.md`).
- Final replay (`--level fit --max-clouds 400`, bags d04_static_bag_204153 / 211835, first 40 s each): `tests/d04/results/summary.json`, `raw.json`.
- Real Humble `nav2_costmap_2d` (domain 77, live local_costmap params, `tests/d04/nav2_real_costmap.py`, `results/nav2_real.json`):
  residual edge lethal cells after the object leaves, A / C: S9 20 / 10, S10 24 / 10, no-object scene 18 / 10 (emulator 27/11, 27/10, 21/10);
  object still present (S6) retained by both: 16 / 16 cells (emulator 19 / 18).
