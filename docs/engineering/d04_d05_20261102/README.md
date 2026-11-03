# D-04 / D-05 engineering — 2026-11-02 (robot feedback, X2)
Statuses: no change deployed. Running X2 level_bridge.py sha256 c817a5ed… (unchanged).

| Item | Status | Summary |
|---|---|---|
| Costmap layers | measured | local+global: 2D ObstacleLayer (+Static global, Inflation). VoxelLayer installed (nav2_costmap_2d 1.1.18) not used; STVL not installed. |
| D-04 root cause | confirmed offline + real bag | marking span ≈ −58.8…+52°, clearing fixed −46…+44.5° → edge marks never cleared; BT ClearEntireCostmap (failure_tolerance 10) = ~10 s delay. |
| D-04 candidate C | OFFLINE_VERIFIED (V1 PASS) | clearing along every observed ray, no fixed window; one function changed (candidate/). Real bag 204153: RPP collision 87.6 % (A) vs 1.3 % (C); persistent object never lost. |
| D-04 open | — | R1 replay levelling must match live R_lv; R2 docs qualification; R3 controlled recording re-do (211835 had 0 returns <1 m). D-04b: stale mark on never-observed bearing (+70.9°/0.37 m) blocks RPP in A and C. Low objects ~0.25 m not detected (D-07 candidate). |
| D-05 | root cause found, fix BLOCKED (needs calibration) | MC LOW speed_mode nonlinear: 1.61× @0.30, 0.93–0.99× @0.80; commands reach MC intact. Calibration sweep 0.10–0.60 needs operator approval. |
| D-06 | measured | leg/LIO speed-dependent: 0.885 @0.30, 0.93–1.00 @0.80. |
| C++ level_bridge | not started (scope only) | candidate/CPP_MIGRATION_SCOPE.md |

Reports: P1P2 (runtime config, AgiBot comparison), P3P4 (A/B/C/D/E), V1 (verification), D1 (D-05/D-06), sitstand-health.
Harness: tests/d04 (54 tests). Recorder: tools/d04/record_d04_static.sh. Bags (3.5 GB each) remain on X2: docs/evidence/nav_20261102_live/d04_static_bag_{204153,211835}.
