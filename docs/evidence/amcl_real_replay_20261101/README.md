# AMCL initialpose on a real X2 recording — offline replay through the real nav2_amcl (2026-11-01)

Data: real X2 bag `RViz_goal2_1216` (2026-10-27 17:15; sha256 in `tests/localization/data/goal2_1216_t5_fixture.json`), map `first_map_nav2_v3`
(pgm sha f6032f75…), recorded scan + odom->base only fed to **production-parameter nav2_amcl**; recorded AMCL/Trust output is reference only. Isolated
domain 87, localhost-only loopback DDS; no robot contact, no Docker/MuJoCo/MC (so this is OFFLINE_VERIFIED, not SIM_VERIFIED/L3).

## What the data show (reproduced by replay)
- Robot stationary (odom speed ~0.001 m/s; displacement over the 7.6 s header-stamp skew = 0.002 m).
- Operator seed #1 at 114.9 s is 3.5 m from the previous estimate -> map->odom steps 3.55 m at +0.13 s; seed #2 at 122.1 s steps back 3.52 m, to within ~0.12 m of the pre-seed estimate. Replay with real AMCL: 3.46 m / 0.63 rad at 122 s, pose vs recording median 0.074 m (max 0.085 m).
- AMCL pose after each seed is 0.08-0.15 m from the seed (recorded `amcl_pose`). The "jump" is the applied reseed, not drift.
- Recorded Localization Trust: at each seed `ready=False (awaiting_initialpose_and_amcl)`, `ready=True` ~0.2 s later; **zero T5 reasons**. Trust behaves as designed (`relocalize()` resets map history).
- AMCL warning "Failed to transform initial pose in time": requested time equals the callback wall time (log-requested = 0.1 ms) while the latest odom TF is 21-24 ms older (historical X2 logs: 8.5-34 ms) -> zero-tolerance lookup into the future -> AMCL falls back to identity odom correction. Independent of the header stamp (here 7.6 s in the past). Harmless for a stationary commissioning robot; would drop odometric correction if the robot moved.

## Defect fixed (root cause, replay metric)
`steps()` in both replay entrypoints used raw map->odom translation and no epoch reset, so the intentional reseeds were reported as T5 trips and as "AMCL jumps".
FAIL (before): recorded t5_trips=2 / max step 3.55 m; replay 1 / 3.46 m (`before_metric_fix_results.json`).
PASS (after): recorded 0 / 0.073 m; replay 0 / 0.064 m (`after_metric_fix_results.json`) = recorded Trust (0 T5).
Tests: `tests/localization/test_amcl_replay_t5.py` (real-bag fixture; legacy 2 false trips, new 0; unseeded 0.5 m correction still trips; only the first update after a seed is exempt; metric is robot displacement) — 32 PASS in tests/localization.

## NOT established
- The 0.1-1.72 m / up to 35.7 deg post-seed deviations of the 13 episodes from 2026-10-30 (`initialpose_episodes.json`): their bags are not available locally, so they are NOT reproduced. This 10-27 bag shows no such deviation. UNVERIFIED.
- Production behaviour with a moving robot, DDS delivery on the robot, AMCL stochastic spread (single replay run). NEEDS_HARDWARE / L3.

---
# Addendum — episode 0 of 2026-10-30 (initialpose_episodes.json #0), real recording

Data: `robot_data/twin_20261030/bags/twin_core_190402/twin_core_190402_0.mcap` (sha256 6e4527f8…c58327; contains initialpose, scan, odom TF, recorded AMCL pose, Trust status). Robot stationary. Only this one of the 13 episodes is available locally (others lie in segments _3/_7/_8 left on the robot).

## Root cause of the 0.445 m / -14.8 deg post-seed move (OFFLINE_VERIFIED, 1 of 13 episodes)
1. The operator seed does not fit the real scan: Trust's own `soft_score` on the real map `main_office` = **0.005** at the seed; **0.904** at the recorded post-update AMCL pose (better in 40/40 scans over +1…+9 s); best local fit around the seed = 0.94 at offset (-0.3, -0.3, -0.3 rad), i.e. the recorded correction (-0.385, -0.223, -0.258 rad). The "jump" is AMCL correcting an operator seed that was ~0.45 m / 15 deg off the map-consistent pose.
2. Trigger: the Trust node requests AMCL measurement updates without motion after each initialpose (`_update_seed_without_motion`: <=3 attempts, >=0.6 s apart, new scan each, within 6 s). Without such a request AMCL on a stationary robot does not update at all.
3. Trust behaved as designed: first valid post-seed AMCL pose = initialization accepted, map history reset (`accept_initialization`), READY at +0.71 s, no T5 reason.

## Replay-fidelity defects found while reproducing (all with tests)
| Replay variant on the real episode | AMCL updates | max step | Verdict |
|---|---|---|---|
| tools/twin launch (max_beams 1440, Nav2 defaults), no refresh | 1 | 0.0 m | NOT reproducing (FAIL): AMCL never updates |
| + `--seed-refresh` (Trust helper emulation) | 4 | 0.098 m | moves, below T5 |
| + production AMCL params (max_beams 180, alpha .3/.2/.15, recovery 0, update_min .1/.05) | 4 | 0.126 m, 1 T5-size step at 78.6 s (recorded 78.95 s) | trigger and timing reproduced |
Recorded: 3 updates, 0.462 m. Deterministic across repeated runs. **The magnitude is NOT reproduced (0.13 vs 0.46 m); final pose differs 0.28-0.30 m** — remaining difference (live seed_loop/second trigger, particle spread, live CPU timing) is UNVERIFIED. The scan-fit result in (1) is independent of this replay.
Tests: `test_amcl_seed_episode_real.py` (4: fixture identity, seed 0.005 vs after 0.904, best local fit = recorded correction, refresh schedule on the real scan timeline), `test_amcl_sim_launch_parity.py` (red before fix), `test_amcl_replay_t5.py` — tests/localization 37 PASS.

## Still NOT established
Episodes 1-12 (bags not local), quantitative AMCL reproduction, moving robot, L3/SIM/HW. Operator seeds that are far off the map (score 0.005) are accepted by the UI; whether the commissioning UI/Trust should warn about a low seed score is a design decision, not changed here (Trust ordering/thresholds untouched).
