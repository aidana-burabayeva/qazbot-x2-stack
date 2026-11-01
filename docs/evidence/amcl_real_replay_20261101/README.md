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
