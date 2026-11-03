# 20261021-x2-nav2-e2e-dryrun-locA-data — Step A data: AMCL alpha1-5 vs FAST-LIO (debugger, read-only)
Date 2026-10-22 ~20:27-20:45, host .66.
Nothing was published, set, restarted or killed (no foreign processes). All CPU work ran under `nice -n 19`. Load avg was 8-14 during the run.
Scripts and intermediate data: /tmp/claude-1001/-home-agi/7b3af5b1-2708-4844-bc74-173c51a4dffc/scratchpad/la/
- ext.py, exs.py: sqlite + rclpy deserialize into *.npz
- upd.py: per-update AMCL-vs-LIO (output upd.txt)
- stats.py, fit.py: statistics and fits
- seg.py: stand/move segmentation
- reg.py, t1.py, t2.py: scan-to-map registration probe
- initposes.txt: AMCL "Setting pose" times from the logs

## 1. Live AMCL params (task 1) — NEEDS_DATA
- One attempt only:
  - Command: `FASTRTPS_DEFAULT_PROFILES_FILE=.../navigation/dds_qazbot.xml timeout -k 5 40 nice -n 19 ros2 param get --no-daemon --spin-time 5 /qazbot_d1_map/amcl alpha1`
  - Result: no output, killed by timeout after 45 s. No leftover process of mine.
  - Per instruction, the other 10 gets (alpha2-5, z_rand, z_hit, laser_max_range, update_min_d/a, max_beams) were NOT attempted.
- Pre-existing hung CLI processes. They were NOT killed because they are not mine:
  - PID 327106 `ros2 param get /qazbot_d1_map/amcl laser_model_type`, started 20:14:44.
  - PID 327937 `ros2 param get /qazbot_d1_map/amcl update_min_a`, started 20:15:43.
  - Both are in state **T (stopped / job-control suspended)**, not blocked on DDS.
  - Parent is an interactive `-bash` PID 327038 (Ss+), so they were probably suspended with Ctrl-Z in someone's terminal.
  - They use FASTRTPS profile /agibot/data/home/agi/.aima/env/ros_dds_configuration.xml (whitelist 10.0.1.41 only). AMCL uses dds_qazbot.xml (10.0.1.41 + 192.168.1.66). Both have SHM. A profile mismatch is not proven to be the cause of the hangs (hypothesis).
  - Suggest the owner kill them: `kill -9 327106 327937`.
- ros2 daemon: PID 26754, running since 13:49.
- Loaded params file /tmp/launch_params_2x1umpt4 (unchanged) sets only frames, topics, max_beams 1440 and OmniMotionModel. Everything else is navigation2 1.1.18 defaults: alpha1-5 0.2, z_hit 0.5, z_rand 0.5, update_min_d 0.25, update_min_a 0.2, laser_max_range 100 (-> scan range_max 8 used).
  - The logs show no runtime param set: no "param set" in the logs; only replay_dense_matched_01_driver.log mentions set_parameters, which is unrelated.
  - Still unconfirmed live.

Motion model, verified from source (navigation2-1.1.18 omni_motion_model.cpp, candidates/nav2_smac_bounds/...). Per filter update with LIO increment dt (m) and dr (rad):
  trans_sd  = sqrt(a3*dt^2 + a4*dr^2)
  rot_sd    = sqrt(a1*dr^2 + a2*dt^2)
  strafe_sd = sqrt(a4*dr^2 + a5*dt^2)
So: a1 = rot<-rot, a2 = rot<-trans, a3 = fwd<-trans, a4 = fwd AND strafe<-rot, a5 = strafe<-trans. This matches the prior report §3.

## 2. Offline data (task 2)
Bags: g_motion_01..04, turn_nav_01, short_nav_01, cycle2_01/02 and manual_ready_turn_01/02. Each was decoded (rclpy deserialize, read-only sqlite) into:
- /qazbot_d1_map/tf (map->odom from AMCL, odom->base_footprint from level_bridge);
- /qazbot_d1_map/amcl_pose with covariance;
- particle_cloud;
- /qazbot_lio/odometry and odom_level;
- /aima/mc/leg_odometry (1/8 subsample; not used further);
- /qazbot_lio/scan;
- trust status JSON.

Facts:
- map->odom TF changes exactly once per amcl_pose message: short_nav_01 has 7 changes and 7 messages. The map->odom stamp is the AMCL stamp + 1.0 s (transform_tolerance).
- amcl_pose is also emitted at each initialpose. Those 14 rows were excluded using the "Setting pose" times from localization_v3_loopfix_0{1,2}.log.
- base_footprint here is the LiDAR position, levelled, with LiDAR-forward yaw (level_bridge.py:106-135). The virtual scan is built from cloud_registered back-transformed with the same LIO pose, so it is LIO-consistent.

Per-update quantities: P_k = AMCL pose, O_k = LIO pose at the same stamp, M = map->odom.
- AMCL-vs-LIO error per update: e_k = (M_{k-1} (+) O_k)^-1 (+) P_k, in the robot frame. This is identical to the map->odom step.

### 2a. Distribution (105 valid updates)
| group | n | err xy p50/p90/p95/max (m) | err yaw p50/p90/p95/max (rad) | per-axis rms fwd/lat (m) |
|---|---|---|---|---|
| no-motion (LIO dt<0.03, dr<0.02; request_nomotion_update) | 51 | 0.021/0.038/0.057/0.069 | 0.004/0.016/0.020/0.024 | 0.019/0.020 |
| motion (dt>=0.1 or dr>=0.15) | 54 | 0.024/0.047/0.054/0.105 | 0.017/0.038/0.052/0.060 | 0.021/0.025 |
- Motion updates are mostly rotation-triggered: median dt 0.079 m, median dr 0.225 rad.
- map->odom step per update: p50 0.022 m, p95 0.060 m, max 0.105 m (g_motion_03 16:57:30.9, dr 0.52 rad). Yaw: p95 0.052 rad, max 0.060 rad.
  - No single filter update reached T5's 0.10 rad limit. Only one reached 0.10 m: 0.105 m with sigma 0.36-0.39.
  - All larger steps (0.24-0.45 m, 1.5 rad) are operator initialposes.
- Key observation: the per-update xy disagreement during motion is nearly the same as while standing still (0.032 vs 0.028 m rms).
  - corr(err_xy, dt) = 0.03. corr(err_yaw, dr) = 0.22.
  - So the per-update AMCL-vs-LIO disagreement is dominated by AMCL's own particle-mean jitter, not by LIO increment error. That jitter is ~2 cm / 0.01 rad per update at sigma 0.15-0.5 m.
- Sigma growth on motion updates matches the "no measurement information" prediction with default alphas, sigma_new ≈ hypot(sigma_old, default noise), to within about ±0.03 (stats.py table).
  - The scan barely contracts the cloud. This re-confirms H1/H4.

### 2b. Loss instants
- 17:01:28 (g_motion_03, trust_v3_07, T5_map_odom_jump):
  - Single-update step at 17:01:28.3: 0.020 m / 0.003 rad. Previous steps 0.006-0.025 m / 0.007-0.053 rad. Not a per-update jump.
  - Cumulative AMCL vs (first update 16:56:44 map->odom (+) LIO): 0.160 m / -0.181 rad after 3.85 m LIO path and 7.7 rad cumulative rotation.
  - AMCL sd at that moment: 0.45 m / 0.33 rad. The trust anchor_err_xy metric was 0.109.
  - Max cumulative over the run: 0.245 m (17:01:17.9). Yaw stayed between -0.19 and +0.01.
  - So this was the old window rule summing AMCL wander inside a 0.45 m cloud. The disagreement is 0.35-0.5 sigma.
- 19:18:48 (short_nav_01, T5 window rule):
  - Single steps 0.010-0.066 m and 0.010-0.033 rad, the last two 0.059 m and 0.066 m. Cumulative over 19:18:43-48 = 0.178 m (map->odom).
  - AMCL vs (19:17:14 map->odom (+) LIO): 0.159 m / -0.032 rad after only 0.27 m path and 1.61 rad of turning. sd 0.376 / 0.230. anchor_err_xy metric 0.157.
  - The error grows monotonically during a 92 deg in-place turn: 0.04 -> 0.07 -> 0.09 -> 0.12 -> 0.16 m. That is consistent with AMCL drift during a rotation with weak scans and also with an LIO rotation-to-translation lever-arm error.
  - Cannot be separated with the data available (see 2c). NEEDS_DATA.
- Other runs with converged AMCL at the start stay small:
  - g_motion_01: 0.038 m / 0.05 rad after 1.21 m / 0.87 rad.
  - cycle2_02 S3b (start sd 0.155/0.044): 0.014 m / 0.026 rad after 0.86 rad.
  - turn_nav_01: 0.023 m over a 0.29 rad turn.

### 2c. Independent ground truth for LIO increment error — NOT obtainable from these bags
- Tried multi-scan (10-20 scans) scan-to-map registration on the D1 map for standing windows (reg.py, distance-field cost, grid + Powell):
  - The cost is flat along one direction: changing dx from 0 to +0.45 m changes the cost only 0.0284 -> 0.0237.
  - Hessian eigenvalues are 10-100x apart. Inlier fraction (<0.10 m) is only 0.43-0.46.
  - The -20..0 deg sector holds an unmapped object at 1.6 m (median dist to map 0.45 m), probably the operator or clutter.
  - Repeatability across 4 subsets of one standing window: 0.38-0.55 m offset vs AMCL, spread ~0.17 m.
  - So scan-to-map cannot bound LIO error below ~0.2 m here.
- Leg odometry (/aima/mc/leg_odometry) is in the bags but is a worse sensor than LIO. LIO-vs-leg would only bound the sum of both errors (not computed).
- Standing-still LIO jitter: the 1.5 s-window displacement is 1-2.5 cm, and summed path grows 0.025 m/s while standing (seg.py). This is real humanoid sway seen by a chest LiDAR, not drift.
  - Net drift over 5-10 min standing windows: 0.013-0.014 m and 0.010-0.015 rad (cycle2_02 20:04:11-20:09:54, 20:10:01-20:20:09). This includes no motion.

## 3. Candidate alpha1-5 (task 3)
Model fitted to the 105 valid updates (no-motion + motion), per axis (fit.py, NNLS, 2000-sample bootstrap):
  e_fwd^2 = c0 + a3*dt^2 + a4*dr^2,  e_lat^2 = c0' + a5*dt^2 + a4'*dr^2,  e_yaw^2 = c0'' + a2*dt^2 + a1*dr^2
- c0 terms absorb AMCL's own per-update jitter: fwd 0.019 m, lat 0.013 m, yaw 0.011 rad.
- e also contains AMCL jitter that grows with sigma during motion. Every coefficient is therefore an UPPER BOUND on LIO increment-error variance.

| coef | point | 90% CI | meaning (sd per unit) |
|---|---|---|---|
| a3 (fwd<-trans) | 0.0000 | [0, 0.00083] | <= 0.029 m/m |
| a5 (strafe<-trans) | 0.0034 | [0, 0.0100] | <= 0.10 m/m (weak: few translation updates) |
| a4 (fwd<-rot) / (lat<-rot) | 0.0015 / 0.0078 | [0, 0.0042] / [0, 0.019] | <= 0.065 / 0.14 m/rad |
| a2 (rot<-trans) | 0.0029 | [0, 0.0061] | <= 0.078 rad/m |
| a1 (rot<-rot) | 0.0046 | [0.0021, 0.0076] | 0.046-0.087 rad/rad |

Rule: candidate alpha_i = upper 90% CI of its coefficient, rounded up. This is conservative: it errs toward under-confidence, so it guards against the overconfidence risk from report §5.
- **alpha1 = 0.008, alpha2 = 0.006, alpha3 = 0.001, alpha4 = 0.02, alpha5 = 0.01** (defaults 0.2 each).
  - a4 is the max of the fwd and lat estimates, because the omni model uses one a4 for both.
- Resulting per-update noise:
  - 0.25 m step: trans 0.008 m, strafe 0.025 m, rot 0.019 rad. Defaults: 0.112 m / 0.112 rad.
  - 0.2 rad turn: trans and strafe 0.028 m, rot 0.018 rad. Defaults: 0.089 / 0.089.
- That is about 4-14x smaller sd than the defaults, and still at or above the measured per-update disagreement p50 (0.024 m / 0.017 rad).
- Confidence:
  - MEDIUM that the defaults are 1-2 orders of magnitude too large in variance.
  - LOW for the individual values, especially a3 and a5:
    - only ~15 updates had dt > 0.2 m;
    - the median motion update was a 13 deg turn;
    - no independent ground truth exists.
  - a1 is the only coefficient whose CI excludes 0.
- Coupling: if update_min_d/update_min_a are made explicit (0.25 / 0.2, the current defaults), the trust node must use the same values (report §5).

Still requires the operator loop walk (NEEDS_DATA):
- A closed loop of >= 5 m with >= 2 full turns, returning to a taped floor mark with a heading mark. Measure LIO closure error per m and per rad (fwd, lat, yaw).
- A separate pure-translation leg (to bound a3/a5) and a pure in-place 360 deg turn (to bound a1/a4).
- The operator must stay out of the forward 90-100 deg sensor sector. That clutter is what currently breaks scan-to-map ground truth.
- Then replace the CI upper bounds above with the measured p95.
- Also needed: confirmation of live AMCL params (§1) and a verification run per report §5 Step A (sigma growth per update <= candidate noise; no T5; no T4 degradation).

## 4. Symptom / proximate / root (unchanged from locpersist, now quantified)
- Symptom: READY loss at 17:01:28 and 19:18:48; sigma 0.3-0.5.
- Proximate cause: AMCL cloud inflation per update ≈ default motion noise with no measurement contraction. The mean wanders 2 cm per update, and up to 0.16-0.25 m cumulative inside that cloud. The old T5 window rule summed this.
- Root cause: alpha1-5 at wheel-odom defaults on top of LIO. The weak 90 deg virtual scan cannot pull the cloud back.

## 5. Next diagnostic commands
- After the owner clears PIDs 327106/327937 and load is low, once each: `timeout 40 ros2 param get --no-daemon /qazbot_d1_map/amcl alpha1` ... `max_beams`. Alternative: `ros2 param dump /qazbot_d1_map/amcl` once.
- Operator loop walk with `ros2 bag record /qazbot_d1_map/tf /qazbot_lio/odometry /qazbot_lio/scan /qazbot_d1_map/amcl_pose /aima/mc/leg_odometry`. Requires explicit user go (motion).
- Offline: re-run fit.py on the loop bag. Also optionally compare LIO vs leg odometry per segment.
