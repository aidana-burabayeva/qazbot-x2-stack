# 20261023-calib-design: LIO incremental-error measurement for AMCL OmniMotionModel alpha1-5 (design + offline tools)

Role: robotics-debugger (read-only). No motion. No publishing, no param set, no node restarts. All CPU work ran as `nice -n 19 taskset -c 0-4,7`, keeping off FAST-LIO cores 5 and 6. Only closed bags were read: each has metadata.yaml and no process holds its db3 open.

STATUS: design READY. Offline tools tested on 3 existing motion bags. The calibration itself is NEEDS_DATA: it needs the physical session below, and every block needs its own user go.

## 0. Deliverable location (deviation, read first)
My role rule lets me write only this report. The scripts are therefore in the session scratchpad and not in tools/lio_calib/:
`/tmp/claude-1001/-home-agi/7b3af5b1-2708-4844-bc74-173c51a4dffc/scratchpad/lio_calib/{lc_common.py,lc_icp.py,lio_legs.py,lc_fit.py}`
(md5 dd82ea41 / 519a7674 / 8e24b70b / 1a76846c; offline outputs in `out/`). Orchestrator action to install them (new dir, nothing overwritten):
`mkdir -p /home/agi/control_backend/tools/lio_calib && cp /tmp/claude-1001/-home-agi/7b3af5b1-2708-4844-bc74-173c51a4dffc/scratchpad/lio_calib/*.py /home/agi/control_backend/tools/lio_calib/`

## 1. System facts used (verified from files/bags)
- Raw cloud: `/aima/hal/sensor/lidar_chest_front/lidar_pointcloud` (sensor_msgs/PointCloud2). frame `lidar_chest_front`, about 25.2k points per scan, 10 Hz, 96 rings.
  - Fields: x,y,z f32 at offsets 0/4/8; intensity; ring u16; timestamp f64 (ns relative, 0.3-95 ms).
  - FOV: azimuth ±59°, elevation ±43°. Median range indoors 2.7 m.
- LiDAR IMU: `/aima/hal/sensor/lidar_chest_front/imu`.
- FAST-LIO odometry `/qazbot_lio/odometry`: camera_init -> body, where body = LiDAR IMU.
  - Its stamp = cloud header + last point offset. Measured on manual_amcl_walk_02: odom - header = 0.095 s = max point timestamp.
- Extrinsic T_BL comes from ws/src/lio/x2.yaml (extrinsic_R/T).
- odom_level (level_bridge.py) = IMU position levelled, with yaw = heading of the LiDAR x axis. AMCL consumes this (base_footprint).
- Bags with raw clouds (all closed): logs/manual_amcl_walk_01, _walk_02, _turn_01, _reverse_01, raw_comparison_input_01. These also contain LIO odometry, odom_level and IMU.
  - logs/bags/liolat_lidar_imu_01 has cloud and IMU only, with no odometry.
  - logs/bags/{g_motion_*, cycle2_*, short_nav_01, turn_nav_01} have NO raw cloud. They cannot be used for this method.
- Libraries on host: Open3D, small_gicp, KISS-ICP, sklearn, rosbags: NOT installed.
  - Available: numpy 1.26.1, scipy 1.8.0 (prints a numpy-version warning; cKDTree worked), rosbag2_py.
  - libpcl-dev 1.12 with registration/gicp.h, plus g++/cmake. There are no pcl_* CLI binaries.
  - Choice: a standard point-to-plane ICP (numpy/scipy). A PCL GICP cross-check was NOT done (NEEDS_DATA, optional).
- step_cmd.py (tests/s3):
  - Forward-only `--lin` ≤0.3 m/s and `--rot` with `--rot-cap` ≤1.0 rad/s.
  - Unwrapped yaw, so `--stop-after-yaw-deg 360` works.
  - Each run does trust arm_check + adapter arm. It aborts on untrusted, latch, foreign publisher, stale odom/status or SIGINT. It leaves the coordinator latched.
  - The S3b data show |wz|=0.6 turning: left actual 0.29 rad/s (ratio 0.49), right 0.92 rad/s (ratio 1.53). Asymmetric.
- record_motion.sh does NOT record the raw cloud or the LiDAR IMU.
  - Its CLOUDS=1 adds /qazbot_lio/cloud_registered_body, which x2.yaml does not publish (scan_bodyframe_pub_en false). Hypothesis: that topic is absent live.
- OmniMotionModel (navigation2-1.1.18 nav2_amcl/src/motion_model/omni_motion_model.cpp:53-87). Per filter update with dt = |Δxy|, dr = Δyaw:
  - `sd_trans = sqrt(a3·dt² + a4·dr²)` along the motion bearing
  - `sd_rot = sqrt(a1·dr² + a2·dt²)`
  - `sd_strafe = sqrt(a4·dr² + a5·dt²)` perpendicular to the bearing
  - So alpha is a VARIANCE coefficient: alpha = (sd per unit)².
  - Update trigger: amcl_node.cpp:768-777, per-axis |dx|>update_min_d or |dy|>update_min_d or |dyaw|>update_min_a. Live values: 0.25 m / 0.2 rad.

## 2. Evaluation of the proposed reference
Accepted, with four changes:
1. Measure open legs, not only closed loops. Out-and-back and ±90/∓90 loops cancel systematic errors (scale, gyro scale), so closed-loop error UNDER-estimates the incremental error. Closed loops are kept only as a consistency check.
2. Overlap limit. The FOV is 118° in azimuth. Start and end clouds of a 90° or 180° turn barely overlap and cannot be registered directly.
   - Rotation legs are therefore 45° steps (≥73° overlap) plus closed 360° turns.
   - A 51° leg with 1 m of translation registered consistently (16/16 pairs, MAD ≤4 mm) at fitness 0.35-0.42.
3. Use per-cloud pairing, not window means. The robot sways 1-6 mm sd while standing. Each cloud is paired with the LIO pose at the cloud's exact end stamp, so sway cancels. k×k pairs per leg give the median and MAD.
4. Use the 3D raw cloud, not the virtual 2D scan. Initialise from LIO. Initial guesses perturbed by 0.2 m / 5° converged to identical results (3/3 per window, spread 0 at print precision).

Error convention (lio_legs.py):
- e = (registration - LIO) end position, in the levelled heading frame of the start window. Up = mean IMU specific force in that window.
- e_yaw = the gravity-axis part of R_lio^T R_reg.
- e_along / e_perp are along / left of the LIO leg direction, which are OmniMotionModel's trans/strafe axes.
- Per leg, the AMCL update sequence is replayed exactly on odom_level, giving S_tt = Σdt_k² and S_rr = Σdr_k².

## 3. Registration accuracy (offline, same pose = clouds inside one stationary window)
Source: lio_legs.py selftest on manual_amcl_walk_02, 3 windows, 4 clouds each, 6 pairs per window. The values are reg minus LIO, so they are an UPPER bound on registration error.

| window | median e fwd / left / up (mm) | max |e|xy (mm) | max |e_yaw| (deg) | fitness | min eig_trans |
|---|---|---|---|---|---|
| 0 (43 s) | 2.4 / 1.5 / -0.8 | 4.6 | 0.022 | 0.73 | 0.139 |
| 1 (2.3 s) | 2.2 / -2.0 / -0.6 | 7.4 | 0.031 | 0.67 | 0.088 |
| 2 (47 s) | 1.4 / 6.0 / -0.7 | 13.0 | 0.191 | 0.60 | 0.090 |

- Window 2's outliers all involve the FIRST cloud after stopping (fitness 0.60 vs 0.74). Cause not proven: a dynamic object (the operator) or LIO settling. Protocol: discard the first 1 s after stop and use the median of pairs.
- The stationary->stationary leg pairs with almost no motion (legs 1->2 in all 3 bags) agree to 0-17 mm and ≤0.1° as medians.
- Working value: σ_reg ≈ 4 mm per horizontal axis and ≈0.03° yaw (EXPERIMENTAL; based on 1 bag and 18 pairs).

## 4. First real LIO-error measurements from existing bags (operator PS5 moves, NOT the planned protocol)
Run: `lio_legs.py legs BAG --k 4 --pairs all`. Shown: the independent motion leg (window 0 -> 1). QC gate: fitness ≥0.30, eig ≥0.02.

| bag | LIO leg | e_fwd / e_left (mm) | e_along / e_perp (mm) | e_yaw (deg) | MAD xy (mm) | pairs ok |
|---|---|---|---|---|---|---|
| walk_02 | 1.043 m, +51° | +11.9 / -21.5 | +11.8 / -21.5 | +0.21 | 4.4/3.4 | 16/16 |
| reverse_01 | 1.223 m, -42° | +6.4 / -34.2 | -27.5 / +23.0 | +0.10 | 8.1/6.0 | 16/16 |
| turn_01 | 0.030 m, +363° in place | +4.8 / +2.2 | n/a | +0.01 | 4.3/5.9 | 16/16 |

- Reading: LIO translation error is about 2-3 cm per 1-1.2 m (about 1% along, about 2-3% lateral). Yaw error is ≤0.2° per leg. A 360° turn shows about 5 mm and 0.01°, which is at the registration noise floor.
- Hypothesis: the Codex operator-mark result "LIO 0.918 m vs 1.000 m" (-8%) is mainly floor-mark or alignment error. In walk_02, registration gives 1.055 m against LIO's 1.042 m.
- Illustrative fit only (n=3, NOT a calibration): `lc_fit.py out/legs_*_g030_indep.json --reg-sd 0.004 --reg-sd-yaw 0.0005`:
  - Point estimates: a1 0, a2 3.0e-5, a3 1.5e-3, a4 2.8e-4, a5 1.2e-3.
  - 95% one-sided upper (×8.5 for n=3): a2 2.6e-4, a3 0.013, a4 0.0024, a5 0.011, a1 0 (unidentified).
  - Defaults are 0.2. Do not apply these values.

## 5. Protocol (orchestrator-commanded; operator = hold the robot and safety only; each block needs its own user go)
Preconditions (read-only checks):
- Stack live; adapter live, registered and idle; trust READY; no Nav2 goal active.
- MC is STAND/STABLE.
- Free space: ≥1.5 m ahead and a 1 m radius around.
- The operator stays BEHIND the robot, outside the ±60° front sensor wedge, holding the robot with the PS5 in hand. If the operator is visible to the lidar, registration is contaminated.
- Static scene with no people walking in front. There must be 3D structure within 10 m (walls/furniture).

Recording: one extra standard recorder in parallel with record_motion.sh. Only after it is running, never read the open db.
```
bash -c 'cd /home/agi/control_backend && source ws/src/navigation/env.sh && exec python3 ws/src/navigation/detach.py rec_NAME_cloud ros2 bag record -o logs/bags/NAME_cloud --max-cache-size 100000000 /aima/hal/sensor/lidar_chest_front/lidar_pointcloud /aima/hal/sensor/lidar_chest_front/imu /qazbot_lio/odometry /qazbot_lio/odom_level'
```
- Rate about 8 MB/s (manual_amcl_walk_02: 839 MB / 100 s); 10 min ≈ 5 GB.
- Stop it with `kill -INT` of the pid in logs/rec_NAME_cloud.pid.
- Unverified assumption: this extra subscriber does not worsen FAST-LIO latency. Precedent: the manual_amcl_* bags were recorded the same way. Check FAST-LIO odom p99 during the session.
- Alternative: an implementer adds a RAW_CLOUD=1 option to record_motion.sh (a code change, needs the orchestrator).

Each leg = one step_cmd.py run. Then wait ≥10 s stationary (the analysis needs ≥2 s after a 1 s trim; extra margin for coast ≤1.8 s) before the next run. Common command prefix:
`bash -c 'cd /home/agi/control_backend && source ws/src/navigation/env.sh && exec python3 tests/s3/step_cmd.py --expect-domain 0 --expect-mode live --csv logs/calib/NAME_legNN.csv ...'`

Block T (translation; forward only):
- T1: 12 × `--lin 0.15 --lin-dur 8 --stop-after-dist 0.85` gives ~1.0 m with the 0.14 m coast. Actual speed ≈0.25-0.28 m/s (1.6-1.9×).
- T2: 12 × `--lin 0.15 --lin-dur 6 --stop-after-dist 0.40` gives ~0.5 m. Two lengths separate error ∝D from ∝√D.
- Turning back between legs uses 4 × 45° steps (block R), so the turn-back legs also count as rotation data.
- Cap: 0.15 m/s command (Nav2 cap 0.3). Space needed: 1.2 m.

Block R (rotation):
- R1: open 45° legs, 16 per direction: `--rot 0.6 --rot-dur 6 --stop-after-yaw-deg 40` and `--rot -0.6 ...`. The 40° limit allows for coast. The 0.6 cap needs `--rot-cap 0.6`, which is user-approved for S3b only. Ask again.
- R2: closed 360° turns, 4 per direction: `--rot ±0.6 --rot-dur 30 --stop-after-yaw-deg 350`. This is a consistency check and also the a1 data point.

Block L (optional loop check): fwd 1 m, 4 × 45° (180°), fwd 1 m, 4 × 45°, repeated twice. Register S0 vs S_end.

Totals and stop path:
- About 24 T legs and 40 R legs. Each run takes about 25 s (arm 1-6 s + motion 3-25 s + stationary 10 s). Total about 27 min: split into 3 sessions of ≤10 min with operator rest.
- Stop: PS5 stick first, Ctrl-C on step_cmd (1 s of zeros + adapter stop), then motion_adapter/stop. After an abort the adapter stays latched; review before resuming.

Analysis:
1. `lio_legs.py selftest BAG_cloud --k 4` and confirm σ_reg in this scene.
2. `lio_legs.py legs BAG_cloud --k 3 --pairs consecutive --out legs.json`. Use `--windows` if auto-detection merges or splits windows.
3. `lc_fit.py legs.json --reg-sd <σ_reg> --reg-sd-yaw <σ_yaw>`.

## 6. Alpha derivation
- Updates are independent, so over a leg:
  - Var_along = a3·S_tt + a4·S_rr
  - Var_perp = a5·S_tt + a4·S_rr
  - Var_yaw = a1·S_rr + a2·S_tt
- For pure rotation the bearing is undefined, so both axes get a4·S_rr.
- The estimator uses SECOND MOMENTS, with any bias included. AMCL noise is zero-mean, so a systematic LIO bias must be covered by variance.
  - Fit: NNLS across legs, with 2σ_reg² subtracted.
  - Upper bound: multiply by n/χ²₀.₀₅(n). That factor is 2.54 at n=10, 1.83 at n=20, 1.61 at n=30.
- Distribution-free p95 of |e|: the sample maximum bounds p95 with confidence 1-0.95^n, which needs n=59 for 95%. Recommended instead: Gaussian upper CI with n ≥20 per class, plus a check that the empirical p95 stays inside 1.96·sd_model.
- Horizon caveat (unproven assumption):
  - The model accumulates variance a3·D·u (u = 0.25-0.354 m per update), i.e. random-walk growth.
  - If LIO error grows ∝D (systematic), alphas fitted at 1 m under-cover longer uninformative stretches.
  - The 0.5 m and 1.0 m legs test the exponent. Calibrate at the longest expected scan-uninformative horizon, which is a user decision (e.g. the trust 2 m limit).

## 7. Unproven assumptions / gaps (NEEDS_DATA)
1. σ_reg comes from one scene and 18 pairs. Needs a selftest in the actual calibration scene.
2. QC gates are experimental: fitness ≥0.30, eig_trans ≥0.02, stationary-window speed <0.04 m/s and <0.04 rad/s over 0.5 s.
3. A PCL GICP cross-check of the numpy ICP was not done. scipy 1.8 vs numpy 1.26 warns, though results were consistent.
4. Leg-to-leg independence and Gaussianity are assumed for the CI.
5. The extra raw-cloud recorder's effect on FAST-LIO latency is unverified.
6. The rotation cap 0.6 needs user approval for this use.
7. Left/right turn speed asymmetry (0.29 vs 0.92 rad/s actual): analyse each rotation direction separately.
8. step_cmd needs trust READY for every leg. If AMCL loses READY mid-session, the session stops. That is safe, but it is a data gap.
9. Stance sway and the posture tilt change (up vector varied by 1.7° between windows) are handled through levelling per start window. Not validated against an external reference.
10. lio_legs `--pairs all` also reports non-independent legs (0->2). For the fit, use consecutive legs only.
