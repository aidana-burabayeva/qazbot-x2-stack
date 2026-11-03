# 20261027 — FAST-LIO translation drift during in-place rotation (trial06) — diagnosis

Role: robotics-debugger (read-only, offline). No motion, no live-stack interaction, no bag replay performed (time budget).
All analysis under `nice -n 19` from the recorded bag; scripts/intermediate data in
/tmp/claude-1001/-home-agi/064b986d-c606-4601-b9e1-dc10e302f794/scratchpad/ (extract.py, a1..a9.py, t06.pkl).

STATUS: ROOT_CAUSE_FOUND (mechanism well supported; the exact source of the ~0.18 m/s^2 horizontal force error is NOT proven)

## 1. System reconstruction (what actually ran on 2026-10-23 20:54 CST)
- Launch: tools/stack/stack_up.sh -> ws/src/navigation/run_localization.sh -> localization.launch.py
  `lio_binary:=/home/agi/control_backend/candidates/fast_lio_portable/install/fast_lio/lib/fast_lio/fastlio_mapping`
  (sha256 63903a5c..., loop-back fix build; note runtime/fastlio_mapping is a different binary 87040c..., NOT used), `lio_cpus:=5,6`.
- Params: /home/agi/control_backend/ws/src/lio/x2.yaml (mtime 2026-10-20, unchanged since):
  lidar_type 5 (AIMA patch), scan_line 96, scan_rate 10, timestamp_unit 3 (ns), blind 0.5, point_filter_num 3,
  feature_extract_enable false, filter_size_surf/map 0.3, max_iteration 3, fov_degree 110, det_range 30,
  acc_cov 0.1, gyr_cov 0.1, b_acc_cov 1e-4, b_gyr_cov 1e-4, time_sync_en false, time_offset_lidar_to_imu 0.0,
  extrinsic_est_en false, extrinsic_T [0.0043,-0.0158,-0.0112] m, extrinsic_R ~ axis permutation (lidar x = -z_imu, lidar y = -x_imu, lidar z = +y_imu).
- IMU used by FAST-LIO: /aima/hal/sensor/lidar_chest_front/imu (the lidar's own IMU, 200.5 Hz), NOT /aima/hal/imu/chest or torso (500 Hz).
- level_bridge.py: base_footprint = IMU position, yaw = lidar forward projected; /qazbot_lio/odom_level.
- FAST-LIO process was started 19:14:53 (log localization_go2_policy23.log), i.e. it ran ~1.7 h through trials 01-05 before trial06; ikd-tree map accumulated. No "No Effective Points" / loop-back lines in that log (log only has startup + AMCL lines).
- Lidar geometry (from data): lidar frame x fwd, y down (up_l = [-0.073,-0.997,0]), z left; FOV ~ +-42 deg horizontal x +-58 deg vertical; ~25.4k pts/scan; ~35 % floor points.

## 2. Key timeline (t = 0 at command on, 1792760042.609; cmd off t=5.87)
LIO pelvis (kinematic_series 'frames/pelvis') vs native leg odometry base:
| t s | LIO disp m | LIO yaw deg | native disp m | native yaw deg |
|---|---|---|---|---|
| -0.4 | 0.022 | -0.1 | 0.001 | 0.0 |
| 2.1 | 0.068 | 19.5 | 0.023 | 19.4 |
| 4.1 | 0.077 | 32.5 | 0.011 | 32.6 |
| 5.1 | 0.124 | 57.7 | 0.019 | 58.5 |
| 6.1 | 0.207 | 68.7 | 0.015 | 70.5 |
| 7.6 | 0.355 | 67.6 | 0.014 | 70.5 |
| 8.6 | 0.625 | 62.9 | 0.004 | 67.5 |
| 10.6 | 1.229 | 62.0 | 0.004 | 67.4 |
| 12.6 | 1.648 | 61.7 | 0.004 | 67.4 |
- MC FSM: LOCOMOTION from t=0.36, STAND_DEFAULT at t=7.86.
- Only 0.207 m of 1.675 m accrued by t=6.1 (rotation end). **1.47 m (88 %) accrued after native yaw stopped; 1.29 m after FSM STAND (t>=7.9, 4.9 s, robot standing).**
- LIO horizontal speed while standing: 0.32 (8-9 s), 0.29, 0.31, 0.23, 0.13 m/s (12-13 s). Velocity in IMU body frame is ~pure +y_imu (= lidar +z = body LEFT), e.g. [−0.01, 0.32, 0.04] m/s.
- LIO yaw tracked native within ~1-2 deg during the turn; after stop LIO yaw drifted −7.2 deg vs native −3.1 deg (extra −4 deg).
- LIO pelvis z dropped 0.33 -> 0.24 m during t=4.6-6 (native z constant 0.58).
- The step tool's 0.3 m distance limit (t=5.87 into step) was triggered by this LIO drift, not by real motion.
- trial04 (50 deg) also drifted 0.215 m; trial03 (39 deg) 0.047 m; trial05 (5 deg) 0.03 m.

## 3. Evidence per hypothesis

### H3 geometric degeneracy (near frontal plane, narrow FOV) — RANK 1 (enabler), strongly supported
Per-scan point-to-plane information matrix (own-scan PCA normals, 0.15 m voxels; approximation of FAST-LIO's map matching), a4.py:
| phase | min transl. eig | mid | weakest direction (lidar frame) | frontal-wall normal fraction |
|---|---|---|---|---|
| pre (t<0) | 140-200 | 250-310 | lidar x (forward) | 0.05-0.08 |
| turning t=2.3-4.1 | 88 -> 38 | 500-555 | rotating fwd->left | 0.01-0.05 |
| t=5.9-7.7 | **6-14** | 495-540 | **lidar +z (body left) [0,0.01,1.0]** | 0.26-0.28 |
| standing t=8.3-12.5 | 20-41 | 410-475 | lidar +z [-0.16,0.01,0.99] | 0.23-0.25 |
- Min/mid ratio dropped from ~0.6 to 0.01-0.07: after the turn the view is a near frontal plane + floor, lateral translation nearly unobservable.
- Drift velocity direction vs weakest-constraint direction (world, horizontal): **13.2 deg apart**; vs lidar forward 103 deg.
- Scan range stats almost unchanged (r10 1.34-1.40 m, median 2.9-3.1 m, f<1 m = 0; no points inside blind) — the problem is constraint direction, not point count.
- Rotational information also fell (~8000 -> ~3500 min eig) — consistent with the extra yaw drift.

### H4 IMU horizontal force error (bias absorbed into gravity at init / ~1 deg attitude error) — RANK 2 (driver), supported, source not proven
- Specific force of the lidar IMU rotated by LIO orientation, minus pre-command mean (a7.py): horizontal residual 0.035 (0-3 s), 0.085 (3-6.1), 0.156 (6.1-7.9), **0.175-0.185 m/s^2 standing after the turn** (= 1.0-1.1 deg tilt equivalent).
- Physical chest tilt change pre vs post is only 0.21-0.27 deg (lidar IMU, chest IMU, torso IMU agree) -> residual is NOT real posture/acceleration. It is either a body-fixed horizontal accel bias that was absorbed as tilt at static init and became visible after 67 deg of yaw (implied |b| ~ 0.17 m/s^2 ~ 17 mg, min-norm b_imu=[0.011,0.139,−0.104]), or an equivalent LIO attitude error.
- Residual direction vs drift velocity: **8.8 deg apart**; vs weakest-constraint dir 22 deg.
- Rough magnitude check: 0.18 m/s^2 integrated in an unconstrained direction would give >0.9 m/s in 5 s; observed peak 0.32 m/s then decaying -> lidar constraint weak but nonzero.
- No saturation: lidar IMU peaks |a| 16.1 m/s^2 (1.6 g), |w| 1.67 rad/s; no clipped repeats; 1 gap >20 ms (20.4 ms). Stepping impacts during cmd: |a| 5.7-16.1 m/s^2.
- Lidar IMU gyro bias at rest is large: [−0.0094, −0.0115, −0.0038] rad/s (0.54/0.66 deg/s); chest/torso IMUs ~0.0001. FAST-LIO held yaw before the cmd, so b_g was estimated; whether its b_g/b_a estimates were disturbed by the turn is NOT observable (FAST-LIO does not publish bias states; covariance fields in /Odometry are all zero, twist zero).
- IMU choice: FAST-LIO uses the lidar's own IMU (rigid with lidar, 200 Hz, higher bias). Chest IMU (500 Hz, ~100x lower gyro bias) is not used. Not proven it would be better (frame/extrinsic to lidar must be known).

### H1 lidar-IMU extrinsic / lever arm — RANK 4, refuted as main cause
- FAST-LIO's lidar-IMU extrinsic is between the lidar and its own IMU: |T| = 2 cm. A 2 cm lever over 67 deg gives <2.3 cm.
- IMU->pelvis planar offset 0.08-0.12 m (body_audit). Max chord for r=0.12 m at 67.4 deg: 0.133 m. Lever-arm fit on rotation phase (t<6.1): |l|=0.12 m, rms 0.026 vs data rms 0.043 (partial fit; cannot separate from early drift). Fit over full window: |l|=0.67 m, rms 0.26 vs 0.42 -> lever arm cannot explain drift accrued while standing (1.29 m with zero rotation).
- Extrinsic rotation error is not excluded at the ~1 deg level but would not create constant-velocity drift while stationary.

### H2 time offset / timestamps — RANK 3 (minor at most), large offset refuted
- Cloud: 10.0 Hz, header->bag latency median 0.109 s (p95 0.159), per-point timestamp 0.33..95.2 ms relative -> header = scan start (consistent).
- Lidar IMU stamp->bag latency median 1.4 ms, p95 0.21 s, max 0.52 s (bursty delivery; FAST-LIO waits for IMU >= lidar_end_time, so bursts add latency, not error).
- LIO odom stamp->bag latency median 37 ms, p95 0.39 s, max 0.78 s.
- 2-D scan-to-scan ICP yaw vs integrated lidar-IMU gyro (a9.py): best offset +0.025 s (rms 0.0094 rad), flat between 0 and +0.05 s; +-0.1 s clearly worse (0.017-0.023). Header=start + mid-scan = +0.0475 is within the flat region. -> no gross offset; a residual <=50 ms is not resolved (ICP gain 0.81, raw non-deskewed scans).
- LIO yaw matched native within 1-2 deg during the turn, which is incompatible with a large time offset.

### H5 deskew off / wrong point time field — RANK 5, refuted
- AIMA handler sets given_offset_time=true, uses 'timestamp' (float64 ns after header) * 1e-6 -> ms, stable-sorts by time; timestamp_unit 3 = NS. Span 95 ms ~ 1/scan_rate. Deskew is active and fed with correct units.

## 4. Symptom / proximate / root cause
- Symptom: LIO reports 1.675 m pelvis translation and 61.6 deg (native 67.4 deg, 3.2 cm max) for an in-place turn; distance limit tripped.
- Proximate cause: FAST-LIO's ESKF carried a ~0.3 m/s lateral (body-left) velocity error that the lidar update did not correct; 88 % of the drift accumulated while the robot was standing still.
- Root cause (most likely, two factors together):
  1. After the 67 deg turn the narrow (~84 deg H) chest lidar faced a near planar surface + floor; lateral translation became nearly unobservable (min info eigenvalue 6-40 vs 140-200 before; weakest direction = drift direction within 13 deg).
  2. An unmodelled horizontal specific-force error of ~0.18 m/s^2 (horizontal accel bias absorbed into gravity at static init, revealed by yaw rotation, or an equivalent ~1 deg attitude error) pushed velocity along exactly that direction (8.8 deg apart).
  The lever arm (H1), timing (H2) and deskew (H5) are not the cause.

## 5. NOT proven
- Whether the 0.18 m/s^2 is accel bias vs FAST-LIO attitude/gravity-state error (FAST-LIO bias/gravity states not logged).
- That FAST-LIO's actual map correspondences (ikd-tree after 1.7 h) have the same degeneracy as the own-scan normals used here; no effective-feature counts were logged.
- What object was in front (prior note "near objects in front"; data show a frontal planar surface, nonfloor r10 ~3.1 m, not <1 m).
- A replay reproduction: not done. Replay starts with an empty map (live run had a 1.7 h map), so results may differ.
- Sub-50 ms lidar-IMU offset.

## 6. Recommended next steps (offline first)
1. **Offline replay, isolated** (ROS_DOMAIN_ID=89, ROS_LOCALHOST_ONLY=1, nice -n 19, taskset to 1 core, use_sim_time): play trial06 bag topics lidar_pointcloud + lidar IMU at 0.5x into the same binary (63903a5c) + x2.yaml copy (never edit the live yaml; pass a scratch copy). a) baseline: reproduce drift (accept if LIO standing-phase speed > 0.1 m/s); b) add runtime logging of effct_feat_num + H eigenvalues (runtime_logging.patch exists in candidates) to confirm degeneracy on map correspondences; c) parameter variants one at a time: point_filter_num 1, filter_size_surf/map 0.15, max_iteration 4-5; b_acc_cov 1e-3 / acc_cov lower; time_offset_lidar_to_imu +0.025. Measure LIO drift after t=7.9 vs native (0 m).
2. Independent safety gate (no FAST-LIO change): in localization trust, flag LIO untrusted when LIO horizontal speed > ~0.05 m/s while native /aima/mc/leg_odometry says stationary / FSM=STAND (here: 0.3 m/s vs 0.004 m drift) — experimental threshold. This would have caught trial06 within ~1 s after stop.
3. Fix direction (design): fuse native leg odometry (velocity / zero-velocity at STAND) or use a degeneracy-aware update; consider IMU accel-bias calibration of the lidar IMU (rotate robot statically through several yaw headings, passive) — requires operator approval.
4. Passive test (operator approval, no locomotion): stand robot facing (a) open room and (b) a near wall, record 60 s each; compare LIO drift and eigenvalues. Any rotation/motion test only with separate explicit user approval.

## 7. Commands used (all offline)
- python3 extract.py <bag_0.db3> t06.pkl (sqlite + rclpy deserialize)
- a1 cloud stamps/ranges; a2 scan similarity/gravity in lidar frame; a3 LIO latency/velocity; a4 degeneracy eigenvalues; a5 IMU stats/saturation; a6 lever-arm fit; a7 gravity leak + direction comparison; a8 body-frame tilt change; a9 ICP-vs-gyro time offset.
