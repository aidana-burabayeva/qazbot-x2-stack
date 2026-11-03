# 20261021-x2-nav2-e2e-dryrun-locpersist — why X2 localization is "lost" / READY false (debugger, read-only)
Date 2026-10-22 ~20:25. Host .66 (Jetson, humble, load avg 25-27). Nothing was published, set, restarted or killed. No ros2 CLI was used
(two hung `ros2 param get /qazbot_d1_map/amcl ...` processes from another agent, started 20:14 and 20:15, were already present).
Scratch scripts: /tmp/claude-1001/-home-agi/7b3af5b1-2708-4844-bc74-173c51a4dffc/scratchpad/lp/{st.py,tr.py,tr2.py,scan.py}. They read bags read-only through sqlite.

## 1. System reconstruction (relevant part)
- FAST-LIO (candidates/fast_lio_portable, PID 314339, since 19:57) -> level_bridge.py (PID 314357)
  - TF odom->base_footprint at 10 Hz.
  - /qazbot_lio/scan at 5 Hz: a height slice of ONE deskewed cloud, 1440 bins, range_max 8 m.
- nav2_amcl (PID 314380). Live params file /tmp/launch_params_2x1umpt4 holds ONLY frames, topics, max_beams 1440 and OmniMotionModel. Everything else is the navigation2 1.1.18 default (amcl_node.cpp:60-232, source in an earlier scratch snap):
  - alpha1-5 = 0.2, update_min_d 0.25, update_min_a 0.2, resample_interval 1
  - likelihood_field, z_hit 0.5, z_rand 0.5, sigma_hit 0.2, laser_likelihood_max_dist 2.0
  - recovery_alpha_slow/fast 0 (recovery disabled), set_initial_pose false, max/min particles 2000/500, pf_err 0.05, pf_z 0.99
  - NEEDS_DATA: no runtime `param get` was done (CLI hangs and load). Any runtime `param set` would not show up in this file.
- Trust (localization_trust_node.py PID 312093, trust_v3_10, since 19:54; loc_trust.py)
  - READY gate (after confirm_anchor) needs T2 sigma_xy<=0.20 and sigma_yaw<=0.15, T3 cloud_std<=0.25, T4, T5, T6 and T8.
  - After READY, T2/T3/T4/T6 are diagnostics only.
  - READY is cleared by T5 (map->odom step >0.10 m / 0.10 rad), T8, initialpose, or T0/T1 loss at goal request.
- Measured virtual-scan content (scan.py, every 50th scan):
  - cycle2_02: 154/206/227 finite bins of 1440 (p10/p50/p90), angular coverage 90-100 deg, median range 2.5 m.
  - g_motion_03: 135/139/193 bins, 90 deg, median range 4.9 m.
  - So AMCL sees about a 90-100 deg forward sector with roughly 150-230 returns. That is a weak, often degenerate measurement.
- Reference code (QazBot/azat_ws, local_control_system_py, robot_driver): NOT on this host. `find /home /opt /mnt -maxdepth 6` returned nothing. This matches DECISIONS 19:0x: it is on the user's PC. Only docs/RPP_MC_ALIGNMENT_20261022.md (motion notes) is available, and it has nothing on pose persistence.

## 2. Evidence: every recorded READY loss / re-pose today
Sources: trust logs logs/trust_v3_07..10.log, AMCL logs logs/localization_v3_loopfix_0{1,2}.log, and trust status JSON in bags g_motion_01..04, turn_nav_01, short_nav_01, cycle2_01, cycle2_02 (tr2.py).

| # | time | trigger (measured) | class |
|---|---|---|---|
| 1 | 15:17-15:25 | FAST-LIO loop-back patch: localization+trust restart; AMCL "Please set the initial pose" until operator initialpose x2 at 15:25 | stack restart, no pose persistence |
| 2 | 16:05-17:01 | READY held, but AMCL sigma_xy rose 0.166->0.336 during G3 (~1.2 m walk), then 0.28->0.49 and sigma_yaw ->0.35 during pad teleop 16:56-17:01. Anchor err (AMCL vs anchor (+) LIO) was 0.04 m while sigma was 0.34; it reached 0.2-0.4 m only after the spread grew | AMCL covariance growth from motion-model noise |
| 3 | 17:01:28 | T5_map_odom_jump (old 1 s window rule, trust_v3_07 code) while sigma 0.486/0.334 and T4 score fell to 0.19-0.30: AMCL mean wandering inside an inflated cloud | AMCL particle spread -> map->odom steps |
| 4 | 17:52-19:05 | operator initialpose x2 (17:52) -> sigma 0.477-0.507 for ~70 min (only 2 nomotion updates, robot standing, AMCL silent by design) -> "READY false sigma 0.48". Re-pose x5 at 19:04-19:05 + 24 nomotion updates -> READY 19:10 (sigma 0.129) | RViz initialpose covariance (0.25 m^2 -> sigma 0.5) never contracted while standing |
| 5 | 19:11:04 | input_loss_at_goal_request T1_odom/odom_msg/scan stale (FAST-LIO latency) -> confirm_anchor 19:14 (no re-pose) | stale inputs (load), not localization |
| 6 | 19:14-19:18:48 | yaw goal +20 deg: ONE AMCL update sigma_yaw 0.045->0.128, sigma_xy 0.129->0.184, anchor err 0.024 m. Short nav: sigma 0.18->0.38, then T5 at 19:18:48 (t5step report: cumulative 0.12 m/1.7 s window rule) | motion noise + old T5 window rule |
| 7 | 19:23, 19:35 | trust restarts (t5step, t5each code) -> anchor lost; operator re-posed x6 and x2 (resetting AMCL to sigma 0.5) | trust restart; re-pose used as reset |
| 8 | 19:43:08 | input_loss_at_goal_request T1_scan_stale -> confirm_anchor 19:43:46 | stale inputs |
| 9 | 19:48:41-20:09 | PTP clock jump -> level_bridge stall -> localization restart 19:57 + trust restart 19:54. 13 initialposes 20:00-20:05 (one with yaw 0.197 vs about -1.2: jump_latched_yaw 1.54). 30 nomotion updates 20:06:41-20:08:53 brought sigma 0.45->0.17 -> READY 20:09:19 | clock fault -> restart, no pose persistence; operator seeding errors |
| 10 | 20:09:56 | S3b turn: ONE update sigma_xy 0.179->0.267, sigma_yaw 0.044->0.185, anchor err 0.020 m / 0.011 rad. READY kept (T2 diagnostic) but would fail any new confirm_anchor gate | motion noise |

Other facts:
- Every RViz initialpose logs "Failed to transform initial pose in time (extrapolation into the future)". AMCL then uses identity odom motion. This is harmless while standing, but it shows that the odom TF lags `now` by ~100 ms.
- FAST-LIO "lidar loop back" with stamp 0.095 s TF_OLD_DATA occurs several times per session (benign after the fixes).
- path_since_anchor grows ~0.7-1.9 m while standing (LIO jitter integrated). The T6 per-m term must not use path_len as is (anchor_xy_per_m=0 now).

## 3. Q2: does map->odom stay constant when scans are uninformative?
No. AMCL injects motion noise on every filter update (OmniMotionModel, omni_motion_model.cpp):
- trans_sd = sqrt(a3*dt^2 + a4*dr^2), rot_sd = sqrt(a1*dr^2 + a2*dt^2), strafe_sd = sqrt(a4*dr^2 + a5*dt^2).
- Defaults a=0.2, applied per update:
  - 0.25 m step: 0.112 m trans, 0.112 m strafe, 0.112 rad yaw.
  - 0.2 rad (or 20 deg = 0.35 rad) turn: 0.089 (0.157) m in x/y and 0.089 (0.157) rad yaw.
- Predicted vs measured growth for the single 20 deg update:
  - yaw: sqrt(0.044^2+0.157^2) = 0.163 rad predicted; 0.122-0.185 measured.
  - xy: sqrt(0.139^2+0.157^2) = 0.21 m predicted; 0.173-0.267 measured.
- The model therefore explains the growth quantitatively. The narrow ~90 deg, ~200-return scan with z_rand 0.5 does not contract it back.
- The particle mean then wanders inside a cloud of sigma 0.3-0.5 m. map->odom moves even though FAST-LIO is right: AMCL vs LIO-predicted error stayed 0.01-0.04 m over the first metres in turn_nav_01, short_nav_01, cycle2_01 and cycle2_02.
- Sometimes it moves by more than 0.10 m per update or window, which gives T5 and READY lost.
- Measured LIO increment error is 1-2 orders smaller than what alpha=0.2 assumes. The defaults model wheel-odometry-class error, not FAST-LIO.
- recovery_alpha=0 means no random injections, so the jumps are not "recovery jumps".
- No FAST-LIO re-init (odom-frame reset) was seen inside a session. LIO restarts happened only together with full localization restarts (#1, #9).
- NEEDS_DATA: map->odom per-update steps were taken from trust metrics only. tf in bags was not decoded per update.

## 4. Ranked hypotheses
- H1 (CONFIRMED, main cause of "covariance grows / READY gate fails / T5 jumps"): AMCL alpha1-5 left at 0.2 (wheel-odom defaults) on top of FAST-LIO odometry.
  - Supporting evidence: the §3 quantitative match; anchor err small while sigma large; growth happens exactly on update events (turns, steps), never while standing.
  - Contradicting evidence: none found.
- H2 (CONFIRMED, cause of "READY false sigma 0.48" while standing): after an RViz initialpose AMCL starts at sigma 0.5 / 0.26 rad (RViz default covariance).
  - Standing, AMCL updates only on request_nomotion_update.
  - It contracts ~3-10 % per nomotion update (20:06-20:08: 0.45->0.17 in ~30 updates). With few updates (17:52-19:04: 2) it stays ~0.48.
  - The operator repeatedly re-posing (5-13 times per episode) RESETS the contraction each time.
- H3 (CONFIRMED, cause of "must re-pose after every relaunch"):
  - Nothing persists the pose. set_initial_pose=false. Humble only writes last_published_pose into in-process params on deactivate, which is lost on process exit.
  - The trust anchor is in memory. Every localization or trust restart today (#1, #7, #9) forced re-pose / confirm.
- H4 (CONFIRMED, contributing): the virtual scan is weak.
  - Data: ~90-100 deg coverage, 135-230 returns, max 8 m. The likelihood is flat with z_rand 0.5 (z_hit 0.5).
  - Effect: it cannot correct the H1 noise quickly, especially when the operator stands in the sector (T4 drops to 0.2-0.3 coincide).
- H5 (CONFIRMED, separate from localization quality): T1 staleness at goal request (#5, #8) is FAST-LIO latency under load. It is classified as input loss, correctly. It is not pose loss.
- H6 (REFUTED for today): FAST-LIO odom-frame reset mid-session; AMCL recovery jumps (recovery disabled); clock fault as a continuous cause (one incident, #9).

Symptom: operator must re-pose; READY false with sigma ~0.48; T5 READY loss during/after motion.
Proximate cause: AMCL covariance and spread grow per update and are not contracted. The READY gate tests AMCL sigma. Restarts drop the pose.
Root cause:
- (a) AMCL motion model not matched to the odometry source (defaults);
- (b) no pose persistence across restarts;
- (c) the trust READY criterion uses AMCL's own covariance as a proxy for "pose wrong". With (a) this conflates "uncertain" with "wrong".
- The operator re-pose workflow amplifies (b) and (c): each re-pose resets sigma to 0.5.

## 5. Q3/Q4: minimal standard fix plan (one component at a time, each verified by a measurement)
Step A — AMCL motion noise matched to FAST-LIO. Component: localization.launch.py AMCL params only. Standard Nav2 tuning, no new code.
- Derive alpha from data, not by guess:
  - alpha3 = (sd of LIO translation error per metre)^2 per metre^2, alpha1 = (sd of LIO yaw error per rad)^2;
  - alpha2/4/5 are the cross terms, from the same data.
- Measurement, read-only plus operator:
  - Walk a closed loop that returns to a floor mark (or use already-recorded segments where LIO and AMCL agree, before the spread grows).
  - Compute the LIO closure error per metre and per radian. Report p95 per update step (0.25 m / 0.2 rad).
  - Available proxy from today's bags: anchor err <=0.04 m over the first 1-4 m and <=0.03 rad after 20 deg turns. That bounds the LIO increment error to ~1 % of the default noise level. Use it only as an upper bound until the loop measurement exists.
- Also make update_min_d/update_min_a explicit, with the same values passed to the trust node (closes verifier F4 coupling).
- Verify (pass/fail):
  - repeat the +20 deg turn and the 0.5 m goal;
  - per update, sigma growth must be <= the measured LIO error and AMCL-vs-LIO err must stay <= the same bound;
  - no T5 over the same cycle;
  - scan_score must not degrade (an overconfident filter would show T4 drops / anchor err growth).
- Risk: alpha too low gives an overconfident filter that cannot recover from a wrong seed. This is mitigated by the READY gate T4 and anchor checks. Note that recovery_alpha stays 0.
Step B — pose persistence across restarts (standard Nav2 mechanism plus a thin saver). Component: localization launch + a small saver; seed_pose.py already publishes PoseWithCovarianceStamped to /qazbot_d1_map/initialpose.
- Save the last CONFIRMED pose to a file: map pose, covariance and map sha. Save at confirm_anchor/READY and at clean shutdown, not every AMCL output.
- On relaunch, publish it once as initialpose WITH its covariance, via the existing seed_pose.py path.
- AMCL set_initial_pose / initial_pose.* are the native alternative, but humble passes ZERO covariance (amcl_node.cpp:271-280 -> handleInitialPose copies msg covariance = 0). Prefer publishing the saved PoseWithCovarianceStamped.
- Validity: the trust READY gate (T4 scan score + T2/T3 after a few nomotion updates) must pass on the seeded pose before READY. If the robot was moved while the stack was down, T4 fails and the operator re-poses (unchanged fallback).
- Verify: restart localization with the robot standing; READY must be reached with no RViz initialpose. Measure the time and the nomotion count to READY.
- Rejected:
  - robot_localization EKF: there is a single odom source (FAST-LIO). An EKF adds nothing to map->odom persistence and does not address H1.
  - slam_toolbox localization mode: a different map format/pipeline is a bigger change. Keep it as an option only if Step A+C fail.
Step C — trust READY criterion (component: loc_trust.py; after A, separately verified).
- Replace "AMCL sigma <= fixed max" as the sole READY-gate pose check with anchor + bounded LIO drift: pose_ok = |AMCL pose - (confirmed anchor (+) LIO displacement)| <= k * sigma_LIO(path).
  - sigma_LIO(path) = measured LIO drift sd from Step A's loop data (per m, per rad of motion, NOT path_len, which integrates standing jitter: 0.7-1.9 m while standing today).
  - k = the same confidence quantile used elsewhere (e.g. p95 of the measured distribution), so no new free constant.
- AMCL covariance is then a diagnostic of "view uninformative". A disagreement beyond the LIO bound means "pose wrong" (T5/T6 class).
- Re-confirm without re-pose:
  - after input loss (#5/#8), if map->odom is unchanged and LIO is continuous, the existing confirm_anchor path suffices;
  - document that the operator should NOT send initialpose when READY is lost for T1 reasons.
- Verify: replay cycle2_02 / short_nav_01 status inputs offline through the new rule. It must keep READY where anchor err <= bound and must clear it on the 17:01 / 19:18 jumps if their err exceeds the bound (NEEDS_DATA: per-update err at those instants).
Operator procedure (immediate, no code):
- Do not repeat initialpose; one pose, then N nomotion updates until sigma <= gate (20:06-20:08: ~30 updates).
- Keep out of the 90-100 deg forward sensor sector.

## 6. Next diagnostic commands (read-only)
1. `ros2 param get /qazbot_d1_map/amcl alpha1` (and alpha2-5, z_rand, laser_max_range) — ONE each, only when the CLI is not hanging. Confirms that no runtime override exists.
2. Offline: decode /qazbot_d1_map/tf map->odom per AMCL update in short_nav_01 and cycle2_02. Get the per-update step distribution and AMCL-vs-LIO err at 17:01:28 and 19:18:48 (extend tr2.py with rclpy.serialization).
3. Operator loop walk to a floor mark (needs explicit go): measure the LIO closure error per m / per rad. This is the source for alpha and sigma_LIO.
4. `free -m; vmstat 1 5` before trials (T1 staleness H5).

## 7. Open questions for the user (reference code on the user's PC)
- In azat_ws / local_control_system_py, is the last pose saved to disk and re-seeded on start? Which fields (pose, covariance, map id), when (periodic, on stop, on operator confirm), and how is it validated before use?
- Which AMCL (or other localizer) motion-noise params did .89 use with LIO odometry (alpha1-5, z_hit/z_rand, laser model)?
- Does the .89 trust/ready logic gate on AMCL covariance, or on LIO-vs-map consistency? Which thresholds, and how were they derived?
- After a stack restart on .89, does the operator have to re-pose, or is the pose restored automatically?

## 8. Comparison with .89 reloc/trust principles (source: reports/20261021-x2-nav2-e2e-dryrun-ref89-reloc-notes.md; user description, no code)
Map onto standard Nav2/AMCL on .66:
- Persisted state, written atomically: map sha, FAST-LIO run id, T(map<-odom), last map pose and covariance. Adopt as Step B.
  - Same LIO run (stack partly restarted, FAST-LIO alive): map<-odom is still valid. Republish the last map pose = map<-odom (+) current odom->base as initialpose, with the saved covariance.
  - New LIO run (FAST-LIO restarted, odom origin reset): seed from the last map pose. Do not assume the robot did not move: the READY gate (T4 scan score + T2/T3 after nomotion updates; Step C anchor rule) must pass before READY. Otherwise the operator re-poses.
  - Save when READY/confirm_anchor is set and at clean stop. Do not save during unconfirmed or degraded states. Nav2 equivalent: initialpose topic via seed_pose.py, not set_initial_pose (zero covariance in humble).
- Coasting on FAST-LIO when scans are uninformative. On .66 this is what AMCL should do natively once alphas match LIO (Step A): small motion noise means the cloud does not inflate, and map->odom stays nearly constant while weak scans carry little weight. The trust rule (Step C) treats AMCL-vs-(anchor (+) LIO) within the measured LIO bound as OK. That is coasting without a custom localizer.
- Slow / gated correction acceptance (.89: median-of-5 slew of <=1 cm / 0.2 deg per match, local ±0.6 m / 25 deg window, far result accepted only if unambiguous twice). Standard counterparts:
  - AMCL with low alphas plus resample_interval>1 corrects slowly by construction;
  - the trust T5 per-update step limit and the Step C bound gate acceptance.
  - Do NOT add a custom slew filter on map->odom (would be a second localizer). .89's freezing of corrections near the goal has no Nav2 analogue and is not needed if T5 holds.
- Jump detection by message stamps (.89: >2 m/s on stamps -> quarantine). Already present on .66 in a stricter form:
  - T5 per accepted map->odom update (0.10 m / 0.10 rad);
  - stamp validation (SourceStamp);
  - across-gap comparison.
  - Keep the .66 version. The .89 2 m/s rule would miss 0.1-0.5 m false locks.
- Source failsafe (.89: no fresh pose <=1 s -> stop). Present on .66 as T1 staleness -> MOTION_OK false.
Reject / do not copy:
- "Robot did not move" assumption on a new LIO run without a scan check. On .66 it must pass the READY gate.
- Global search every 15 s with score 0.55 acceptance (.89 produced a 9 m / 32 deg false fix at score 0.45). No periodic global relocalization; global_localization only on operator request.
- pose_trusted = fresh + no recent jump only (slow drift undetected; .89 notes it). .66 keeps the anchor + LIO-drift consistency check (Step C).
- No trust gate on the walker (.89 gap). .66 keeps READY/MOTION_OK gating of the adapter.
- Fixed uncalibrated thresholds (5 s, 15 misses, 0.3 m / 5 deg, ±0.6 m). .66 thresholds must come from the measured LIO drift (Step A loop walk).
