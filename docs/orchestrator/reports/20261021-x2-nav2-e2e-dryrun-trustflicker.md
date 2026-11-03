# 20261021-x2-nav2-e2e-dryrun — live trust flicker on a stationary X2 (debugger, read-only)

Date: 2026-10-21 22:14-22:20 (host .66, ROS 2 humble, rmw_fastrtps_cpp, domain 0, FASTRTPS profile
ws/src/navigation/dds_qazbot.xml, use_sim_time false). Nothing was published, no services called, no
processes restarted. Probes: scratchpad gapprobe.py (60 s), chain.py (30 s), analyze.py, moving.py in
/tmp/claude-1001/-home-agi/c9b41d51-5b1b-455f-88cf-eb93053f257e/scratchpad (raw data rec1.pkl, rec2.pkl),
run with the trust node's own environment (from /proc/408263/environ).

## 1. System reconstruction (relevant part)
- FAST-LIO (PID 387621, candidates/fast_lio_portable) -> /qazbot_lio/odometry (stamp = lidar scan time).
- level_bridge.py (PID 387640, single-threaded rclpy) -> TF odom->base_footprint on /qazbot_d1_map/tf
  (10 Hz, one per LIO odom), /qazbot_lio/odom_level (10 Hz), /qazbot_lio/scan (5 Hz, scan_every:=2).
- nav2_amcl (PID 387664) params: ONLY base/global/odom frames, map_topic, scan_topic, max_beams 1440,
  OmniMotionModel (/tmp/launch_params_kfmfdnvb, localization.launch.py:49-54). update_min_d /
  update_min_a / resample_interval are therefore Nav2-humble defaults (0.25 m / 0.2 rad / 1; from Nav2
  docs, not read via param get because services were forbidden). AMCL publishes amcl_pose / particle_cloud
  only on a filter update = odom delta >= update_min_d/a, or request_nomotion_update.
- localization_trust_node.py (PID 408263, map v3): subscriptions + 5 Hz timer in the default
  MutuallyExclusive group, services (arm_check) in a second group, MultiThreadedExecutor(2)
  (localization_trust_node.py:82-91, 227-235). So arm_check does NOT starve the verdict; the tick is light
  (node 10.9 % CPU).
- goal_coordinator: good() = trusted AND trusted-msg age <= trust_max_age_s 0.6 (goal_coordinator.py:37,71).
- Load at measurement: load avg 27-46, top showed 2.7 % idle, kswapd0 43 % (510 MB free, swap in use),
  vendor perception/soc1_hal/nvargus 20-30 % each, plus concurrent `ros2 topic echo` x3, `ros2 action
  send_goal`, and repeated `ros2 service call .../request_nomotion_update` launched by another agent.
  The "54 % idle" in the brief is no longer true during these runs.

## 2. How "moving" and amcl/particle staleness are decided (code)
- loc_trust.py:301-304:
  `T1_amcl_pose_stale` if amcl_cov is None or (inp.moving and now - amcl_stamp > pose_max_age_moving_s=3.0);
  `T1_particle_cloud_stale` if cloud seen and inp.moving and now - cloud_stamp > 3.0.
  `inp.moving` is used nowhere else.
- localization_trust_node.py:158-163 `_moving()`: odom_hist = TF odom->base_footprint poses keyed by
  RECEIVE time (time.monotonic(), line 100/111), 0.6 s window; moving if |dxy|/dt > 0.02 m/s or
  |dyaw|/dt > 0.03 rad/s with dt = receive-time span.
- Ages: SourceStamp.accept (source_stamp.py:12) returns mono_now - (ros_now - header.stamp), i.e. the age
  INCLUDES the upstream source latency (stamp -> receipt). Correct by design.

## 3. Evidence
### 3.1 Stationary pose jitter vs. the 0.02 m/s threshold (60 s, robot standing)
- odom->base_footprint span over 60 s: x 24.3 mm, y 18.1 mm, yaw 0.0039 rad (humanoid standing sway + LIO noise).
- Speed over 0.6 s (source-time stamps): p50 0.0098, p95 0.0245, max 0.0545 m/s; yaw p95 0.0023 rad/s.
  10.1 % of windows exceed 0.02 m/s; yaw never exceeds 0.03 rad/s. -> moving=True is translational jitter.
- Receive-time denominator inflates it further when delivery bunches after LIO lag spikes, e.g. tick
  t=13.87: dt_recv 0.515 s vs dt_src 1.600 s (x3.1); t=31.89: 0.066 vs 0.403 s (x6).
- Replica of _moving() on probe data agrees with the node's reported `moving` on 249/275 ticks.
- Reported moving=True on 37/291 ticks (12.7 %).
- level_bridge's own twist (0.3 s baseline) at standstill: |v| p95 0.029, max 0.055 m/s.

### 3.2 AMCL publication while stationary
- First (latched) amcl_pose was 206 s old at probe start: without nomotion calls AMCL never updates when
  standing, so EVERY moving=True tick immediately gives T1_amcl_pose_stale (+ particle_cloud_stale). This
  matches the brief's 20 s sample (3 x amcl/cloud stale, each followed by 5 s "settling" = 23 ticks).
- During the probe AMCL updated every ~5 s (t = 13.4, 18.4, 23.6, 28.2, 31.4, 36.5, 41.8, 47.2, 51.7,
  56.2 s) — coincides with the other agent's repeated request_nomotion_update calls (seen in ps). Even then
  the 5 s period > 3 s, so moving=True ticks still fail (t=2.06, 5.47, 5.87, 10.46, 29.27, 34.35, 35.67,
  39.27-39.67, 41.77, 50.27, 55.07 all show amcl/cloud stale exactly when moving=True).
- AMCL pose latency (ros_now - stamp) 0.36-2.0 s = scan latency (amcl_pose stamp = scan stamp).

### 3.3 T1_odom_stale / T1_odom_message_stale / T1_scan_stale / T1_tf_stale (real upstream lag)
- 60 s probe (heavy load window): odom_level recv gap max 1.12 s, source latency med 0.31, p99 2.30,
  max 2.81 s; scan recv gap max 1.69 s, latency med 0.32, p99 2.32 s. Status reasons: 92 ticks
  odom+odom_msg+scan stale, 29 + tf stale, 128 settling — almost no trusted ticks in this window.
- 30 s chain probe: IMU (driver) latency med 0.001 s; FAST-LIO /qazbot_lio/odometry latency med 0.237,
  p95 0.375, max 0.788 s; level_bridge adds med 4 ms (p95 13, max 41 ms). => latency is created inside
  FAST-LIO (CPU-starved), not in level_bridge, DDS or the trust node.
- Independent probe vs trust-node reasons (age>0.5 s): odom 123 both / 2 probe-only / 10 node-only /
  151 neither; scan 145/2/8/131 => ~95 % agreement, so this staleness is real input age, not trust-node
  executor latency. Trust status publication gaps: max 0.565 s, 0 gaps > 0.6 s (no starvation seen).
- With odom_max_age_s 0.5 and FAST-LIO latency p95 0.375 + up to 0.1 s period, the margin under the current
  load is ~0; these withdrawals are CORRECT behaviour of the gate (stale LIO), not a trust bug.

### 3.4 Side observation
- One odom->base_footprint TF on /qazbot_d1_map/tf with header.stamp = 0.095 s (sec=0) at mono 33084.48,
  pose consistent with LIO. SourceStamp rejects it (odom_t=-inf until next TF), but _tf still stores the
  pose and the `a < src` filter wipes odom_src history (localization_trust_node.py:107-111). Publisher not
  identified (ros2 topic info -v did not finish discovery under load). Open question; minor.

## 4. Ranked hypotheses
H1 (CONFIRMED, cause of the amcl/particle flicker): `moving` is a false positive on a standing humanoid
   (LIO translational jitter p95 0.0245 m/s > 0.02 m/s threshold, amplified by receive-time dt), and the
   amcl/cloud freshness rule "age > 3 s while moving" is semantically wrong because AMCL does not update
   by time but by odom displacement (update_min_d 0.25 m / update_min_a 0.2 rad). Standing: amcl_pose is
   minutes old -> any moving tick fails. Contradicting evidence: none.
   Also wrong for real slow motion: at v < 0.25/3 = 0.083 m/s (or w < 0.067 rad/s) AMCL legitimately
   stays silent > 3 s -> false withdrawal during approach/slow turns.
H2 (CONFIRMED, cause of odom/scan/tf stale flicker): real FAST-LIO output latency spikes under CPU/memory
   saturation (kswapd 43 %, load 27-46, extra ros2 CLI processes). Trust gate is correct to withdraw.
H3 (secondary, unconfirmed): coordinator good() needs a trusted msg <= 0.6 s old; trust publish gap max
   0.565 s observed -> under heavier load a gap > 0.6 s could also yield "trust=False" with trusted=True.
   Not observed in data; rejection at 1792591997.6 has no recorded status context.
H4 (REFUTED): trust-node executor starvation by arm_check/scan scoring — services are in a separate
   callback group on a 2-thread executor; probe and node agree on staleness; no publish gap > 0.6 s.

## 5. Symptom / proximate / root cause
- Symptom: verdict flickers trusted -> untrusted -> 5 s settling on a stationary X2; goal rejected trust=False.
- Proximate: (a) T1_amcl_pose_stale + T1_particle_cloud_stale on ticks where moving=True;
  (b) T1_odom/odom_message/scan(/tf)_stale when FAST-LIO lags (age = latency + gap > 0.5 / 1.0 s).
- Root: (a) freshness of AMCL outputs is gated on a time limit conditioned on a noisy velocity flag
  instead of on the event that makes AMCL publish (odom displacement since the last filter update);
  (b) CPU/memory saturation of the Jetson delays FAST-LIO (system resource, not trust logic).

## 6. Recommended fix (minimal, no threshold bump)
Semantic: "amcl_pose / particle_cloud must be fresh only after AMCL was due to update", i.e. after LIO
odometry has moved >= update_min_d or turned >= update_min_a since the odom pose of the last AMCL update.
A. ws/src/navigation/loc_trust.py
   - TrustConfig (after line 71): `amcl_update_min_d: float = 0.25`, `amcl_update_min_a: float = 0.2`
     (must equal AMCL's update_min_d/update_min_a; the launch does not override the Nav2 defaults — better:
     set them explicitly in localization.launch.py:51-54 and pass the same values to the trust node).
   - Inputs (line 227-244): `odom_at_amcl: tuple = None`, `odom_at_cloud: tuple = None` (odom->base at
     the AMCL update).
   - evaluate, replace lines 301-304: due_pose = displacement(odom_at_amcl, odom_base) exceeds min_d or
     |dyaw| exceeds min_a; keep `self.amcl_due_since` (set on first due tick, cleared when amcl_stamp
     changes); `T1_amcl_pose_stale` if amcl_cov is None or (due and now - amcl_due_since >
     pose_max_age_moving_s). Same for the cloud with odom_at_cloud / cloud_stamp. Existing 3.0 s keeps its
     meaning as "max AMCL lag after an update became due"; it must still be calibrated (AMCL latency
     measured here 0.36-2.0 s under load, so 3 s is not generous). `inp.moving` no longer gates safety.
B. ws/src/navigation/localization_trust_node.py
   - _amcl (129-132) / _cloud (134-138): store the odom pose AMCL used: interpolate odom_src at the
     amcl_pose header stamp (= scan stamp; AMCL pose latency up to 2 s -> extend odom_src history from 1 s
     to >= pose_max_age_moving_s, line 110), fallback to self.odom at receipt. particle_cloud is stamped
     with AMCL's now(), so use the odom at receipt (or the paired amcl_pose stamp).
   - _inputs (165-171): pass odom_at_amcl / odom_at_cloud.
   - _moving (158-163, informational after the fix): use source stamps (odom_src) instead of receive time.
   - _tf (107-111): do not store pose / wipe odom_src when the stamp is rejected (side issue 3.4).
C. Not a code fix for H2: reduce load before live goals (stop ad-hoc `ros2` CLI echo/service loops,
   watch kswapd/memory) and re-measure FAST-LIO latency; only then discuss odom_max_age_s with data.

## 7. Unit tests to add (tests/loc_trust/test_trust_gaps.py, using make_inputs)
1. test_standing_jitter_with_old_amcl_is_trusted: now=10, amcl_stamp=cloud_stamp=1, odom_at_amcl=pose,
   odom_base=pose+(0.024, 0.018, 0.004) (measured 60 s standing span), moving=True -> no T1_amcl_pose_stale
   / T1_particle_cloud_stale.
2. test_slow_motion_below_update_min_d_is_not_stale: 0.20 m over 10 s with no AMCL update -> not stale.
3. test_amcl_silent_after_due_displacement_is_stale: odom moves 0.30 m (> 0.25) at t=10, no new amcl_stamp;
   evaluate t=10 (not yet), t=10+pose_max_age_moving_s+0.1 -> T1_amcl_pose_stale and T1_particle_cloud_stale.
4. test_amcl_silent_after_due_rotation_is_stale: yaw 0.25 rad (> 0.2) -> stale after the lag.
5. test_new_amcl_update_clears_due: after case 3, new amcl_stamp + odom_at_amcl=current -> clean.
Update existing test_trust_gaps.py:38-46 (stale amcl/cloud "while moving") to the displacement form.
ROS-level: in run_ros_synthetic/ros_e2e_synthetic, publish TF with 12 mm 0.5 s jitter + latched old
amcl_pose and assert trusted stays True for 30 s.

## 8. Next diagnostic commands (read-only)
- `ros2 param get /qazbot_d1_map/amcl update_min_d` / `update_min_a` / `resample_interval` (service call —
  needs authorization) to confirm defaults 0.25 / 0.2 / 1.
- Re-run gapprobe.py 60 s with no concurrent ros2 CLI loops; compare FAST-LIO latency and moving rate.
- `ros2 topic info -v /qazbot_d1_map/tf` (with daemon) to identify the stamp-0 odom->base_footprint publisher.
- `free -m; vmstat 1 10` to quantify swap pressure (kswapd 43 %).

## 9. Open questions
- Who sent the periodic request_nomotion_update during the probe (another agent?) — it masks H1 partially.
- Exact trust status at the 22:13 (1792591997.6) rejection is not recorded; H1 vs H2 vs H3 for that goal
  cannot be distinguished retroactively.
- Source of the stamp-0 TF.
