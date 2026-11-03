# Physical Nav session 2026-11-02 (X2 #1, 10.0.1.41 / wifi 192.168.1.72)

Evidence dir: `docs/evidence/nav_20261102_live/` (start_loc.sh, start_nav_dry.sh, stop.sh, stop_nav.sh, logs, probes).

## Baseline
- Installed tree `~/control_backend/ws` is NOT a git checkout; clone `qazbot-x2-stack` on `nav/first-goals-20261028` @982f618 (2 dirty files, not installed).
- Hashes: x2_motion_adapter.py blob e54d6c7f (sha256 c76ac1d1, P23); map.yaml blob 7ccad24e; map.pgm sha256 72ee4094…b354 (match).
- Installed = 30.10 post-P26-rollback state (rollback_verified.json sha256 match for 5 files).
- ROS 2 Humble, FastDDS, ROS_DOMAIN_ID 0, process profile dds_nav_two_iface_log.xml.
- MC: STAND_DEFAULT, input source rc prio 80. /aima/mc/locomotion/velocity publishers: 4 vendor only.
- Effective params (snapshot 30.10 actual_parameters + start_nav.sh): xy_goal_tol 0.35, yaw_goal_tol 0.17, max_linear 0.3,
  max_angular 0.6, stateful goal checker; Trust go2, jump_xy_max 1.0, jump_yaw_max 0.5; AMCL beams 180, update_min .10/.05,
  alpha 0.3/0.2/0.15/0.15/0.15, likelihood_field; odom /qazbot_native/odom (native_imu owns odom->base_footprint); adapter yaw_loop=true.
  Older handoff 0.20/0.35 superseded by live change 30.10 (operator_actions 1793353273, 1793350646).

## Stage A preflight — PASS with findings
- Lidar 10 Hz age p50 0.105 s, IMU 200 Hz, n_future 0. PTP s2 now; clock jump/SYNC_FAULT at 16:43:48 (10 min before start).
- CPU load 8–11 on 8 cores. Vendor camera/perception stack running (was trimmed on 30.10).
- `qazbot-speech.service` (user unit) crash-loops: KeyError SONIOX_API_KEY, restart #1410, every ~7 s.

## Stage B dry-run — running
- 16:55 localization+trust (start_loc.sh), adapter live=false + nav_full allow_dry_run_driver (start_nav_dry.sh).
- TF on /qazbot_d1_map/tf: publishers amcl + qazbot_native_odom_bridge only. Scan/obstacles/clearing: level_bridge only.
- Adapter: dry_run, published 0, no MC publisher, foreign_publishers []. Nav2 waits for initial pose (expected).
- Defect D-01 (hypothesis): stream stalls. /qazbot_lio/scan age p99 0.84 s, gaps up to 0.73 s; odom gaps 0.47–0.67 s.
  Gaps follow each speech-service restart by 3.7–4.6 s (gap_probe_baseline.txt). Trust shows T1_scan_stale.

### D-01 fix 1: stop qazbot-speech (operator OK 17:02)
| Metric | Before | After | Result |
|---|---|---|---|
| scan age p99 | 0.84 s | 0.16 s | better |
| scan gaps >0.3 s / 40 s | 7 (max 0.73) | 1 (0.37); 15 s window max 0.44 | better |
| odom rate | 26.1 Hz | 29.9 Hz | nominal 30 |
| odom age p99 / gap max | 0.12 / 0.67 s | 0.048 / 0.11 s | better |
Root cause confirmed by A/B on live robot: speech crash-loop (python start+traceback each ~7 s) starves nav processes.
Rollback: `systemctl --user start qazbot-speech`. Residual: single scan drop 0.37–0.44 s (< T1 0.5 s), vendor camera stack ~140% CPU.

### D-01 controlled 60 s measurement, speech OFF (freshness_speech_off_1.json, 17:2x)
scan 5.00 Hz, age p50/p95/p99/max 0.046/0.057/0.087/0.158 s, gap max 0.293 s, gaps>0.3 s: 0.
odom 29.93 Hz, age p50/p95/p99/max 0.008/0.016/0.030/0.071 s, gap max 0.130 s, gaps>0.3 s: 0.
Trust T1 (excluding pre-seed T1_tf_stale): 0 events. CPU busy p50 51 %, max 89 %; loadavg 6.6.
Speech ON baseline (gap_probe_baseline.txt): 7 gaps >0.3 s in 40 s, max 0.73 s; each 3.7–4.6 s after a service (re)start.
Status: STRONG A/B + timing correlation; reverse test (speech ON again, same probe) pending operator OK for causal closure.
Speech unit: Restart=always/3 s, enabled; env file ~/.config/qazbot_speech/env MISSING (not just a key) -> will crash-loop after every reboot.
Proposed permanent fix (config only, no retries/timeouts): user drop-in `ConditionPathExists=%h/.config/qazbot_speech/env`.

## Lab candidate registry (robot/stable 1212d7ed vs lab/stable 55ad923e; ws/src diff = maps only)
| ID | Defect | Repro before | Change | Class | Prod ws/src | On X2 | Needs for HW_VERIFIED |
|---|---|---|---|---|---|---|---|
| LAB-P20 | turn stall / low wz | yes (HW 29.10 P19) | adapter YawRateLoop | SIM_VERIFIED | yes, already | P23 installed (c76ac1d1) | after-walk + in-goal turns on X2 |
| LAB-EPS | L2 stall s1_0046 | sim only | QAZBOT_SIM_YAW_LIN_EPS | sim-only, causal OPEN | no | n/a | not transferable |
| LAB-TOL | CCW loop at 0.10/0.10 | sim | tol 0.20/0.35 in twin | sim alignment | no | X2 uses 0.35/0.17 (lab doc stale) | n/a |
| LAB-AMCLR | replay T5 false trips | yes offline | replay metric epoch reset | OFFLINE tooling | no | n/a | n/a |
| LAB-AMCL4 | replay readers lose seeds | 14 FAIL offline | readers/clock align | OFFLINE tooling, review pending | no | n/a | n/a |
| LAB-EP0 | 0.445 m/14.8° post-seed move | 1/13 episodes | none (diagnosis: bad seed, go2 nomotion helper corrects) | diagnosis | no | n/a | check on live seed |
| LAB-TWIN02 | odom reset recovery | sim/DDS | candidates/native_odom_reanchor | EXPERIMENTAL | no | NOT to install | physical evidence provider |
Verdict: **NO_DEPLOYABLE_LAB_FIX**. Highest-priority unconfirmed defect for a production candidate: D-01 (config), then P1 after-walk turn stall with P23.

### D-01 permanent fix (operator decision 2026-11-02)
`systemctl --user disable qazbot-speech` (service not needed). Drop-in not used; reverse A/B test skipped by operator.
D-01 status: FIXED on X2 (stopped+disabled); causal attribution = A/B before/after + restart-timing correlation (reverse test not run).
Rollback: `systemctl --user enable --now qazbot-speech`. Re-check freshness after next reboot.

## Stage C — localization, seed 1 (17:35:46, operator RViz)
AMCL set pose (-1.947, -0.953, -1.660 rad); "Failed to transform initial pose in time" (known future-TF lookup, stationary = harmless).
Trust 60 s: ready=True, motion_ok=True, BUT scan_score 0.10–0.11 (floor 0.35), sigma_xy 0.51, second_mode 0.59, no amcl_pose updates (stationary).
FINDING S-01: go2 admission (post-P26 rollback) grants READY/motion_ok with scan_score 0.10 — scan does not match map. Not moving on this pose.
seed_fit.py (5 live scans, Trust soft_score, ±1.0 m / ±0.6 rad): seed 0.113; best 0.407 at (-1.60, -0.05, -110.6°) = 0.97 m / -15.5° off,
competing modes 0.39 (-2.35,-1.15,-98°), 0.36, 0.36 -> AMBIGUOUS (narrow FOV, ~167 endpoints/scan). Evidence: seed_fit_1.txt, trust_watch_seed1.txt.

## OFFICE vs CORRIDOR experiment (operator request 17:5x)
OFFICE baseline snapshot `baseline_office_snapshot.json` (current operator seed, stationary):
map->base (-2.46, -1.05, -92.8°); Trust ready/motion_ok TRUE; scan_score 0.27; sigma_xy 0.51; second_mode 0.83;
Trust ambiguity: best 0.44 at dyaw -3°, competitor 0.39 at (+0.5, +0.9, -11°) -> not flagged ambiguous by margin, but 2nd-mode weight 0.83.
AMCL params live: beams 180 likelihood_field, z_hit .5 z_rand .5 sigma_hit .2, beamskip off, recovery_alpha 0/0, 500–2000 particles.
Native bridge: no fault, not latched. Latch risk: MC origin reset (|dp|>0.05+1.5·dt or |dyaw|>0.3) e.g. stand/sit/mode change -> LATCHED until restart.
Recorder: oc_bag_175920 (mcap, 13 light topics, hard stop 1800 s).

### Transit OFFICE -> CORRIDOR (operator RC, 18:07, oc_bag_175920 + oc_bag_175920_analysis.json)
MC: STAND_DEFAULT/LOCOMOTION_DEFAULT, source rc only. native bridge: no fault/latch. No new initialpose (last seed 17:39:30 (-2.66,-1.11,-94.3°)).
Walk 26 s. Net displacement: native 8.04 m / -124°, AMCL 8.10 m / -106°, FAST-LIO 8.80 m (3D; LIO world x≈up). No physical ground truth.
map->odom: yaw rotated 113° -> 128–132° during t=+2..+8 s (≈17° heading correction), then stable ±4°; max single step 0.62 m / 9.7°; 22 steps >0.1 m; total 1.26 m.
Trust: READY 100 %, motion_ok 99.8 % (T1_scan_stale 4, T1_odom_stale 2); no T5 (limit 1.0 m / 0.5 rad).
scan_score median: office 0.23 (2nd mode 0.83) -> walk 0.61 -> corridor 0.58 (2nd 0.37, sigma .27); stationary corridor 20 s stable 0.55–0.56.
Corridor ambiguity: nominal 0.59, best 0.90 at (0, -0.2 m, -5.7°), competitor 0.70 at -0.5 m along y -> unique but AMCL ~0.2 m/6° from best fit (no stationary update).
Interpretation: office seed heading ≈17° off (consistent with seed_fit_1 −15.5°); AMCL corrected it once corridor walls were observed. Office map-mismatch vs seed error NOT separated yet.
S-01: by design — go2 (loc_trust.py header, _verdict, arm_check) makes T2–T7 diagnostic; arm-time ambiguity() disabled. Existing ambiguity() with margin .05 would NOT have flagged the office pose (competitor .387 vs .389) -> a criterion must be validated on recorded data before proposal.
Load note: 1 T1_scan_stale with recorder (36 % core) + probes running.

## First Nav2 goal in corridor — preparation (18:1x)
Operator RViz screenshot: corridor scan on walls (right side matches; left points enter unmapped rooms / free area = unmapped objects).
Pre-goal snapshot `snap_corridor_prego.json`: map->base (-7.70, -7.34, 150.2°); Trust ready/motion_ok; score 0.45; sigma .27; 2nd .37;
independent scan fit: best 0.95 at offset (+0.1, -0.1 m, -5.7°), competitor 0.67 at +0.5 m -> unique (margin 0.28). AMCL ≈0.14 m / 6° from best fit.
Stationary drift: map->base moved 0.10 m since 18:07 with no AMCL update (below update_min_d) -> native odom creep while standing.
native bridge: no fault/latch. Recorder: nav_bag_181810 (23 topics incl. action status/feedback, cmd chain; no costmaps), hard stop 1800 s.
Live switch procedure (same as p20_restart_adapter.sh 30.10, minus DELETE: dry adapter never registered): MC STAND_DEFAULT+rc, GC idle,
adapter idle -> SIGINT dry adapter -> start live adapter (P23 c76ac1d1, prio 30) -> ONE register. GC keeps allow_dry_run_driver=true (live path: live&registered&ready).

### D-03 freshness regression with monitoring (18:2x) — 30 s freshness_probe each
| Config | scan age p99 | scan gaps>0.3 | odom rate | T1 events |
|---|---|---|---|---|
| 17:2x no monitors (speech off) | 0.087 | 0 | 29.9 | 0 |
| recorder + joints 500 Hz + IMUs + status_logger + goal_watch | 0.97 | 1 | 29.7 | 5 |
| same, no IMUs | 0.75 | 4 | 28.6 | 10 |
| no joints (light) | 1.52 | 5 | 29.1 | 19 |
| light, monitors pinned 0-3 nice 10 | 1.20 | 8 | 25.4 | 1 (+ lidar loop-back 18:25:19) |
| 18:27 all monitors stopped | 0.70 | 0 | 30.1 | 4 |
Conclusion: joints were not the main load; system degraded since 17:2x even without monitors (CPU busy 51 -> 54-62 %).
Vendor camera chain (hal_camera, camera_container, nvargus, perception) ≈130 % CPU; trimmed on 28.10/30.10 with operator OK via `aima em stop-app`.
D-02: FAST-LIO "lidar loop back, clear buffer" at 17:00:04 and 18:25:19 -> one scan with stamp 0.095 s, level_bridge cache clear; recovered.
Monitoring redesign: one recorder (no 500 Hz joints/IMUs; pelvis via /aima/mc/body_pose 48 Hz + leg odom), status_logger without `top` subprocess.

### D-03 fix: vendor trim (operator OK 18:28) — `aima em stop-app` perception, orbbec_camera, hal_camera, drp, lx_webrtc_app
Kept: MC, hal_sensor_orin (lidar/IMU), pnc, slam/map_manager/auto_integrated. Restart: `aima em start-app hal_camera|orbbec_camera|perception|drp|lx_webrtc_app`.
| Metric | Before (no monitors) | After trim | After trim + light monitoring |
|---|---|---|---|
| CPU busy p50 | 54 % | 24 % | 30 % |
| scan age p99 | 0.70 s | 0.048 s | 0.096 s |
| scan gaps >0.3 s | 0 | 0 | 0 |
| odom rate | 30.1 | 28.3 | 30.1 Hz |
| T1 events / 30 s | 4 | 0 | 0 |
MC STAND_DEFAULT / rc after trim. Light monitoring: nav_bag_183029 (24 topics, no 500 Hz joints/IMUs) + status_logger (2 Hz, /proc/stat).

### DRY-1 — dry-run NavigateToPose (operator RViz goal 18:32:56; nav_bag_183029, nav_bag_183029_nav_analysis.json)
Goal (-9.44, -2.89, 85.7°); start AMCL (-7.72, -7.43, 149.8°); straight 4.86 m.
Plan 5.31 m, 101 poses; min distance plan->map wall 0.87 m = 0.42 m beyond robot_radius 0.454. 61 plans (BT replan rate).
Chain consistent: cmd_vel_nav = cmd_vel = cmd_vel_authorized = adapter dry_run_cmd: vx 0.30, |wz| ≤ 0.31 (mean 0.29) — curve walking, no in-place rotation;
P23 yaw loop inactive at vx 0.3 (|vx| < 0.02 condition) -> walking turns go open-loop to MC (the after-walk turn weakness from 29.10 applies).
No our publisher on /aima/mc/locomotion/velocity; MC STAND_DEFAULT/rc; native speed 0 (robot static). Trust score median 0.35, no events.
Nav2 "Failed to make progress" every 15 s (expected: no motion) -> stopped via goal_coordinator/stop; GC latched operator_stop (needs explicit resume).
Event monitor (tail of nav_full/adapter/trust/localization logs) armed so goals and failures are caught without operator notice.

### LIVE-1 — first live NavigateToPose (operator OK; nav_bag_183911, nav_bag_183911_nav_analysis.json)
Live switch 18:37:48 (go_live.sh): adapter P23 live prio 30, source sdk_qazbot_nav2_1102a registered; GC resume; goal (-9.44,-2.89,85.7°) 18:39:27.
t+0..10 s: RPP "detected collision ahead" 201× (20 Hz), collision arc 0.15 m, cmd 0, robot static; operator: front physically EMPTY.
t+10 s: "Controller patience exceeded" -> BT ClearEntireCostmap local+global -> collision gone immediately -> robot walked.
t+10..18 s: cmd vx 0.26–0.29, measured native vx 0.40–0.43 (≈1.5× command), wz cmd ≤0.33. AMCL 12 map->odom steps >5 cm, total 0.62 m; Trust score med 0.62, 0 events.
t+18 s: orchestrator stopped goal (goal_coordinator/stop) — PREMATURE, misread collision log as stalled robot. Final AMCL (-9.12,-4.22, 84.5°): 1.36 m short, yaw -1.2°.
Path native 3.79 m in 18 s; plan 5.36 m, clearance 0.42 m beyond radius. MC LOCOMOTION_DEFAULT under sdk source during walk, back to STAND_DEFAULT.
D-04 (HYPOTHESIS): stale lethal marks ~0.15 m ahead in local costmap not cleared by raytrace (near blind zone / person trail); only BT full clear removes them -> 10 s start delay.
D-05 (observation): measured forward speed 1.5× commanded (MC or leg-odom scale) — not yet separated (AMCL displacement check pending).

### D-05 analysis (nav_bag_183911 = LIVE-1; nav_bag_183029 is the dry-run and has no motion)
Walking window 18:39:37.3–18:39:45.2 (7.9 s): cmd_vel_authorized vx mean 0.271 (integral 2.14 m).
Adapter publish path at |vx| ≥ 0.02 is passthrough (x2_motion_adapter.py: P20/P23 yaw loop only for |vx|<0.02, vy=0); MC input = authorized cmd clamped to MC limits.
MC velocity topic NOT captured by recorder (QoS incompatible: adapter publishes best-effort; adapter log "incompatible QoS") -> gap to fix in recorder, not in adapter.
Measured: native odom pose 2.84 m net / 7.8 s = 0.364 m/s (twist mean 0.375); /aima/mc/leg_odometry pose 2.87 m / 7.9 s = 0.364 m/s.
Independent scale check (oc_bag_175920, RC walk 26 s): native net 8.04 m vs FAST-LIO (lidar, independent of legs) 8.80 m -> leg odom does NOT over-read (if anything -9 %).
Historical (lab real_mc_response_summary.json, 30.10): cmd .203->.241, .245->.351, .300->.427/.406, .800->.454 (saturates ~0.45).
ROOT CAUSE (HYPOTHESIS, strongly supported): MC locomotion tracks forward command with gain ~1.2–1.45 in 0.2–0.3 m/s and saturates ~0.45 m/s; not an estimator error.
Missing for CONFIRMED: physical distance reference (tape) — no external reference in any bag.
FIX CANDIDATE (not applied): measured inverse command map in adapter (feed-forward, like P20 u_ff), only after tape-confirmed curve. No feedback loop, no gain change now.

### D-04 analysis
Clearing source bounded_rays (level_bridge.clearing_endpoints): FOV −46°..+44.5°, RANGE_MIN 0.45 m; marking source scan_obstacles (marking only).
Footprint front corners (0.17, ±0.229)/(0.008, ±0.34) lie at bearings ≥53°: cells there and in the ±45–90° sectors are never re-observed -> never cleared.
LIVE-1: collision arc 0.15 m for 10 s while robot static; disappeared exactly at BT ClearEntireCostmap (18:39:37.21); not re-marked after clear (robot walked 8 s, no collision warnings).
Costmap was not recorded in LIVE-1 (excluded for CPU) -> exact blocking cells UNKNOWN for LIVE-1.
Live snapshot 18:5x (costmap_near_1.txt, robot static): 49 cost≥99 cells within 1 m; LETHAL (100) cells at (0.54,0.58),(0.50,0.62),(0.57,0.61): bearing 47–51°, r 0.79–0.84 m = just outside clearing FOV, front-left.
TF / sensor origin / stamps: no T1 events in LIVE-1, map->odom steps unrelated to start window; not implicated (no evidence for TF error).
ROOT CAUSE (HYPOTHESIS): marks from a real object (person) at the FOV edge persist after it leaves because no clearing ray covers |bearing|>45°; footprint corners overlap that blind sector.
D-04 follow-up (18:4x–18:50): operator confirmed she stands BEHIND the robot, far away.
Code fact: level_bridge.on_cloud publishes marking points (scan_obstacles) over ALL bearings (no FOV filter); clearing_endpoints only −46..+44.5° -> any return beyond +44.5/−46° marks but can never clear.
costmap_near_2.txt: 9 LETHAL cells within 1.5 m, all on a ray at bearing +44.8..+49.7°, map pos (-9.69..-10.11, -3.60..-3.04), 0.29–0.99 m from map walls (not mapped structure).
Diagnostic local costmap clear 18:50:20 (robot static): within ~30 s LETHAL re-marked at the SAME map points (-9.73,-3.63) bearing +49.1° and (-9.95,-3.29) +45.8° -> real lidar returns beyond the clearing FOV (not a stale artifact of a departed person).
=> Mechanism CONFIRMED: marking FOV (≈±52°) ≠ clearing FOV (−46..+44.5°). Returns in the 44.5–52° sectors persist forever; transient objects there (people) leave permanent marks; real objects there are never re-verified.

### LIVE-2 — straight 3 m goal for D-05 (nav_bag_185510, live2_d05.txt, live2_action.out)
Goal pose topic publish was lost twice (ros2 topic pub --once / script: no GC log at all) -> sent via GC action server /qazbot_d1_map/navigate_to_pose (same gate) 18:57:28.
t+0..10 s: RPP collision ahead (D-04 again, operator: nothing physical; glass door front-left suspected reflections) -> patience -> BT full clear -> walk 7.2 s -> SUCCEEDED 18:57:46.
Command integral (cmd_vel_authorized) 1.899 m; MC-published (/aima/mc/locomotion/velocity, now recorded via best_effort QoS override) 1.898 m, vx 0.273 — adapter passthrough confirmed.
Measured net: leg odom / native 2.865 m; FAST-LIO 3.109 m horizontal (3.117 3D); AMCL 3.039 m (not independent).
Ratio actual/command: legs 1.51, LIO 1.64. Tape measurement by operator: PENDING.
Tape (operator, pencil foot marks): start feet staggered (right behind, left ahead); after walk right foot 3.24 m, left 2.97 m -> body (foot midpoint) ≈ 3.105 m; slight drift to the left (not measured).
| Source | Distance | vs tape |
|---|---|---|
| Tape (foot midpoint) | 3.105 m | — |
| FAST-LIO | 3.109 m | +0.1 % |
| leg odom / native | 2.865 m | −7.7 % |
| commanded integral (MC input) | 1.898 m | −39 % |
D-05 ROOT CAUSE: CONFIRMED — MC walks ≈1.64× the commanded forward speed at cmd 0.27 m/s (MC input recorded = authorized cmd). Not an estimator error.
Secondary finding D-06: leg odometry under-reads distance by ≈8 % (FAST-LIO matches tape); affects native odom (Nav2/AMCL odom) scale.
Historical 30.10 "measured" values were leg odom -> real speeds ≈8 % higher than listed.
FIX CANDIDATE (not applied): measured inverse forward-speed map in adapter (feed-forward, like P20 u_ff) -> needs ≥3 tape points across 0.1–0.3 m/s; until then each map point is extrapolation.

## Evening 2026-11-02: D-04/D-05 offline
- Withdrawn conclusion: the earlier claim that marks re-appeared after the 18:50:20 local costmap clear (real returns beyond the clearing FOV) is withdrawn. /costmap was not republished while the robot was static, so the observation does not show re-marking.
- Two static recordings: 204153 (during lab cleaning, uncontrolled; used as real data: 10 edge events, D-04 reproduced for baseline) and 211835 (controlled protocol 21:18:35-21:23:35, but the LiDAR saw 0 returns < 1 m, so no events: not effective, scenarios stay NOT_COVERED; re-record with an on-the-spot check of /qazbot_lio/scan).
- Sit/stand health check (~21:45): processes unchanged, adapter live but idle and untrusted, tilt change 0.24 deg (< 0.5 deg), no floor ring in costmap, trust awaiting initialpose/AMCL.
- Stack stopped via stop.sh at ~22:00.
- Feedback branch robot/feedback-d04-d05-20261102 pushed (reports, harness tests/d04, candidate C, record script); robot/stable untouched.
- Results: see docs/engineering/D04_D05_CPP_REPORT.md and candidates/d04/CANDIDATE.md (candidate C OFFLINE_VERIFIED, not deployed).
