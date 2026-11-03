# 20261021-x2-nav2-e2e-dryrun — G3 debug (offline bag g_motion_01)

Read-only. Source: /home/agi/control_backend/logs/bags/g_motion_01 (bag t = 1792656322.1 … 670.3; all times below = t − 1792656000).
Joints npz was NOT produced: tests/footprint/joint_record.py crashed at start (logs/rec_g_motion_01_joints.log:
`RuntimeError: Unable to convert call argument to Python object` in take_message), so there is no joint data.
Extraction scripts (scratch, not part of repo): /tmp/claude-1001/-home-agi/c9b41d51-5b1b-455f-88cf-eb93053f257e/scratchpad/{ext,a1..a10}.py
(ext.py → g1.pkl; run with ws/src/navigation/env.sh + aimdk_env.sh sourced).

Coordinator correction applied: the robot DID walk (~1.15 m). Original "did not move" premise is superseded.

## 1. System facts used (verified)
- Controller params actually loaded (/home/agi/.ros/log/qazbot_ctl_67nimooi.yaml, running controller_server PID 97962):
  RPP desired 0.3, rotate_to_heading true, rotate_to_heading_min_angle 0.6, rotate_to_heading_angular_vel 0.5,
  **max_angular_accel 1.0**, controller_frequency 20 → per-cycle angular step 0.05 rad/s; use_collision_detection true,
  max_allowed_time_to_collision_up_to_carrot 1.5; approach_velocity_scaling_dist 0.6, min_approach 0.12;
  **goal_checker xy 0.10 / yaw 0.10**; progress_checker 0.1 m / 15 s; odom_topic /qazbot_lio/odom_level; robot_radius 0.454.
- Velocity smoother OPEN_LOOP, accel [0.4,0,1.0], decel [-0.6,0,-1.5].
- Local costmap: odom frame, 4×4 m rolling, obstacle_layer on /qazbot_lio/scan (marking+clearing, inf_is_valid default = false).
- /qazbot_lio/scan (lio/level_bridge.py): 1440 bins, frame base_footprint, 5 Hz, rmin 0.45 rmax 8.0; empty bins = +inf.
  Typical frame: 207 valid / 1233 inf (85 % of beams inf).
- MC: speed_mode LOW(1), fwd bounds −1.0…0.8 (no saturation occurred).

## 2. MC velocity topic content
| source | msgs | nonzero | window |
|---|---|---|---|
| rc (vendor, 0 values) | 16786 | 0 | whole bag |
| sdk_qazbot_nav2 (ours) | 953 | 943 | 426.54–427.30, 624.14–642.46 |
Ours: 49.9 Hz, header.stamp ≈ receive time (p50 0.8–2.5 ms), source field set, lateral always 0.
Attempt 2 fwd: max 0.300, integral of commanded fwd = 0.62 m; ang: −0.018…+0.500.
Format/units match AIMDK (m/s, rad/s, sdk_ source, 50 Hz) — and MC obviously executed them (below). No pad driving in bag
(rc stream all zero), so no vendor reference values.

MC common state transitions (input_source, prio, fsm, body):
324.8 rc/80 fsm4 → 426.59 sdk_qazbot_nav2/30 fsm4 → 426.79 fsm5 → 429.19 fsm4 → 624.59 fsm5 → 644.39 fsm4.
action_desc STAND_DEFAULT → LOCOMOTION_DEFAULT for 22.3 s total (= both attempts). MC auto-switches to locomotion; no mode switch needed.

## 3. Attempt 2 timeline (odom = /qazbot_lio/odom_level; map pose = map->odom ∘ odom->base_footprint TF)
| t | event |
|---|---|
| 624.04 | goal; plan 20 poses (−7.475,5.525)→(−7.275,4.575) |
| 624.14 | first nonzero sdk cmd (smoother ramps 0.16→0.30) |
| 624.61 | first odom motion (yaw kick −0.075 rad, then translation) → **startup lag ≈0.47 s** (att.1: 0.28 s) |
| 625.0–627.0 | walking; commanded 0.30→0.12; **actual 0.39–0.49 m/s** (net 0.97 m in 2.5 s) |
| ≈627.0 | robot abreast of goal, map (−7.40,4.555): dist 0.105 m > xy_tol 0.10, laterally offset ~0.13 m from path; RPP switches to rotate-to-carrot (vx=0, wz+) |
| 627.054 | last fwd>0 command |
| 628.81 | speed <0.03 m/s: **stop time 1.76 s, coast 0.14 m** after last fwd cmd |
| 627.8–628 | final pose map (−7.338,4.374,−0.80); goal (−7.295,4.559,−1.393): 0.19 m past goal, yaw err 0.59 rad |
| 628.5–642.3 | RPP cmd vx 0, **wz ≈ +0.05 rad/s for ~14 s** (671 msgs with 0.005<|wz|≤0.06); MC fsm=5 (LOCOMOTION) but odom yaw constant −0.943±0.001 → **no rotation** |
| 642.33 | "Failed to make progress" (last ≥0.1 m displacement ≈627.3 + 15 s = 642.3 ✔) → ABORTED |
| 642.354 / 642.456 | last nonzero (wz 0.05) / last zero-burst msg; then silence |
| 644.39 | MC fsm 5→4; settle 0.022 m, 0.038 rad after abort. No re-motion. |
Totals: odom net 1.155 m, leg_odometry 1.063 m, AMCL 1.18 m; commanded integral 0.62 m → actual/commanded distance ≈1.86.
Note: leg_odometry child_frame=lidar_imu_chest_front, its yaw is noisy (±1 rad jumps) — do not use its yaw.

Why wz stuck at 0.05: after overshoot the goal is behind the robot (bearing ≈+1.34 rad world vs heading −0.80 → +2.1 rad),
RPP rotate-to-heading requests 0.5 rad/s but clamps to measured_wz ± max_angular_accel·dt = 0 ± 1.0·0.05 = 0.05 rad/s
(measured odom wz ≈0 because robot does not rotate). X2 does not start in-place rotation at 0.05 rad/s
(AIMDK doc threshold "~0.03–0.05", docs/X2_MOTION_ADAPTER.md row 5: measured on .89 = 0.4 rad/s, unconfirmed on .66).
Deadlock → progress checker abort. (Clamp mechanism = RPP humble source behaviour, inferred from params + data; source not re-read here.)

## 4. Attempt 1 (goal 426.44, abort 427.13)
- Cmds 426.54–427.18: fwd 0.10–0.16, wz −0.155. Robot moved 0.05 m and yawed −0.2 rad (measured wz −0.47, overshooting command); stop 0.53 s.
- "collision ahead": no current scan point <1.06 m at 426–427. But at **403.6–404.6** (22 s before goal) a moving object (person,
  hypothesis: operator) crossed 0.47–0.8 m front-right of the robot; odom points (−0.5,−0.30)/(−0.5,−0.34) lie 0.24–0.39 m
  from the projected RPP arc (< robot_radius 0.454 → inscribed cost). From 404.7 to 426.6 (110 scans) **zero** beams through those
  two cells had a finite return beyond them (all inf) → obstacle_layer never cleared them (inf not used for clearing) → stale
  "ghost" lethal cells → RPP collision abort. After att.1 the robot had yawed −0.17 rad; between 427.2–624 those cells got
  826–954 clearing rays → cleared → att.2 had no collision warning. (Costmap itself not recorded: strong but indirect evidence.)

## 5. AMCL quality after walking
- AMCL updated 5× during walk (update_min_d 0.25), none after 627.78 (robot static) → covariance frozen at walk-end value.
- Covariance grew monotonically: σx/σy 0.08/0.17 m at start → 0.27/0.33 m; particles 501→948 (KLD expanding); cloud is one broad
  blob ±0.3 m (top weighted 10 cm cells spread −7.45…−7.05, 4.05…4.65), yaw std 0.22 rad — **second_mode_weight 0.48 most likely
  reflects the broad cloud, not a distinct second hypothesis** (hypothesis).
- AMCL vs LIO: map->odom changed only 0.05 m / −0.046 rad over the walk; trust anchor_err 0.04 m / 0.043 rad → AMCL mean ≈ LIO dead-reckoning.
- scan_score dropped 0.63–0.69 (at start) → 0.37–0.39 at end (< floor 0.507, T4 diagnostic).
- Verdict: pose mean is consistent with LIO (±~5 cm relative), but AMCL is not independently confirming it (σ ~0.3 m, low scan score).
  NOT trustworthy as an absolute ground truth for G4 return-to-start without a static re-check (tape/marker, or AMCL nomotion update at rest).
- path_since_anchor: grows ~1.6 cm/s at rest all session (32.9 at 325 → 37.5 at 605) = LIO jitter integration, not stepping in place; walk added ~1.2 m.

## 6. Ranked hypotheses / causes
Symptom: G3 ABORTED "Failed to make progress"; robot 0.19 m past goal, yaw off 0.59 rad.
Proximate: robot stuck after overshoot; RPP commanded 0.05 rad/s in-place rotation which X2 does not execute; progress checker fired correctly.
Root causes (confidence):
1. HIGH — Accel-limited rotate-to-heading from rest (max_angular_accel·dt = 0.05) is below X2 in-place rotation start threshold → in-place rotation deadlock whenever RPP must rotate from standstill. Would also hit any goal needing final in-place yaw alignment.
2. HIGH — Goal tolerance 0.10 m < X2 stop behaviour (coast 0.14 m / 1.76 s after vx→0, actual speed ≈1.6–1.9× command, cross-track ≈0.13 m after gait-start yaw kick) → overshoot past goal is expected, after which the goal lies behind the robot.
3. MED-HIGH — Attempt 1: stale costmap obstacle (person at 403.6–404.6) not cleared because 85 % of virtual-scan beams are inf.
4. Open/unexplained — MC over-delivers speed (0.62 m commanded → 1.155 m actual). Sensors agree (LIO, leg odom, AMCL), so real. Cause unknown (MC gait minimum stride / LOW-mode mapping?). Safety-relevant: max_linear 0.3 behaves like ~0.45–0.5 m/s.
Refuted: message format/units/rate/source, MC acceptance, need for action switch, startup threshold for forward (0.1 m/s moved the robot), re-motion after abort (none).

## 7. Fix direction (no edits made; values need measurement, not guesses)
a. In-place rotation: make RPP able to reach X2's rotation start threshold from standstill — e.g. max_angular_accel such that one
   20 Hz step ≥ measured threshold, or rotate_to_heading only above a measured threshold; value MUST come from the S3 measurement.
b. Goal checker / approach: xy_goal_tolerance ≥ measured (stop coast + lateral tracking error) p95 over repeats; approach_velocity_scaling_dist / min_approach_linear_velocity chosen from measured stop distance vs commanded speed; consider
   commanded→actual speed scale before setting max_linear (F1 dynamic envelope depends on actual speed).
c. Costmap ghosts: before each new goal call local costmap clear (standard Nav2 service `…/local_costmap/clear_entirely_local_costmap`
   from the coordinator), or evaluate `inf_is_valid:=true` — caution: inf in this virtual scan means "no point in this height band in this sparse frame", so it can also clear real obstacles; needs a test.
d. Fix tests/footprint/joint_record.py crash before next recording (type support for /aima/hal/joint/leg/state not loadable in that env — hypothesis).

## 8. Minimal safe S3 measurement (needs explicit user go; operator with PS5, clear 2 m, record_motion.sh)
- Rotate-in-place: wz steps 0.05, 0.1, 0.2, 0.3, 0.4 rad/s, 3 s each, vx 0, with zeros between; measure LIO yaw rate, start lag, stop lag.
- Straight: vx 0.05, 0.1, 0.15, 0.2, 0.3, 2 s each; measure actual speed (odom_level), commanded→actual ratio, coast distance/time after zero.
- Repeat G3 ×3 afterwards to get p95 overshoot/cross-track.
Publishing to MC requires the live adapter path or a dedicated tool — must be authorized by the user; not done here.

## 9. Open questions
- Why actual speed ≈1.6–1.9× command (vendor speed semantics in LOW mode?) — AIMDK docs do not state it; measure (S3).
- Exact in-place rotation threshold on this robot (.66).
- Local costmap not recorded; add /qazbot_d1_map/nav2/local_costmap/costmap to WANT for next run to confirm ghost cells directly.

---
# Addendum — G4 back-to-start (bag g_motion_02, goal 1075.04–1090.06) — CONFIRMED same mechanism

Recorder actions (authorized by coordinator): stopped g_motion_02 (tools/stop_motion.sh; metadata + joints npz 6.8 MB finalized),
started g_motion_03 (tools/record_motion.sh g_motion_03 1800, PIDs bag 145460 / joints 145458).

## Data gap
g_motion_02 did NOT record our MC messages: rosbag2 subscribed /aima/mc/locomotion/velocity with TRANSIENT_LOCAL (matched the
first-discovered publisher) and then rejected VOLATILE publishers incl. ours (logs/rec_g_motion_02_bag.log 676.77–676.96 WARN;
adapter log WARN at 676.76). Only 'rc' (all-zero) messages are in that topic. g_motion_03 log has no such warning so far, but the
same race can recur → record_motion.sh should pass a QoS override for that topic (best_effort/volatile) — needs implementer.
Our MC values are therefore inferred from cmd_vel_authorized (adapter passes values through, saturated=0, rejected=0, latched=0,
counter 943→1690, state active/driving, mc_source sdk_qazbot_nav2) — identical pipeline to G3 where MC msgs were recorded and
matched cmd_vel_authorized 1:1 (fields forward_velocity / lateral_velocity / angular_velocity, m/s & rad/s, source sdk_qazbot_nav2).

## angular.z sequence (G4)
- cmd_vel_nav (RPP): vx 0 throughout; wz p50 0.050, range −0.38…+0.161 (299 msgs); brief 0.08–0.15 spikes only.
- cmd_vel (smoother) and cmd_vel_authorized: same values (≈0.050) → our MC angular_velocity ≈ +0.05 rad/s for 15 s.
- **wz_cmd − measured odom wz (odom_level twist.angular.z at same time) = 0.050 (p5 0.047, p95 0.055); corr 0.90.**
  → each cycle RPP outputs measured_wz + max_angular_accel·dt = measured + 1.0·0.05. Closed loop on actual speed: robot does not
  turn at 0.05 → measured stays ≈0 → command stays 0.05. Spikes = odom yaw-rate jitter (+0.03…0.1) plus 0.05.
- Robot: odom yaw −0.901 → −1.031 … −0.941 (net −0.04 rad, opposite sign, = gait sway), MC fsm 5 / LOCOMOTION_DEFAULT 16.6 s,
  i.e. MC accepted and stepped in place but did not turn. Same in G3 628.5–642.3.
- MC side: no separate turn field and no mode switch needed — G3 attempt 1 (wz −0.155 with vx 0.1) and G3 627.5 (wz 0.47–0.5)
  produced real yaw rates (−0.47, +0.44 rad/s) through the same angular_velocity field; sign convention CCW+ consistent (+0.5 cmd → yaw increased).
  Minimum in-place yaw rate: AIMDK doc ~0.03–0.05 rad/s; docs/X2_MOTION_ADAPTER.md row 5 measured on .89 = 0.4 rad/s (unconfirmed on .66). Data here: 0.05 → no turn.
  No vendor pad-yaw sample exists in either bag (rc stream all zeros).

## Source (navigation2 1.1.18, local copy candidates/nav2_smac_bounds/src/navigation2-1.1.18)
- nav2_regulated_pure_pursuit_controller/src/regulated_pure_pursuit_controller.cpp:386–399 rotateToHeading():
  `angular_vel = sign * rotate_to_heading_angular_vel_; dt = control_duration_;`
  `min/max_feasible = curr_speed.angular.z ∓ max_angular_accel_ * dt; angular_vel = clamp(...)` (lines 396–398).
- nav2_controller/src/controller_server.cpp:473–481: `twist = getThresholdedTwist(odom_sub_->getTwist())` is passed as curr_speed
  (odom_topic = /qazbot_lio/odom_level). So rotate-in-place is closed-loop on MEASURED speed.
- max_angular_accel_ is used only in rotateToHeading (grep: lines 170, 396–397, 868).
- nav2_velocity_smoother/src/velocity_smoother.cpp:268–272: OPEN_LOOP → current_ = last_cmd_ (its own previous output, not odom);
  applyConstraints (225–246) limits per step to max_accel/smoothing_frequency. Loaded: max_accel [0.4,0,1.0], max_decel [−0.6,0,−1.5],
  max_velocity wz 0.5, smoothing 20 Hz, deadband 0 → angular ramp 0.05 rad/s per step, 0→0.5 in 0.5 s, independent of measured motion.

## Root cause G4 (and G3 stall)
RPP rotate-to-heading ramp is anchored to measured yaw rate; with max_angular_accel 1.0 and 20 Hz it can only ask 0.05 rad/s above
the measured rate, which is below X2's in-place turn start threshold → permanent 0.05 rad/s, no rotation → progress checker abort
(15 s, 0.1 m; rotation does not count as progress either way).

## Minimal fix (direction; implementer + verifier)
Set FollowPath.max_angular_accel ≥ rotate_to_heading_angular_vel × controller_frequency = 0.5 × 20 = 10.0 rad/s² so RPP requests the
full 0.5 rad/s from standstill; the OPEN_LOOP velocity smoother keeps doing the physical ramp (1.0 rad/s², 0→0.5 in 0.5 s, decel 1.5).
This value is derived from existing params, not a robot guess. Preconditions/caveats:
- 0.5 rad/s must be above the real .66 in-place start threshold (doc 0.03–0.05 vs .89 measurement 0.4): measure in S3 (wz 0.1/0.2/0.3/0.4/0.5, 3 s each).
- The smoother must stay in the chain and OPEN_LOOP (CLOSED_LOOP would reintroduce the same measured-speed anchoring).
- Separately (G3): xy_goal_tolerance 0.10 < measured coast 0.14 m + cross-track ~0.13 m → overshoot puts goal behind robot; tolerance/approach params need measured stop data.
