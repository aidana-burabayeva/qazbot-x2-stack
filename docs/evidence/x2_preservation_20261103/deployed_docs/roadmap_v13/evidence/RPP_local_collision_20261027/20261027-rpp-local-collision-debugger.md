# 20261027-rpp-local-collision — debugger report (read-only, offline)

Scope: offline only (bags read via sqlite + rclpy deserialization with ROS_DOMAIN_ID=199, ROS_LOCALHOST_ONLY=1; no ros2 CLI on the live domain).
I stopped early on the coordinator's instruction, so the trial03 analysis is incomplete (see Unproven).
Offline checker: /home/agi/.orchestrator/reports/20261027-rpp-local-collision-rpp_offline.py (a Python port of Humble 1.1.18 RPP isCollisionImminent/inCollision + FootprintCollisionChecker + LineIterator).

## System reconstruction (loaded params, not disk defaults)
- controller_server argv: `--params-file /home/agi/.ros/log/qazbot_ctl_ewgwlyx7.yaml` (produced by nav_control.launch.py).
- RPP (nav2_regulated_pure_pursuit_controller 1.1.18): use_collision_detection=true, max_allowed_time_to_collision_up_to_carrot=1.5,
  lookahead_dist=0.5 (velocity-scaled lookahead=false), desired_linear_vel=0.3, use_rotate_to_heading=true (min_angle 0.6, w 0.6),
  use_cost_regulated_linear_velocity_scaling=false, transform_tolerance=0.2. controller_server failure_tolerance=0.3 s, controller_frequency=20 Hz.
- BT bt/navigate_to_pose.xml = ComputePathToPose -> FollowPath, with no recovery and no clear-costmap. Any controller failure therefore aborts NavigateToPose.
- local_costmap: odom frame, rolling 4x4 m, res 0.05, update 5 Hz, publish 2 Hz, robot_radius 0.454, footprint_padding 0.01, plugins obstacle_layer+inflation_layer,
  inflation_radius 0.464, cost_scaling 3.0, scan source /qazbot_lio/scan, marking/clearing true, obstacle_max_range 3.0, raytrace_max_range 3.5.
  Not set, so these take defaults: inf_is_valid=false, observation_persistence=0, track_unknown_space=false, footprint_clearing_enabled=true, min/max_obstacle_height 0/2.
- Scan producer ws/src/lio/level_bridge.py takes a height slice of the deskewed LIO cloud and publishes LaserScan in frame base_footprint: 1440 bins over 360 deg,
  range_min 0.45, range_max 8.0 (launch arg). Bins with no point default to `np.inf` (line 193: `ranges = np.full(N_BINS, np.inf)`).
  Measured: only about 180-210 of 1440 bins are finite, all within roughly -46..+44 deg.

## Collision semantics (answers Q1)
- In Humble 1.1.18, inCollision = footprintCostAtPose(padded 16-gon, r≈0.455-0.468) >= LETHAL(254). Only the polygon perimeter is rasterized.
  INSCRIBED(253) cells and the inflation band therefore do NOT trigger the check. The 0.454+0.01 radius is not compared against 253.
  (Source is not on disk; the logic was matched to the .so symbols/strings `inCollision`, `footprintCostAtPose`, "detected collision ahead!".)
- isCollisionImminent first checks the current pose. It then projects the commanded (v, w) in steps of res/|v| (0.1667 s at 0.3 m/s) while i*dt < 1.5 s and the
  distance is <= carrot_dist (about 0.5). At 0.3 m/s the footprint is projected about 0.40 m ahead, i.e. it reaches about 0.87 m ahead of base center.
  With v=0 (rotate to heading) only the ring at about 0.46 m is swept.
- Previous docs say "lethal cells (cost253)" and "cost>=253 at 0.22 m". Those are inscribed/inflation cells, which this check does not use. The 0.22 m figure is not the trigger.

## Goal01 (1793103181) — verdict class (b) stale/phantom marks caused by a clearing defect (inf rays never clear). Not (a) or (d).
Evidence:
- controller log /home/agi/.ros/log/controller_server_234469_1793099298187.log: goal received at 181.4277, first "collision ahead" at 181.4420 (first cycle),
  7 warnings, "Controller patience exceeded" at 181.7304 (failure_tolerance 0.3). cmd_vel_nav had 7 messages, all zero.
- Robot stationary: odom->base_footprint = (-1.8842,-1.6232, yaw 2.0615) for the whole window. map->odom = (-8.8449, 3.6338, 2.7509).
  The robot in map is (-6.4845, 4.4171) heading -1.471. The goal is 1.869 m away at +4.8 deg, so RPP drives straight (no rotate), v≈0.3.
- Before-clear snapshot (First_RViz_goal_C3_1205/local_costmap_before_clear.npy, t=3257.8, robot moved only 7 mm, no motion in between):
  254 cells = 246. Of these, 121 are within 1.6 m. A cluster sits directly ahead at r=0.822-1.3 m, bearing -3.4..-12.6 deg (cells mx 27-34, my 54-65).
- Offline RPP replay on this snapshot: current pose is clear. v=0.3 with w in {0, ±0.05, ±0.1} gives COLLISION at projection step 7-8 on cells (34,55) = odom(-2.125,-0.825)
  and (32,54) = odom(-2.225,-0.875). That is about 0.82 m ahead, about 0.36 m beyond the body front. v=0.15 or a pure rotate gives no collision.
- Scans 1793103151-181.4 (151 scans): no return < 1.5 m (min 1.75 m at -45 deg). The bearing band [-14, -2) deg has 0/1208 finite returns (all inf).
  The neighbouring band at -16..-18 deg has median range 7.2-7.8 m, which indicates an open, long-range direction.
  So nothing is physically there now, which matches the operator report. Because inf rays are dropped (inf_is_valid=false),
  ObstacleLayer never raytraces through that sector and the mark persists indefinitely.
  Dead bands also exist at +26..+38 deg and at everything outside ±46 deg.
- clear_entirely_local_costmap (1793103310) dropped 254 cells from 246 to 10/42, consistent with stale memory.
Symptom: RPP collision -> patience -> abort. Proximate: LETHAL cells 0.82-1.3 m ahead inside the 1.5 s projection.
Root cause: obstacle-layer marks in scan directions that only ever return inf are never cleared. level_bridge encodes "no point in slice" as inf, and inf_is_valid=false.
Mark origin: not proven. Likely a person or object that was earlier ~1 m ahead (operator during seeding/RC), or marks made while the robot faced differently. Needs a replay to confirm.

## Trial03 first goal (1793104301) — verdict: UNPROVEN; leading hypothesis (b)+(c)
- Log: goal 301.584. Five cycles with v=0.30, w=-0.129 (cmd_vel_nav 301.597-301.788). First collision 301.840, patience at 302.138.
- Collision onset coincides with an odom pose step at the start of MC locomotion (STAND->LOCOMOTION at 301.825).
  odom->base_footprint goes from yaw 2.0340 (stationary) to 2.0057 at stamp 301.812, then 1.9958 / 1.9353 / 1.8311 by 302.21 (-11.5 deg in 0.4 s).
  The commanded w would explain only about -0.03 rad. The translation step was about 1 cm.
- Scans 296-302: no return < 1.5 m (min about 1.85 m). Any lethal cell that triggered was therefore stale (class b), not real (d).
- The only costmap available (Static_map_trial_1230/local_costmap_before_clear.npy, t=4349) was captured 47 s later, after 3 attempts and 0.78 m of motion.
  Replaying it at the trial03 start pose gives NO collision for v=0.3 w=-0.129, v=0.3 w=0, or rotate ±0.6.
  Lethal cells within 1 m are at +47..+57 deg, 0.69-0.96 m, just outside the ±45 deg FOV and therefore never cleared. So that snapshot cannot explain the trial03 abort.
- Hypothesis (not verified): a costmap update or pose change at locomotion start brought a stale mark into the swept perimeter.
  Possible mechanisms: footprint-clearing polygon tied to the old pose, plus the yaw step (c), with marks hugging the footprint or in dead bands.
  This needs a costmap replay from bags.

## Other questions
- Q2: raytrace clearing covers only finite-return bearings (about 90 deg front, with holes). Side and rear marks are never cleared and leave only by rolling out of the 4 m window.
  Self-marks: range_min 0.45 drops points < 0.45 m. No returns < 1.5 m were observed while standing, so there is no evidence of self-marking from body or legs. Behaviour during walking was not checked.
- Q3: Goal01 shows no plan/odom misalignment (straight path, +4.8 deg, stationary). For trial03 the odom yaw step at locomotion start is real in TF; its role is unproven.
- Q4: Goal01 was time-aligned using the stationary-robot argument. Trial03 is not aligned.

## Proposed fix (staged, needs implementer + verifier; no live apply without approval)
Keep the radius, padding and collision detection unchanged. The checker semantics are correct: LETHAL only, 1.5 s horizon, firing about 0.36 m ahead of the body.
1. Primary: make "no return within range" clear. Two options:
   (a) In level_bridge.py, publish bins that have no slice point but lie inside the sensor's valid FOV as range_max (8.0) or +inf, and set
       `obstacle_layer.scan.inf_is_valid: true` together with `raytrace_max_range` 3.5. Expected effect: phantom cells in open bearings (Goal01: -14..-2 deg) clear within one update (0.2 s).
   (b) Safer variant: only treat inf as free beyond the lidar near-blind radius. Clear to range_max only for bins whose 3D beams actually swept the slice.
   Safety residual: for a head lidar, low or close obstacles in the vertical blind zone could be cleared once they leave the slice. The blind-zone geometry must be measured first.
   Bins outside the real FOV (|bearing| > ~46 deg) must stay NaN/no-clear, not inf. Otherwise the rear will be "cleared" without being observed.
2. Secondary (robustness, no safety reduction): add a ClearCostmapAroundRobot / ClearEntireCostmap(local) + retry recovery around FollowPath in bt/navigate_to_pose.xml.
   This is the standard Nav2 recovery and is bounded to 1-2 retries. The cleared costmap re-marks real obstacles from fresh scans in front.
   Also consider `observation_persistence` 0 (keep) and an optional obstacle-layer decay; Humble has no decay, so this would need a separate plugin, which is out of scope.
3. Do NOT raise failure_tolerance or shrink the radius. The evidence shows the radius is not the cause.
Expected quantified effect on Goal01: with clearing in the -14..-2 deg band, the cells (32-34, 54-56) are cleared and the offline replay predicts no collision.
To verify: rerun the offline checker on a replayed costmap.

## Unproven / open items
- Trial03 root cause (no time-aligned costmap). Next step: offline ObstacleLayer replay (mark/clear from /qazbot_lio/scan + /qazbot_d1_map/tf, footprint clearing, 5 Hz)
  over the bags RViz_goal2_1216 -> RViz_goal3_1220 -> Static_map_trial_1230, starting after the clear at 1793103310. Validate against the 4349 snapshot,
  then evaluate the checker at 301.79/301.84.
- Origin of the Goal01 phantom cluster. Replay First_RViz_goal_1135 + C3_1205 to find the scan that marked it.
- Vertical blind zone of the head-lidar slice (required before inf_is_valid=true).
- Whether the yaw step at locomotion start is real body motion or a native_imu odom artifact.
- Static_map_retry_1257 bag (recorded during this session) was not analyzed.
