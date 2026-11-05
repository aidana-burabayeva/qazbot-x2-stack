# 2026-11-05 P0 test: free 3 m NavigateToPose with the production profile (SimpleGoalChecker 0.30 m / 0.35 rad, stateful)

Result: **SUCCEEDED in 6.6 s** (E2E-02, same speed mode: 60 s, 4 turn-arounds). Verdict against the agreed criteria: **PARTIAL PASS** — functional completion restored,
final yaw is at/over the 0.35 rad limit. Not a clean PASS; no automatic repeat.

Conditions: Nav2 restarted with the new parameters (verified live), Trust READY, GC ready, adapter registered (odom_topic odom_level), no SpeedLimit change (0.30 configured),
RC in hand, corridor free. Start (-8.275,-6.193,yaw 1.438) -> goal (-7.877,-3.219,1.438), Smac 3.03 m, lateral dev 0.116 m, cost 0.
Recording: Nav2 plan/RPP/smoother/authorized, MC (source/FSM/speed_mode), TF, AMCL, FAST-LIO, leg odometry, GC/Trust (bag in `docs/evidence/p0_goal3m_profile_20261105`).

| Criterion | Result | Evidence |
|---|---|---|
| SUCCEEDED | YES | "Reached the goal!" +6.5 s, status 4 |
| No repeated turns / returns around the goal | YES | one approach, commands zero from +7.0 s, no rotate-to-path |
| Robot physically stopped | YES | MC cmd 0 from +7.0 s, FSM MOVING->STABLE at +10.1 s, leg odometry static (0.00 m between +10 and +13 s) |
| End position within 0.30 m | YES | 0.086 m from the goal at the stop (LIO); later LIO values 0.07-0.29 m are an odometry artifact (below) |
| End yaw within 0.35 rad | **NO (marginal)** | accepted at -0.35, LIO -0.36 at the stop, -0.42 later (part of the later change is the same LIO artifact) |
| Trust / MC normal | YES | trusted throughout, MC source sdk_qazbot_nav2, body STAND, speed_mode 1, GC/adapter no fault, no latch |
Collision: none. Max measured speed 0.57 m/s (cmd 0.30, D-05 overspeed unchanged).

Timeline (map pose composed from TF; goal-relative): cross-track +0.36 m at +4.5 s (lateral excursion), yaw error +0.27 rad at +2.5 s, -0.20..-0.36 rad from +5 s; distance to goal
0.52 m at +6.0 s, 0.24 m at +6.5 s (inside 0.30 m, yaw -0.35 -> stateful checker accepts), 0.086 m at +7.0 s when the command went to zero. The robot stopped about 0.09 m short of the goal (no overshoot).

First (only) deviation: the heading at arrival. The lateral swing (0.36 m) and the correction turn leave a -0.35..-0.36 rad yaw error when the xy tolerance is entered; with stateful and yaw 0.35 the checker
accepts at exactly the boundary, so the robot is not rotated to the goal heading. This is the expected property of the chosen 0.35 rad tolerance, not a controller fault.

Finding (not part of the criteria): after the stop FAST-LIO position wandered 0.21 m between +10 and +13 s while leg odometry stood still (0.00 m) and MC was STABLE: FAST-LIO standstill drift after motion
(also moves the odom yaw by ~0.09 rad). It feeds map->base and therefore any final-pose check; physical rest is judged from leg odometry + MC state here. To be investigated separately (not now).

Next step: if this profile is accepted, obstacle navigation (separate permission); if the yaw at rest must be within 0.35 rad, a yaw margin (not a tolerance change) is a separate decision.
Rollback of the profile: Git 4329077 files + controlled Nav2 restart (see `20261105_goal_checker_profile.md`).
