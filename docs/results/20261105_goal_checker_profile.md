# 2026-11-05 production goal-checker profile (operator decision): SimpleGoalChecker, xy 0.30 m, yaw 0.35 rad, stateful

Decision: restore the 2026-10-30 behaviour. `goal_checker.plugin = nav2_controller::SimpleGoalChecker`, `xy_goal_tolerance 0.30`,
`yaw_goal_tolerance 0.35`, `stateful true`, `goal_checker_stopped` not used (the StoppedGoalChecker switch from branch
`feat/e2-goal-checker-stopped` is NOT included). Nothing else changed: RPP, adapter, Trust, AMCL, smoother, MC, projector, speeds.

## 1. Where the defaults lived (all real launch paths)
Entry `run_nav_full.sh -> nav_full.launch.py` (declares `xy_tol`, `yaw_tol`, `goal_checker_stateful`, calls `nav_control.setup`),
`nav_control.launch.py` (own declarations + `setup` fallback `.get('goal_checker_stateful','false')`). No systemd/cron autostart exists for Nav2
(started by hand with `run_nav_full.sh`; only the two files build the controller parameters; `nav_plan`/`nav_debug_bounds` have no goal checker).
Changed: defaults 0.10/0.10/false -> 0.30/0.35/true in both files, the `setup` fallback false -> true (so a missing argument cannot restore false), the comment,
and the constants of `tests/nav_e2e/run_suite_real.py`. Diff: `git diff 4329077..HEAD -- ws/src/navigation` (branch `fix/goal-checker-profile`): 2 files, +9/-8 lines.

## 2. Offline checks (no ROS graph, no motion)
`tests/controllers/test_goal_checker_profile.py` 6 PASS: declared defaults of both files; generated controller_server YAML through `nav_full.setup` (robot env,
Smac prefix) and through `nav_control.setup`; fallback when the stateful argument is missing; RPP/progress-checker values unchanged; no `trans_stopped_velocity`.
Previous test `tests/controllers/test_goal_checker_stopped.py` is on the other branch and not part of this profile.
The deployed robot files generate: plugin SimpleGoalChecker, stateful True, xy 0.3, yaw 0.35.

## 3. E2E-02 recorded trajectory under the new profile (replay of data, NOT a physical PASS)
First inside xy 0.30 m: +6.6 s, d 0.261 m, yaw error -0.18 rad, speed 0.47 m/s. A stateful SimpleGoalChecker would accept at the same instant (yaw 0.18 <= 0.35).
Prediction (not a measurement): if the command went to zero there, rest = pose + 0.38 s * v -> error ~0.085 m (along -0.07, lateral 0.05).
Recorded movement after that point (the old 0.10 checker kept the robot driving and turning, this is not what the new profile would do):
+0.5 s 0.25 m moved (0.092 m from goal), +1 s 0.36 m (0.11 m), +2 s 0.41 m (0.15 m), +5 s 0.52 m (0.29 m, yaw -2.41), +10 s 0.37 m (0.21 m).
Consequence: SUCCEEDED will be reported while the robot still walks at ~0.5 m/s; the final rest distance must be measured separately (done in the test below).

## 4. Deployment state and rollback
Robot files updated (no restart yet): `ws/src/navigation/nav_full.launch.py` sha256 1be39f89...; `nav_control.launch.py` sha256 e1ab9809...; backups next to them
`*.bak_20261105_pre_gcprofile` (sha256 9a9f963a... and 39b8a6ff...). The running Nav2 still uses the old parameters until the controlled restart.
Rollback: `cp nav_full.launch.py.bak_20261105_pre_gcprofile nav_full.launch.py` (same for nav_control), restart Nav2 the same way as below.

## 5. Controlled Nav2 restart (needs operator on site; not done)
Preconditions: robot standing, no active Goal (GC busy false), RC and emergency stop checked, operator confirms. Stop only `nav_full` (SIGINT to the pid in
`logs/nav_full_20261105.pid`; it contains Nav2 and the Goal Coordinator), Motion Adapter/localization/Trust stay running. Start with
`ROBOT_RADIUS=0.454 run_nav_full.sh allow_dry_run_driver:=true map_yaml:=...d1_qazbot_20261029/map.yaml` via `detach.py`. Then verify the REAL parameters of
`/qazbot_d1_map/nav2/controller_server` (ros2 param get: plugin, xy, yaw, stateful, desired_linear_vel, rotate_to_heading_min_angle), GC status, Trust READY, adapter registered.
SpeedLimit returns to the configured 0.30 on restart.

## 6. One physical test (after a separate permission)
3 m free straight Goal, same speed mode (no SpeedLimit change), full recording as in E2E-02 (plan, RPP, smoother, authorized, adapter output, MC source/FSM/speed_mode,
TF, AMCL, FAST-LIO, GC/Trust). PASS: SUCCEEDED; no repeated turns/returns around the goal; the robot physically stopped; REST position within 0.30 m and 0.35 rad
(checked independently of SUCCEEDED, because stateful=true can keep XY accepted while the robot rotates); Trust and MC normal. At the first significant deviation end
with the normal procedure, analyse, no automatic repeat.
