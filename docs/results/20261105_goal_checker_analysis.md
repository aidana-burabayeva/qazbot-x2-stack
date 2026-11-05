# 2026-11-05 P0: why the 3 m Goal does not finish cleanly — goal checker / RPP analysis of the E2E-02 bag (no motion)

Data: bag_e2e02_20261105_1707 (evidence copy in `docs/evidence/e2e02_20261105/`), scripts `docs/results/20261105_goal_checker/` (`tol.py`,
output `tol_output.txt`). Production unchanged, no Goal sent. Goal checker evaluated on the 10 Hz map pose composed from TF.

## 1. Parameters of the E2E-02 run (live dump `~/.ros/log/qazbot_ctl_39kblq9x.yaml`)
Goal checker `SimpleGoalChecker`: xy 0.10 m, yaw 0.10 rad, `stateful: false` (Nav2 default: 0.25 / 0.25 / stateful true; 11-03 run: 0.35 / 0.17 / true).
RPP: lookahead 0.5 (0.3-0.9), `use_rotate_to_heading` true, `rotate_to_heading_min_angle` 0.6, `allow_reversing` false, approach scaling 0.6 m,
min approach 0.12, desired 0.3 (SpeedLimit 0.20 applied), collision time 1.5 s. Progress checker 0.1 m / 15 s / 0.25 rad. Smoother OPEN_LOOP.

## 2. Tolerance comparison on the recorded trajectory
| Checker | first OK | pose error then | predicted rest error* |
|---|---|---|---|
| actual: xy 0.10 yaw 0.10 stateless | reported at +60.2 s while moving | — | **recorded rest: 0.21 m short, yaw 0.26 rad** (outside its own tolerance) |
| xy 0.10 yaw 0.10 + stopped (v<0.08, w<0.05) | never in 75 s | — | best recorded rest 0.195 m |
| Nav2 default xy 0.25 yaw 0.25 (stateful or not) | +6.7 s (0.49 m/s) | 0.211 m, yaw -0.15 | 0.04 m |
| xy 0.25 yaw 0.10 stateless | +57.0 s | 0.161 m | 0.31 m |
| xy 0.15 yaw 0.15 stateless | +56.9 s | 0.135 m | 0.30 m |
| xy 0.25 yaw 0.25 + stopped | +61.6 s (recorded rest) | 0.200 m, yaw 0.12 | recorded |
*predicted rest = pose + 0.38 s * velocity (D-05 stop fit); a counterfactual, not a measurement.
Conclusions: (1) the current SUCCEEDED is a moving pass through the tolerance disc, the robot never rested inside 0.15 m in 75 s; (2) with 0.10 the
condition is not reachable by this robot as it behaves now (rest error 0.2 m, lateral 0.10-0.18 m, yaw error 0.15-0.27 rad at arrival);
(3) yaw tolerance 0.10 is the binding one when xy is relaxed. The tolerance is NOT being raised to obtain SUCCEEDED: the finding is that the
checker must test rest pose, and which numbers are achievable is an operator decision (section 5).

## 3. RPP transition that causes the unnecessary turn
Recorded at 0.1 s (`tol_output.txt`): pure pursuit (carrot = path end, angle -0.3..-0.4) until +6.8 s; at **+6.9 s, 0.14 m from the goal, carrot angle
-0.61 rad (> rotate_to_heading_min_angle 0.6) -> rotate-to-path** (cw, MC wz -0.45..-0.62, vx -> 0). The lateral offset 0.10 m and the closing distance make the
angle to the final carrot grow hyperbolically. The robot (0.5 m/s, coast 0.2 m) crosses the 0.10 m disc in <0.3 s while turning cw; the commanded wz sign
(-0.62 with yaw error -0.26, which would need +) shows rotate-to-path, not rotate-to-goal-heading. By +7.4 s the carrot is behind (-2.57 rad): U-turn.
Yaw tolerance: arrival yaw error -0.15..-0.27 rad > 0.10 forces an in-place rotation even when xy is inside; body drift during in-place turns is
0.13 m/s with no preferred direction (12 s turn: 0.5-1.2 m net), which re-exits the disc.

## 4. Governor (existing regression tests re-run, defects from the verifier fixed)
Fixed: throttled WARN + `no_data_reason` in status + `odom_clock_offset` flag when a forward command passes unrestricted; estimate above 1.0 m/s
(project real-speed NO-GO) is ignored (no premature stop far from the goal; test); path projection windowed around the previous fix (no jump on a path
that doubles back; test); GC contract test (exactly one publisher on the cmd topic, GID stable over 20 commands, zero forwarded unchanged).
GC behaviour (read from goal_coordinator.py): it trips on publisher count != 1 or a changed publisher GID; a governor restart mid-goal therefore
stops the robot safely, a dead governor means silence -> adapter timeout -> zero. Tests: unit 21, simulation 7, node launch 5, all PASS.
Sufficiency: UNCHANGED — simulation 142 -> 153 of 200 successes at the measured lateral error; the along-track governor does not remove the
rotate-to-path transition of section 3. **Not installed, not proposed for installation.**

## 5. Proposed experiments (no motion without separate operator consent)
Prepared config switch (branch `feat/e2-goal-checker-stopped`, 7d06b8d): launch arg `goal_checker_stopped:=true` selects
`nav2_controller::StoppedGoalChecker` (within tolerance AND at rest; velocity from the odom_level twist, verified 0.48 +- 0.075 while walking);
default off = unchanged; 3 launch tests PASS. `trans_stopped_velocity` 0.08 m/s and `rot_stopped_velocity` 0.05 rad/s come from the measured
stationary noise (speed sd 0.02, yaw rate max 0.015), they are proposals to be confirmed by E1/E2 data.
**E1 (one run):** Goal 2.5 m straight from the current pose, Nav2 SpeedLimit 0.10 m/s (nav2_msgs/SpeedLimit, no GC/adapter bypass), checker unchanged,
recorder as E2E-02. Measures: whether the robot starts at 0.10 (the vendor start threshold is ~0.1 m/s), steady speed (LIO), stop distance from steady.
If it does not start, the existing progress checker ends the goal after 15 s (no new watchdog); RC in hand, free corridor 4 m.
**E2 (one run, after E1 and your decision):** Goal 3 m with the stopped checker. Decision needed: tolerance for E2 — (A) keep 0.10/0.10: predicted
not reachable; (B) Nav2 default 0.25 m / 0.25 rad + stopped: counterfactual finish in ~7-10 s with rest error 0.04-0.2 m (prediction). Acceptance
is judged on the recorded REST pose (SUCCEEDED, no U-turn, no repeated approaches, MC/Trust normal), reported against both 0.10 and 0.25.
Recording: Nav2 plan/RPP/smoother/authorized, adapter output and MC (source, FSM, speed_mode), TF, AMCL, LIO, GC and Trust status.
FAIL handling: first divergence point, fix that component only, no repeat without a changed cause.
