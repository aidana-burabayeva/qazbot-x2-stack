# stopodom: PARTIAL (analysis halted on coordinator request, host load). Offline, read-only.
Scratch: /tmp/claude-1001/-home-agi/c9b41d51-5b1b-455f-88cf-eb93053f257e/scratchpad/so/{ext.py,ext2.py,met.py,fw.py,*.pkl}. g_motion_04 extraction was killed unfinished (not analysed). No ROS graph access.
Method: MC cmd (/aima/mc/locomotion/velocity filtered by source) vs speed from /qazbot_lio/odom_level pose differences over 0.25 s (header stamp) and leg_odometry. Receive-time cmd. Speed estimator smears ~0.12 s.

## Q1 measured (forward segments available: G3 att1, G3 att2 (g_motion_01), pad fwd +0.8, pad back -0.8 x2 (g_motion_03); g_motion_02 has no SDK MC msgs, no forward cmd)
| segment | cmd peak | plateau LIO speed p50/p90 | speed at last cmd | last cmd -> v<0.05 | coast LIO / leg |
|---|---|---|---|---|---|
| G3 att1 (0.10-0.16, 0.64 s) | 0.16 | 0.07/0.08 (never reached gait speed) | 0.10-0.13 | 0.28-0.38 s | 0.004-0.013 / 0.012-0.016 m |
| G3 att2 (0.30 ramp down to 0.03 by RPP) | 0.30 | 0.47/0.53 | 0.50 | 1.20-1.24 s | 0.076-0.121 / 0.117-0.129 m |
| pad fwd (rc 0.8, onset spike 1.8) | 1.8/0.8 | 0.70/0.78 | 0.78 | 1.5 s | 0.32 / 0.30 m |
| pad back -0.8 | | 0.44/0.49 | 0.42 | 1.1 s | 0.20 / 0.12 m |
| pad back1 -0.8 | | 0.32/0.44 | 0.47 | 1.1 s | 0.14 / 0.12 m |
Onset lag cmd->v>0.10: 0.32-0.51 s (all). Actual/commanded: att2 1.6-1.8 (distance 1.12 m vs 0.62 m integral); pad fwd ~0.9-0.97; back ~0.55.
Observation: in att2 speed stayed 0.45-0.57 while command fell 0.30 -> 0.03 (300.5-302.0 s); it fell only after cmd reached 0 (0.33, 0.18, 0.13, 0.04 at 0.25 s steps). RPP approach scaling/min_approach therefore did not slow the robot; the only effective stop lever was cmd=0.
Scaling: coast d = k v^2: att2 0.12/0.5^2 = 0.48 s^2 (0.30 m/s command), pad 0.32/0.78^2 = 0.53 s^2 (a_eff ~1 m/s^2). Linear d=tau*v is inconsistent (0.24 s vs 0.41 s). n=2 forward points, so tentative.
Speed vs command hypothesis (2 points): v ~0.35+0.5*cmd (0.30->0.5, 0.8->0.75) = speed floor ~0.45 m/s once gait is running; unconfirmed. att1 shows 0.1-0.16 cmd for 0.64 s only produced the start-up, not steady speed.
Latency facts (g_motion_03): odom_level 10.0 Hz, receive-header p50 45 ms (p95 59 ms, max 1.17 s); leg_odometry 169 Hz, latency p50 4.9 ms (p95 8 ms).

## Q1 derivation (Nav2 mechanisms only)
Inputs: v_a = 0.5 m/s (actual at approach, att2), d_c = 0.12 m (LIO and leg agree 0.117-0.129), t_lat = odom latency 0.045 s + mean 10 Hz staleness 0.05 s + controller 20 Hz 0.025 s ~0.12 s (estimated; pipeline cmd_vel_nav -> MC latency NOT measured), e_y = 0.13 m (g3debug, n=1).
D = d_c + v_a*t_lat = 0.12 + 0.06 = 0.18 m travelled after the goal checker fires.
GoalChecker succeeds at first entry into the xy circle. For the robot to stop within tol with cross-track e_y: derived tol^2 >= e_y^2 + (D/2)^2, so xy_goal_tolerance >= sqrt(0.13^2 + 0.09^2) = 0.158 m (e_y=0 gives 0.09 m; current 0.10 only works if cross-track ~0.04 m).
(Derivation: entry chord s = sqrt(tol^2 - e_y^2); final error hypot(D - s, e_y) <= tol  =>  s >= D/2.)
- xy_goal_tolerance: ~0.16 m, provisional. This is the only parameter I can derive. It rests on e_y n=1, d_c n=2, t_lat estimated.
- min_approach_linear_velocity (0.12) / approach_velocity_scaling_dist (0.6): no change justified. MC speed does not follow the command below ~0.3 (floor hypothesis), so lowering min_approach cannot reduce the actual speed (also keeps the floor at ~0.45-0.5). Approach distance must cover v_a * T_resp; T_resp (speed response to a command step down) is unmeasured; at 0.5 m/s, 0.6 m = 1.2 s, which exceeds the onset lag 0.3-0.5 s, so 0.6 is not contradicted.
- max_linear 0.3: actual ~0.5 m/s; F1 envelope should use actual speed.
- Smoother decel -0.6 m/s^2: irrelevant to physical stop (robot decel ~1 m/s^2 after cmd=0 and does not follow the command ramp).
- yaw_goal_tolerance 0.10: insufficient data in this run (padcompare: quantised ~30 deg pivots, hunting). I did not compute yaw coast (halted).
Data gaps and the single measurement that closes them: constant-command forward steps 0.10/0.15/0.20/0.30 for >=4 s each, without returning to STAND between steps, x3 repeats, with cmd->zero. This gives v(cmd) (floor vs gain), d_c(v), the p95 cross-track and a stop-response lag. Also record cmd_vel_nav and MC stamps (t_lat) and run the SDK goal G3 x3 for p95 overshoot. g_motion_04 contains no confirmed forward segments (not analysed).

## Q2 (partial)
Measured (g_motion_03): leg_odometry frame leg_odom -> lidar_imu_chest_front, 169 Hz, latency p50 4.9 ms; LIO odom_level frame odom -> base_footprint, 10 Hz, latency p50 45 ms. Both quaternions are normalised. leg orientation at rest (0.396, -0.572, 0.421, 0.582): the sensor frame is rotated roughly 90 deg off level, so naive atan2 yaw extraction is probably the cause of the +-1 rad "jumps" (HYPOTHESIS, not tested; check by computing the relative rotation R_leg^-1 * R_lio, which should be constant). LIO base_footprint is level (z 0.006-0.094).
Translation vs LIO (LIO = reference, no tape): G3 att2 1.056 vs 1.124 m (-6%), pad fwd 1.434 vs 1.532 (-6%), pad back 0.816 vs 0.981 (-17%), 1.184 vs 1.461 (-19%); G3 (earlier docs): 1.063 vs 1.155. At rest, leg odom is flat (v ~0.000-0.001) while LIO jitters 0.01-0.03 m/s.
Verdict (preliminary): not yet qualified for the odom -> base_footprint role. Promising: rate 17x higher, 9x lower latency, zero rest drift. Missing: heading from a correct orientation transform plus the fixed child-frame offset (lever arm), yaw-rate behaviour in the turn segments (pad L +96 deg, R -95 deg), asymmetric reverse scale error 17-19%, the chest vs base frame lever arm during turns, ground truth (tape). Not done: per-metre drift in turns, yaw jump count. Resume with ext2.py output g_motion_03.q.pkl (already written).
