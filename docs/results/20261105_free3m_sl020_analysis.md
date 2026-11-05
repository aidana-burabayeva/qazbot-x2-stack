# 2026-11-05 free 3 m SL0.20 — read-only analysis (run: 20261105_free3m_sl020.json, commit 478d289)

Read-only: no Goal, no latch reset, no restart, no parameter/MC change. Sources: the 0.5 s timeline, nav_full log
(controller/BT/GC), adapter code+status counters, nav_control.launch.py params, a 15 s passive stationary sample of
odom topics (odq.py). NOT recorded during the run (so several items stay OPEN): Smac path, adapter output command,
MC state/speed_mode, FAST-LIO yaw time series, TF.

Geometry: start (-8.16,-5.33) yaw 1.482 -> goal (-7.88,-2.35) yaw 1.4825; straight start->goal heading 1.477 rad
(start heading error 0.005 rad). Goal tolerances xy 0.10 / yaw 0.10; RPP rotate_to_heading (wmax rad/s, min angle 0.6);
progress checker 15 s / 0.1 m / 0.25 rad; OPEN_LOOP smoother.

| Observation | Evidence | Cause | Status |
|---|---|---|---|
| Early small wz (+-0.07..0.37 rad/s, sign flips) | path heading = start yaw (0.005 rad); cross-track to straight line within -0.07..+0.18 m; odom wz noise +-0.3..0.8 | RPP corrections of cross-track/gait sway, not plan geometry; actual Smac path not recorded (assumed straight) | CONFIRMED (path itself OPEN) |
| Overshoot past goal | along-track 3.12..3.27 m vs 2.99 m at actual 0.49 m/s, cmd 0.20 | open-loop MC overspeed (D-05), SpeedLimit only limits RPP command | CONFIRMED |
| Real turn at ~8 s: cmd -0.6 for 5 s, then +0.6 | odom wz mean ~-0.5 during 8..13 s (robot did rotate), cmd flips at 14.6 s; xy already inside goal | RPP goal-heading rotation after reaching xy. Why heading error was large: unknown (TF/AMCL yaw vs real yaw not recorded) | CONFIRMED stage; trigger OPEN |
| Translation during rotation with cmd vx=0 | odom v 0.2..0.4 m/s, pose along 3.27 -> 2.30 (0.9 m) while turning | real walking drift in turn, or AMCL/TF jump; not separable without TF/FAST-LIO record | OPEN |
| From 19 s: cmd +0.6 constant, odom w=v=0 for 70 s | timeline; nav log: 4x "Failed to make progress" (t=34,49,64,79 s) -> BT clears costmaps and retries; GC trips only at my 90 s cancel | robot physically stationary under ccw 0.6; RPP/GC/auth chain intact (cmd == cmd_vel_authorized, gate true) | CONFIRMED (stall); cause OPEN |
| Yaw loop P20/P23 never active | adapter counters: yaw_loop 0, yaw_meas_stale 4043 of 4597 published; /qazbot_native/odom has 0 publishers; adapter odom_topic default is that topic | wz passed through open-loop to MC | CONFIRMED (loop inactive); NOT shown to be the cause of the stall |
| Adapter output / MC source / FSM / speed_mode during stall | not recorded; adapter status now idle, mc_source rc, registered | candidate: ccw u=0.6 sits at MC sustained-turn deadband (adapter u_min ccw 0.6 vs cw 0.45; cw 0.6 did rotate) | OPEN (hypothesis only) |
| Final AMCL pose inconsistent | last latched amcl_pose (-7.78,-0.48,yaw -1.64) vs run end (-7.84,-2.99); AMCL silent when standing; Trust: sigma_xy 0.34, second_mode_weight 0.82, scan_score 0.74 | AMCL second mode / jump after rotation possible; unverified | OPEN |
| odom_level for yaw-rate (stationary) | 10 Hz, dt med 0.100 max 0.101, 0 gaps >0.2 s, latency 36 ms, yaw-rate sd 0.005 rad/s; FAST-LIO raw sd 0.016 | meets YawRateEstimator limits (dt<=0.5 s, age<=0.3 s, stamp-based) at rest | CONFIRMED at rest |
| odom_level for yaw-rate (dynamic) | no 10 Hz FAST-LIO/odom_level yaw record during gait/turn | gait ripple, lag under rotation not measured | OPEN |

Minimal next steps (no code change yet, no P20/P23 input switch):
1. Next physical run (goal completion) must record a bag: cmd_vel, cmd_vel_authorized, adapter output topic to MC,
   MC state/source/fsm/speed_mode, /qazbot_lio/odometry, odom_level, amcl_pose, TF, plan, controller log.
2. Qualify odom_level offline from that bag (yaw rate vs commanded during turns) before any odom_topic change; the
   adapter change itself needs a restart and separate operator permission.
3. Before the next run, re-seed pose (AMCL last pose is inconsistent) and confirm Trust.
