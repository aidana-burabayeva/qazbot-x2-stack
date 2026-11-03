# 20261103-odom-hold-launch -- robotics-tester (isolated launch/replay verification)
Overall: checks 1, 2, 4 PASS; check 3 FAIL (slow displacement is partly hidden; cap is not cumulative). Candidate NOT deployable as "does not hide body motion" until 3 is fixed.

## Setup (isolated; nothing running was modified)
- Copy tree: /tmp/claude-1001/-home-agi/875283d9-732e-4c1e-ba4d-0d8188cfcd4d/scratchpad/odomhold_launch/ (control_backend/ws/src/{lio,navigation}: candidate native_odom_bridge.py, odom_hold.py, native_odom.py, run_native_odom_bridge.sh, env.sh, aimdk_env.sh; runtime/ symlinked read-only).
- ROS_DOMAIN_ID=80, ROS_LOCALHOST_ONLY=1, DDS profile = loopback copy (dds_qazbot.xml replaced by dds_loopback.xml in the COPY). Domain 0 never used. Own processes only were stopped; domain-80 ros2 daemon stopped.
- Bridge started with `ros2 launch launch_bridge.launch.py`: the ExecuteProcess is identical to the candidate localization.launch.py (wrapper run_native_odom_bridge.sh + same --ros-args), plus only `-p use_sim_time:=<bool>` (sim time used for ros2 bag play --clock; wall time for crafted inputs). The launch process itself sources only /opt/ros/humble, so aimdk_msgs comes solely from the wrapper.
- Scripts: drv.py (launch+recorder+feeder), rec.py, feeder.py, ana.py, trans.py, ramp_ana.py, outputs in .../odomhold_launch/out/.

## 1. Launch / startup -- PASS
- Log: `native_odom_bridge started: mode=native_imu publish_tf=True /aima/mc/leg_odometry -> TF odom->base_footprint + /qazbot_native/odom, 30 Hz, max_age=0.2 s standing_hold=True`; no import ERROR.
- `ros2 topic info -v /aima/mc/common/state`: bridge SUBSCRIPTION, type aimdk_msgs/msg/McCommonState, BEST_EFFORT/VOLATILE. Bag metadata (both bags) for /aima/mc/common/state and /aima/mc/leg_odometry: types aimdk_msgs/msg/McCommonState, nav_msgs/msg/Odometry; offered BEST_EFFORT + TRANSIENT_LOCAL -> compatible with the sensor-data subscriber. Functionally proven: hold became held=True/standing=True from replayed state messages.
- /qazbot_native/status carries `hold{held,standing,offset_m,held_samples,entries,exits[,cap_releases]}` (55/215 status messages recorded). Published odom 30 Hz, TF == odom pose (max dev 0.0), frames odom/base_footprint.
- Not checked: live MC topic QoS on the robot (domain 0 not touched); only recorded bag metadata.

## 2. Bag replay (ros2 bag play --clock, topics leg_odometry + common/state only) -- PASS
nav_bag_183911 (walking, 54 s): leg 8861/8872 received, odom 1601, TF 1610.
- State: standing -> NOT standing at bag 24.245 s -> standing at 34.045 s. Hold entries 6 / exits 5 (published-sample derived).
- Walk start: last held published sample ~28 ms after the state flip, first non-held ~55-65 ms after (30 Hz decimation + leg backlog ordering before the 10 Hz state in the executor); offset change in that window 0.06 mm. Held samples while state not standing: 1 (this one). No hold during locomotion otherwise (0 held samples with state not standing).
- Stand after walk: first hold entry 3.69 s after state returns to standing (needs 0.5 s quiet + ~1 s settling + walk residue).
- No jump: max |published step - raw step| over all non-absorbing steps = 0.0; step at hold entry/exit = raw step (max 0.32 mm). Net displacement raw 3.0956 m vs published 3.0959 m; path raw 3.805 / published 3.778 m (drift removed only in standing). Twist while held <= 1.4 mm/s, up to 0.53 m/s when walking. Matches offline replay numbers in the implementer report.
nav_bag_183504 (standing 215 s, includes 25 mm shove; state standing the whole time): leg 35323/35323, odom 6345. Hold entries 4 / exits 3 on published samples (offline replay: 11/10 raw-rate; 30 Hz decimation merges short exits). Held while not standing 0. Step dev 0.0. Max step at transitions 0.74 mm == raw step.
- Shove window 162-166 s: raw 9.4 mm / published 9.4 mm (passed through, not hidden). 0-150 s: raw 25.1 mm / published 0.1 mm (vendor standing drift ~1 cm/min absorbed, by design).

## 3. Slow real displacement (crafted from standing segment 20-80 s of nav_bag_183504, wall time, real state at 10 Hz standing, injected world-x ramp starting 15 s into the segment) -- FAIL
Control (0 mm/s): hold stays on all 60 s, 0 releases; hides the real vendor drift 7.45 mm in 45 s (0.17 mm/s) = intended behaviour.
| inject | first release after ramp start | hidden at first release | later behaviour | hidden at +45 s |
|---|---|---|---|---|
| 0.5 mm/s | 16.74 s (cap_release) | 8.0 mm (7.5 mm injected + drift) | cap releases 2, hold re-enters (5 entries), 46% held afterwards | 13.9 mm of 22.5 mm injected (62-72%) |
| 1 mm/s | 0.44 s | 0.41 mm | hold RE-ENTERS 53 times (120 entries total), 29% held after first release; hidden grows linearly ~0.23-0.28 mm/s | 10.5 mm (vector, since ramp start); injected-component 12.3 mm at +43 s (28% of 43 mm) |
| 2 mm/s | 0.30 s | 0.41 mm | no re-entry | 0.41 mm |
| 5 mm/s | 0.14 s | 0.41 mm | no re-entry | 0.41 mm |
Result: 2 and 5 mm/s PASS (hidden ~0.4 mm = D_POS). 1 mm/s FAIL: > 1 cm hidden (10.5 mm in 45 s, unbounded linear growth ~1.4-1.7 cm/min, = 23-28% of the real motion). 0.5 mm/s (below the 1 mm/s criterion): 13.9 mm in 45 s, rate ~0.31-0.36 mm/s, consistent with the documented cap rate (0.38 mm/s) but not bounded in total.
Root cause: the drift cap (3.8e-4 m/s * t + 1 mm) is per hold run. `capped` is cleared as soon as the quiet/standing condition drops once (odom_hold.py update(): `if not hold: self.capped=False`), and entry resets t_enter and the 1 mm slack. At ~1 mm/s (D_POS/WINDOW = 0.9 mm/s edge) leg noise (+-0.1 mm) makes the 0.5 s window excursion drop under 0.45 mm intermittently, so the hold re-enters again and again; each run hides up to 0.45 mm + slack, with no cumulative budget. Same at 0.5 mm/s via the cap latch dropping.
Suggested fix (not implemented): cumulative hidden-displacement budget over a rolling window (e.g. absorbed total over the last 60 s <= 0.38 mm/s*60 s + 1 mm, not reset on re-entry), or require a longer quiet window (2 s: D 0.8 mm -> 0.4 mm/s) / hysteresis (re-entry only after >= N s of quiet since the last release). Then re-run this test (feeder.py --ramp-rate-mms 0.5/1).

## 4. Faults -- PASS
- MC state topic stops (feeder stops state at 19.97 s, leg continues, standing, quiet): last held published sample 1.006 s after the last state message = STATE_MAX_AGE_S 1.0 s as documented. Offset before == after release (3.52 mm, no jump, 0.000 mm step); afterwards pass-through (status held=False, standing=True flag stays from the last message but is stale-gated).
- aimdk import failure (wrapper started with AIMDK_PREFIX=/nonexistent): log `[ERROR] ... standing hold ENABLED but aimdk_msgs import failed (No module named 'aimdk_msgs'): hold DISABLED, odometry is pass-through (status hold=null)` then `standing_hold=False`; all 38 status messages have hold=null; odom 741 / TF 750 published, published == raw (max dev 0.0).

## Other observations
- Stopping through launch SIGINT prints a KeyboardInterrupt traceback from the bridge (spin in native_odom_bridge.py main, during shutdown of the wrapper-exec'ed python); cosmetic, exit code -2, not related to the hold. Not checked against the currently deployed bridge.
- A launch SIGTERM to `ros2 launch` once left the exec'ed bridge process orphaned (observed in my own early run, killed by PID); SIGINT shuts down cleanly. Not investigated further.
- Standard compliance: new work here was test tooling only (Python, no ROS nodes added to the project); candidate keeps legacy in-code params (accepted earlier as F3).
