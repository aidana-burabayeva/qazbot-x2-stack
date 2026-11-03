# odom_hold candidate (standing hold for native leg odometry) -- fix cycle 2

Task 20261103-odom-standing-hold. Nothing here is deployed; running files are untouched.
Logic: odom_hold.py (pure, tested). Bridge diff: native_odom_bridge.diff (vs ws/src/lio/native_odom_bridge.py).

## Fix-cycle-1 changes
- Deploy path (verifier F1): run_localization.sh sources only env.sh, so aimdk_msgs was not importable and the hold
  silently disabled. run_native_odom_bridge.sh sources env.sh + aimdk_env.sh and execs python3 for the bridge process
  only (FAST-LIO/AMCL keep the unmodified env). localization.launch.diff starts the bridge through the wrapper.
  Import failure with standing_hold=true now logs ERROR ("standing hold ENABLED but aimdk_msgs import failed") and
  status hold=null. stack_up_check.snippet.sh makes stack_up.sh fail unless the start line has standing_hold=True.
- Drift cap (F2, cycle 1 = per hold run, SUPERSEDED by the per-episode budget above): hold released when offset absorbed since entry > DRIFT_RATE_MAX_MPS*held_time + DRIFT_SLACK_M.
  DRIFT_RATE_MAX_MPS = 3.8e-4 m/s = 2 x 1.9e-4 (max measured held drift 0.19 mm/s, nav_bag_183504, 155.7 s run);
  slack 1 mm > window noise (0.45 mm). After a cap release the hold stays off (latched) until stand/quiet drops once.
  Counter hold.cap_releases appears in status when it fires. Test: test_drift_cap_releases_fast_ramp_but_not_real_drift.

## Fix-cycle-2 changes (launch test check 3: per-run cap was defeated by hold re-entry, 1 mm/s hid 10.5 mm/45 s)
- Absorption budget is now CUMULATIVE per MC standing episode (odom_hold.py). Episode = continuous MC standing state
  (fresh state message with the standing fields); it ends only when MC leaves standing or the state goes stale.
  budget(t) = 2 x 0.19 mm/s x (t - episode_start) + 1 mm (DRIFT_RATE_MAX_MPS = 3.8e-4, DRIFT_SLACK_M = 1e-3, unchanged).
  absorbed = |offset - offset_at_episode_start| (vector), accumulated over ALL hold entries of the episode.
  absorbed > budget -> hold released and LATCHED released for the rest of the episode (no re-entry until MC leaves
  standing and comes back). Yaw never held; exits jump-free (offset kept constant on release).
- Bridge main: catches KeyboardInterrupt and ExternalShutdownException, destroy_node/try_shutdown in finally (clean
  SIGINT/SIGTERM exit through the rclpy shutdown path, no traceback).
- Tests: `python3 -m pytest -q` -> 21 passed (new: episode budget, cumulative over re-entries, latch, new episode, stale
  state, jump-free release).

### Fix-cycle-2 results (domain 80 / offline pure-python; replay/ramp_off.py, replay/replay_results_f2.json)
- 13 offline replays (all bags of cycle 1): hold counts, free-step deviation (<= 1.3e-15 m), walking and standing drift
  identical to cycle 1, 0 cap releases; standing drift pub 0.00-0.008 cm/min vs raw 1.0 cm/min (183504: 0.278 vs 0.769 incl. shove).
  183911 launch replay (ros2 bag play): 6 entries / 5 exits, step dev 0.0, net 3.0959 m -- unchanged.
- Slow displacement (183504 standing segment, ramp from +15 s), published-vs-raw hidden vector:
  0.5 mm/s: no release in 60 s (hidden 21.4 mm < budget); in 150 s release at episode t = 90.4 s, hidden at release
  35.34 mm = budget 35.37 mm, latched (0 held samples afterwards). 1 mm/s: hold flickers (150-400 short entries), absorbs
  ~0.2 mm/s < 0.38 mm/s budget rate, never released: hidden along the ramp = 11.8 mm/45 s (26% of motion), 34.5 mm/135 s;
  inside the budget by construction (rate cap 0.38 mm/s) but NOT released. 2 and 5 mm/s: released after 0.3 / 0.1 s,
  hidden 2.8 mm (pre-ramp vendor drift absorbed before the ramp started + 0.4 mm), no re-entry.
- Bridge shutdown: main catches KeyboardInterrupt/ExternalShutdownException; SIGINT is ignored during cleanup (the double
  SIGINT from launch + process group interrupted destroy_node in the first cycle-2 run -> traceback). Live re-test pending.

## Deploy (robot, stack stopped)
1. cd /home/agi/control_backend/ws/src; backup: cp lio/native_odom_bridge.py lio/native_odom_bridge.py.bak_pre_hold;
   cp navigation/localization.launch.py navigation/localization.launch.py.bak_pre_hold
2. cp candidates/odom_hold/odom_hold.py candidates/odom_hold/native_odom_bridge.py -> ws/src/lio/
   cp candidates/odom_hold/run_native_odom_bridge.sh -> ws/src/navigation/ (chmod +x)
   cp candidates/odom_hold/localization.launch.py -> ws/src/navigation/ (diff: localization.launch.diff)
3. Insert stack_up_check.snippet.sh into tools/stack/stack_up.sh after the native_odom_bridge wait_log (line ~60).
4. Start with QAZBOT_ODOM_SOURCE=native_imu.

## Post-deploy checks
- Startup log: `native_odom_bridge started: ... standing_hold=True`; no `standing hold ENABLED but` ERROR.
- Robot standing: `ros2 topic echo /qazbot_native/status` -> hold.held=true, hold.standing=true (after ~0.5 s).
- Walk/goal: hold.held=false during motion; odom->base_footprint continuous, no jump at hold exit.
- hold.cap_releases stays 0 during normal standing (a release = >0.38 mm/s absorbed on average: real slow displacement).

## Rollback
Restore the *.bak_pre_hold files (bridge, launch), delete odom_hold.py and run_native_odom_bridge.sh from the
deployed dirs (optional), remove the stack_up check, restart the stack. Quick disable without redeploy:
add `-p standing_hold:=false` to the bridge args (pass-through = previous behaviour).

## Tests
`source env.sh; source aimdk_env.sh; ROS_DOMAIN_ID=88 ROS_LOCALHOST_ONLY=1 python3 -m pytest -q` -> 21 passed (cycle 2).
Replay (cycle 1): nav_bag_183504 11 entries/10 exits, 0 cap releases, free step dev 0.0; nav_bag_183911 (walking)
7/6, free step dev 0.0 -- identical to the pre-cap results; drift in standing still absorbed (pub ~0 vs raw 1 cm/min).
