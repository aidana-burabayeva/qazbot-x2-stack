# odom_hold candidate (standing hold for native leg odometry) -- fix cycle 1

Task 20261103-odom-standing-hold. Nothing here is deployed; running files are untouched.
Logic: odom_hold.py (pure, tested). Bridge diff: native_odom_bridge.diff (vs ws/src/lio/native_odom_bridge.py).

## Fix-cycle-1 changes
- Deploy path (verifier F1): run_localization.sh sources only env.sh, so aimdk_msgs was not importable and the hold
  silently disabled. run_native_odom_bridge.sh sources env.sh + aimdk_env.sh and execs python3 for the bridge process
  only (FAST-LIO/AMCL keep the unmodified env). localization.launch.diff starts the bridge through the wrapper.
  Import failure with standing_hold=true now logs ERROR ("standing hold ENABLED but aimdk_msgs import failed") and
  status hold=null. stack_up_check.snippet.sh makes stack_up.sh fail unless the start line has standing_hold=True.
- Drift cap (F2): hold released when offset absorbed since entry > DRIFT_RATE_MAX_MPS*held_time + DRIFT_SLACK_M.
  DRIFT_RATE_MAX_MPS = 3.8e-4 m/s = 2 x 1.9e-4 (max measured held drift 0.19 mm/s, nav_bag_183504, 155.7 s run);
  slack 1 mm > window noise (0.45 mm). After a cap release the hold stays off (latched) until stand/quiet drops once.
  Counter hold.cap_releases appears in status when it fires. Test: test_drift_cap_releases_fast_ramp_but_not_real_drift.

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
- hold.cap_releases stays 0 during normal standing.

## Rollback
Restore the *.bak_pre_hold files (bridge, launch), delete odom_hold.py and run_native_odom_bridge.sh from the
deployed dirs (optional), remove the stack_up check, restart the stack. Quick disable without redeploy:
add `-p standing_hold:=false` to the bridge args (pass-through = previous behaviour).

## Tests
`source env.sh; source aimdk_env.sh; ROS_DOMAIN_ID=88 ROS_LOCALHOST_ONLY=1 python3 -m pytest -q` -> 16 passed.
Replay (cycle 1): nav_bag_183504 11 entries/10 exits, 0 cap releases, free step dev 0.0; nav_bag_183911 (walking)
7/6, free step dev 0.0 -- identical to the pre-cap results; drift in standing still absorbed (pub ~0 vs raw 1 cm/min).
