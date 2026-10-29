# Stable final alignment — simulation candidate

This is a small controller around the installed RPP plugin, not a copy of a reference robot driver. Ordinary path tracking stays in RPP. The requested motion remains forward-only with no lateral velocity, linear cap 0.3 m/s and angular cap 0.5 rad/s.

RPP switches immediately between path heading and final goal heading around the goal-checker XY threshold. Small localization corrections can reverse angular commands repeatedly. This wrapper enters final alignment at a tighter approach radius (3 cm for the 5 cm goal tolerance), preserves that phase across replans of the same goal, and uses a proportional angular command capped at 0.5. Small XY corrections are reassessed after finishing the heading phase. A displacement beyond the 10 cm exit window returns to tracking.

After heading alignment, a remaining XY error outside the goal tolerance starts a short correction whose target is frozen in the local `odom` frame. RPP tracks this two-point path until within 1 cm, then the latest global plan is restored and the map-frame goal is checked again. Replans for the same goal cannot redirect that correction as AMCL changes with viewing direction. A new goal resets both phases. Ground truth never enters the controller.

The exit window is **not** a relaxed success tolerance. The installed `StoppedGoalChecker` still requires the original current XY/yaw tolerances and measured rest. Goal completion and action ownership remain in Nav2/Coordinator. The wrapper checks the current circular envelope against the local costmap before stationary rotation; the ordinary RPP collision checks remain active while tracking, including the short correction.

The C++ plugin refuses to configure outside QAZBOT_SIM=1, domain 87, ROS_LOCALHOST_ONLY=1. This candidate is for the ideal kinematic plant. Its small final angular commands are not qualified for the real MC turn threshold or braking dynamics.

Build from the host, with the isolated twin container running:

```bash
bash tools/twin/build_pose_refine.sh
```

An existing loaded library is not overwritten. The build manifest fingerprints source and binary. `QAZBOT_SIM_POSE_REFINE=1` selects the candidate in isolated simulation; the hardware RPP path remains separate.
