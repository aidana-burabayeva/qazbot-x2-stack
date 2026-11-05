# qazbot_approach_governor

Forward-command governor for the X2 Motion Controller (D-05). It sits between the Nav2 velocity smoother and the Goal
Coordinator and can **only shorten** the forward command: when the remaining path length is not larger than the
predicted stopping distance (`measured_speed * stop_time_s + stop_margin_m`) it forces `vx = 0` and keeps it at zero until
the robot has really stopped. `vy`, `wz` and every other field are forwarded untouched (P20/P23 yaw loop and RPP
rotate-to-heading are unaffected). Missing or stale inputs never add a restriction: the command passes unchanged.

**Status: prepared offline, NOT installed, NOT wired into production.** See `docs/results/20261105_d05_governor.md`
for the data, the limits of this fix and the physical verification plan.

## Why
E2E-02 (3 m free goal, `docs/results/20261105_e2e02_free3m.md`): MC walks at ~0.5 m/s for any forward command >= ~0.1 m/s,
has ~0.4 s dead time and coasts 0.07-0.31 m after the command goes to zero. RPP commands zero only inside the goal
tolerance, so the robot overshoots (0.19 m) and RPP turns it around in place.

## Nodes
### approach_governor (`qazbot_approach_governor::ApproachGovernorNode`, composable; executable `approach_governor`)
Single-threaded executor. Logic is in the ROS-free core (`ApproachGovernor`, `SpeedEstimator`, `PathProgress`).

| Interface | Type | Notes |
|---|---|---|
| `cmd_vel_in` (sub) | geometry_msgs/Twist | smoother output (remap) |
| `cmd_vel_out` (pub) | geometry_msgs/Twist | to Goal Coordinator; absent when `shadow:=true` |
| `plan` (sub) | nav_msgs/Path | Nav2 `received_global_plan`, frame must be `plan_frame` |
| `odom` (sub) | nav_msgs/Odometry | `/qazbot_lio/odom_level`, position only (speed = displacement/time over `speed_window_s`) |
| `/tf` (sub) | TF | `plan_frame` -> `base_frame` (remap to `/qazbot_d1_map/tf`) |
| `status` (pub) | diagnostic_msgs/DiagnosticStatus | state, speed, remaining, stop distance, counters |

### Parameters (`config/approach_governor.yaml`; dynamic: all doubles)
| Name | Type | Default | Description |
|---|---|---|---|
| `stop_time_s` | double | 0.40 | stop distance = speed * this (E2E-02: fit 0.38 s, 8 stops, residual sd 0.054 m) |
| `stop_margin_m` | double | 0.0 | extra distance added |
| `speed_window_s` | double | 0.30 | displacement window of the speed estimate |
| `max_speed_age_s` | double | 0.50 | pose older than this -> speed unknown |
| `max_plan_age_s` | double | 3.0 | plan older than this -> remaining unknown |
| `max_pose_age_s` | double | 0.30 | TF pose older than this -> remaining unknown |
| `stopped_speed_mps` | double | 0.08 | below this the robot is stopped (latch released) |
| `max_brake_hold_s` | double | 2.0 | latch released after this long without a stop (fail-open) |
| `min_forward_cmd` | double | 0.02 | vx below this is not a forward command |
| `shadow` | bool | false | report only, publish no command |
| `plan_frame`, `base_frame` | string | map, base_footprint | |
| `status_period_s` | double | 0.5 | |

## Usage
`ros2 launch qazbot_approach_governor approach_governor.launch.xml` (copy the YAML to change values). Production wiring
(remap smoother output to `cmd_vel_smoothed`, governor output `cmd_vel`) is described in the launch file and is NOT applied.
Shadow observation on a live graph: `shadow:=true` (no command is published).

## Safety properties (tested)
- never increases |vx|, never changes vy/wz (property test, 20000 random cases);
- latch keeps vx = 0 while the speed estimate falls (no re-acceleration), released when stopped or after `max_brake_hold_s`;
- no plan / no pose / no speed -> pass-through (previous behaviour); node death -> no command (Goal Coordinator zeroes);
- zero, reverse and pure-rotation commands are never modified.

## Tests
`colcon test`: gtest core (18, line coverage 94.6 %), simulation regression (labelled SIMULATION, data-derived plant),
launch_testing node integration (domain 79). The DDS vendor profile is not needed for tests.
