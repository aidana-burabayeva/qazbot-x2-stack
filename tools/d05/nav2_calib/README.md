# tools/d05/nav2_calib — D-05 calibration through real Nav2 goals (P-D05, option B)

Prepared 2026-11-03, dry-tested only (domain 81, real Humble 1.1.18 controller_server + velocity_smoother, live params).
Replaces `tools/d05/const_cmd.py` for P-D05 (direct `cmd_vel_authorized` publishing was rejected: it bypassed GC
admission, needed manual arming and had a step profile).

| File | Role |
|---|---|
| `nav2_calib.py` | operator runner (rclpy node `d05_nav2_calib`) |
| `calib_core.py` | logic without ROS (envelope/percentage, preflight, goal tracking, steady/ramp analysis) |
| `record_calib.sh` | rosbag2 recorder: commanded chain + actual odometry + status topics |

## What the runner does — and does not
1. Reads (GetParameters, read-only) the live `controller_server` / `velocity_smoother` params; refuses a target above
   `min(FollowPath.desired_linear_vel, max_velocity[0])` (today 0.30) or below 0.05 m/s (exit 2).
2. Preflight, read-only (exit 3 on failure): GC status fresh, `ready`, not busy, not latched; adapter live + registered,
   no latch, no foreign MC publisher; `controller_server` subscribed to `<ns>/nav2/speed_limit`; no other publisher there.
3. Starts `record_calib.sh` (unless `--no-record`).
4. Publishes `nav2_msgs/SpeedLimit{percentage: true, speed_limit: 100*target/desired}` on `<ns>/nav2/speed_limit`
   (refresh 1 Hz while the run is open). Percentage <= 100 can only LOWER RPP's speed.
5. Waits for the operator's RViz goal (it never sends goals), follows it via `goal_coordinator/status` (`busy`),
   keeps recording `--post-roll-s` after the goal ends, restores `NO_SPEED_LIMIT` (0.0), stops the bag, writes
   `calib_<run>_<HHMMSS>.meta.json` (live params, limit, estimate, events, goal timing, commanded summary:
   limit honoured, steady command duration, observed smoother ramps; operator tape fields to fill in).
6. If stopped (Ctrl-C/timeout) while a goal is still busy it LEAVES the lower limit (safe side) and prints the restore
   command: `nav2_calib.py --run-id restore --restore-only`.

Never: publishes velocity, calls GC/adapter/trust services, changes params, sends goals. GC admission, gate, trust,
T1 pause, adapter arming per goal, MC checks and RC takeover are all unchanged and in the path.

## Usage (robot domain, stack as LIVE-2)
```
source /home/agi/control_backend/ws/src/navigation/env.sh
export FASTRTPS_DEFAULT_PROFILES_FILE=/home/agi/control_backend/docs/evidence/nav_20261102_live/dds_nav_two_iface_log.xml
python3 tools/d05/nav2_calib/nav2_calib.py --run-id r01_030 --target-vx 0.30 --path-len-m 4.4
```
| Arg | Default | Meaning |
|---|---|---|
| `--run-id` | required | `[A-Za-z0-9_.-]`, used in bag/meta names |
| `--target-vx` | required | m/s, 0.05 .. live envelope |
| `--ns` | `/qazbot_d1_map` | stack namespace |
| `--path-len-m` | 4.0 | planned goal distance (metadata + steady estimate) |
| `--gain` | 1.61 | assumed real/cmd ratio, estimate only |
| `--goal-wait-s` / `--max-goal-s` / `--post-roll-s` | 300 / 120 / 4 | timeouts, recording tail |
| `--record/--no-record`, `--out-root` | on, `docs/evidence/d05_nav2calib` | recorder |
| `--restore-only` | off | publish NO_SPEED_LIMIT if GC not busy |
Exit: 0 done, 2 invalid/envelope, 3 refused, 4 aborted/timeout, 5 internal. Recorder env: `DDS_PROFILE`, `ROS_DOMAIN_ID`.

## Topics
Pub: `<ns>/nav2/speed_limit` (nav2_msgs/SpeedLimit). Sub: `<ns>/goal_coordinator/status`, `<ns>/motion_adapter/status`
(std_msgs/String JSON), `<ns>/nav2/cmd_vel_nav`, `<ns>/cmd_vel` (Twist). Services used: `<ns>/nav2/{controller_server,
velocity_smoother}/get_parameters`. Recorded: MC velocity/state, leg odom, `/qazbot_lio/odometry`, `/qazbot_native/odom`,
`cmd_vel_nav` -> `cmd_vel` -> `cmd_vel_authorized`, speed_limit, GC/adapter/trust status, `nav2/plan`, `goal_pose`,
both navigate_to_pose action status topics.

## Tests
`python3 -m pytest tests/d05/nav2_calib -q` (52 unit tests, 100 % of `calib_core.py`, no sleeps).
`bash tests/d05/nav2_calib/run_dry_nav2.sh` (17 checks, ROS_DOMAIN_ID=81, localhost only; harness sends FollowPath
goals to a stationary controller to emulate the operator goal).

Standard deviations: CLI args instead of YAML config and no launch file (one-shot operator tool, not a stack node).
