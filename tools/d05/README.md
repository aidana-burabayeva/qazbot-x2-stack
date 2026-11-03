# tools/d05 — fixed-command source for protocol P-D05 (MC speed calibration, tape)

Prepared 2026-11-03, NOT run on hardware. Protocol: `docs/hardware/protocols/HW_PROTOCOLS_20261103.md` (P-D05).
Python tooling (not a control node). Parameters are CLI arguments by explicit task requirement
(`--help` is the reference); there is no launch/config file.

| File | Role |
|---|---|
| `const_cmd.py` | rclpy node `d05_const_cmd`: start checks, constant-vx publisher, zero on any end, CSV stamps |
| `const_cmd_core.py` | all logic without ROS (limits, schedule, start/abort conditions); unit-tested |
| `record_d05.sh` | rosbag2 recorder with the protocol topics and best_effort QoS overrides |

Tests: `python3 -m pytest tests/d05 -q` (67 unit tests, 100 % of `const_cmd_core.py`, no sleeps) and
`bash tests/d05/run_dry_integration.sh` (isolated `ROS_DOMAIN_ID=79`, localhost only, stub status publisher, 29 checks).

## 1. Which path the command takes (evidence from code)

```
Nav2 RPP -> cmd_vel_nav -> velocity_smoother (20 Hz) -> /qazbot_d1_map/cmd_vel
         -> goal_coordinator.command() [needs busy goal + gate + trust, sole-publisher check]
         -> /qazbot_d1_map/cmd_vel_authorized (Twist, depth 1)
         -> x2_motion_adapter (50 Hz, live, source sdk_qazbot_nav2_1102a, prio 30) -> /aima/mc/locomotion/velocity
```
* The adapter consumes `/qazbot_d1_map/cmd_vel_authorized` (param `cmd_vel_topic`; `x2_motion_adapter.py`). The tool publishes
  exactly there, with `geometry_msgs/Twist{linear.x=vx, all else 0}`, at 20 Hz (= `controller_frequency` and `smoothing_frequency`
  in `nav_control.launch.py`). The adapter then does what it did in LIVE-2: pass-through for |vx| >= 0.02 (saturation only at the
  live MC bounds, fwd [-1.0, 0.8]), republish to MC at 50 Hz, zero burst of 5 when the command goes zero/stale (timeout 0.15 s).
* `/qazbot_d1_map/cmd_vel` is NOT usable: goal_coordinator trips (`command_publisher_count`, cancel + latch) when it sees a second
  publisher next to the smoother, and forwards only while a goal is busy. Nothing in the tool touches adapter, GC, Trust, T1/T5,
  priorities or the MC topic (the MC velocity topic is only *subscribed*, to see RC activity).
* The tool is a stand-in for the GC output, so it publishes on a topic whose only normal publisher is `goal_coordinator`.
  The adapter-side gates stay fully in force; the GC-side gates are not exercised (see section 2).

## 2. Admission state that must exist (nothing is bypassed in the adapter)

The adapter ignores commands until `motion_armed` is true. `motion_armed` is set only by the adapter service
`/qazbot_d1_map/motion_adapter/arm_motion` (std_srvs/Trigger), which goal_coordinator calls once for each NEW accepted goal
(adapter comment: "called only for a NEW, validated public navigation goal"). `arm_motion` succeeds only if: registered (live),
adapter idle with no zero burst pending, and `_health_fault` is empty = MC state fresh (<0.5 s) + standing + FSM stable/moving,
`map->odom` TF fresh (<1 s), `localization_trust/trusted` true and fresh (<0.6 s), no foreign publisher on the MC velocity
topic, MC source not `pnc`/foreign SDK source. Stop/latch/resume clear `motion_armed`.

Required state before a run (the tool verifies the adapter-visible part and refuses otherwise):
1. LIVE-2 stack as in `go_live.sh`: localization + trust up (`start_loc.sh`), live adapter registered
   (`registration: registered`, `live: true`), MC STAND_DEFAULT, idle source `rc`, adapter `state: idle`, `latch: ""`.
2. Goal coordinator (if running) `busy: false`, `latched: false`; no Nav2 goal. Nobody else publishes on `cmd_vel_authorized`.
3. `motion_armed: true`. The tool never calls services. Arming is a separate explicit step done by CLAUDE with the operator's OK:
   `ros2 service call /qazbot_d1_map/motion_adapter/arm_motion std_srvs/srv/Trigger '{}'`
   This is the one thing that is normally done by GC. It is the same service with the same adapter checks, but GC's own
   admission (trust `arm_check`, `bt_navigator` active, source-zero seen, goal reservation) is skipped. Decision needed from
   orchestrator/operator before the session: accept this manual arm, or run the calibration via real Nav2 goals instead
   (D1 report alternative: `desired_linear_vel` per run; then the command is not constant and this tool is not used).
4. Trust must hold during the walk. A trust loss longer than `t1_pause_max_s` (3 s), TF loss, or MC state change latches the
   adapter (`operator_takeover`, `localization_untrusted`, ...); the tool sees the latch and sends zero. If localization/trust
   cannot be READY in the corridor, `arm_motion` fails (`localization_untrusted`). The only way around is restarting the adapter with
   `require_localization/require_trust:=false` — an adapter configuration change that was NOT made and needs a separate decision.
5. After the last run: `ros2 service call /qazbot_d1_map/motion_adapter/stop std_srvs/srv/Trigger '{}'` disarms (latches) and
   `.../resume` clears the latch (resume does not re-arm). The adapter stays armed (idle) after each run otherwise; any later publisher on
   `cmd_vel_authorized` would move the robot.

RC priority: `rc` has priority 80, the adapter source 30. RC takeover wins at MC; the adapter observes `mc_source` leaving the
SDK source and latches `operator_takeover` (live only). The tool additionally stops on adapter latch, status loss (> 1.5 s),
a non-zero `rc` velocity message and `mc_source == rc` after 2 s.

## 3. Usage

```
source /opt/ros/humble/setup.bash; source ws/src/navigation/aimdk_env.sh      # aimdk_msgs (RC check fails closed without it)
# same DDS profile/domain as go_live.sh (FASTRTPS_DEFAULT_PROFILES_FILE=.../dds_nav_two_iface_log.xml ROS_DOMAIN_ID=0)
tools/d05/record_d05.sh r01_v030 40                      # background; prints bag dir and pid
python3 tools/d05/const_cmd.py --vx 0.30 --duration 10 --run-id r01 [--check-only]
```
Per-run order: place robot -> arm (section 2) -> `record_d05.sh` -> `const_cmd.py` -> after exit `kill -INT $(cat $OUT_ROOT/record_d05.pid)`
(or let the hard stop run out). Protocol runs: (vx, duration) = (0.30,10) (0.10,12) (0.15,12) (0.20,10) (0.25,10) (0.40,8) (0.50,8) (0.60,7);
all accepted by the limits. Ctrl-C / SIGTERM at any time -> zero burst, exit 4.

Command profile: STEP at t=0 to vx, held for `duration` s (the first `settle` s = 2 s are part of it), then explicit zero.
No ramps. Nav2 itself ramps (smoother ~0.4 m/s^2): the abrupt stop corresponds to the "abrupt" stopping distances of the D1 report
(0.17-0.30 m), not the 0.10 m of a ramped Nav2 stop. Reduced vx is NOT lifted or mapped; values go through unchanged.

## 4. API

| Direction | Name | Type | Notes |
|---|---|---|---|
| publishes | `--cmd-topic` (default `/qazbot_d1_map/cmd_vel_authorized`) | `geometry_msgs/Twist` | depth 1, reliable, `rate` Hz; zero burst x10 at the end |
| subscribes | `--adapter-status-topic` (`/qazbot_d1_map/motion_adapter/status`) | `std_msgs/String` (JSON) | start checks + abort monitor |
| subscribes | `--gc-status-topic` (`/qazbot_d1_map/goal_coordinator/status`) | `std_msgs/String` (JSON) | optional; busy/latched refuses |
| subscribes | `--mc-velocity-topic` (`/aima/mc/locomotion/velocity`) | `aimdk_msgs/McLocomotionVelocity` | observe only: RC activity |
| services/actions | none | | arming is external |

Start refusal (exit 3, nothing published): adapter status missing/older than 1 s; not live; registration != registered; not armed;
state != idle; latch set; ready != true; foreign publishers; mc_source pnc; t1_pause; GC busy/latched; any other publisher on the
command topic; no subscriber on the command topic; a non-zero `rc` velocity within the last 1 s (the tool always listens 1.2 s first);
`aimdk_msgs` missing.

Exit codes: 0 ok, 2 limits, 3 refused, 4 aborted (signal / adapter latch / status loss / RC), 5 exception. Zero is sent in every
path after the run started (and on exceptions).

## 5. Parameters (CLI)

| Arg | Type | Default | Description |
|---|---|---|---|
| `--vx` | float | required | constant forward velocity, 0.02 <= abs(vx) <= 0.60 m/s (negative = reverse; not in the protocol) |
| `--duration` | float | required | total command time incl. settle, 0 < d <= 12 s, and >= settle + 1 s |
| `--settle` | float | 2.0 | settle phase inside the duration (only the `settle_end` stamp; command unchanged) |
| `--rate` | float | 20.0 | publish rate [Hz], 20..50 (Nav2 rate = 20) |
| `--run-id` | str | "" | label in every CSV row |
| `--log-csv` | path | `tools/d05/logs/const_cmd_log.csv` | appended; header on creation |
| `--cmd-topic`, `--adapter-status-topic`, `--gc-status-topic`, `--mc-velocity-topic` | str | see API | topic names |
| `--preflight-timeout` | float | 5.0 | seconds to collect status/RC observations before refusing |
| `--check-only` | flag | off | run the start checks, publish nothing |

CSV columns: `run_id,event,wall_iso,ros_time_ns,mono_s,vx_cmd,duration_s,settle_s,rate_hz,msgs_sent,reason,adapter_state,adapter_mc_source`;
events `start`, `settle_end`, `cmd_end` (last command instant, before the zero burst), `stop` | `abort`, or `refused`.
`wall_iso` is UTC; `ros_time_ns` is the node clock (system time) for joining with the bag.

## 6. record_d05.sh

`record_d05.sh RUN_TAG [MAX_SECONDS=45]` (hard cap 120): `ros2 bag record -s mcap` of `/aima/mc/locomotion/velocity`,
`/aima/mc/common/state`, `/aima/mc/leg_odometry`, `/qazbot_lio/odometry`, `/qazbot_native/odom`,
`<NS>/motion_adapter/status`, `<NS>/cmd_vel_authorized` (no clouds). A generated `<bag>.qos.yaml` sets best_effort/keep_last/50 on the five vendor/odometry topics
(needed on the MC velocity topic where `rc` and `sdk_*` sources publish with different QoS). Output default
`docs/evidence/d05_20261103/`; env overrides: `OUT_ROOT`, `ROS_DOMAIN_ID` (default 0), `DDS_PROFILE` (default the
two-iface profile of LIVE-2; `none` = localhost-only for isolated tests), `NS`. `nice -n 10`, `timeout -s INT`.

## 7. Verification status and limits

* Unit tests and the domain-79 dry run pass (nominal 4 s run: 80 messages, mean dt 49.4 ms, max 53 ms, 10 zeros, CSV
  start/settle_end/cmd_end/stop; refusals for not armed / RC active / dry adapter; limits; latch mid-run; SIGINT).
* Not verified (needs hardware or the real adapter): real status JSON values after `arm_motion`; that `rc` streams non-zero values on this
  topic exactly when the stick moves (D1 report saw ~49 Hz rc traffic); acquisition of the MC source by the first command.
* Stub status is hand-built from `x2_motion_adapter.py` `_report()`; if the adapter's status keys change, `adapter_refusals()` fails closed.
