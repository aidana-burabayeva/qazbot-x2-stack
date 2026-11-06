# qazbot_motion_control

`x2_motion_adapter.py`: the only bridge from Nav2 velocity to the AgiBot MC. Reads `/qazbot_d1_map/cmd_vel_authorized`
(goal_coordinator output) and, when `live:=true`, registers the SDK input source and publishes
`aimdk_msgs/McLocomotionVelocity` on `/aima/mc/locomotion/velocity` at 50 Hz (BEST_EFFORT). In dry run
(`live:=false`, default) it publishes only `~/dry_run_cmd` and creates no `/aima/*` publisher or client.

Docs consulted (AgiBot first): `Interface/control_mod/locomotion.html` 5.1.2 "走跑控制话题" (topic, 50 Hz recommendation,
QoS), 5.1.3 "MC控制信号设置" (input-source arbitration, `SetMcInputSource`, SDK priority 20..39, `sdk_` prefix),
`~/aimdk/topics_and_services`.

**C++ migration candidate (HIGH):** 50 Hz command loop incl. yaw loop on the safety path in Python (GIL jitter).
Not rewritten in the consolidation (no logic change allowed); tracked in AGENTS.md.

## Usage
```bash
source <install>/qazbot_bringup/lib/qazbot_bringup/env.sh; source <install>/qazbot_bringup/lib/qazbot_bringup/aimdk_env.sh
ros2 run qazbot_motion_control x2_motion_adapter.py --ros-args -r __ns:=/qazbot_d1_map            # dry run
# production (operator permission): qazbot_bringup x2_navigation.launch.xml mode:=production
```

## API (`/qazbot_d1_map/motion_adapter`)
Sub: `cmd_vel_topic` (Twist), `tf_topic` (TFMessage), `odom_topic` (Odometry), `trust_topic` (Bool), `trust_status_topic`
(String), `/aima/mc/common/state` (McCommonState). Pub: `~/status` (String JSON, 2 Hz), `~/dry_run_cmd` (Twist),
live only `/aima/mc/locomotion/velocity`. Srv: `~/stop`, `~/resume`, `~/arm_motion`, live only `~/register` (Trigger;
kept for compatibility, refused once the automatic registration sequence has run).
Client (live): `/aimdk_5Fmsgs/srv/SetMcInputSource`. Live mode takes the cooperative lock `lock_path`.

| Parameter | Type | Default | Description |
|---|---|---|---|
| `live` | bool | false | publish to MC (production profile: true) |
| `priority` | int | 30 | SDK input-source priority (20..39) |
| `registration_action` | string | add_then_enable | only accepted value (anything else, incl. the former ADD / ENABLE, refuses to start). Live: once per process, automatically when `/aimdk_5Fmsgs/srv/SetMcInputSource` is discovered: ADD; if ADD code != 0 exactly one ENABLE (official example `mc_locomotion_velocity.py`, "likely already registered"); ENABLE != 0 or an exception -> `registration=failed`, no MC publisher, no further call. Never DELETE |
| `source_name` | string | sdk_qazbot_nav2 | input source name (must start with sdk_) |
| `publish_hz` / `controller_hz` | float | 50.0 / 20.0 | MC publish rate / expected Nav2 rate |
| `cmd_vel_topic` | string | /qazbot_d1_map/cmd_vel_authorized | input velocity |
| `tf_topic` | string | /qazbot_d1_map/tf | localization liveness |
| `odom_topic` | string | /qazbot_native/odom | odometry (production: /qazbot_lio/odom_level) |
| `trust_topic` / `trust_status_topic` | string | /qazbot_d1_map/localization_trust/{trusted,status} | trust gate |
| `lock_path` | string | ~/control_backend/run/x2_motion_owner.lock | motion owner lock (live mode only; the directory is created on demand). On the robot `~/control_backend` is the workspace repository, so this is `<repo>/run/` (gitignored); the legacy stack used the same path, so legacy and new stack exclude each other |
| `allow_lateral`, `require_localization`, `require_trust` | bool | false, true, true | safety gates |
| `tf_max_age_s`, `t1_pause_max_s` | float | 1.0, 3.0 | freshness / pause limits |
| `yaw_loop`, `yaw_u_min_ccw`, `yaw_u_min_cw`, `yaw_u_max`, `yaw_ki` | bool/float | true, 0.6, 0.45, 1.0, see code | in-place yaw-rate loop (P20/P23) |
| `vendor_publishers` | string[] | VENDOR_PUBLISHERS | regexes of vendor MC publishers to report |

Docs: `docs/X2_MOTION_ADAPTER.md`.
