# qazbot_navigation

Nav2 for the X2 (namespace `/qazbot_d1_map/nav2`): Smac 2D planner (bounds-patched build from `ws/third_party`),
Regulated Pure Pursuit controller, velocity smoother, bt_navigator, plus the **goal coordinator** gate.
Command chain (nothing here talks to MC or the vendor `/cmd_vel`):
`controller_server → nav2/cmd_vel_nav → velocity_smoother → /qazbot_d1_map/cmd_vel → goal_coordinator →
/qazbot_d1_map/cmd_vel_authorized → x2_motion_adapter (qazbot_motion_control)`.

Docs consulted: AgiBot `Interface/control_mod/locomotion.html` "走跑控制话题" (MC velocity input is owned by
qazbot_motion_control), `common/warnings.html` (≤ 2 nodes/s → 0.6 s stagger). Nav2 parameters: docs/X2_NAV2_CONTROLLER.md.

## Usage
```bash
source <install>/qazbot_bringup/lib/qazbot_bringup/env.sh
ros2 launch qazbot_navigation nav_full.launch.py robot_radius:=0.454 scan_topic:=/qazbot_lio/scan \
  odom_topic:=/qazbot_lio/odom_level allow_dry_run_driver:=true map_yaml:=...
```

## Launch files (Python on purpose)
Parameters are computed from arguments (footprint padding from the map resolution, RPP angular acceleration from
`max_angular`, SIM-only gates) and written to `$ROS_LOG_DIR/qazbot_{ctl,plan}_*.yaml`; this legacy logic stays in Python
(Henki: legacy launch kept; not rewritten during the structural consolidation).

| File | Starts |
|---|---|
| `nav_full.launch.py` | controller_server, velocity_smoother, planner_server, bt_navigator, lifecycle_manager_navigation, goal_coordinator (production) |
| `nav_control.launch.py` | controller_server + velocity_smoother + lifecycle_manager_control (command path only) |
| `nav_plan.launch.py` | planner + bt_navigator with a plan-only BT (no velocity output) |
| `nav_debug_bounds.launch.py` | debug variant of nav_full for the Smac bounds issue |

`nav_full.launch.py` arguments: `ns` (`qazbot_d1_map`), `scan_topic` (required), `odom_topic` (required), `max_linear` (0.3),
`max_angular` (0.6), `xy_tol` (0.30), `yaw_tol` (0.35), `goal_checker_stateful` (true), `local_frame` (odom),
`robot_radius` (required), `map_yaml` (`qazbot_maps/maps/first_map_nav2_v2/map.yaml`), `allow_dry_run_driver` (false),
`autostart` (true), `cell_margin` (true).
The qualified Smac overlay is required: `QAZBOT_SMAC_PREFIX` if set, else the `nav2_smac_planner` selected by the ament
index; its `qazbot_bounds_fix.json` must match the library sha256 (written by `ws/tools/build.sh`).

## goal_coordinator (`/qazbot_d1_map/goal_coordinator`, Python, 20 Hz gate)
Sub `goal_pose` (PoseStamped), `cmd_vel` (Twist), `localization_trust/trusted|status`, `motion_adapter/status`,
`nav2/bt_navigator/transition_event`; pub `cmd_vel_authorized` (Twist), `~/status` (String JSON); srv `~/stop`, `~/resume`;
clients `localization_trust/arm_check`, `motion_adapter/arm_motion`, `nav2/bt_navigator/get_state`, action NavigateToPose.

| Parameter | Type | Default | Description |
|---|---|---|---|
| `allow_dry_run_driver` | bool | false | accept a dry-run adapter as driver |
| `trust_max_age_s` | float | 0.6 | trust verdict max age |
| `t1_pause_max_s` | float | T1_PAUSE_MAX_S | pause (not latch) on T1-only loss up to this long |
| `alignment_stagnation_s` | float | 30.0 | bound for final alignment without progress |

## Data
`bt/*.xml` (navigate_to_pose: PipelineSequence of RateController(1 Hz) ComputePathToPose + FollowPath), `rviz/localization.rviz`
+ `rviz/run_local_viewer.sh` (PC viewer, DDS profile `rviz/dds_pc_x2.xml`).

## Tests
`colcon test --packages-select qazbot_navigation`; tests that evaluate nav_full parameters skip unless the qualified
Smac overlay is resolvable.
