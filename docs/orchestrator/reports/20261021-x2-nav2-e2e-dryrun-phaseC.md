# 20261021-x2-nav2-e2e-dryrun — phase C: live dry-run bring-up on X2 hardware DDS (domain 0)

Status: DONE (bring-up left running for operator; see §6, §9, §10)
Artifacts: /home/agi/control_backend/tests/integration_results/live_dryrun_01/

## 0. Preconditions observed (2026-10-21)
- Hardware localization: ros2 launch localization.launch.py PID 284182 (sid/pgid 284182, up 2h15m), children:
  fastlio 284214, level_bridge 284243, map_server 284252, amcl 284289, lifecycle_manager_localization 284308.
  Env: domain unset (=0), ROS_LOCALHOST_ONLY=0, FASTRTPS profile ws/src/navigation/dds_qazbot.xml. Old map in memory.
- CONCURRENT (not mine, not touched): verifier run tests/nav_e2e/run_suite_real.py nav_e2e_verify_01 (PID 384885)
  in domain 87, ROS_LOCALHOST_ONLY=1, dds_domain87_unicast.xml, ns /qazbot_e2e -> isolated from hardware DDS.
- Vendor: orbbec camera launch PID 2411 etc. — not touched.

## 1. BEFORE snapshot (21:4x, hardware DDS domain 0, profile dds_qazbot.xml)
Tool (new, read-only, one subscriber-only node, publishes nothing): tests/nav_e2e/live_graph_snapshot.py OUT [discovery_s=8] [sample_s=4]
(observation windows for evidence, not runtime timeouts). Files: before.json, before.topic_info.txt (`ros2 topic info -v`).
- 93 nodes, 264 topics. Ours: /qazbot_lio_mapping, /qazbot_level_bridge, /qazbot_d1_map/{map_server,amcl,lifecycle_manager_localization}.
- /aima/mc/locomotion/velocity publishers: vendor only (aimrt_planning_planning_node, soc0_rc, app_proxy, soc2_interaction_slave; CLI saw 3 incl. one unnamed). No qazbot node.
- /cmd_vel publishers: vendor /controller_server_ros_node only.
- /qazbot_d1_map/tf: publishers level_bridge (odom->base_footprint 11.5 Hz) + amcl (map->odom 5.75 Hz). /qazbot_d1_map/tf_static: none.
- Vendor /tf: mc_ros2_node, aimrt_perception, camera — joint chain only; no map/odom/base_footprint frames. Vendor /tf_static incl. map->foot_contact (vendor, separate tree).
- SetMcInputSource server present (vendor); our nodes have no clients to it. No action servers in graph.
- MC common state (one read-only sample): action STAND_DEFAULT, fsm_state 4, input_source rc prio 80, body_status 1.
- Side effect: `ros2 topic info` has no --no-daemon -> ros2 daemon (domain 0) started by me (PID 386654); read-only.

## 2. Stop old localization (PID 284182, logs/localization_manual_01.*)
`kill -INT 284182` at 21:44:02. All 6 processes gone within ~1 s (ps). Launch log:
fastlio 284214, map_server 284252, amcl 284289, lifecycle_manager 284308: "finished cleanly".
**level_bridge 284243: exit code 1** — rclpy `take_message` RuntimeError "Unable to convert call argument" raised from
rclpy.spin during SIGINT (bridge catches only KeyboardInterrupt). Shutdown race in level_bridge.py:192, not masked, not fixed (out of scope; finding).
No orphan processes (ps by PID empty).

## 3. Resume (new agent, 21:46) — state found
- Localization relaunched by predecessor 21:44:42: `ros2 launch localization.launch.py lio_binary:=candidates/fast_lio_portable/install/fast_lio/lib/fast_lio/fastlio_mapping map_yaml:=ws/src/maps/first_map_nav2_v2/map.yaml`
  launch PID 387538 (pgid/sid 387538): fastlio 387621, level_bridge 387640, map_server 387645, amcl 387664, lifecycle_manager_localization 387681. Log: /home/agi/.ros/log (launch) — no detach log file name recorded by predecessor.
- Also found started by predecessor (standalone, parent 1, own session), not yet recorded in this report:
  trust `run_trust.sh` -> PID 388248 (logs/trust_dryrun_01.{log,pid}, expected_map_sha256=545630c1… = v2 manifest pgm),
  adapter `run_x2_driver.sh -p live:=false` -> PID 388336 (logs/adapter_dryrun_01.{log,pid}). Kept (correct config, see §4).
### 3.1 lio_binary check
- Launch default `runtime/fastlio_mapping` sha256 87040c81… (the binary named in docs/NAVIGATION_STATUS.md).
- Relaunch uses `candidates/fast_lio_portable/.../fastlio_mapping` sha256 4097314b…  -> DIFFERENT file from the launch default,
  but the SAME binary the previous accepted hardware localization ran with (PID 284182 since 19:25, recorded in
  reports/...-debugger.md §1: `lio_binary:=candidates/fast_lio_portable/...`; it is the stack the operator used in RViz).
- What the candidate is: same FAST-LIO source + `runtime_logging.patch` (does not open build-machine debug paths unless
  runtime_pos_log is requested; removes the destructor fclose(NULL)). Replay evidence logs/replay_portable_01/comparison.json:
  137 common odometry stamps, max_abs_pose_component_difference 0.0, identical scan ray counts; shutdown.json lio exit 0.
  §2 above also shows fastlio "finished cleanly" on SIGINT with this binary.
- Decision: keep PID 387621 (portable) — it is the "previously used" binary on hardware; no relaunch (a relaunch would also
  require the robot standing still for LIO init and adds load). Doc gap (phase B): docs still name only the 87040c81 binary.
### 3.2 Host load (21:46, load avg 11.8/12.5/11.1, 8 cores), top %CPU (ps, lifetime average)
- Vendor: hal_camera 39.8, nvargus-daemon 32.0, orbbec camera_container 30.3, perception 27.6, aima-drp-app 18.0,
  vectorflux 14.0, hal_sensor 12.8, atop recorder 12.7, irq host1x 14.1.
- Ours: fastlio 29.5 (startup average), adapter 17.2 (startup avg incl. imports), trust 12.6, level_bridge 8.2, amcl 4.0,
  lifecycle_manager 3.5, map_server 3.3, ros2 daemon 1.7.
- Other: Claude Code CLI processes (VS Code extension) — not robot software.

## 4. Nav dry-run bring-up #1 (21:49:37, hardware DDS domain 0, dds_qazbot.xml via env.sh)
Command (cwd ws/src/navigation): `ROBOT_RADIUS=0.454 python3 detach.py nav_full_dryrun_01 bash run_nav_full.sh allow_dry_run_driver:=true`
(radius 0.454 = conservative circumscribed rectangle about base_footprint, docs/X2_FOOTPRINT.md; same value as e2e suite; not a physical qualification.)
launch PID 391761: controller_server 391794, velocity_smoother 391825, planner_server 391844, bt_navigator 391872,
lifecycle_manager_navigation 391901, goal_coordinator 391937 (allow_dry_run_driver:=true). Log logs/nav_full_dryrun_01.log.
Trust 388248 and adapter 388336 (live=false) from §3 reused (they are separate processes, not in this launch).
Lifecycle: controller_server, velocity_smoother ACTIVE; planner_server blocked in on_activate: global_costmap waits for
map->base_footprint ("Invalid frame ID map") because AMCL has NO initial pose yet (new map; no stored pose for v2;
trust reasons T1_tf_stale/T1_amcl_pose_stale/T3_no_particles/T4_no_baseline/T6_no_anchor). bt_navigator inactive.
Standard Nav2 behaviour: activation completes once map->odom exists. I did NOT inject an initial pose (wrong pose = hazard;
the operator sets it with "2D Pose Estimate" in RViz -> /qazbot_d1_map/initialpose).
### 4.1 AFTER snapshot: tests/integration_results/live_dryrun_01/after_nav1.{json,topic_info.txt}
- 108 nodes / 309 topics (before 93/264). Nodes added: only /qazbot_d1_map/{goal_coordinator,localization_trust,motion_adapter,
  nav2/*} (+ ros2cli daemon node, tf listeners). Removed: none except an old tf-listener id. Topics removed: none.
- Actions added: /qazbot_d1_map/navigate_to_pose (coordinator, public), /qazbot_d1_map/nav2/{navigate_to_pose,
  navigate_through_poses,compute_path_to_pose,compute_path_through_poses,follow_path}. Before: none.
- `ros2 topic info -v` publishers:
  /aima/mc/locomotion/velocity: aimrt_planning_planning_node4571, soc0_rc3107, app_proxy, soc2_interaction_slave1577 (vendor only; 0 qazbot).
  /cmd_vel: /controller_server_ros_node (vendor) only; 0 subscribers; no qazbot publisher.
  /qazbot_d1_map/cmd_vel: nav2/velocity_smoother only. /qazbot_d1_map/cmd_vel_authorized: goal_coordinator only (sub: motion_adapter).
  /qazbot_d1_map/motion_adapter/dry_run_cmd: motion_adapter. /qazbot_d1_map/goal_pose: 0 pub, 1 sub (coordinator).
- Our nodes: no client of /aimdk_5Fmsgs/srv/SetMcInputSource (per-node client lists empty), no publisher on MC or /cmd_vel.
  Adapter live=false: lock and registration client and ~/register exist only in live mode (x2_motion_adapter.py:300-305);
  status: mode dry_run, registration dry_run, sdk_service_ready false, counters.published 0. No flock file taken.
- TF: /qazbot_d1_map/tf publishers = qazbot_level_bridge (odom->base_footprint 11.75 Hz) + amcl (no map->odom yet, no pose).
  /qazbot_d1_map/tf_static: none. Vendor /tf (aimrt_perception, camera, mc_ros2_node; 31 edges) and /tf_static (22 edges,
  incl. vendor map->foot_contact) contain no odom/base_footprint and no ->map child; our tree is on the private topic -> no conflict.
- MC state sample: STAND_DEFAULT, fsm 4, input_source rc prio 80 (unchanged vs before).
- Note: bt_navigator also subscribes /qazbot_d1_map/nav2/goal_pose (stock Nav2). A pose there would bypass the coordinator's
  goal ownership, but its velocity would NOT be forwarded (coordinator publishes cmd_vel_authorized only while it owns a goal:
  goal_coordinator.command requires self.busy). RViz must use /qazbot_d1_map/goal_pose.

## 5. Shutdown test of nav part: `kill -INT 391761` at 21:52:50.50 (state: planner blocked in activation, see §4)
All 7 processes gone by 21:52:53.59 (~3.1 s), process group empty, no orphans.
- goal_coordinator 391937, velocity_smoother 391825, controller_server 391794, bt_navigator 391872: "finished cleanly".
- lifecycle_manager_navigation 391901: "finished cleanly" (it logged "Failed to change state for node: planner_server ...
  async_send_request failed" / "Aborting bringup" first).
- **planner_server 391844: exit code -6 (SIGABRT)**: SIGINT arrived while on_activate was blocked in the costmap TF wait;
  "could not create service: rcl node's context is invalid" -> errorprocessing -> "terminate called without an active
  exception". Nav2-Humble shutdown-during-activation defect (same family as the known lifecycle shutdown defect). Recorded, not masked.
  Normal-state (all active) shutdown cannot be exercised until the operator sets the AMCL pose; to be re-checked then.

## 6. Nav dry-run bring-up #2 — LEFT RUNNING (21:53:23)
Command: `cd ~/control_backend/ws/src/navigation && ROBOT_RADIUS=0.454 python3 detach.py nav_full_dryrun_02 bash run_nav_full.sh allow_dry_run_driver:=true`
launch PID 394586 (pgid 394586; logs/nav_full_dryrun_02.{log,pid}): controller_server 394618, velocity_smoother 394645,
planner_server 394663, bt_navigator 394683, lifecycle_manager_navigation 394715, goal_coordinator 394732.
State: controller+smoother active; planner_server activating (waits for AMCL map->odom), bt_navigator inactive -> becomes
active automatically after the operator's 2D Pose Estimate. Log grows ~2 lines/s ("Timed out waiting for transform") until then.
Snapshot tests/integration_results/live_dryrun_01/after_nav2.{json,topic_info.txt}: identical publisher sets to §4.1:
MC velocity = 4 vendor nodes, /cmd_vel = vendor controller_server_ros_node, /qazbot_d1_map/tf = level_bridge + amcl,
/qazbot_d1_map/cmd_vel = velocity_smoother, cmd_vel_authorized = goal_coordinator. Coordinator status: busy false, gate false,
driver_mode dry_run, trusted false, ready false. Adapter: idle/driver_not_armed, mode dry_run, registration dry_run, published 0.
No process holds run/x2_motion_owner.lock (file from 20:13 exists, no fd open in adapter/coordinator, not in /proc/locks).
CPU 21:55 (ps lifetime %): ours fastlio 28.4, adapter 11.4, trust 7.5, level_bridge 6.4, coordinator 6.4, controller 4.4,
lifecycle mgr 3.5, smoother 3.3, bt 3.1, amcl 2.9, planner 2.7 (~80 % of one core total); load avg 11.1 (vendor dominates).

### Running set and exact stop commands (stop nav first, then trust/adapter, then localization)
| part | PID | stop |
|---|---|---|
| Nav2 + coordinator (launch) | 394586 | `kill -INT $(cat ~/control_backend/logs/nav_full_dryrun_02.pid)` |
| adapter live=false | 388336 | `kill -INT $(cat ~/control_backend/logs/adapter_dryrun_01.pid)` |
| localization trust | 388248 | `kill -INT $(cat ~/control_backend/logs/trust_dryrun_01.pid)` |
| localization (launch) | 387538 | `kill -INT 387538` (no pid file written by predecessor) |
Check after stop: `pgrep -af "fastlio_mapping|level_bridge|amcl|map_server|planner_server|controller_server|bt_navigator|velocity_smoother|lifecycle_manager|goal_coordinator|x2_motion_adapter|localization_trust"` -> empty.

## 7. RViz (ws/src/navigation/rviz/localization.rviz; backup localization.rviz.bak_20261021_pre_goal_tool)
- Tool `rviz_default_plugins/SetGoal` ("2D Goal Pose") -> `/qazbot_d1_map/goal_pose` (Reliable/Volatile, depth 1). Subscriber is
  goal_coordinator (ns /qazbot_d1_map, `create_subscription(PoseStamped,'goal_pose')`, Reliable/Volatile) -> its own public
  NavigateToPose gate -> single internal bt_navigator. Not /qazbot_d1_map/nav2/goal_pose (bypass, see §4.1).
- Displays added: Path /qazbot_d1_map/nav2/plan (planner, global plan), Path /qazbot_d1_map/nav2/received_global_plan (RPP local
  plan), PointStamped /qazbot_d1_map/nav2/lookahead_point, Polygon /qazbot_d1_map/nav2/local_costmap/published_footprint,
  Pose /qazbot_d1_map/goal_pose, Map /qazbot_d1_map/nav2/local_costmap/costmap (disabled by default — Wi-Fi load).
  QoS of each matches the live publishers (`ros2 topic info -v`: all RELIABLE; footprint TRANSIENT_LOCAL -> volatile sub OK).
- dry_run_cmd (geometry_msgs/Twist): no Twist display in Humble rviz_default_plugins -> not added; watch with
  `ros2 topic echo /qazbot_d1_map/motion_adapter/dry_run_cmd`.
- TF remap unchanged and consistent with run_local_viewer.sh (`-r __ns:=/qazbot_d1_map -r /tf:=/qazbot_d1_map/tf -r /tf_static:=/qazbot_d1_map/tf_static`);
  all topics absolute. run_local_viewer.sh, dds_pc_x2.xml, dds_qazbot.xml, vendor config: not changed.
- Validated: YAML parses (11 displays, 6 tools). Not opened in RViz here (no display on robot; avoid load) -> operator check.

## 8. level_bridge SIGINT exit code 1 (finding, NOT fixed)
Traceback (logs/localization_manual_01.log:102-119): rclpy.spin -> executors._take_subscription -> `sub.handle.take_message`
-> RuntimeError "Unable to convert call argument to Python object" while rclpy's SIGINT handler shuts the context down
(race between signal-triggered context shutdown and an in-progress take). level_bridge.main catches only KeyboardInterrupt.
Not trivial to fix correctly: catching RuntimeError = swallowing; disabling rclpy's signal handler needs a guard-condition/
signal wake-up design (a Python handler does not run while the executor blocks in the C wait) or a periodic spin timeout
(arbitrary). Candidate proper fix for a later task: SignalHandlerOptions.NO + signal.set_wakeup_fd/guard condition, or check
whether ros2/rclpy humble backport fixes take during shutdown. Impact: cosmetic exit code 1; all publishers already stopped.

## 9. Operator procedure (dry-run on hardware DDS; no MC output possible)
1. PC: `bash rviz_live/run_local_viewer.sh` (copy the updated localization.rviz from X2 ws/src/navigation/rviz/ first).
2. "2D Pose Estimate" at the robot's true pose -> /qazbot_d1_map/initialpose. Check Nav2 finished activating:
   `ros2 lifecycle get /qazbot_d1_map/nav2/bt_navigator` -> active [3] (also `grep "Managed nodes are active" logs/nav_full_dryrun_02.log`).
3. Robot standing still at the confirmed pose (on X2, `source ws/src/navigation/env.sh`):
   `ros2 service call /qazbot_d1_map/localization_trust/baseline_start std_srvs/srv/Trigger`, wait for >=20 scan scores (~5 s),
   `.../baseline_finish` (must say success), `.../confirm_anchor`. Then `ros2 topic echo --once /qazbot_d1_map/localization_trust/status` -> trusted true.
4. "2D Goal Pose" tool in RViz -> /qazbot_d1_map/goal_pose. Watch: global/local plan in RViz, `ros2 topic echo /qazbot_d1_map/motion_adapter/dry_run_cmd`,
   `/qazbot_d1_map/goal_coordinator/status`, `/qazbot_d1_map/motion_adapter/status`. Robot does not move (adapter live=false, no MC publisher).
5. Stop: `ros2 service call /qazbot_d1_map/goal_coordinator/stop std_srvs/srv/Trigger` (latches; `.../resume` allows only a NEW goal).

## 10. Open issues
1. Nav2 activation cannot complete until an AMCL pose is given; normal (all-active) SIGINT shutdown therefore not yet re-tested
   on hardware; the activation-blocked SIGINT gave planner_server SIGABRT (-6) (§5).
2. bt_navigator `default_server_timeout` (Nav2 default 20 ms ack) is not set; verify_01 aborted with "Timed out while waiting for
   action server to acknowledge goal request for compute_path_to_pose" at load 11.8. Load here is ~11–12 -> the operator's
   first RViz goal may ABORT the same way (safe outcome, no motion). Needs a measured value, not an arbitrary bump.
3. Physical anchor/initial pose for map v2 not stored (no home.yaml); operator sets it by eye in RViz.
4. Docs name only FAST-LIO sha 87040c81; live stack runs the portable candidate 4097314b (§3.1) -> document in phase B.
5. level_bridge SIGINT exit 1 (§8). Stale run/x2_motion_owner.lock file content from 20:13 (not held).
6. Dry-run PS5 takeover on hardware observable only if MC input source changes (idle source already rc) — implA risk 1.

Status: phase C bring-up DONE, stack LEFT RUNNING for operator RViz dry-run test (no goal sent by me).
