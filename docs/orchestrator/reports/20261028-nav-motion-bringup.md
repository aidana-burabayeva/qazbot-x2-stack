# 20261028-scan-latency-rviz-goal — P5b: Nav2 + MC registration + Motion Adapter + Goal Coordinator up (no goal)
Agent: robotics-debugger on X2, 2026-10-28 20:06-20:13 CST. STATUS: UP, IDLE, NO GOAL SENT. No goal, cmd_vel, initialpose, arm_motion or param set were sent by this agent.
Authorization: DECISIONS.md 2026-10-28 operator answer (2) + PLAN P5b "operator confirmed RC 'можно'" + coordinator message.
Evidence dir: /home/agi/control_backend/docs/evidence/nav_bringup_20261028_195035/

## Pre-checks (pre_nav_mc_state.txt)
- Uptime 6:05 at 20:06 (robot rebooted today ~14:00) -> fresh SDK source_name used (sdk_qazbot_nav2_1028a).
- Vendor MC is NOT a local process we start (MC lives on another SoC: subscriber mc_ros2_node3201). Nothing "started" for MC except ONE SDK input-source ADD.
- MC before: GetCurrentInputSource rc/prio 80; /aima/mc/common/state action_desc STAND_DEFAULT; speed mode 1 (LOW) fwd [-1.0, 0.8], lat [-1,1], ang [-1,1].
- What MC "startup" does (x2_motion_adapter.py:413-453): live:=true starts the node with no ADD and no MC publisher; ~/register sends ONE SetMcInputSource ADD (priority 30, lease 1000 ms), no retry/ENABLE. On ACK it creates the MC velocity publisher but publishes nothing while idle (publishes only while an armed goal is ACTIVE, plus a 5-zero burst on stop). No stance/action change. Verdict: does not move the robot -> proceeded.
- No GC/MA/Nav2 processes were running; localization 372186 + Trust 372377 alive.
- Trust one sample before: ready true, motion_ok false (T1_scan_stale) — transient.

## Started (start_nav.sh; log start_nav_run.txt, adapter.log, nav_full.log; PIDs owned_nav_pids.txt)
Env: source env.sh + aimdk_env.sh, QAZBOT_SMAC_PREFIX=ws/src/navigation/../../../candidates/nav2_smac_bounds/install/nav2_smac_planner, then FASTRTPS_DEFAULT_PROFILES_FILE=<EV>/dds_nav_two_iface_log.xml (SHA 918f941d...), ROS_DOMAIN_ID=0, ROS_LOCALHOST_ONLY=0. Direct python/ros2 launch (not run_x2_driver.sh/run_nav_full.sh, which re-source dds_qazbot.xml).
File SHAs: nav_full 995a0688, nav_control ebea6637, goal_coordinator 3ee7b76c, x2_motion_adapter 09a31d62 (same as 27.10 robot-run versions).
1. Motion adapter PID 387158: x2_motion_adapter.py -r __ns:=/qazbot_d1_map -p live:=true -p priority:=30 -p source_name:=sdk_qazbot_nav2_1028a.
2. nav_full launch PID 387282: robot_radius 0.454, scan /qazbot_lio/scan, odom /qazbot_native/odom, map_yaml first_map_nav2_v3 (NOT the v2 default), max_linear 0.3, max_angular 0.6, xy_tol 0.20, yaw_tol 0.1745 (10 deg), goal_checker_stateful true, allow_dry_run_driver false. = last robot-run config v13 27.10 (~/.ros/log/qazbot_ctl_ewgwlyx7.yaml) and DECISIONS "M1 xy 0.20 stateful".
   Children: controller_server 387302, velocity_smoother 387327, planner_server 387345, bt_navigator 387365, lifecycle_manager_navigation 387372, goal_coordinator 387399.
   nav_full.launch.py contains NO localization/AMCL/FAST-LIO/Trust/level_bridge nodes (only control+planner+bt+lifecycle+GC). Confirmed: no duplicates.
   "Managed nodes are active" at 20:10:04. nav_full.log: 0 error/exception lines.
3. ONE ADD at 20:10:16: adapter status before = idle/not_requested/mc_source rc; Trigger "one request submitted"; adapter log "SDK source registered".

## Verification (post_nav_checks.txt, post_nav_cmd_sample_20s.txt, post_nav_tf_owners.txt)
- Lifecycle: controller_server, velocity_smoother, planner_server, bt_navigator = active [3].
- Actions: /qazbot_d1_map/navigate_to_pose (GC public server) and internal /qazbot_d1_map/nav2/navigate_to_pose listed. (`ros2 action info` crashed in CLI with RCLError "Failed to get action client names" — CLI/daemon issue, not stack; not retried.)
- GC status (20 s sample end): busy false, latched false, gate false, fault "", source_zero true, trusted true, driver_ready true, driver_mode live, ready true, child_uuid null => idle, no goal.
- Adapter: state idle, live, registration registered, mc_source rc, trusted true (= receives Trust), motion_armed false, ready true, foreign_publishers [], published 0.
- Command topics, 20 s single-participant sample: /qazbot_d1_map/cmd_vel 0 msgs, /qazbot_d1_map/cmd_vel_authorized 0 msgs, /aima/mc/locomotion/velocity 0 msgs (all publishers incl. 4 vendor) => no velocity commands at all.
- Graph: cmd_vel 1 pub (velocity_smoother) -> GC; cmd_vel_authorized 1 pub (GC) -> adapter; MC velocity: our adapter (BEST_EFFORT) + 4 known vendor publishers. /qazbot_d1_map/goal_pose: pub rviz (operator PC already connected), sub goal_coordinator. /qazbot_d1_map/nav2/goal_pose: sub bt_navigator only (bypass — do NOT use).
- TF owners /qazbot_d1_map/tf unchanged: amcl (map->odom) + qazbot_native_odom_bridge (odom->base_footprint). tf_static no publishers.
- MC after: input_source rc/80, STAND_DEFAULT (unchanged).
- Trust over 20 s (101 statuses): ready 101/101, motion_ok 83/101; blocked 18 by T1_scan_stale (+2 T1_odom_message_stale, 3 T1_tf_stale). Before Nav2 start (P5a): 293/293. Load avg 29.7 (8 cores) after Nav2 start vs ~17-24 before.

## Operator: RViz 2D Goal Pose
- Topic: /qazbot_d1_map/goal_pose (Fixed Frame map). GC calls trust arm_check + motion_adapter/arm_motion and forwards to nav2. One short goal 1-2 m in free space, along current heading preferred.
- Speed in effect: RPP desired/max linear 0.3 m/s, max angular 0.6 rad/s, smoother max [0.3,0,0.6], accel [0.4,0,1.0], decel [-0.6,0,-1.5]; min approach 0.12. Measured real walking ~0.4-0.5 m/s for commands 0.12-0.3 (G3 22.10), coast 0.14-0.3 m after zero. MC LOW bounds fwd <=0.8.
- Stop paths: (1) physical RC/PS5 stick (rc prio 80 > our 30) / vendor e-stop — fastest; NOTE live takeover latch never proven on hardware (S4 27.10 not proven). (2) software, fastest: pre-sourced terminal:
  ros2 service call /qazbot_d1_map/goal_coordinator/stop std_srvs/srv/Trigger {}   (gate closed, zero, cancel, latch)
  ros2 service call /qazbot_d1_map/motion_adapter/stop std_srvs/srv/Trigger {}      (latch, 5 zeros if active)
  (env: cd ws/src/navigation; source env.sh; export FASTRTPS_DEFAULT_PROFILES_FILE=<EV>/dds_nav_two_iface_log.xml ROS_DOMAIN_ID=0 ROS_LOCALHOST_ONLY=0). CLI start costs seconds under load 30 — prefer RC. (3) bash <EV>/stop_nav.sh (does both services, then SIGINT adapter, then nav_full).
- After any stop: latches require explicit resume + NEW goal; no auto-resume.

## Scripts
- start_nav.sh (this phase), stop_nav.sh (motion first: GC stop -> adapter stop -> SIGINT adapter -> SIGINT nav_full; TERM/KILL only owned PIDs), stop.sh extended (calls stop_nav.sh first, then Trust, then localization); original saved as stop_localization_only.sh.orig.

## Risks / blockers
1. Trust motion_ok 82 % after Nav2 start (T1_scan_stale under load ~30). Goal may be refused at admission or tripped mid-goal (safe side: adapter latches, zeros). Operator may need to retry once with a NEW goal after resume; do not weaken T1.
2. No bag recorder started (minimal observers). Goal analysis will rely on logs (nav_full.log, adapter.log, trust.log) unless orchestrator starts a recorder with BEST_EFFORT override for MC topic (B5).
3. PS5 takeover latch unverified on hardware (operator waived 27.10).
4. fastdds.log growth ~11 MB/h (P4b).
