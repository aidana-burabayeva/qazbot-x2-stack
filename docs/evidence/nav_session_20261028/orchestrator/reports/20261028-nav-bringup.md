# 20261028-scan-latency-rviz-goal — P4b nav bring-up (robotics-debugger, on X2)

Status: localization + Trust RUNNING, waiting for operator initialpose. Nav2 NOT started (see Blocker 1).
Evidence dir: /home/agi/control_backend/docs/evidence/nav_bringup_20261028_195035 (start.sh, stop.sh, localization.log, trust.log, fastdds.log, owned_pids.txt, owned_env.txt, checks*.txt, trust_status.txt, top_*.txt, preflight.txt, dds_provenance.txt)

## Preflight (19:50:35)
- No QazBot stack procs (pattern incl. FAST-LIO/level_bridge/native/AMCL/map_server/lifecycle/Trust/GC/MA/Nav2 servers/probes): NONE.
- Vendor HAL hal_sensor_orin PID 204720 running (untouched). Camera 3361/2936, perception 4674, pnc 4787, ros2-daemon 74445 untouched.
- PTP: clock jump + SYNCHRONIZATION_FAULT at 19:22:31 (before P1 run A); at 19:50 ptp4l/phc2sys in s2, offset ~600 ns. Not re-checked with age_probe (no probes rule).
- Load 10.1 (8 cores) before start.

## DDS log fix (operator-approved)
- New file (original untouched, no vendor/global XML touched): /home/agi/control_backend/docs/evidence/nav_bringup_20261028_195035/dds_nav_two_iface_log.xml
  SHA256 918f941dd5654a94c4654e3c544ab3f930cc6eacc4f9cbf48fb4b67da534b3a0
  Source: docs/evidence/latency_20261028_192628/dds_nav_two_iface.xml SHA 29d4c736a364691689ccc8f27c62f566c65af946df1b6fc9dd5ec03ae239ec63 (UDPv4 only, whitelist 10.0.1.41 + 192.168.1.72).
- Diff: comment + top-level <log><use_default>FALSE</use_default><consumer>FileConsumer filename=/home/agi/control_backend/docs/evidence/nav_bringup_20261028_195035/fastdds.log append TRUE</consumer></log>.
- Why FileConsumer, not "verbosity Error": Fast DDS 2.14.6 (libfastrtps.so.2.14.6) XML <log> has only use_default/consumer/thread_settings, no verbosity knob; StdoutErrConsumer would still print warnings. The vendor build tags discovery/match events as [Warning], so they cannot be filtered by threshold. Same pattern as dds_qazbot.xml (FileConsumer).
- Result: localization.log has 0 DATA_FRAG and 0 Fast DDS [Warning] lines (was 15,590 DATA_FRAG in ~4 min in P1 run A). fastdds.log: 24 DATA_FRAG in first 3 min; steady growth ~3 KB/s (~11 MB/h) without CLI; mostly discovery churn (qazbot_speech participant appears/disappears repeatedly). Disk free 366 GB.

## How it was started (reproducible: /home/agi/control_backend/docs/evidence/nav_bringup_20261028_195035/start.sh)
cd ws/src/navigation; source ./env.sh; then OVERRIDE: FASTRTPS_DEFAULT_PROFILES_FILE=/home/agi/control_backend/docs/evidence/nav_bringup_20261028_195035/dds_nav_two_iface_log.xml ROS_DOMAIN_ID=0 ROS_LOCALHOST_ONLY=0.
1) setsid nohup ros2 launch ws/src/navigation/localization.launch.py (DIRECT, not run_localization.sh)
   lio_binary:=/home/agi/control_backend/candidates/fast_lio_portable/install/fast_lio/lib/fast_lio/fastlio_mapping
   map_yaml:=/home/agi/control_backend/ws/src/maps/first_map_nav2_v3/map.yaml lio_cpus:=5,6
   seed_pose_file:=/nonexistent/qazbot_seed_disabled.json odom_source:=native_imu   > localization.log
   localization.launch.py SHA feebadbb875b240d... (AMCL P2a params). Ready 19:51:05: "Managed nodes are active", levelled=1, native_odom_bridge started.
2) setsid nohup python3 localization_trust_node.py --ros-args -p map_yaml:=<v3 map> -p expected_map_sha256:=f6032f75ffd386361993a4b4864cf0816713fc27f6d037c88d929ae5d48562fb -p admission_policy:=go2 -p odom_source:=native_imu -p odom_topic:=/qazbot_native/odom   > trust.log (pose_store default '' = off). Started 19:51:10. localization_trust_node.py SHA 6b36ea0e... (unchanged).
Stop: bash /home/agi/control_backend/docs/evidence/nav_bringup_20261028_195035/stop.sh (SIGINT Trust, SIGINT launch, TERM then KILL only for owned PIDs that ignore INT; known rclpy/FastDDS shutdown futex deadlock from P1 A0).

## Owned PIDs (sessions 372186 launch, 372377 Trust)
372186 ros2 launch | 372200 fastlio_mapping (affinity 5,6) | 372225 level_bridge (publish_tf=false via odom_source native_imu) | 372237 map_server | 372271 amcl | 372290 lifecycle_manager_localization | 372313 native_odom_bridge (mode native_imu, publish_tf true) | 372377 localization_trust.
/proc/<pid>/environ for ALL 8: FASTRTPS_DEFAULT_PROFILES_FILE=/home/agi/control_backend/docs/evidence/nav_bringup_20261028_195035/dds_nav_two_iface_log.xml, ROS_DOMAIN_ID=0, ROS_LOCALHOST_ONLY=0, dds.shm threads=0 (owned_env.txt).

## Checks (checks.txt / checks_clean.txt / checks2.txt / trust_status.txt)
- /qazbot_lio/cloud_registered: 1 publisher (qazbot_lio_mapping), sub level_bridge.
- /qazbot_lio/scan: 1 publisher (qazbot_level_bridge), sub amcl.
- /qazbot_native/odom: 1 publisher (qazbot_native_odom_bridge), sub Trust.
- /qazbot_d1_map/tf publishers: amcl + qazbot_native_odom_bridge only (level_bridge not a TF publisher).
- TF odom->base_footprint (remapped /qazbot_d1_map/tf): present, t=[-2.195, 2.037, 0], yaw 38.6 deg (native bridge; odom frame origin arbitrary).
- TF map->odom: ABSENT ("frame map does not exist") = expected before initialpose. AMCL log: "Please set the initial pose..." every 2 s.
- Lifecycle: /qazbot_d1_map/map_server active [3], /qazbot_d1_map/amcl active [3].
- AMCL param dump (running node): max_beams 60, alpha1..alpha5 0.1, recovery_alpha_slow 0.0, recovery_alpha_fast 0.0, scan_topic /qazbot_lio/scan, laser likelihood_field, particles 500/2000, base_frame base_footprint, odom_frame odom, global map, tf_broadcast true, transform_tolerance 1.0, set_initial_pose false, update_min_d .25 / a .2. CONFIRMED = disk.
  (First attempt with --no-daemon returned "Node not found" for some params = CLI discovery race; param dump retry succeeded.)
- Trust status (19:53, one echo): trusted false, ready false, motion_ok false, reasons [T1_tf_stale, not_ready:awaiting_initialpose_and_amcl]; /trusted false. No T1_scan_stale / T1_odom_stale in that sample. Expected state before initialpose.
- /cmd_vel and /qazbot_d1_map/cmd_vel: unknown topics (no publishers in our graph). No MC/MA/GC started.
- Foreign RViz already connected (node /qazbot_d1_map/rviz publishes /qazbot_d1_map/initialpose; subscribes nav2/* topics). Not touched.
- CPU snapshot 19:53/19:54: FAST-LIO 37-47 %, native bridge 25-40 %, Trust 20 %, level_bridge 13 %, AMCL 13 %, map_server/lifecycle ~7 % (single sample). HAL 204720 19-53 %. Load avg 27-34 right after start + my CLI checks (each ros2 CLI started a participant); dominant non-owned loads: perception 56 %, camera 38 %, camera container 38 %, nvargus 31 %. Not a root-caused number; recheck load after a quiet minute.
- Note: the first CLI batch used the original profile (no <log>), so CLI Fast DDS noise went to my tool output only; stack processes not affected.

## Blockers / remaining
1. Nav2 NOT started: nav_full.launch.py (the only full Nav2 launch: planner/controller/smoother/bt_navigator) unconditionally adds goal_coordinator.py (ExecuteProcess) -> Goal Coordinator not authorized. nav_control.launch.py + nav_plan.launch.py are a different config (plan-only BT, different lifecycle/namespace) and would be duplicated/replaced by nav_full in the next phase; not started to avoid extra CPU and a non-target configuration. Next phase (with GC/MA permission) per X2_QUICKSTART_BRINGUP_RU.md: same env override, then
   ROBOT_RADIUS=0.454 QAZBOT_ODOM_TOPIC=/qazbot_native/odom ... ros2 launch nav_full.launch.py robot_radius:=0.454 scan_topic:=/qazbot_lio/scan odom_topic:=/qazbot_native/odom map_yaml:=<v3> [allow_dry_run_driver:=true for dry-run adapter]  (call ros2 launch directly, NOT run_nav_full.sh, which re-sources env.sh -> dds_qazbot.xml; QAZBOT_SMAC_PREFIX must be exported = ws/src/navigation/../../../candidates/nav2_smac_bounds/install/nav2_smac_planner before sourcing env.sh). Note nav_full default map_yaml is v2 -> must pass v3.
2. Load: system load is high from vendor processes; T1 margins remain tight (P1).
3. fastdds.log grows ~11 MB/h; rotate/truncate not configured.

## Operator action in RViz (PC, profile per RVIZ_LOCAL.md / dds_pc_x2.xml, X2 wifi 192.168.1.72)
- Fixed Frame: map. TF topic remapped: /qazbot_d1_map/tf, /qazbot_d1_map/tf_static (as in localization.rviz).
- Map: /qazbot_d1_map/map (D1 first_map_nav2_v3, sha f6032f75...). Scan: /qazbot_lio/scan. AMCL pose: /qazbot_d1_map/amcl_pose; particles /qazbot_d1_map/particle_cloud.
- 2D Pose Estimate must publish to /qazbot_d1_map/initialpose, at the true robot pose (base_footprint, heading), robot standing still; within ~0.7 m / 20 deg.
- Expected after: map->odom appears (~5 Hz, AMCL stamp scan+1.0 s), Trust drops T1_tf_stale / not_ready:awaiting_initialpose. Then (orchestrator) nomotion updates /qazbot_d1_map/request_nomotion_update, baseline + confirm_anchor via /qazbot_d1_map/localization_trust/*.
