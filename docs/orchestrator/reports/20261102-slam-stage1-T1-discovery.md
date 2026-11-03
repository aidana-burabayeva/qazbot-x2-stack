# 20261102-slam-stage1-mapping — T1 read-only discovery
Date: 2026-11-02 ~14:30-14:40 (+0800), robot booted 14:25. Agent: robotics-debugger. Strictly read-only: nothing published, no state-changing service called, no node/process started/stopped (only short-lived CLI/rclpy probe nodes, sequential).
Stopped early at coordinator request; gaps marked UNKNOWN.

## 0. Verdict summary
- Official doc (v1.1.0) contract: start/stop via `/integrated_command` std_msgs/String (RELIABLE+TRANSIENT_LOCAL), payloads `start_mapping`, `stop_mapping:<map_name>`; save timeout default 120 s; verify with `/aimdk_5Fmsgs/srv/GetStoredMapByName` (aimdk_msgs/srv/GetStoredMapByName), code==0 success.
- LIVE GRAPH (at ~14:38): `/integrated_command` topic NOT present (0 pubs/0 subs); `/aimdk_5Fmsgs/srv/GetStoredMapByName` NOT present (wait_for_service 3 s failed). Present instead: `/aimdk_5Fmsgs/srv/GetStoredMap` (aimdk_msgs/srv/GetStoredMap), `/aimdk_5Fmsgs/srv/GetMapVersion`, and internal `/aimdk_2Eprotocol_2EMappingService/{StartMapping,StopMapping,GetStoredMapNames,...}` (ros2_plugin_proto/srv/RosRpcWrapper, NOT documented).
- Official SLAM processes ARE running on this PC (slam, map_manager, perception, vectorflux, auto_integrated), but `/agibot/info/robot_mode` says `nav=false dc=false`. SLAM optional-module activation: UNKNOWN (leaning "not fully activated / interface not exposed") — see §5.
- LiDAR + IMU OK: PointCloud2 10.0 Hz (frame `lidar_chest_front`, ~25.5k pts), IMU 199 Hz (frame `lidar_imu_chest_front`).
- control_backend stack NOT running (no amcl/nav2/agi-owned ROS processes). But a vendor `static_transform_publisher map -> foot_contact (0 0 -0.6)` and a `/map` OccupancyGrid topic already exist (publisher UNKNOWN).
- map.db: `/agibot/data/var/MapManagerModule` does not exist on this PC (PC2); `/agibot/nfs/soc0/var/MapManagerModule` and `/agibot/nfs/soc2/...` exist-or-not UNKNOWN (Permission denied for user agi).
- BLOCKERS for Stage 1 as specified: (B1) GetStoredMapByName service absent; (B2) /integrated_command has no subscriber; (B3) map.db not readable from agi on PC2. Must be resolved (after-sales activation / correct PC / permissions) before start_mapping.

## 1. Official documentation (local copy == online)
Sources: online https://x2-aimdk.agibot.com/en/latest/Interface/perception/SLAM.html (title "5.5.2 SLAM (Optional) — AimDK_X2 1.1.0 documentation"); local /home/agi/aimdk/docs/cn/_build/html/en/dev/Interface/perception/SLAM.html. Extracted texts were diffed: IDENTICAL. Changelog local == online, latest v1.1.0 (2026.05.11); map retrieval/relocalization introduced v1.0.0 (2026.03.02).

### 1.1 SLAM.html facts
- "Important: This is an optional component. To use the interfaces described on this page, please contact after-sales technical support."
- Map Building: topic `/integrated_command`, `std_msgs/String`, "Publish mapping control commands", QoS `RELIABLE+TRANSIENT_LOCAL`, frequency "User (on-demand)".
- Commands: `start_mapping` = Start mapping; `stop_mapping:<map_name>` = Stop mapping and save the map as <map_name>.
- Note: "After sending the stop_mapping command, map saving takes some time. The save may time out (default 120 seconds) if the system encounters an anomaly (e.g. SLAM module unresponsive, slow disk I/O). Use the GetStoredMapByName service to verify whether the map was saved successfully."
- No mapping status/confirmation topic is documented (only GetStoredMapByName verification).
- Map Retrieval: service `/aimdk_5Fmsgs/srv/GetStoredMapByName`, type `GetStoredMapByName` (mm/srv/GetStoredMapByName.srv).
  - Request: `std_msgs/Header header`, `string map_name`
  - Response: `uint64 code` (0 = success), `std_msgs/Header header`, `nav_msgs/MapMetaData map_info`, `uint64 map_version`, `string map_path` (PNG path), `int8[] data` (-1/0/100), `NaviPoint[] navi_points`, `QRPoint[] qr_points`, `HRPoint[] hr_points`, `Path[] paths`, `Region[] regions`, `uint64 map_id`.
  - Local srv (/home/agi/aimdk/src/aimdk_msgs/interface/robot/mm/srv/GetStoredMapByName.srv) comment: `int8[] data` "此字段暂时因T2跨板通信保留" (field temporarily reserved due to T2 cross-board comms) => data[] may be empty; use map_path/map_info.
- map_id/map_name: "check them in the mobile app, or query the map database on the robot": `sqlite3 /agibot/data/var/MapManagerModule/map.db "SELECT * FROM map;"`. Schema NOT documented (only fields map_id, map_name mentioned).
- Map info file: `/agibot/data/var/MapManagerModule/<map_id>/grid_map_info.txt` (3 lines: resolution "1m corresponds to 20 pixels"; origin pixel coords; width height px).
- `/slam/lidar_odom` nav_msgs/Odometry, frame_id `map`, child_frame_id `torso_link`, RELIABLE+TRANSIENT_LOCAL, 10 Hz (only after relocalization; relocalization is out of scope).
- Caution: "standard ROS DO NOT handle cross-host service (request-response) well, please refer to SDK examples ... (exception safety and retransmission)".
- Caution: "While the robot is in Stable Standing Mode or Locomotion Mode, DO NOT launch ROS nodes in rapid bulk (no more than 2 nodes per second is recommended) ... may cause the robot to lose balance and fall".
### 1.2 Perception index (Interface/perception/index.html, online == local)
- Optional; "X2 EDU does not support the features in this section. This section applies only to X2 Ultra and X2 Ultra(new version)."
### 1.3 Data Visualization (Interface/perception/visualization.html; online fetched)
- Mapping/relocalization prerequisites: robot in stable standing mode; apps `slam`, `map_manager`, `perception`, `pnc`, `auto_integrated` running ("usually start automatically"; verify `ros2 node list | grep -E "slam|perception|pnc"`). Warning: do not enter autonomous development mode in developer mode or these apps may not start.
- Frames: `map` = global world frame established during SLAM mapping; `gravity_aligned_base_link` robot body frame; `ros2 run tf2_ros tf2_echo map gravity_aligned_base_link`.
- RViz: "RViz relies on direct DDS network connectivity and currently only supports wired connections (no VPN / wireless)". "After completing the wired connection per Network Connection and setting ROS_DOMAIN_ID to match the robot": `source /opt/ros/humble/setup.bash; rviz2`. "TF, LiDAR point cloud, maps ... subscribed directly via DDS". URDF: run robot_state_publisher with `$URDF_DIR/x2_ultra.urdf` (sed mesh path).
- LiDAR live topic: `/aima/hal/sensor/lidar_chest_front/lidar_pointcloud # sensor_msgs/PointCloud2`.
- No rviz config file is provided by docs. Local SDK has only /home/agi/aimdk/extra/mc-rl/x2_urdf/X2_URDF-v1.3.0/launch/config.rviz (URDF viewer, not SLAM).
### 1.4 Network (quick_start/prerequisites.html, local)
- Host PC static IP 10.0.1.2/24 on rear SDK Ethernet port; dev unit PC2 = 10.0.1.41; PC1 (motion control) 10.0.1.40 (secondary dev prohibited); PC3 interaction 10.0.1.42. Wi-Fi jump host 192.168.88.88.
- Docs give no concrete ROS_DOMAIN_ID value ("match the robot").
### 1.5 SDK examples (example/Python.html 6.1.27 Map Building, 6.1.29 Retrieve a Stored Map)
- Local /home/agi/aimdk/src/py_examples/py_examples/slam.py and get_map.py are byte-equivalent (whitespace-diff) to the online doc code.
- slam.py: node `slam_command_publisher`, QoS depth=10 TRANSIENT_LOCAL RELIABLE, publishes `start_mapping` / `stop_mapping:{map_name}`; run `ros2 run py_examples slam`. Doc: "After starting the mapping process, push the machine or control the robot to move".
- get_map.py: client on `/aimdk_5Fmsgs/srv/GetStoredMapByName`, wait_for_service loop 1 s, spin_until_future_complete timeout 60 s, success iff `result.code == 0`, prints map_path, map_info.resolution/width/height, map_id. Input is map_name not map_id.
- No official read-only "list maps" interface documented (only sqlite3 / mobile app).

## 2. Local SDK
- /home/agi/aimdk/version: `aimdk v0.0.0-g14fe2e8`; docs bundled = AimDK_X2 1.1.0.
- aimdk_msgs built & installed: /home/agi/aimdk/install/aimdk_msgs (Oct 23 18:18; prebuilt_aarch64). Source: `source /opt/ros/humble/setup.bash && source /home/agi/aimdk/install/setup.bash`. Python import `from aimdk_msgs.srv import GetStoredMapByName` works.
- py_examples is NOT built/installed (install/ contains only aimdk_msgs) => `ros2 run py_examples slam` unavailable unless built.
- /home/agi/aimdk/topics_and_services lists `/integrated_command`, `/aimdk_5Fmsgs/srv/GetStoredMapByName`, `/relocalization_pose`, `/slam/lidar_odom`, `/aima/hal/sensor/lidar_chest_front/{imu,lidar_pointcloud}` (the SDK's expected set).
- Doc changelog: "From v1.2.0 on, secondary development programs must be built and run inside a container".

## 3. Robot environment
- /agibot/info/sku: `X2 Ultra`; series x2; hardware_version t2d5; project_id lx2501_3; sn X2B0028C6Z0034; board LCFC_ORIN_NX; soc_index 1 of {0,1,2} => this is PC2 (dev unit).
- /agibot/info/robot_mode: `nav=false dc=false` ("由 set_robot_mode.sh 生成" — nav = enable autonomous navigation); robot_use: dc.
- hostname `agi`; IPs 10.11.1.1, 10.0.1.41, 192.168.1.72, 10.0.200.41; Ubuntu 22.04.5, kernel 5.15.148-tegra aarch64; ROS 2 Humble.
- RMW_IMPLEMENTATION unset (Humble default rmw_fastrtps_cpp); ROS_DOMAIN_ID unset (=0); ROS_LOCALHOST_ONLY=0; FASTRTPS_DEFAULT_PROFILES_FILE=/agibot/data/home/agi/.aima/env/ros_dds_configuration.xml: UDPv4 interfaceWhiteList 10.0.1.41 only + SHM, builtin transports off => DDS only on 10.0.1.0/24 interface (external RViz must be on 10.0.1.x wired, consistent with doc).
- Connection: SSH from 192.168.1.70 -> 192.168.1.72 (VS Code remote). DISPLAY unset.
- PC1/PC3 data mounted via sshfs: nfs@10.0.1.40:/agibot/data on /agibot/nfs/soc0, nfs@10.0.1.42:/agibot/data on /agibot/nfs/soc2.
- Also running (agi user): /home/agi/qazbot-speech/robot/lingxi_off.sh loop, tailscaled (userspace).

## 4. Live graph / processes
Probe: one rclpy node, 8 s discovery: 248 topics, 937 services, 89 nodes.
- Processes (user run, started 14:25-14:26): `/agibot/software/nav/scripts/slam/start_slam.sh` -> `./slam --cfg_file_path=../config/slam/slam_config.yaml` (pid 4280); `start_map_manager.sh` -> `./map_manager` (4559); perception (4564); vectorflux (4741); `auto_integrated` (5006); `static_transform_publisher 0 0 -0.6 0 0 0 map foot_contact` (4737/4788, parent start_vectorflux.sh 4582).
- ROS nodes (filtered): aimrt_slam_node4280, aimrt_auto_integrated_node5006, aimrt_perception_node4564, perception_ros_node, costmap_ros_node, static_transform_publisher_wN0AlKgWcfPQetOg, tf_base_imu_subscriber. No map_manager ROS node name matched.
- `/integrated_command`: NOT in topic list; 0 publishers, 0 subscribers.
- Services: `/aimdk_5Fmsgs/srv/GetStoredMapByName` ABSENT (wait_for_service 3 s => not available). Present: `/aimdk_5Fmsgs/srv/GetStoredMap` [aimdk_msgs/srv/GetStoredMap], `/aimdk_5Fmsgs/srv/GetMapVersion`, `/aimdk_5Fmsgs/srv/GetSystemState`, `/aimdk_2Eprotocol_2EMappingService/{StartMapping,StopMapping,GetStoredMapNames,Get2DWholeMap,GetCurrentWorkingMap,DeleteMap,RenameMap,...}` [ros2_plugin_proto/srv/RosRpcWrapper], `/aimdk_2Eprotocol_2ESmNodeService/{slam,soc1_slam,map_manager,soc1_map_manager}/ExecuteAction`. None of these is in the official SLAM doc => must NOT be used under "official only" rule (not called).
- Topics: `/aima/hal/sensor/lidar_chest_front/lidar_pointcloud` PointCloud2, `/aima/hal/sensor/lidar_chest_front/imu` Imu, `.../lidar_pointcloud_down_sampling`, `.../lidar_status`; `/slam/lidar_odom`, `/slam/localization/odometry`, `/slam/mapping/odometry` (Odometry); `/map` nav_msgs/OccupancyGrid + `/map_updates` (publisher UNKNOWN); `/mapping_tele/realtime_2d_map/...` and `/mapping_tele/saving_percent/...` (ros2_plugin_proto RosMsgWrapper, undocumented); `/tf`, `/tf_static`; `/aima/sm/system_state`.
- Rates (6 s window, SensorDataQoS): LiDAR PointCloud2 10.0 Hz, frame_id `lidar_chest_front`, width*height 25468; IMU 199.0 Hz, frame_id `lidar_imu_chest_front`.
- `ros2` CLI (daemon not running) returned nothing within 15 s timeouts under CPU load (camera/hal ~40 % CPU per proc) — use rclpy probe or `--no-daemon` with longer timeouts.
- Vendor runtime configs (NOT official docs; supporting evidence only): /agibot/software/nav/config/map_manager/map_manager_config.yaml `saving_timeout_s: 120`, db_path default "module data location", pubs `/mapping_tele/saving_percent`, `/aima/mm/building_process`; /agibot/software/nav/config/slam/slam_config.yaml MappingModule subs `/aima/hal/sensor/lidar_chest_front/{imu,lidar_pointcloud}`, `/aima/mc/leg_odometry`; RelocalizationModule `map_dir: /agibot/data/var/MapManagerModule/`; slam publishes `/tf`.
- map.db: `/agibot/data/var/MapManagerModule` -> No such file or directory on PC2; `/agibot/nfs/soc0/var/MapManagerModule` and `/agibot/nfs/soc2/var/MapManagerModule` -> Permission denied (cannot tell existence). sqlite3 available (/usr/bin/sqlite3). Schema: UNKNOWN.
- Current robot mode (MC state / SM state): NOT queried (UNKNOWN). Doc path: `/aima/sm/system_state`, `/aima/mc/common/state`, GetSystemState service.

## 5. SLAM optional-module activation
Evidence "running": slam/map_manager/auto_integrated processes alive; slam node publishes /slam/* odometry topics; internal MappingService RPCs exist.
Evidence "not exposed/activated for SDK": `/integrated_command` has no subscriber; documented `GetStoredMapByName` absent; robot_mode `nav=false`; docs state optional + contact after-sales.
Conclusion: UNKNOWN / likely NOT activated for the AimDK interface. Resolve by: (a) after-sales confirmation; (b) re-probe after full boot (graph probed ~13 min after boot); (c) check whether auto_integrated subscribes to /integrated_command only when nav=true (requires vendor info — do NOT edit robot_mode without vendor guidance).

## 6. RViz
- rviz2 installed on robot (/opt/ros/humble/bin/rviz2). No DISPLAY in this SSH session => run on external wired PC (10.0.1.2, same ROS_DOMAIN_ID=0 unless vendor says otherwise, RMW fastrtps) per doc, or X-forward (not documented).

## 7. Conflicts
- control_backend: not running now (no amcl/nav2/map_server). Re-check before T3.
- Existing `map` frame publishers: vendor static TF map->foot_contact; slam publishes /tf; `/map` OccupancyGrid topic exists from an unknown vendor publisher — a custom node must not publish `/map` or `map` TF (RViz config should not assume a single source; identify /map publisher first).

## 8. Open items / next commands (read-only)
1. `ros2 topic info -v /integrated_command --no-daemon` and rclpy probe again after operator confirms mode (both returned empty/absent here).
2. Identify /map publisher: rclpy `get_publishers_info_by_topic('/map')`.
3. `ros2 interface show aimdk_msgs/srv/GetStoredMap` (present service, compare with GetStoredMapByName) — informational only; doc-sanctioned is GetStoredMapByName.
4. Read-only mode: `ros2 topic echo --once /aima/sm/system_state`, `/aima/mc/common/state`.
5. map.db: ask operator to run with run/sudo: `sqlite3 -readonly /agibot/nfs/soc0/var/MapManagerModule/map.db ".schema"` or on PC1 (10.0.1.40) — which PC hosts map.db is UNKNOWN.
6. Ask after-sales: is the SLAM optional module activated on SN X2B0028C6Z0034; why GetStoredMapByName/integrated_command missing; does robot_mode nav=true gate them.
