# 20261102-slam-stage1-mapping T1b: read-only discovery on robot #2 (192.168.1.89) vs robot #1 (.72)
Date 2026-11-02 ~14:40 (+0800); .89 booted 14:01 (up ~39 min at probe). Strictly read-only: only ssh shell reads, sqlite3 -readonly, and two short rclpy probe nodes (graph queries + 6 s passive subscribe), run sequentially. Nothing published, no service called, no process touched.

## Side-by-side
| # | Item | Robot #1 (.72, from T1) | Robot #2 (.89) | Same? |
|---|------|-------------------------|----------------|-------|
| 1 | hostname / IPs | agi; 10.11.1.1, 10.0.1.41, 192.168.1.72, 10.0.200.41 | agi; `10.0.1.41 10.11.1.1 192.168.1.89 10.0.200.41` | internal IPs identical (same PC2 layout), only LAN IP differs |
| 1 | machine-id | not recorded | 1708f65920254226a891e15872fae978 | n/a |
| 1 | model / SN | X2 Ultra, series x2, hw t2d5, project lx2501_3, SN X2B0028C6Z0034, LCFC_ORIN_NX, soc_index 1 | sku `X2 Ultra`, series x2, hw t2d5, project lx2501_3, form with_casing, board LCFC_ORIN_NX_391400001801_391400001802, soc_index 1 of {0 1 2}; **SN UNKNOWN** (`/agibot/info/sn` and `board_sn` Permission denied for agi) | same model/PC2; SN not readable |
| 1 | OS / ROS | Ubuntu 22.04.5, 5.15.148-tegra, Humble | same (22.04.5, 5.15.148-tegra, /opt/ros: humble) | yes |
| 1 | RMW / DOMAIN / DDS profile | RMW unset (fastrtps), domain unset=0, LOCALHOST_ONLY=0, profile ros_dds_configuration.xml whitelist 10.0.1.41 | shell env: only FASTRTPS_DEFAULT_PROFILES_FILE=/agibot/data/home/agi/.aima/env/ros_dds_configuration.xml, ROS_LOCALHOST_ONLY=0 (in agi procs); whitelist address 10.0.1.41; ros2 daemon (pid 1817) runs `--ros-domain-id 0 --rmw-implementation rmw_fastrtps_cpp` | yes |
| 1 | robot_mode | `nav=false dc=false`; robot_use dc | `nav=false dc=false`; robot_use dc | yes |
| 1 | AimDK SDK | /home/agi/aimdk, version `aimdk v0.0.0-g14fe2e8` (docs 1.1.0); install has only aimdk_msgs | /home/agi/aimdk, version **`aimdk v1.0.0-gf8a92e8`** (older); install/ has aimdk_msgs, arms_msgs, arms_speak_dialogue, kazbot, py_examples, ruckig, setup.bash; src also QazBot-Consultant | **differs** (older SDK, py_examples is installed here) |
| 2 | SLAM-related procs (`ps aux | grep -iE 'slam|map_manager|perception|auto_integrated|mapping'`) | slam, map_manager, perception, vectorflux, auto_integrated, static_transform_publisher map->foot_contact all running | **EMPTY: none running** (312 procs total; only hal_sensor `aima-hal_sensor-app-main`, drp, hds, housekeeper, camera_container, etc.) | **MAJOR DIFFERENCE** |
| 3 | /integrated_command | not present, 0 pub / 0 sub | not present, type none, 0 pub / 0 sub | same: no subscriber |
| 4 | GetStoredMapByName | absent (but GetStoredMap + GetMapVersion present) | **absent**; GetStoredMap also **absent**; only `/aimdk_5Fmsgs/srv/GetMapVersion` [aimdk_msgs/srv/GetMapVersion] present | worse on .89 (no GetStoredMap) |
| 4 | other map/slam services | MappingService/{StartMapping,StopMapping,GetStoredMapNames,...}, SmNodeService/{slam,soc1_slam,map_manager,soc1_map_manager} | 15 `/aimdk_2Eprotocol_2EMappingService/*` (StartMapping, StopMapping, GetStoredMapNames, Get2DWholeMap, GetCurrentWorkingMap, DeleteMap, RenameMap, SetCurrentWorkingMap, ...) and `SmNodeService/soc1_5Fslam`, `soc1_5Fmap_5Fmanager` ExecuteAction/ExecuteFallback (no non-soc1 `slam`/`map_manager`), plus EmAppService/{StartApp,StopApp,...} | RPC surfaces registered in both, but no slam/map_manager process on .89 to back them (provider unknown; not called) |
| 5 | Lidar `/aima/hal/sensor/lidar_chest_front/lidar_pointcloud` | present, 10.0 Hz, ~25.5k pts | **ABSENT**: 0 pub / 0 sub; 0 msgs in 6 s | **MAJOR DIFFERENCE** |
| 5 | IMU `/aima/hal/sensor/lidar_chest_front/imu` | present, 199 Hz | **ABSENT**: 0 pub / 0 sub; 0 msgs in 6 s. (Other IMUs exist: `/aima/hal/imu/chest/state`, `/aima/hal/imu/torso/state`, not rate-measured) | **MAJOR DIFFERENCE** |
| 5 | lidar/imu/map/slam topics | lidar_pointcloud(+down_sampling,status), imu, /slam/{lidar_odom,localization/odometry,mapping/odometry}, /map, /map_updates, mapping_tele/*, etc. | only: `/aima/hal/imu/{chest,torso}/state`, `/aima/internal/event/probe/soc0/hal_imu`, `/aima/mc/leg_odometry`, `/aima/mm/building_process/pb_..MapBuildProcessData`, `/loc_tele/relocalization_{pcd,result}/...`, `/mapping_tele/realtime_2d_map/...`, `/mono_perception/debug/line_color_map`, `/slam/localization/odometry`, `/slam/mapping/odometry` (declared by remaining subscribers/stale entries; no live lidar). Graph total 106 topics / 505 services / 36 nodes (vs 248/937/89) | much smaller graph |
| 6 | map.db | /agibot/data/var/MapManagerModule did not exist (PC2) | **EXISTS and READABLE**: `/agibot/data/var/MapManagerModule/map.db` -rw-r----- run:run 69632 B (Oct 19 15:46); agi is in group `run` (gid 1000) so sqlite3 -readonly works. 76-entry dir (74 maps). `/agibot/nfs/soc0`, `/agibot/nfs/soc2`: Permission denied (same as #1) | **MAJOR DIFFERENCE** (robot #2 has stored maps) |
| 6 | schema | UNKNOWN | see below | n/a |
| 7 | /map publisher | topic exists, publisher unknown | `/map`, `/map_updates`, `/slam/lidar_odom` have 0 pubs/0 subs (not even in the topic list) | differs; no /map owner |
| 7 | TF | static map->foot_contact vendor pub; slam publishes /tf | `/tf` pub: mc_ros2_node3259 (subs: soc2_drp_ros2_node, tf_base_imu_subscriber, mc_ros2_node). `/tf_static` pubs: soc2_hal_sensor, mc_ros2_node3259. No `map`-frame publisher process seen (no static_transform_publisher; TF content not echoed) | differs |
| 8 | control_backend / AMCL / Nav2 | none | no amcl/nav2/map_server/controller_server processes. BUT agi runs custom NDT/LIO stack (see below) | partially differs |

## map.db (readonly)
```
sqlite3 -readonly /agibot/data/var/MapManagerModule/map.db .schema
CREATE TABLE db_version (version INTEGER);
CREATE TABLE map (map_id TEXT PRIMARY KEY NOT NULL, map_version TEXT NOT NULL, map_name TEXT NOT NULL, map_info TEXT NOT NULL, topo TEXT NULL);
CREATE TABLE current_map (map_id TEXT PRIMARY KEY NOT NULL);
select count(*) from map -> 74 ; db_version -> 1 ; current_map -> 1791184924974 (earliest map, "standing-test-20261005")
latest rows (map_id|map_version|map_name|map_info(trunc)):
1792396000848|1792396000848|offload-20261019-154638|{"index":74,"map_path":"/agibot/data/var/MapManagerModule/1792396000848/","width...
1792389150649|...|offload-20261019-135228|{"index":73,...
1792387411603|...|offload-20261019-132329|{"index":72,...
sample names: standing-test-20261005, room-20261006-1615, room-20261006-1557, room-20261006-1337, room-20261005-2104, offload-2026101*...
map dir 1791184924974/: 0.pcd, map.pcd, frame_poses.txt, grid_map_info.txt, index.txt, keyframes/, occupancy_map{,_debug,_prob}.png, occupancy_map.yaml, trajectories.txt
```
SELECT * FROM map was only done in truncated form (map_info/topo are large JSON blobs). Many "offload-*" maps (Oct 16-19) indicate prior mapping activity on this robot. Schema is now documented (map_id==map_version typically a ms timestamp).

## Custom agi stack on .89 (not on robot #1)
Running as agi since boot (not vendor): `/home/agi/x2-pad-park/ndt/install/lidar_localization_ros2/.../lidar_localization_node` (node `ndt_localization`, remaps /cloud -> `/aima/hal/sensor/lidar_chest_front/lidar_pointcloud`, /imu -> `.../lidar_chest_front/imu`, /pcl_pose -> /ndt/pose, /map -> /ndt/map_in, /tf -> /ndt/tf), `x2-pad-park/orin/{ndt_bridge,ndt_odom,ndt_reloc,listen}.py`, `/home/agi/azat_ws/agibot/lio/x2pub.py` (node lio_x2pub, /tf -> /lio/tf, prefix /x2/localization), qazbot_speech, tailscaled x2. Notably these ROS nodes (ndt_localization, lio_x2pub) did NOT appear in the DDS graph from my probe despite same domain 0 (cause UNKNOWN: possibly hung, or no discovery); they consume the lidar/imu topics that currently have no publisher. `ros2 daemon` is running (pid 1817), so CLI is usable here.

## Verdict
- .89 is an X2 Ultra PC2 with identical network/ROS config to #1 but SLAM stack is NOT running (no slam/map_manager/perception/auto_integrated processes), there is no chest-front lidar/IMU publisher, and no /integrated_command, GetStoredMapByName, or GetStoredMap. Likely lidar HAL (hal_sensor on this PC is running but publishes no lidar topic; /agibot/software is Permission denied so config could not be checked) or the slam bundle is down/disabled; root cause UNKNOWN. Robot #1 is the better target for Stage 1.
- Positive: map.db readable here with 74 maps; schema known.
- SDK on .89 is older (v1.0.0).
- Gaps: SN, TF contents, /agibot/software configs, why lidar absent; re-probe after hal/lidar check if .89 is to be used.
