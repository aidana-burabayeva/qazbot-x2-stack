# TASK 20261102-slam-stage1-mapping
Stage 1: official AimDK X2 SLAM mapping -> save -> read back (GetStoredMapByName + map.db) -> RViz.
Project dir: /home/agi/x2_slam_stage1 (NEW, fully independent of /home/agi/control_backend: no imports, no shared launch/params/deps, no edits there).
Previous task archived: reports/TASK_20261030-twin-data-collection.archived.md
Full user spec: user prompt 2026-11-02 (sections A–G, response format 1–8, verdict PASSED/BLOCKED).

## Sources (official only)
- https://x2-aimdk.agibot.com/en/latest/Interface/perception/SLAM.html
- https://x2-aimdk.agibot.com/en/latest/Interface/perception/index.html
- AimDK docs: Data Visualization / RViz Visualization; SDK examples Map Building, Retrieve a Stored Map, RViz Visualization
- Local SDK: /home/agi/aimdk (src, docs, topics_and_services)

## Constraints / non-goals
- Never invent topics/services/params/paths/commands: each must be cited to official doc or local SDK file.
- NO motion commands. NO relocalization, POI, navigation, PncTaskRequest.
- Max 2 ROS node launches per second (balance risk in Stable Standing/Locomotion).
- control_backend stack (AMCL/Nav2/Trust/recorders) may be running: do not stop/restart/modify it; detect and report conflicts with official SLAM before start_mapping.
- start_mapping/stop_mapping on live robot require explicit operator go (operator walks the robot? robot carried/teleop by operator — not by us).
- If official SLAM unavailable: record exact error first, then fallback order a) official SDK/support b) agibot-x2-navigation if compatible c) custom node; fallback = mapping/visualization only.

## Acceptance
- A/B checks recorded (model, SDK ver, distro, host/IP, connection, mode, SLAM activation, LiDAR+IMU rates).
- slam_mapping_client.py (param map_name; publishes start_mapping / stop_mapping:<name> on /integrated_command std_msgs/String; logs elapsed; waits save ≤120 s; never declares saved without GetStoredMapByName code==0).
- output/stage1_map_check.json with real values; sqlite map.db cross-check.
- output/stage1_rviz_config.rviz (map, TF map->torso_link, LiDAR PointCloud2).
- Deliverable doc: output/STAGE1_RUNBOOK.md (format 1–8 + verdict).
- Independent verifier PASS.
