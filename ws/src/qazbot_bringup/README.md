# qazbot_bringup

Canonical single entry point of the X2 navigation stack, the process environment and DDS profiles.

Docs consulted (AgiBot first): `common/warnings.html` "二次开发边界与声明" (≤ 2 new ROS nodes/s while standing/walking;
do not modify vendor DDS/system config; never run on PC1 10.0.1.40), `quick_start/prerequisites.html` 4.2 and
`container_dev.html` 4.4 (container mode mandatory from AimDK v1.2.0; this robot runs v1.1.x where host mode is allowed,
see docker/README.md), `Interface/control_mod/locomotion.html` 5.1.2/5.1.3 (MC velocity input source).

## Usage
```bash
source <install>/qazbot_bringup/lib/qazbot_bringup/env.sh
# lab (default): adapter dry run; refuses ROS_DOMAIN_ID 0 and a non-local graph
ROS_DOMAIN_ID=88 ROS_LOCALHOST_ONLY=1 ros2 launch qazbot_bringup x2_navigation.launch.xml
# production (operator permission; ROS_DOMAIN_ID unset/0): live MC output
ros2 launch qazbot_bringup x2_navigation.launch.xml mode:=production
```
At start it logs one INFO line: `QAZBOT BRINGUP mode=… profile=… workspace=… git_sha=… dirty=… manifest=…
odom_source=lio lidar_source=/aima/hal/sensor/lidar_chest_front/lidar_pointcloud -> FAST-LIO tf_owner=[…]
control_channel=[…]`. The SHA comes from `<release>/manifest.yaml` (docker/deploy) or the build manifest
`share/qazbot_bringup/build_manifest.yaml` (generated at every build from the source checkout).

## x2_navigation.launch.xml
Starts, 0.6 s apart: `qazbot_localization/localization.launch.py` (t=0), `localization_trust_node.py` (3.6 s),
`qazbot_navigation/nav_full.launch.py` (4.2 s), `x2_motion_adapter.py` (7.8 s). Names, namespaces, remaps, topics and
parameter values equal the live fb18dc1 deployment of 2026-11-05 (verified, see report P2).

| Argument | Default | Description |
|---|---|---|
| `mode` | `lab` | `lab` (dry-run adapter, isolated domain) or `production` (live MC) |
| `profile` | `x2_d1` | `config/profiles/<profile>/` (adapter per mode, trust) |
| `odom_source` | `lio` | owner of odom→base_footprint (only lio) |
| `map_yaml` | `qazbot_maps/maps/d1_qazbot_20261029/map.yaml` | map (production map) |
| `lio_binary` | `$(find-pkg-prefix fast_lio)/lib/fast_lio/fastlio_mapping` | FAST-LIO executable |
| `lio_cpus` | `5,6` | FAST-LIO CPU pinning |
| `seed_pose_file` | `/nonexistent/qazbot_seed_disabled.json` | pose re-seed disabled (as production) |
| `projector_prefix` / `projector_params` | installed qazbot_lidar_projector / `lidar_projector_switch.yaml` | C++ projector |
| `robot_radius` | `0.454` | body envelope radius (m) |
| `scan_topic` / `odom_topic` | `/qazbot_lio/scan` / `/qazbot_lio/odom_level` | Nav2 inputs |
| `allow_dry_run_driver` | `true` | goal_coordinator accepts a dry-run adapter (as production) |
| `start_adapter` | `true` | start the motion adapter |
| `aimdk_prefix` | `/agibot/software/common` | vendor prefix with aimdk_msgs (adapter process only) |

## Profiles (`config/profiles/x2_d1`)
`motion_adapter.production.yaml` (live true, priority 30, ENABLE, odom /qazbot_lio/odom_level),
`motion_adapter.lab.yaml` (live false), `localization_trust.yaml` (expected map sha256). Package defaults: copy, don't edit.

## Scripts (lib/qazbot_bringup)
`env.sh` (source first: ROS, pinned runtime debs `QAZBOT_RUNTIME` = `<release>/deps`, workspace, vendor env,
`FASTRTPS_DEFAULT_PROFILES_FILE` by `QAZBOT_MODE`: lab (default) = `config/dds_loopback.xml` + `ROS_LOCALHOST_ONLY=1`, production = `config/dds_qazbot.xml`; `QAZBOT_DDS_PROFILE` overrides), `aimdk_env.sh`,
`bringup_info.py` (guard + summary), legacy single-component runners `run_localization.sh`, `run_nav_full.sh`,
`run_nav_plan.sh`, `run_trust.sh`, `run_trust_go2_shadow.sh`, `run_x2_driver.sh`.

## DDS profiles
`config/dds_qazbot.xml` (robot: SHM + UDP on 10.0.1.41 and the Wi-Fi address for the operator RViz),
`config/dds_loopback.xml` (lab: 127.0.0.1 only).

No topics/services of its own. Tests: `test/test_bringup_info.py`.
