# qazbot_localization

Localization of the AgiBot X2 on a prior occupancy map: FAST-LIO (third-party, pinned in `ws/third_party.repos`)
→ C++ `qazbot_lidar_projector` (node `qazbot_level_bridge`, owner of `odom→base_footprint`) → `nav2_amcl`
(`map→odom`) + `nav2_map_server`, plus the **localization trust** gate and the confirmed-pose store/seed.
All TF of this stack lives on `/qazbot_d1_map/tf` (never the vendor `/tf`).

Docs consulted (AgiBot first): `~/aimdk/topics_and_services` and `Interface/hal/sensor.html` "Lidar话题"
(`/aima/hal/sensor/lidar_chest_front/{lidar_pointcloud,imu}`, PointXYZIRT, 10 Hz / 200 Hz, RELIABLE+TRANSIENT_LOCAL);
`common/warnings.html` "二次开发边界与声明" (≤ 2 new ROS nodes per second → `launch_stagger`, 0.6 s).

## Usage
```bash
source <install>/qazbot_bringup/lib/qazbot_bringup/env.sh
ros2 launch qazbot_localization localization.launch.py map_yaml:=... lio_cpus:=5,6   # or via qazbot_bringup
ros2 run qazbot_localization localization_trust_node.py --ros-args -p map_yaml:=... -p expected_map_sha256:=...
```

## localization.launch.py
Starts (0.6 s apart): `qazbot_lio_mapping` (FAST-LIO, params `config/x2.yaml`, all outputs remapped to
`/qazbot_lio/*`, its TF to the unused `/qazbot_lio/tf`), `qazbot_level_bridge` (C++ projector), `map_server`,
`amcl` (`config/amcl.yaml`), `lifecycle_manager_localization` and, unless `seed_pose_file:=''`,
the one-shot `seed_pose.py`. Any of these exiting shuts the launch down.

| Argument | Default | Description |
|---|---|---|
| `lio_binary` | `lib/fast_lio/fastlio_mapping` of package `fast_lio` (resolved at launch) | FAST-LIO executable |
| `map_yaml` | `share/qazbot_maps/maps/first_map_nav2_v2/map.yaml` | map for map_server (production passes d1_qazbot_20261029) |
| `lio_cpus` | `''` | `taskset -c` CPU list for FAST-LIO (production `5,6`) |
| `seed_pose_file` | `pose_store.DEFAULT_PATH` (`~/.ros/qazbot_x2_last_confirmed_pose.json`) | confirmed pose to re-seed once; `''` disables |
| `odom_source` | `lio` | only `lio`; `native_imu` was removed with the python native bridge (tag `pre-consolidation-fb18dc1`) |
| `projector_prefix` | install prefix of `qazbot_lidar_projector` | C++ projector prefix; `''` (old python fallback) is refused |
| `projector_params` | `share/qazbot_lidar_projector/config/lidar_projector_switch.yaml` | projector parameters |

## Nodes
### qazbot_level_bridge (from qazbot_lidar_projector)
Sub `/qazbot_lio/odometry`, `/qazbot_lio/cloud_registered`; pub `/qazbot_lio/scan`, `/qazbot_lio/scan_obstacles`,
`/qazbot_lio/scan_clearing`, `/qazbot_lio/odom_level`, TF `odom→base_footprint` on `/qazbot_d1_map/tf`. See its README.

### amcl / map_server (`/qazbot_d1_map`)
Parameters in `config/amcl.yaml` (package default; copy it to change). map_server: `yaml_filename=map_yaml`,
`topic_name=map`, `frame_id=map`.

### localization_trust (`/qazbot_d1_map/localization_trust`, Python, 5 Hz)
Decides whether the AMCL pose may be used for motion. Pub `~/trusted` (std_msgs/Bool), `~/status` (std_msgs/String, JSON);
srv `~/arm_check`, `~/confirm_anchor`, `~/baseline_start`, `~/baseline_finish` (std_srvs/Trigger); client
`request_nomotion_update` (AMCL). Subscribes map, AMCL pose/particles, `/qazbot_lio/scan`, odometry and `/qazbot_d1_map/tf`.

| Parameter | Type | Default | Description |
|---|---|---|---|
| `map_yaml` | string | `<script dir>/../maps/first_map_nav2_v2/map.yaml` (not valid when installed: always pass it) | grid used for scan scoring |
| `expected_map_sha256` | string | `''` | refuse trust if the map image hash differs |
| `pose_store` | string | `''` (off) | path to persist the confirmed pose |
| `odom_topic` | string | `/qazbot_lio/odom_level` | odometry |
| `odom_source` | string | `lio` | reported/checked odometry source |
| `map_topic` | string | `map` | served map |
| `go2_nomotion_on_initialpose` | bool | `true` | request AMCL nomotion update after initialpose |
| all fields of `loc_trust.TrustConfig` | float/str | see `scripts/loc_trust.py` | thresholds (σ, hit ratio, jumps, ambiguity...); `config/go2_admission.yaml` is the GO2 shadow profile |

Docs: `docs/X2_LOCALIZATION_TRUST.md`.

### seed_pose.py (one shot)
Publishes the last confirmed pose once on `/qazbot_d1_map/initialpose` (geometry_msgs/PoseWithCovarianceStamped) after AMCL is up;
never grants READY.

## Python helpers (lib/qazbot_localization)
`loc_trust.py` (pure numpy trust evaluator), `pose_store.py`, `source_stamp.py`, `launch_stagger.py` (shared by the
qazbot_navigation launch files: process start pacing).

## Tests
`colcon test --packages-select qazbot_localization` (pytest on `test/`, ROS_DOMAIN_ID != 0; some tests need the
installed workspace sourced).
