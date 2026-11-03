# collision_monitor offline evaluation (test-only)

Offline, isolated evaluation of the official `nav2_collision_monitor` (ros-navigation/navigation2 tag `1.1.18`,
SHA `1c68c212db01f9f75fcb8263a0fbb5dfa711bdea`, Apache-2.0) on the real X2 LiDAR cloud from bag
`d04_static_bag_183015` (see `tests/d04/BAGS.md`). It is not production configuration: nothing here is launched
on the robot stack, and it publishes only `/cm_test/*` in `ROS_DOMAIN_ID=82` with `ROS_LOCALHOST_ONLY=1`.
Report: `.orchestrator/reports/20261103-reuse-R2-collision-monitor.md`.

`nav2_collision_monitor` is not installed in `/opt/ros/humble`. Build it from the official tag into a scratch
overlay (no system install), against the installed Nav2 1.1.18:
```
git clone --depth 1 --branch 1.1.18 --filter=blob:none --sparse https://github.com/ros-navigation/navigation2.git
cd navigation2 && git sparse-checkout set nav2_collision_monitor
mkdir -p ../cm_ws/src && ln -s $PWD/nav2_collision_monitor ../cm_ws/src/
cd ../cm_ws && source /opt/ros/humble/setup.bash && MAKEFLAGS=-j2 colcon build --parallel-workers 1 \
  --cmake-args -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTING=OFF
```

## Usage
```
export CM_OVERLAY=<cm_ws>/install/setup.bash CM_OUT=<scratch>/runs
tools/run_all.sh                                   # 9 short scenarios, ~6 min sim time
python3 tools/offline_emulate.py <scratch>/emu.pkl # full-bag emulation of the 1.1.18 decision logic (~2 min)
python3 tools/emu_summary.py <scratch>/emu.pkl     # per ground-truth window
python3 tools/analyze_run.py <CM_OUT>/<scenario> <scratch>/emu.pkl
python3 -m pytest -q test                          # unit tests of lib/cm_eval_lib.py
```

## Nodes (harness, `nodes/`)
| node | role | publishes | subscribes |
|---|---|---|---|
| `collision_monitor` (official) | system under test | `/cm_test/cmd_vel_out` (Twist) | `/cm_test/cmd_vel_in`, `/cm_test/lidar_pointcloud`, `/cm_test/tf`, `/cm_test/tf_static` |
| `cm_cmd_vel_source` | constant synthetic cmd_vel_in | `/cm_test/cmd_vel_in` (Twist) | – |
| `cm_fixture_static_tf` | fixture static TF base_footprint→lidar_chest_front | `/cm_test/tf_static` | – |
| `cm_topic_gate` | fault injection (cloud dropout, odom TF stop), only with `use_gate:=true` | `/cm_test/lidar_pointcloud`, `/cm_test/tf` | `/cm_test/lidar_pointcloud_raw`, `/cm_test/tf_raw` |
| `cm_probe` | records cloud stamp / cmd in / cmd out (sim + wall time) to CSV | – | cloud, cmd in, cmd out |

No services or actions. The collision_monitor lifecycle is driven by `ros2 lifecycle set` in `tools/run_scenario.sh`.

## Parameters
`config/collision_monitor_x2_offline.yaml` (CM, defaults) and `config/harness.yaml` (harness nodes, defaults).
Per-scenario changes are override YAMLs in `config/overrides/`; do not edit the defaults for a run.

| node | parameter | type | default | description |
|---|---|---|---|---|
| cm_cmd_vel_source | topic | string | /cm_test/cmd_vel_in | output topic |
| | rate_hz | double | 20.0 | publish rate (sim time) |
| | linear_x / angular_z | double | 0.30 / 0.0 | constant command |
| | stop_after_s | double | -1.0 | stop publishing after N sim s (≤0 = never) |
| cm_fixture_static_tf | parent_frame / child_frame | string | base_footprint / lidar_chest_front | frames |
| | translation | double[3] | fitted, see YAML | metres |
| | rotation_xyzw | double[4] | fitted, see YAML | quaternion |
| cm_topic_gate | cloud_in / cloud_out / tf_in / tf_out | string | see YAML | relay topics |
| | cloud_drop / tf_drop | double[] | [0, 0] | drop windows [start, end, ...] in sim s from first message |
| | tf_drop_child | string | base_footprint | drop only transforms with this child (empty = all) |
| cm_probe | cloud_topic / cmd_in_topic / cmd_out_topic | string | see YAML | recorded topics |
| | output_csv | string | /tmp/cm_probe.csv | CSV path (run_scenario.sh overrides it per run) |

## Notes
- The fixture TF is a floor-plane fit on the static bag; body sway while walking is not modelled.
- The bag contains no boxes (operator only), so low-obstacle behaviour is not evaluable with it.
