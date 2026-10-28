# X2 Nav bring-up runbook (2026-10-28)

Одна страница. Скрипты: `EV=/home/agi/control_backend/docs/evidence/nav_bringup_20261028_195035` (копии в Git: [evidence/nav_session_20261028/nav_bringup_20261028_195035](evidence/nav_session_20261028/nav_bringup_20261028_195035/)). Текущее состояние и блокеры: [HANDOFF.md](HANDOFF.md).

## Окружение (обязательно)

- `ros2 launch` вызывать напрямую, **не** `run_localization.sh` / `run_nav_full.sh` (они пересоурсят env.sh → dds_qazbot.xml).
- После `source ws/src/navigation/env.sh`: `export FASTRTPS_DEFAULT_PROFILES_FILE=$EV/dds_nav_two_iface_log.xml ROS_DOMAIN_ID=0 ROS_LOCALHOST_ONLY=0` (UDP 10.0.1.41 + 192.168.1.72; Fast DDS лог → `$EV/fastdds.log`).
- Nav2: `export QAZBOT_SMAC_PREFIX=$NAV/../../../candidates/nav2_smac_bounds/install/nav2_smac_planner` **до** `source env.sh`, иначе «Qualified Smac prefix is not selected».
- Adapter: `QAZBOT_SOURCE_NAME=sdk_qazbot_nav2_1028b` (default в start_nav.sh всё ещё 1028a), `REGISTER=1`.
- ros2 CLI ненадёжен (daemon убит) — для статусов/сервисов использовать rclpy (`post_seed.py`).

## Порядок старта

1. Preflight: нет второго FAST-LIO/AMCL/Trust/level_bridge (`start.sh` сам проверяет); HAL 204720 жив; lidar ~10 Гц.
2. `bash $EV/start.sh` — localization (FAST-LIO cpus 5,6, level_bridge I1 publish_tf=false, native_imu odom, map_server despeckled, AMCL) + Trust (go2, T5 0.25 через `-p jump_xy_max:=0.25 -p jump_yaw_max:=0.25`).
3. `QAZBOT_SOURCE_NAME=sdk_qazbot_nav2_1028b bash $EV/start_nav.sh` — adapter (live) → nav_full (+GC) → один MC ADD. Код выхода 6 = Nav2 ждёт map→odom: нормально до initialpose.
4. Recorder: `bash $EV/record_goal.sh` (mcap, без PointCloud2).
5. Оператор: RViz 2D Pose Estimate → `python3 $EV/post_seed.py` (12 AMCL no-motion updates, печатает sigma_xy/second_mode/scan_score, resume GC+adapter **только если READY**).
6. Оператор: RViz 2D Goal Pose на `/qazbot_d1_map/goal_pose`.

## Мониторинг

- `/qazbot_d1_map/localization_trust/status` (trusted/ready/motion_ok/t1_transient/reasons/metrics), `goal_coordinator/status` (busy/paused/latched/fault), `motion_adapter/status` (state/latch/t1_pause/registration).
- Логи: `$EV/nav_full.log` (grep `Begin navigating|Goal succeeded|TRIP|collision|patience`), `$EV/trust.log`, `$EV/localization.log`, `$EV/adapter.log`.
- Сброс latch без рестарта: initialpose → READY → `post_seed.py` (или `goal_coordinator/resume`, `motion_adapter/resume` Trigger).

## Остановка (motion first)

- `bash $EV/stop.sh` = `stop_nav.sh` (GC stop → adapter stop/SIGINT → nav_full SIGINT) → Trust → localization; INT→TERM→KILL только owned PID. Никогда pkill.
- Перед остановкой adapter: DELETE своего MC source (adapter не делает его сам), иначе следующий ADD code=1.
- Recorder: `kill -INT $(cat $EV/record.pid)` — только с разрешения оператора.
- Рестарты по частям: `restart_trust.sh` (только Trust), `CONFIRM=1 restart_gc.sh` (весь nav_full, adapter не трогается; GC нельзя рестартовать отдельно — OnProcessExit→Shutdown).

## Rollback каждого изменения (на X2, затем копия в repo)

| Изменение | Rollback |
| --- | --- |
| AMCL max_beams 180 | `ros2 param set /qazbot_d1_map/amcl max_beams 60` + `cp localization.launch.py.bak_*pre_beams180 localization.launch.py` |
| AMCL update_min 0.10 | `ros2 param set … update_min_d 0.25 / update_min_a 0.2` + `.bak_20261028_pre_p12` |
| AMCL odom-trust (beams 60, alpha .1) | `cp localization.launch.py.bak_20261028_pre_amcl_odomtrust localization.launch.py`, рестарт localization |
| T5 0.25 | убрать два `-p jump_*` из start.sh/restart_trust.sh (или `start.sh.pre_P12`), `restart_trust.sh` |
| T1 pause/resume | `*.bak_20261028_pre_t1pause` для loc_trust.py, localization_trust_node.py, goal_coordinator.py, x2_motion_adapter.py; рестарт Trust/nav_full/adapter |
| GC TRIP logging (P10) | `goal_coordinator.py.bak_20261028_pre_p10` |
| I1 2d240dbd | после остановки стека: `python3 /tmp/i1_stage_2d240dbd/tools/deploy_i1.py --restore --backup /home/agi/control_backend/docs/evidence/i1_deploy_2d240dbd_r2` |
| Despeckled map | `MAP=…/first_map_nav2_v3/map.yaml` и SHA f6032f75… в start.sh/restart_trust.sh/start_nav.sh (`*.pre_P9`) |
| Replanning/global obstacles/tolerance (P11) | `bt/navigate_to_pose.xml.bak_20261028_pre_replan`, `nav_full.launch.py.bak_20261028_pre_replan` |
| RecoveryNode/failure_tolerance 3.0 (P13) | `*.bak_20261028_pre_p13` (3 файла) |
| failure_tolerance 10 | `nav_control.launch.py.bak_*pre_ft10` + `ros2 param set /qazbot_d1_map/controller_server failure_tolerance 3.0` |
| DDS log profile | использовать `latency_20261028_192628/dds_nav_two_iface.xml` (без `<log>`) |
| MC source 1028b | DELETE 1028b, ADD новым именем |
| Vendor apps | `aima em start-app hal_camera; … orbbec_camera; … perception; … drp; … lx_webrtc_app` |
| HAL nice (не сделано) | `sudo kill 204720` → `aima em start-app hal_sensor_orin`; не запускать em HAL при живом 204720 |
