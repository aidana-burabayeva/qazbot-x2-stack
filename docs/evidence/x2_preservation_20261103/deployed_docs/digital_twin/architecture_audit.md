# QazBot X2 Digital Twin — Phase 0: архитектурный аудит

Дата: 2026-10-23. Источник: `.66` (только чтение: ssh, rsync-pull) + локальная копия `~/QazBot/control_backend`.
На `.66` ничего не изменялось, процессы не запускались и не останавливались, движения не выполнялись.

## 1. Расхождения с постановкой (важно)

| В постановке | Факт |
|---|---|
| `~/qazbot/{docker,ws/src}` | Такого каталога на `.66` нет. Стек живёт в `~/control_backend/{ws/src,runtime,candidates,tests,docs,logs}` |
| Docker-окружение | На `.66` Docker для пользователя `agi` недоступен (`Cannot connect to the Docker daemon`). Стек — нативные процессы под `ros2 launch`, запуск через `navigation/detach.py` |
| «x86 как на роботе» | `.66` = Jetson, `aarch64`, kernel `5.15.148-tegra`, Ubuntu 22.04, ROS 2 **Humble**, 8 ядер, 15 GiB. Бинарники в `runtime/` и `candidates/*/install` — **ARM64**, на x86 не запустятся, нужна пересборка/apt |
| Motion Adapter «со специфичным SDK» | Да: `aimdk_msgs` (`McLocomotionVelocity`, `SetMcInputSource`, `McCommonState`), исходники `/agibot/software/aimdk/src/aimdk_msgs` |

Локальная машина: Ubuntu 22.04 x86_64, 12 ядер, 62 GiB, ROS Humble (apt), Nav2 **1.1.20**, `ros-humble-ros-gz`, `slam-toolbox`, `rviz2`, `colcon`, Docker 29.8.1. `stage_ros2` в локальном apt не найден.

Правило пользователя из памяти («вся работа только на роботе, локальных копий нет») противоречит этой задаче; явное требование задачи — изолированная копия, поэтому работа ведётся в `~/QazBot/control_backend`, `.66` не трогается.

## 2. Состояние `.66` на момент аудита

Запущено (нативно, namespace `/qazbot_d1_map`, приватный TF `/qazbot_d1_map/tf`):
`fastlio_mapping` (`qazbot_lio_mapping`), `level_bridge.py`, `map_server`, `amcl`, `localization_trust_node.py`, `controller_server`, `velocity_smoother`, `planner_server`, `bt_navigator`, 3× `lifecycle_manager`, `goal_coordinator.py` (`allow_dry_run_driver:=true`), `x2_motion_adapter.py` (**`live:=false`**, dry-run). Параллельно вендорские: `housekeeper`, `lx_webrtc_app`, `orbbec_camera`.

Вывод: на момент аудита адаптер в dry-run, на MC ничего не публикуется.

Запись в `20261023-nav-blockers.md` («проблема ограничения 0,5→0,6») — по коду уже **`max_angular` default = 0.6** в `nav_control.launch.py` и `nav_full.launch.py`. Это не мой вывод, а факт файла; согласован ли предел — вопрос к оператору (см. §9).

## 3. Цепочка и источники ключевых топиков (по коду)

```
aima lidar/imu ─► fastlio_mapping ─► /qazbot_lio/odometry, /cloud_registered
                       │
                level_bridge.py ─► /qazbot_lio/odom_level, /qazbot_lio/scan, TF odom→base_footprint (приватный tf)
                       │
 map_server ─► /qazbot_d1_map/map ─► amcl ─► map→odom (приватный tf), amcl_pose, particle_cloud
                       │
 localization_trust_node ─► localization_trust/{trusted,status}
                       │
 RViz goal ─► goal_coordinator ─► bt_navigator(NavigateToPose) ─► planner_server(SmacPlanner2D, патч)
                                          └► controller_server(RPP) ─► cmd_vel_nav ─► velocity_smoother(OPEN_LOOP)
                                                                   ─► /qazbot_d1_map/cmd_vel
 x2_motion_adapter ─► /aima/mc/locomotion/velocity (McLocomotionVelocity, BEST_EFFORT, 50 Hz)
```

Читает адаптер: `/aima/mc/common/state` (McCommonState), приватный TF, `…/localization_trust/trusted`, `cmd_vel`.
Регистрация источника: сервис `/aimdk_5Fmsgs/srv/SetMcInputSource` (только `live:=true`).
Статический `map→foot_contact` публикует отдельный `static_transform_publisher` (вендорский, не наш).

Реальные bag (получены с `.66`, `logs/bags/`): `cycle2_01` (292 с, 79k сообщений), `turn_nav_01` (383 с), `short_nav_01`, `manual_ready_turn_01`. Содержат: `/qazbot_lio/{odometry,odom_level,scan}`, `amcl_pose`, `particle_cloud`, `localization_trust/{trusted,status}`, `goal_coordinator/status`, `nav2/{plan,cmd_vel_nav}`, `cmd_vel`, `cmd_vel_authorized`, `motion_adapter/{status,dry_run_cmd}`, `/aima/mc/{common/state,leg_odometry,locomotion/velocity}`, приватный `tf`. **Сырых LiDAR/IMU в них нет** — только на `.66` в `logs/bags/liolat_lidar_imu_01` (487 MB, не стянут), там же более тяжёлые `g_motion_0{1..4}`, `manual_amcl_*`.

## 4. Ответы на вопросы A–H

**A. Можно ли собрать на отдельном компьютере?** Да для Python-слоя и конфигов (запускается без правок). Nav2 — через apt (1.1.20 вместо 1.1.18 overlay на роботе: версию фиксировать). Нужны пересборка под x86: `aimdk_msgs` (исходники получены, `third_party/`), патченый Smac (`candidates/nav2_smac_bounds/bounds.patch` — **launch отказывается стартовать без него**, проверяет sha256 библиотеки), опционально lifecycle-shutdown overlay, FAST-LIO (`candidates/fast_lio_portable`, исходники не стянуты).

**B. Что требует робота/SDK?** Только `x2_motion_adapter` (MC publisher, регистрация источника, `McCommonState`), FAST-LIO (вендорские топики лидара/IMU), `level_bridge` (вход — вывод LIO). Nav2, AMCL, Trust, Coordinator, RPP, smoother — аппаратно-независимы.

**C. Где Virtual MC?** На границе ROS-интерфейса `aimdk_msgs`: Virtual MC подписан на `/aima/mc/locomotion/velocity`, публикует `McCommonState`, обслуживает `SetMcInputSource`. Тогда **настоящий адаптер запускается с `live:=true` без изменений** — проверяется реальный live-код (регистрация, владение lock, latch-логика), а не dry-run. Рефакторинг адаптера не нужен: граница уже чистая (`AdapterCore` без ROS-импортов + единственный MC-publisher).

**D. Симулятор.** Для Mode B 3D не нужен. В репозитории уже есть `tests/controller_dry/sim_robot.py` — кинематический стенд-ин (лаг, пороги страгивания, 0.2 с timeout, odom/TF/scan от одной стены) и `tests/nav_e2e/` (модель в изолированном домене 87). Решение: **развивать их в пакет Virtual X2** (диффпривод/yaw + raycast-скан по `map.pgm`), а не писать параллельную реализацию и не тащить Stage (в apt для Humble не найден). Gazebo (`ros_gz` установлен) — только Mode C.

**E. Какие bag воспроизводят ошибки?** См. §3. `turn_nav_01`/`cycle2_01` содержат отказ поворота и ABORT возврата (Блокер 1), Trust-статусы и AMCL (Блокеры 3, 4 на уровне сообщений), `leg_odometry`+`velocity` (Блокер 2, идентификация выбега). Для Блокера 4 в части LIO нужен сырой `liolat_lidar_imu_01`.

**F. Как сохранить реальный стек?** Ничего не форкать: `ws/src` копируется as-is; различия SIM/REAL — только (1) что публикует `/aima/*`, `/qazbot_lio/*` и приватный odom-TF, (2) DDS-профиль/домен, (3) `use_sim_time`. Параметры — те же launch-файлы.

**G. Изоляция REAL/SIM.** Уже есть прецедент: `ROS_DOMAIN_ID=87`, `ROS_LOCALHOST_ONLY=1`, unicast-профиль. Этого недостаточно как единственной защиты (переменные можно забыть). Добавить: Docker с `--network none` (или отдельный netns) для SIM; в SIM-запуске отказ при `ROS_DOMAIN_ID`≠87 и при достижимом адресе `192.168.1.66`; без `aimdk` прошивочного профиля.

**H. Что воспроизводимо без робота?**

| Блокер | Без робота | Как |
|---|---|---|
| 1 повороты | Нет (свойство MC). Только реплей записанного и проверка логики RPP/smoother/adapter на **параметризованной** модели порога | Mode A (bag) + Mode B с моделью по измерениям S3b |
| 2 перелёт | Частично: логика остановки/approach; сама характеристика v→MC — только по измерениям | идентификация по `leg_odometry`+`velocity` из bag |
| 3 AMCL/Trust | Да на записанных odom+scan (Mode A, уровни 2–3); регрессия T5 уже есть | реплей bag в AMCL и Trust |
| 4 stale | Не воспроизводится (нагрузка CPU/DDS Jetson); логику Trust — да | на x86 нагрузки нет → только детектор по bag |
| 5 costmap inf-лучи | Да в Mode B (synthetic scan с inf) | тест clearing |
| 6 lifecycle/restart | Да (есть `run_restart.py`, `repro_manager_*`) | Mode B |

## 5. Выбранная архитектура

Mode A: `ros2 bag play` с `--clock`, фильтром топиков по уровню. Уровни: (1) сырой LiDAR/IMU → реальный FAST-LIO; (2) odom+scan → AMCL; (3) AMCL pose → Trust. На каждом уровне фильтр исключает топики, которые перепубликует живой узел (один авторитетный издатель).
Mode B: `Virtual X2` (одна нода или пакет) = Virtual MC + интегратор + odom/TF + raycast-скан; реальные AMCL, Trust, Coordinator, Nav2, адаптер `live:=true`.
Mode C: отложен.

## 6. Дефекты, найденные при аудите

```
Observation: tests/loc_trust/test_loc_trust.py::Pieces::test_real_map_loads берёт карту из ~/control_backend, а не относительно репозитория.
Impact: падает на любой машине кроме робота/пользователя; блокирует CI.
Priority: medium
```
```
Observation: nav_full.launch.py жёстко требует патченый Smac (QAZBOT_SMAC_PREFIX + sha256 по ARM64-сборке).
Impact: на x86 стек не стартует без пересборки патча; артефакт нужно воспроизводимо собирать (Docker/colcon).
Priority: high (блокер Phase 1)
```
```
Observation: .66 работает вне Docker, версии Nav2 на роботе (overlay 1.1.18 bt_navigator и др.) отличаются от apt 1.1.20.
Impact: поведение Nav2 в симуляции может отличаться от робота; нужна фиксация версий.
Priority: medium
```
```
Observation: max_angular default=0.6 уже в launch-файлах, тогда как постановка запрещает менять 0.5→0.6 без согласования.
Impact: SIM и REAL должны использовать одинаковое значение, но решение о безопасности за оператором.
Priority: high (нужно подтверждение)
```

## 7. Результаты запусков в Phase 0 (факты)

| Команда | Результат |
|---|---|
| `pytest tests/motion_adapter/{test_adapter_core,test_dry_run_shadow,test_early_takeover,test_new_faults}.py` | **43 passed** |
| `pytest tests/loc_trust/{test_loc_trust,test_ready_motion,test_source_failures,test_trust_gaps}.py` | **77 passed, 1 failed** (`test_real_map_loads`, `FileNotFoundError` по пути `~/control_backend`) |
| `ros2 bag info` для `cycle2_01`, `turn_nav_01` | читаются, топики перечислены в §3 |

Не запускалось: launch-стек, ROS-интеграционные тесты, реплей bag, FAST-LIO.

## 8. План на ближайшие шаги (Phase 1–3)

1. `Dockerfile` на `ros:humble` + apt Nav2, фиксированные версии; сборка `aimdk_msgs` и патченого Smac (`bounds.patch`), `--network none` для SIM.
2. Починить `test_real_map_loads` (относительный путь) — минимально.
3. Запустить существующий `tests/nav_e2e/run_model.sh` на x86 (домен 87): первая проверка «реальный стек без робота».
4. Virtual X2: из `sim_robot.py` → пакет, интерфейс `aimdk_msgs`; адаптер `live:=true` против Virtual MC.
5. Mode A: проигрыватель bag по уровням + сравнение до/после.

## 9. Что нужно от оператора / что требует робота

- Подтвердить: `max_angular=0.6` одобрено или вернуть 0.5 (влияет на SIM и REAL одинаково).
- Разрешить стянуть с `.66` (только чтение): `candidates/nav2_smac_bounds/{bounds.patch,src}`, `candidates/fast_lio_portable/src`, `logs/bags/liolat_lidar_imu_01` (487 MB), `g_motion_0{2,3}` для идентификации выбега.
- Калибровка Virtual MC (порог страгивания, асимметрия, лаг, выбег) — только по измерениям S3b/G-серий; до этого модель помечается «идеализированная».
- Любые движения X2 — только с отдельным разрешением.
