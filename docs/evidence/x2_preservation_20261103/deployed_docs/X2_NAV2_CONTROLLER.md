# Nav2 для X2: полная цепочка NavigateToPose (сухой режим, без движения робота)

Состояние: 2026-10-21. Только сухой режим: адаптер `live:=false`, издателя MC нет, источник в MC не регистрируется. Документ сверен с кодом
(`nav_full.launch.py`, `nav_control.launch.py`, `localization.launch.py`, `goal_coordinator.py`, `x2_motion_adapter.py`, `bt/*.xml`).

Установлено в `runtime/qazbot_nav_deps` (пользовательский overlay; системный ROS и вендорские сервисы не менялись), версия 1.1.18
из подписанного снимка `snapshots.ros.org/humble/2025-08-15` (apt проверил индекс): `nav2_controller`,
`nav2_regulated_pure_pursuit_controller`, `nav2_velocity_smoother`, `nav_2d_msgs`, `nav_2d_utils`. Хэши .deb в `docs/SOURCES.md` не внесены.

Особенность: вендор пересобрал системный Fast-DDS (fastrtps 2.14, fastcdr .so.2), dpkg-записи устарели. typesupport-библиотеки
`nav_2d_msgs` из снимка ссылаются на `libfastcdr.so.1`, поэтому `ldd` показывает `not found`. Они грузятся только при создании
издателя или подписчика этих типов; Controller/RPP/smoother их не создают (6 сценариев без ошибок в журнале). Не добавлять
`libfastcdr.so.1` в `LD_LIBRARY_PATH`: смешение ABI опаснее.

## 1. Архитектура
```
X2 3D лидар -> FAST-LIO (qazbot_lio_mapping) -> /qazbot_lio/{odometry,cloud_registered}
   -> level_bridge (выровненная по гравитации плоская поза + срез облака 0,35..1,8 м над полом)
        -> TF odom->base_footprint, /qazbot_lio/odom_level, виртуальный LaserScan /qazbot_lio/scan (1440 лучей, base_footprint)
   -> AMCL (OmniMotionModel) на 2D-карте D1 (map_server, first_map_nav2_v2) -> TF map->odom, amcl_pose, particle_cloud
   -> localization_trust (только подписки; вердикт trusted, 5 Гц)

RViz 2D Goal Pose -> /qazbot_d1_map/goal_pose
   -> goal_coordinator (публичный NavigateToPose = шлюз владения целью)
      -> ОДИН bt_navigator -> ОДИН planner_server (SmacPlanner2D) и ОДИН controller_server (RPP) -> ОДИН velocity_smoother
      -> /qazbot_d1_map/cmd_vel -> goal_coordinator (шлюз команд) -> /qazbot_d1_map/cmd_vel_authorized
      -> x2_motion_adapter (live:=false) -> ~/dry_run_cmd -> кинематическая модель (только в тестах)
```
- **Координатор не навигатор.** Он не планирует, не управляет и не исполняет дерево поведения. Он принимает публичный `NavigateToPose`,
  проверяет готовность (trust, драйвер, lifecycle `bt_navigator`, кадр `map`, нормированный кватернион, пустое поле `behavior_tree`),
  пересылает цель во внутренний стандартный Nav2 и открывает/закрывает шлюз команд. Любой отказ закрывает шлюз раньше, чем уходит отмена.
  Решение зафиксировано в `DECISIONS.md`; открытый вопрос владельца: допустим ли второй (публичный) сервер `NavigateToPose` в смысле правила
  «нет второго навигатора».
- Ровно один `bt_navigator`, один `planner_server`, один `controller_server` (RPP), один `velocity_smoother`. `nav2_controller` не переписывался.
- `plan_to_pose.xml` (только планирование) сохранён для `nav_plan.launch.py`; полный запуск использует отдельное дерево `navigate_to_pose.xml`.

## 2. Топики, действия, сервисы (namespace `/qazbot_d1_map`)
| Что | Имя | Владелец / издатель |
|---|---|---|
| Действие (публичное) | `navigate_to_pose` | `goal_coordinator` |
| Действия (внутренние, стандартный Nav2) | `nav2/navigate_to_pose`, `nav2/navigate_through_poses`, `nav2/compute_path_to_pose`, `nav2/compute_path_through_poses`, `nav2/follow_path` | `bt_navigator`, `planner_server`, `controller_server` |
| Цель из RViz | `goal_pose` (PoseStamped, Reliable/Volatile) | подписчик `goal_coordinator`; **не** `nav2/goal_pose` (его читает `bt_navigator` напрямую; цель там планировалась бы в обход шлюза владения, хотя скорость не прошла бы: шлюз пропускает команды только при активной цели координатора) |
| Скорость контроллера | `nav2/cmd_vel_nav` | `controller_server` |
| Скорость после сглаживания | `cmd_vel` (приватный, не глобальный `/cmd_vel`) | `nav2/velocity_smoother` |
| Скорость после шлюза | `cmd_vel_authorized` | `goal_coordinator` (единственный издатель) |
| Выход адаптера | `motion_adapter/dry_run_cmd` (Twist), `motion_adapter/status` (JSON) | `x2_motion_adapter` |
| Статус | `goal_coordinator/status`, `localization_trust/{trusted,status}` | координатор, trust |
| Локализация | `map`, `amcl_pose`, `particle_cloud`, `initialpose`, `/qazbot_lio/{scan,odom_level,odometry}` | map_server, AMCL, level_bridge, FAST-LIO |
| Сервисы | `goal_coordinator/{stop,resume}`, `motion_adapter/{stop,resume,arm_motion}` (+ `register` только live), `localization_trust/{baseline_start,baseline_finish,confirm_anchor,arm_check}` | |

Глобальный `/cmd_vel` (там вендорский `controller_server_ros_node`) и `/aima/mc/locomotion/velocity` в цепочке не используются: в живом
сухом стеке на них 0 наших издателей (снимок графа до/после, `tests/integration_results/live_dryrun_01/`).

## 3. Владельцы TF
Наше дерево целиком на приватных топиках: `/qazbot_d1_map/tf` и пустой `/qazbot_d1_map/tf_static`.
| Преобразование | Владелец |
|---|---|
| `odom → base_footprint` | `level_bridge` (≈ 11,5 Гц; из FAST-LIO, выровнено по гравитации) |
| `map → odom` | AMCL (≈ 5,8 Гц), единственный |
| TF самого FAST-LIO | отведён на `/qazbot_lio/tf`, в наше дерево не попадает |
Вендорские `/tf` и `/tf_static` (суставы, камера, `map→foot_contact`) не содержат наших кадров `map/odom/base_footprint` и не пересекаются.
Все наши узлы подключены к приватному дереву через `-r /tf:=/qazbot_d1_map/tf -r /tf_static:=/qazbot_d1_map/tf_static`.

## 4. Конфигурация Nav2 (по коду)
Launch: `nav_full.launch.py` (аргументы `scan_topic`, `odom_topic`, `robot_radius` **без значения по умолчанию**, `max_linear` 0,3,
`max_angular` 0,5, `xy_tol` 0,10, `yaw_tol` 0,10, `map_yaml` = `first_map_nav2_v2`, `allow_dry_run_driver` false, `cell_margin` true).
Обёртка `run_nav_full.sh` требует `ROBOT_RADIUS`, задаёт `scan_topic=/qazbot_lio/scan`, `odom_topic=/qazbot_lio/odom_level`, `QAZBOT_SMAC_PREFIX`.
Все параметры ниже **кандидаты для сухих тестов, не одобренные ходовые лимиты**; единственный постоянный элемент — отсутствие рекавери.

| Узел | Параметр | Значение (кандидат) |
|---|---|---|
| planner_server | плагин | `SmacPlanner2D` (сборка с проверкой границ, `candidates/nav2_smac_bounds`), `allow_unknown=false`, `tolerance=0` |
| global_costmap | слои | `static_layer` + `inflation_layer`; `robot_radius = robot_radius + res/√2` (при 0,454 → ≈ 0,489), `footprint_padding` 0,01, `inflation_radius = inscribed + 0,01`, `track_unknown_space` |
| local_costmap | слои | `obstacle_layer` (скан, дальность 3,0 м, трассировка 3,5 м) + `inflation_layer` (радиус = `robot_radius` + 0,01, масштаб 3,0); окно 4×4 м, 0,05 м, 5 Гц |
| controller_server | частота; `failure_tolerance` | 20 Гц; 0,3 с |
| | ProgressChecker | `SimpleProgressChecker`, 0,1 м за 15 с |
| | GoalChecker | `SimpleGoalChecker`, xy 0,10 м, yaw 0,10 рад, `stateful=true` |
| | RPP (`FollowPath`) | `desired_linear_vel`=`max_linear`; `lookahead` 0,5 (0,3…0,9, без масштабирования по скорости); `allow_reversing=false`; `use_rotate_to_heading=true` (порог 0,6 рад, `rotate_to_heading_angular_vel`=`max_angular`); `use_collision_detection=true`, время до столкновения 1,5 с; регулирование по кривизне min радиус 0,9 м / min скорость 0,15; подход 0,6 м, min 0,12 м/с |
| velocity_smoother | пределы | вперёд 0…`max_linear`, боковая 0, поворот ±`max_angular`; ускорение 0,4 м/с² и 1,0 рад/с²; торможение −0,6 м/с² и −1,5 рад/с²; `OPEN_LOOP`; `velocity_timeout` 0,5 с |
| bt_navigator | деревья | `navigate_to_pose.xml`; `NavigateThroughPoses` → `unsupported_through_poses.xml` (`AlwaysFailure`); загружены только плагины `compute_path_to_pose` и `follow_path` |
| lifecycle_manager_navigation | один, на `controller_server`, `velocity_smoother`, `planner_server`, `bt_navigator`; `attempt_respawn_reconnection=false`; выход любого критического процесса завершает всю группу launch |

Боковая скорость 0 по замыслу (порог страгивания вбок MC 0,5–0,7 м/с). Процессы стартуют с шагом 0,6 с (AimDK: ≤ 2 новых ROS-нод в секунду).
Не измерено и не задано: `bt_navigator` `default_server_timeout` (по умолчанию Nav2 20 мс на подтверждение цели серверу) и `wait_for_service_timeout`
(1000 мс). См. «Известные дефекты».

## 5. Рекаверы и безопасное завершение
- В стеке **нет** `BackUp`, `DriveOnHeading`, `Spin`, `RecoveryNode` и `behavior_server` (проверено поиском по `bt/`, `nav_full.launch.py`, `nav_control.launch.py`).
  В дереве `navigate_to_pose.xml` только `ComputePathToPose` → `FollowPath`; `allow_reversing=false`; минимальная скорость сглаживателя по x равна 0.
  У X2 нет датчика сзади, движение назад не допускается.
- **Безопасное завершение = `ABORTED` + нулевая скорость + простой.** Отказ планировщика, контроллера, прогресса (0,1 м / 15 с), столкновение по
  прогнозу, потеря `map→odom` у Nav2: цель завершается `ABORTED`. Координатор закрывает шлюз и публикует нулевой Twist; адаптер выдаёт пачку из 5 нулей
  и переходит в `idle` (это нормальный конец цели, не авария). Отказы безопасности (trust, перехват, чужой издатель, stop) дополнительно защёлкивают
  адаптер и координатор (см. `X2_LOCALIZATION_TRUST.md`, «Защёлка и возобновление»). Автоматического повтора цели нет.
- Отмена цели оператором (`cancel` публичного действия, `goal_coordinator/stop`): шлюз закрывается, публикуется нуль, внутренняя цель отменяется.
  `cancel` приводит к статусу «отменена» (5), без защёлки адаптера; `stop` защёлкивает.

## 6. Тесты и счётчики
| Набор | Команда | Результат (2026-10-21) |
|---|---|---|
| адаптер (core 33 + early_takeover 1 + new_faults 3 + dry_run_shadow 6) | `cd tests/motion_adapter && python3 -m unittest -v test_adapter_core test_early_takeover test_new_faults test_dry_run_shadow` | **43 OK** (до работ 37) |
| trust (loc_trust 16 + source_failures 13 + trust_gaps 17) | `cd tests/loc_trust && python3 -m unittest -v test_loc_trust test_source_failures test_trust_gaps` | **46 OK** (до работ 29) |
| критерий остановки | `cd tests/nav_e2e && python3 -m unittest -v test_stop_criterion` | **7 OK** |
| controller_dry (модель MC, без карты) | `bash tests/controller_dry/run_dry.sh` | **19 проверок ok** |
| launch_stagger (нужен `QAZBOT_SMAC_PREFIX=$PWD/candidates/nav2_smac_bounds/install/nav2_smac_planner`) | `python3 -m unittest -v tests/launch_stagger/test_launch_stagger.py` | **5 OK** (без переменной 4 OK + 1 ERROR, зависит от окружения) |
| footprint | `tests/footprint`: `test_vendor_polygon` 4, `test_envelope_rest` 1 (нужен `X2_URDF_DIR`) | **4 + 1 OK** |
| легаси nav_e2e (идеальная локализация, подмена статуса адаптера) | `bash tests/nav_e2e/run_model.sh --legacy NAME` | **73 проверки, 0 ошибок** (`nav_e2e_legacy_impl2b_01`) |
| **сквозной, реальная цепочка** (настоящий адаптер live=false + настоящий trust + Nav2 + координатор + модель) | `bash tests/nav_e2e/run_model.sh NAME` | **`nav_e2e_real_dryrun_08`: 160/160**, rc 0, завершение всех групп 0, `abnormal_children` 0 |

Все ROS-прогоны — изолированный домен 87, `ROS_LOCALHOST_ONLY=1`, профиль `tests/nav_e2e/dds_domain87_unicast.xml`; рабочий DDS не затрагивается.
Единый источник карты и поз фикстуры: `tests/nav_e2e/e2e_map.py` (карта `first_map_nav2_v2`, хэш PGM читается из манифеста).
`START (-1,275; -4,825; 0)` — **поза фикстуры**, не физическая домашняя поза (у новой карты нет `home.yaml`).

Покрытие реальной цепочки (в каждом случае критерий остановки и, при успехе, сверка позы модели с допусками цели):
прямая 1,5 м (ошибка xy 0,064 м); поворот на месте (ошибка yaw 0,055 рад); уже в цели (без движения); повтор цели (повтор при активной отклонён, та же цель после
успеха не двигает); отмена; препятствие (неотмеченный в карте ящик, зазор 0,745 м > 0,464, `ABORTED`); недостижимая цель, цели вне карты (4) и в занятой клетке без движения;
RViz `goal_pose`; **перехват PS5 через настоящий адаптер** (поддельный `McCommonState` меняет источник на `rc`: цель A завершается, внутренняя цель терминальна,
защёлка `operator_takeover`, новая цель при защёлке отклоняется, `resume` один движения не даёт, A не оживает, цель B доходит); потеря скана, одометрии, AMCL;
ложная локализация (неверная начальная поза, скачок позы во время цели); оператор stop; драйвер не готов и потеря драйвера; **перезапуск Nav2** (старая цель не
возобновляется, новая с новым UUID доходит; ошибка xy цели B 0,057 м); проверки графа (нет издателя MC, нет клиента `SetMcInputSource`, нет fd блокировки у адаптера, нет глобального `/cmd_vel`).
Критерий остановки (`tests/nav_e2e/stop_criterion.py`): терминальный статус; последняя команда и её время; нет ненулевых команд после терминала плюс период управления 0,05 с;
модель |v|,|w| < 0,01 и дрейф < 5 мм за 1 с; та же проверка после восстановления (нет повторного движения). «Последние 3 нуля» не требуются.
Фикстура: `reset_pose` телепортирует модель, поэтому trust один раз фиксирует плохое совпадение и ждёт 5 с стабильности: ожидание только в тесте.

**Открытый провал**: `nav_e2e_verify_01` (независимый проверяющий) упал на `straight_succeeded` (статус 6): `bt_navigator` «Timed out while waiting for action server to acknowledge
goal request for compute_path_to_pose» при нагрузке ≈ 11,8 на 8 ядрах (параметр по умолчанию 20 мс не задан в launch). Не маскируется, таймаут не менялся, причина не устранена; нужен измеренный
параметр, а не произвольное увеличение. Предыдущие одиночные срывы старта `bt_navigator` («server not available after waiting for 1.00s», `wait_for_service_timeout`)
в прогонах 06/07/08 не повторились; гипотеза причины (не доказана): обнаружение DDS нового клиента при >1 с на загруженном Jetson и перечень initial peers только для participant id 0–7
в тестовом профиле.

## 7. Запуск и остановка (сухой стек на аппаратном DDS, домен 0)
Порядок запуска (робот стоит; каждая пара запусков ≥ 2 с, внутри launch шаг 0,6 с). Файлы PID пишет `detach.py` в `~/control_backend/logs/NAME.pid`; значения PID ниже на 2026-10-21 и со временем устареют, перед остановкой проверять `ps`.
1. Локализация (FAST-LIO + level_bridge + map_server + AMCL + lifecycle_manager): `run_localization.sh lio_binary:=… map_yaml:=…/first_map_nav2_v2/map.yaml`
   (текущий запуск: launch PID 387538, `logs/localization_v2_dryrun_01.pid`). Используется переносимый бинарник `candidates/fast_lio_portable/install/fast_lio/lib/fast_lio/fastlio_mapping`
   (sha256 `63903a5c…` (после loop-back fix 22.10; прежний 4097314b…); тот же исходник FAST-LIO плюс `runtime_logging.patch`, воспроизведение на записи дало нулевую разницу поз). Умолчание launch — `runtime/fastlio_mapping` (`87040c81…`).
2. trust: `run_trust.sh` (закрепляет `expected_map_sha256` из манифеста v2), `logs/trust_dryrun_01.pid` (PID 388248).
3. адаптер: `run_x2_driver.sh -p live:=false`, `logs/adapter_dryrun_01.pid` (PID 388336). **Без `-p live:=true`, без вызова `~/register`.**
4. Nav2 + координатор: `cd ws/src/navigation && ROBOT_RADIUS=0.454 python3 detach.py nav_full_dryrun_02 bash run_nav_full.sh allow_dry_run_driver:=true`,
   `logs/nav_full_dryrun_02.pid` (launch PID 394586). Радиус 0,454 — консервативный описанный прямоугольник (`X2_FOOTPRINT.md`), не квалификация габарита.
   Планировщик остаётся в активации (costmap ждёт `map→base_footprint`), пока оператор не поставит начальную позу AMCL (`RVIZ_LOCAL.md`); после этого `bt_navigator` активируется сам.

Остановка в обратном порядке, каждый процесс адресно SIGINT (без `pkill`):
`kill -INT $(cat logs/nav_full_dryrun_02.pid)`; `kill -INT $(cat logs/adapter_dryrun_01.pid)`; `kill -INT $(cat logs/trust_dryrun_01.pid)`; `kill -INT $(cat logs/localization_v2_dryrun_01.pid)`.
Проверка: `pgrep -af "fastlio_mapping|level_bridge|amcl|map_server|planner_server|controller_server|bt_navigator|velocity_smoother|lifecycle_manager|goal_coordinator|x2_motion_adapter|localization_trust"` пусто.
Отмена текущей цели из терминала: `ros2 service call /qazbot_d1_map/goal_coordinator/stop std_srvs/srv/Trigger`.

Проверено на аппаратном DDS в сухом режиме: до/после граф 93→108 узлов, 264→309 топиков; добавлены только наши узлы и действия; издателей `/aima/mc/locomotion/velocity` 4 вендорских (наших 0),
издатель `/cmd_vel` один вендорский; у наших узлов нет клиента `SetMcInputSource`; `status`: `mode=dry_run`, `registration=dry_run`, `counters.published=0`; блокировку владельца никто не держит.

## 8. Известные дефекты (не маскируются)
1. **Завершение lifecycle manager** (Nav2 Humble): периодически SIGABRT/зависание при shutdown; кандидат `candidates/nav2_lifecycle_shutdown` НЕ принят (SIGSEGV в ожидании RMW /
   `Bond::~Bond`). Прогон 03: 0 аварийных дочерних, повторный старт группы при перезапуске Nav2 иногда даёт `abnormal_children`. Не лечить увеличением времени ожидания и отключением bond.
2. **`planner_server` SIGABRT (код −6)**, если SIGINT пришёл, пока `on_activate` ждёт TF costmap («rcl node's context is invalid»). Воспроизведено на живом стеке до установки позы AMCL.
   Завершение штатного (всё active) стека на аппаратном DDS ещё не проверено.
3. **`level_bridge` выходит с кодом 1 по SIGINT**: `RuntimeError` в `take_message` внутри `rclpy.spin` при завершении (ловится только `KeyboardInterrupt`). Косметика: все издатели уже остановлены. Не исправлено.
4. **`default_server_timeout` bt_navigator = 20 мс** (умолчание Nav2) не измерен для нагрузки X2; первая цель из RViz при нагрузке ≈ 11–12 может завершиться `ABORTED` (безопасный исход, без движения). См. открытый провал `verify_01`.
5. Бонд и обнаружение потери серверов оставлены включёнными; автоматического возобновления цели при перезапуске нет.
6. `tests/planner_clearance` на карте v2 не запускался (нет базовой линии v2).

## 9. Сухой замкнутый тест контроллера (`tests/controller_dry/run_dry.sh [radius]`)
Кинематический симулятор MC (отсечка команды 0,2 с, пороги страгивания 0,1 м/с и 0,05 рад/с, лаг 0,25 с) в приватном namespace
`qazbot_ctl_test`, 6 нод; скрипт принудительно использует домен 87, localhost и unicast-профиль. Результат 2026-10-21: 19 проверок ok (прямая 1,5 м, поворот на месте,
отмена без ползучести, цель достигнута без команд, стена на 1,3 м: отказ `collision ahead`, остановка; плюс проверки остановки после терминала, отмены и аборта). Раньше проверка конца
использовала «последние нули smoother»; она заменена критерием с ограничением торможения 0,6 м/с², так как в 1.1.18 выход smoother не гейтится.
Ранняя оценка загрузки 2026-10-21 (`docs/evidence/controller_dry_2026-10-21.json`): controller 6,9 %, smoother 3,8 % одного ядра.
Наблюдение: при разгоне smoother даёт ~0,25 с команд ниже порога страгивания 0,1 м/с; MC такие, вероятно, не исполнит. Поднимать малые скорости до минимума (как Nova-Valley) нельзя; порог надо измерить на .66.
Ловушка: `env.sh` содержит `set -e`; в скриптах после `source` нужен `set +e`.

## 10. Исправление Smac bounds
На исходной версии 1.1.18 цель вне карты воспроизвела SIGSEGV; gdb показал
Costmap2D::getCost → GridCollisionChecker → Node2D → AStarAlgorithm. Исходник
не проверял return worldToMap и использовал неинициализированные индексы.
Отдельная сборка сохраняет алгоритм 1.1.18 и добавляет две проверки границ с
явной ошибкой. Источник, patch и build.log: candidates/nav2_smac_bounds
(каталог `install` не входит в архив-снимок; нужен отдельный экземпляр).
Ни системный ROS, ни вендорские пакеты не заменены.

Full launch требует QAZBOT_SMAC_PREFIX и проверяет hash библиотеки по манифесту.
Новый плагин подтверждён через /proc/PID/maps. 20 прежних пар планирования дали
побитно равные JSON cases: 17 путей, 0 нарушений R=0,65, min clearance 0,66123 м
(на прежней карте; на карте v2 базовой линии нет).
Это регрессия алгоритма; R=0,65 не объявляется физическим footprint X2.

NavigateThroughPoses в данном профиле не реализован: его приватный BT явно
AlwaysFailure без движения. Публичный API здесь только NavigateToPose.


## 2026-10-22: блокировка страгивания поворота RPP

См. [RPP_MC_ALIGNMENT_20261022.md](RPP_MC_ALIGNMENT_20261022.md): подтверждение
по G3, одна правка max_angular_accel с сохранением OPEN_LOOP smoother,
адресная проверка до/после и статус физической приёмки. FAST-LIO не менялся.
