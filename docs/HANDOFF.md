# QazBot X2: продолжение — первые физические RViz goals (2026-10-28, вечер)

Обновлено 2026-10-28 ~23:00 CST (+08), X2 agi@192.168.1.72. Задача оркестратора `20261028-scan-latency-rviz-goal`. Начать с этой передачи, [X2_NAV_BRINGUP.md](X2_NAV_BRINGUP.md) и AGENTS.md, не с нового аудита.

**Итог дня:** первый физический NavigateToPose **SUCCEEDED** (21:05, ~2.2 м, 12.5 с), затем ещё два SUCCEEDED (22:12 A 4.9 с, 22:42 #1 8.8 с). Цель оператора «3 goals подряд» **НЕ достигнута**: серии прерываются T5 (AMCL не сходится) и RPP collision abort. Стек на момент записи **работает** (оператор тестирует), recorder пишет.

## ТЕКУЩИЕ БЛОКЕРЫ (главное)

Влияние на «3 goals подряд»: H = прерывает серию прямо сейчас, M = может прервать, L = риск/долг.

| # | Блокер | Доказательства | Влияние | Предложенное решение | Владелец / нужно решение |
|---|---|---|---|---|---|
| 1 | **AMCL не сходится после seed** → T5 map→odom jump отменяет goal даже при 0.25 м | P10: goal B 22:12:55 отменён через 4.6 с, jump 0.1045 м, sigma_xy .71→1.26 м, scan_score .02–.16 (порог .35), second_mode .82–.89; 22:45:10 goal отменён через 2.2 с `TRIP localization_untrusted [T5_map_odom_jump]`; 22:48 sigma_xy .82, second_mode .98, scan_score .14 | **H** | Применено, **не проверено**: max_beams 60→180 (live + launch), `post_seed.py` = 12 no-motion updates после initialpose, resume только при READY. Далее при неудаче: проверить качество скана/карты для AMCL (scan_score низкий при 90° FOV), уточнить initialpose | Оркестратор + оператор (initialpose) |
| 2 | **HAL с приоритетом nice 0** (ручной экземпляр PID204720 через em_run.sh) против nice -19 у em на эталонном роботе .89 | P8d: PID204720 NI 0, cgroup user session, em не отслеживает; .89: HAL nice -19 все 66 потоков; P1: первая задержка = доставка raw cloud HAL под CPU нагрузкой (schedstat HAL run 30.6 с / wait 89.9 с) | **M** (~5–10 % статусов T1_scan_stale; T1 pause теперь не отменяет goal ≤3 с) | `sudo kill 204720` (владелец run) затем `aima em start-app hal_sensor_orin`; никогда не запускать em HAL пока 204720 жив | **Оператор (sudo), отложено оператором** |
| 3 | **Lethal marks вне FOV** | P7: лидар даёт конечные лучи только ±45°; ObstacleLayer без decay, inf_is_valid=false → метки от прежних поз/людей не очищаются | M | I1 bounded clearing установлен (2d240dbd); replanning 1 Гц обходит; глобальные метки остаются до выхода из окна. Далее: STVL/decay или ClearCostmap recovery в BT | Решение оператора (decay = архитектурное изменение costmap) |
| 4 | RecoveryNode x3 / failure_tolerance 10 с / replanning **не проверены** на реально заблокированном goal | P11/P13 только xml/py_compile + live param read-back 10.0; 22:42 goal #2 abort был при failure_tolerance 0.3 (до P13) | M | Контролируемый тест: goal за препятствием, оператор рядом | Оператор (разрешение теста) |
| 5 | Resume скорости 0→0.3 м/с обходит smoother accel (P8v F2) | P8v: после T1 pause adapter может применить свежую Nav2 команду сразу | L | Наблюдать вживую; MC ramp не проверен | Оркестратор, проверка по bag |
| 6 | Adapter **не делает DELETE** MC source при остановке | P9b: stale `sdk_qazbot_nav2_1028a` → ADD code=1; ручной DELETE (code 0) | L (блокирует следующий старт) | DELETE в destroy_node или one-shot DELETE перед stop; `start_nav.sh` default всё ещё `1028a` → использовать `QAZBOT_SOURCE_NAME=sdk_qazbot_nav2_1028b` | Оркестратор (код) |
| 7 | ros2 CLI нестабилен (daemon убит), чужие зависшие shells 450989/473127 | P12/P13: `ros2 topic echo` в restart_gc.sh → «status unavailable» | L | Использовать rclpy-скрипты (post_seed.py); чужие PID не трогать | — |
| 8 | Recorder sqlite «database is locked» → смерть записи | goal_bag_201325 умер 21:03:58, goals #3/#4 не в bag | L (исправлено) | Теперь mcap (`record_goal.sh`) | — |
| 9 | RC takeover latch **не доказан на железе** | нет live теста в этой сессии | L (безопасность) | Отдельный тест с оператором | Оператор |

## Что работает сейчас (проверять живые PID, старые PID не являются состоянием)

Evidence/скрипты: `EV=/home/agi/control_backend/docs/evidence/nav_bringup_20261028_195035` (копия в Git: [evidence/nav_session_20261028](evidence/nav_session_20261028/)).

| Компонент | Последний PID / источник | Примечание |
| --- | --- | --- |
| localization launch (FAST-LIO cpus 5,6, level_bridge I1, native_imu bridge, map_server, AMCL) | 464092 (`start.sh`, P9) | карта first_map_nav2_v3_despeckled, AMCL params ниже |
| Localization Trust (go2, T5 0.25) | 494295 (`restart_trust.sh`, P12) | despeckled map + SHA 6fc67526… |
| Motion adapter, MC source `sdk_qazbot_nav2_1028b` prio 30 | 479952 | registered |
| nav_full (controller/planner/smoother/bt_navigator + Goal Coordinator) | 509608 (P13) | failure_tolerance 10.0 live |
| Recorder mcap | 471147 (`goal_bag_mcap_215612`) | **не останавливать без оператора** |
| DDS профиль всех owned | `$EV/dds_nav_two_iface_log.xml` | UDP 10.0.1.41 + 192.168.1.72, Fast DDS лог в файл |
| Vendor HAL | 204720 (run, nice 0) | не трогать без sudo-решения |

## Изменения 2026-10-28 (значения, причина, одобрение)

Код: коммит 5b17bd9 в ветке `nav/first-goals-20261028`; repo ws/src = deployed ws/src (SHA256 сверены). Backups `*.bak_20261028_pre_*` рядом с файлами на X2.

| Изменение | Значения | Причина | Одобрение / нарушенное правило AGENTS.md |
| --- | --- | --- | --- |
| AMCL odom-trust (P2a) | max_beams 1440→60, alpha1–5 0.2→0.1, recovery_alpha_slow/fast 0 | оператор: доверять одометрии больше карты; 1440 лучей — overconfident + CPU | оператор 2026-10-28 |
| AMCL update (P12) | update_min_d 0.25→0.10, update_min_a 0.2→0.10 (live param set + launch) | меньшие, частые коррекции вместо скачков | оператор 22:2x |
| AMCL beams | max_beams 60→180 (live + launch, bak `pre_beams180`) | сходимость (scan_score .14) | оркестратор 22:48 по мандату оператора «make it work»; эффект не проверен |
| Trust T5 (P12) | jump_xy_max/jump_yaw_max 0.10→0.25 м/рад через ROS params (`start.sh`, `restart_trust.sh`); loc_trust.py defaults неизменны | P10: реальные коррекции 4–10 см, ложные global locks ≥0.5 м | **оператор, нарушает правило «не ослаблять T5»** |
| T1 pause/resume (P8c/P8v/P10) | `t1_transient` в Trust status; GC и adapter: T1-only потеря → ноль, goal сохраняется; cancel+latch если >3.0 с (`t1_pause_max_s`); T1 пороги не менялись (scan .5, odom .5, TF 1.0); GC логирует TRIP причину | P1/21:07: goal #4 отменён транзиентной T1 при нагрузке | оператор 21:20; **отходит от правила latch-on-T1**; verifier PASS |
| I1 2d240dbd redeploy (P8b/P9) | level_bridge c817a5ed…, nav_control I1 bounded_rays; backup `i1_deploy_2d240dbd_r2` | out-of-FOV метки | оператор 21:20 |
| Карта despeckled | first_map_nav2_v3_despeckled: 8-связные occupied компоненты ≤4 px → free (461 комп./614 px), pgm 6fc67526…; v3 f6032f75 не тронута | одиночные пиксели давали phantom lethal → goal #1 «goal occupied» | оператор 21:02; launch-файлы по умолчанию всё ещё v3, скрипты передают despeckled |
| DDS log section | `dds_nav_two_iface_log.xml`: Fast DDS log → файл (DATA_FRAG flood через ros2 launch stdout) | CPU/шум | P4b |
| Nav2 replanning (P11) | BT PipelineSequence + RateController 1 Гц; global obstacle_layer (I1 sources); GridBased.tolerance 0→0.3 | оператор: «планировщик должен объезжать» | оператор 22:15 («just make it work»; tolerance явно не подтверждён) |
| RecoveryNode (P13) | RecoveryNode(retries 3){PipelineSequence; ComputePathToPose}; `nav2_recovery_node_bt_node` в plugin_lib_names | 22:42 goal #2: RPP collision → abort без попытки перепланировать | оператор |
| failure_tolerance | 0.3 → 3.0 (P13) → **10.0** (live + launch, bak `pre_ft10`) | дать время replanning | оператор 22:52 |
| MC source | `sdk_qazbot_nav2_1028b` (stale 1028a удалён DELETE) | ADD code=1 из-за stale регистрации | P9b |
| Vendor apps остановлены (`aima em stop-app`) | perception, orbbec_camera, hal_camera, drp, lx_webrtc_app; idle 66.9→79.5 % | CPU (камерная цепь ~1.4 ядра, не нужна nav) | оператор 21:20 и P8d; **правило «не трогать vendor» нарушено по решению оператора** |
| ClearEntireCostmap local+global | 20:57:47 | по просьбе оператора | **правило запрета нарушено по решению оператора** |
| Тесты | 22:28 оператор: больше не запускать unit-suites, только минимальные live проверки | | оператор |

Restart vendor: `aima em start-app hal_camera; aima em start-app orbbec_camera; aima em start-app perception; aima em start-app drp; aima em start-app lx_webrtc_app`.

## Находки

- **Латентность (P1, 19:20–19:47):** первая задержка = доставка raw cloud HAL под CPU нагрузкой (load 9–11 без стека, 17–23 со стеком/probe, 8 ядер). Raw cloud age vs scan end p50 .012/p99 .082/max .376 с; FAST-LIO +p50 25 мс; level_bridge +p50 5 мс (p99 32–44 мс). **Накопление очередей level_bridge опровергнуто**; UDP drops на owned сокетах 0. Уверенность средняя (probe сам нагружает). Таблица: `evidence/nav_session_20261028/orchestrator/reports/20261028-scan-latency-debugger.md`.
- **HAL nice 0 vs -19** (см. блокер 2) — сильный кандидат первопричины.
- **Lidar loop-back stamps:** FAST-LIO «lidar loop back, clear buffer» 59×, level_bridge «odometry stamp went back» 86× (часть ~эпоха 0) с 19:50 — сбрасывают кэш позы → пропуски скана. Не исследовано глубже.
- **AMCL не сходится** (блокер 1): T5 срабатывает на реальных шагах коррекции.
- **Phantom marks** (P7): не скачки одометрии (native creep 0.0106 м/мин), а метки вне ±45° FOV без очистки + одиночные пиксели карты.
- **Recorder sqlite lock death** → mcap.
- **ros2 CLI** нестабилен; daemon убит агентом; использовать rclpy.
- A0: `run_localization.sh` молча переключает DDS на dds_qazbot.xml — вызывать `ros2 launch` напрямую (скрипты так и делают).

## Результаты физических goals (2026-10-28, X2, оператор рядом, RViz → /qazbot_d1_map/goal_pose)

| Время CST | Goal | Откуда → куда (map) | Результат | Причина / примечание | Bag |
| --- | --- | --- | --- | --- | --- |
| 20:54 | #1 | (-6.56,5.82)→(-5.52,3.76) | ABORTED | planner «goal occupied» (одиночный пиксель карты, tolerance 0) | goal_bag_201325 |
| 20:56 | #2 | (-6.54,5.83)→(-6.05,3.64) | ABORTED | RPP collision ahead ×7 / 0.9 с (phantom), один шаг | goal_bag_201325 |
| 21:05 | #3 | (-6.42,5.59)→(-5.82,3.46) ~2.2 м | **SUCCEEDED 12.5 с** | первый физический SUCCEEDED (despeckled map, новая initialpose) | нет (recorder умер 21:03:58) |
| 21:07 | #4 | (-5.61,3.59)→(-3.92,1.11) | CANCELLED 2.0 с | GC localization_untrusted: транзиентная T1 + старый latch → привело к T1 pause | нет |
| 22:12:37 | A | (-5.72,5.49)→(-5.06,3.82) | **SUCCEEDED 4.9 с** | | goal_bag_mcap_215612 |
| 22:12:55 | B | (-5.06,3.86)→(-5.69,0.15) | CANCELLED 4.6 с | T5 jump 0.1045 м > 0.10 (P10) | goal_bag_mcap_215612 |
| 22:42:06 | 1 | (-5.45,5.40)→(-4.96,3.17) | **SUCCEEDED 8.8 с** | T5 0.25, replanning 1 Гц | goal_bag_mcap_215612 |
| 22:42:37 | 2 | (-4.41,3.59)→(-3.41,0.76) | ABORTED 11.2 с | RPP collision ahead ×6 → patience (failure_tolerance 0.3, до P13); 22:44:19 GC TRIP operator_stop | goal_bag_mcap_215612 |
| 22:45:10 | 3 | (-3.62,0.86)→(-5.82,4.84) | CANCELLED 2.2 с | T5_map_odom_jump при 0.25 (AMCL не сошёлся) | goal_bag_mcap_215612 |

Bags в Git **не публикуются** (решение оператора): пути, размеры и SHA (для живого goal_bag_mcap_215612 — копия на 22:51:19 CST, sha256 d3f43303…) в `docs/evidence/nav_session_20261028/manifest.json` → `bags_not_published`; файлы на X2 в `$EV`.

## Следующие действия

1. Оператор: RViz 2D Pose Estimate в истинной позе → `post_seed.py` (12 no-motion updates, метрики, resume GC+adapter только при READY). Проверить sigma_xy/second_mode/scan_score с max_beams 180.
2. 3 goals подряд 1–3 м; при T5 — собрать метрики AMCL из bag, не ослаблять T5 дальше без решения оператора.
3. Решение оператора по HAL vendor-restart (sudo) — блокер 2.
4. Тест заблокированного goal (RecoveryNode/failure_tolerance 10 с) с оператором.
5. Код: adapter DELETE MC source при остановке; `start_nav.sh` default source; launch-файлы default → despeckled map.
6. Остановка: только `$EV/stop.sh` (motion first), recorder SIGINT отдельно с разрешения оператора. Bag summaries по goal-окнам (±30 с) — не сделаны, следующий шаг документации.

Продолжение: «продолжи задачу 20261028-scan-latency-rviz-goal из .orchestrator/PLAN.md».

## История (кратко)

- До 2026-10-28 18:56: I1 2d240dbd пассивно установлен, LIVE freshness FAIL (55/1356 статусов, scan max 1.207 с) → rollback; T5 стационарный LIVE принят (4 seeds, 0 ложных T5, READY 124.97 с). Данные: evidence/t5live1_summary_20261028.json, I1_*_20261028.json, i1_cloud_registered_udp_20261028_092700Z (parts). Детали — история Git ветки handoff/first-rviz-goal-20261028 (коммит 2b0f9a7).
- I1 / PR #1: 2d240dbd09ca69de87e9ac4b1acde735293ad6b2 (не merged в main); main 6e70c0f9. Чужие незакоммиченные файлы сохранены: docs/X2_QUICKSTART_BRINGUP_RU.md, tests/loc_trust/run_ros_synthetic.py, tests/nav_e2e/*.
- X2 /home/agi/control_backend — deployed-каталог без .git; Git checkout — /home/agi/control_backend/qazbot-x2-stack.
- HAL PID2623 ранее упал в FastDDS SHM; восстановлен один штатный HAL PID204720 через em_run.sh (отсюда nice 0).
