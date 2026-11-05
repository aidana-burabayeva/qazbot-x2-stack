## 2026-11-05 вечер: C++ projector на роботе, E2E-02 (3 м) SUCCEEDED, D-05 governor подготовлен офлайн

- Robot runtime (не Git): C++ projector из `9417bed` (z_min −0.65) развёрнут; Motion Adapter перезапущен 17:00 с `odom_topic=/qazbot_lio/odom_level` (yaw-loop P20/P23 получает свежие данные, stale 0), регистрация ENABLE. Production-код больше не менялся; проверить живые PID.
- E2E-02 (свободная прямая 3 м, SpeedLimit 0.20, через GC): NavigateToPose SUCCEEDED за 60 с, но качество НЕ принято: перелёт цели и циклические развороты. Результат: [20261105_e2e02_free3m.md](results/20261105_e2e02_free3m.md) (ветка `feat/projector-switch-prep`, SHA 4329077); bag lossless: [evidence/e2e02_20261105](evidence/e2e02_20261105/README.md) (ветка `feat/d05-approach-governor`).
- Первопричина (уточнена): MC ~0.5 м/с при cmd 0.2, задержка 0.4–0.5 с, выбег 0.07–0.31 м; боковая ошибка 0.10–0.18 м > допуск 0.10 м → RPP выбирает rotate-to-path и разворачивается. Подробно: [20261105_d05_governor.md](results/20261105_d05_governor.md).
- Подготовлено (НЕ установлено): C++ пакет `qazbot_approach_governor` (ветка `feat/d05-approach-governor`): только уменьшает vx при остатке пути ≤ предсказанного тормозного пути. Один недостаток: один governor проблему не закрывает (боковая ошибка). Следующий шаг — эксперименты E1/E2 из отчёта и решение оператора по боковой ошибке; установка/движение только по отдельному разрешению.
- 2026-11-05 P0 анализ (без движения): допуск xy/yaw 0.10/0.10, `stateful false`; «SUCCEEDED» E2E-02 был проходом в движении, фактический покой в 0.21 м/0.26 рад от цели; разворот вызывает переход RPP pure-pursuit → rotate-to-path (угол к carrot > 0.6 при боковом смещении 0.10 м). Отчёт: [20261105_goal_checker_analysis.md](results/20261105_goal_checker_analysis.md). Подготовлен (не применён) переключатель `goal_checker_stopped` в ветке `feat/e2-goal-checker-stopped` (7d06b8d). Governor доработан (потеря данных, GC-контракт), остаётся недостаточным и не устанавливается. Следующее: E1 (SpeedLimit 0.10, один Goal 2.5 м) и E2 — только по разрешению оператора.
- Не менять: xy/yaw tolerance, planner, AMCL, costmap, MC arbitration, GC, Trust.

## 2026-11-03 preservation and V continuation

Current source/deploy audit, backup scope, remote verification, fresh 11-test result and corrected C1/C2 analysis: [preservation report](evidence/x2_preservation_20261103/REPORT.md). Current source snapshot branch: `robot/preservation-20261103`. D-05 and D-07 remain OPEN; full V hardware/replay validation is incomplete. No production logic or running nodes were changed. Historical entries below retain their original dates.

# QazBot X2: продолжение — первые физические RViz goals (2026-10-28, вечер)

## Проверка .72 и передача для симуляции — 2026-10-30

Последнее указание оператора: не скачивать все bags; проанализировать и документировать. [Актуальный отчёт и план воспроизведения](digital_twin/X2_ROBOT_EVIDENCE_20261030.md). Снимки: `docs/evidence/x2_twin_20261030/`. Этот раздел новее исторического состояния ниже. Установленный backend также опубликован в обычных `ws/src/` путях ветки `baseline/x2-robot-20261030`: [проверка 73 SHA256 и ограничения симуляции](digital_twin/PINNED_ROBOT_BASELINE_20261030.md).

- По явной просьбе оператора выполнен откат на `bak_20261030_pre_p26`; 5 основных файлов сверены с бэкапами, фактические параметры 12 узлов сохранены. Цели/команды движения агент не отправлял.
- Core recorder Claude завершён штатно 20:32:03 UTC+08. Сохранены на роботе 87.9 мин / 2 237 829 сообщений; SHA256 14 MCAP и чтение summaries проверены. На ПК только небольшие снимки (~2 МБ), не bags.
- Установленная карта отличается от Git: PGM SHA256 `72ee4094b394241e7b2761878774efbdd99e93731d7cb52709b2231c3975b354`. Использовать сохранённые deployed PGM/YAML.
- Срыв initialpose включает XY и yaw; относительное смещение до 1.72 м, yaw до 35.7° в поздней серии. Это не независимая абсолютная ground-truth ошибка.
- Контрольный возврат модели likelihood_field дал меньшие смещения в четырёх следующих эпизодах, но сравнение не было рандомизированным/парным. Причина не объявлена доказанной.
- Новых SIM PASS, LIVE navigation PASS, доказанного устранения срыва и гарантии sim→real нет. Большие данные остаются только на роботе; SHA256 не заменяет резервную копию.


## 2026-10-29 (самая свежая запись; ниже сохранена история 2026-10-28)

### 2026-10-29 вечер (21:45-22:20): карта D1 включена, P20 yaw-loop развёрнут живьём

Обновлено 2026-10-29 ~22:30 CST (+08), задача P22. Источники: `.orchestrator/PLAN.md` (записи 20:2x-22:1x; копия [orchestrator/PLAN.md](evidence/nav_session_20261029/orchestrator/PLAN.md)), [DECISIONS.md](evidence/nav_session_20261029/orchestrator/DECISIONS.md), live-тест P20 ([calib/p20_yaw_test_*](evidence/nav_session_20261029/calib/), [logs/p20_yaw_test_run2.out](evidence/nav_session_20261029/logs/p20_yaw_test_run2.out), [logs/p20_restart_run.txt](evidence/nav_session_20261029/logs/p20_restart_run.txt)), [logs/operator_actions.txt](evidence/nav_session_20261029/logs/operator_actions.txt). Repo `ws/src` = deployed `ws/src` (sha256 сверены в P22: adapter, nav_full/nav_control/localization launch, bt xml, карта D1). Требования к цифровому двойнику по данным этого дня: [DIGITAL_TWIN_FINDINGS.md](DIGITAL_TWIN_FINDINGS.md).

**Итог вечера:** (1) демо на старой карте 2/2 SUCCEEDED (20:5x, 9.2 с / 3.9 м и 11 с / 4.9 м); (2) переключились на карту D1 `Qazbot_29.10`, исправили ошибку порога free_thresh в её map.yaml, scan_score 0.47 (лучше, чем 0.35-0.40 на старой); (3) P20 yaw-rate loop развёрнут и проверен живьём: in-place поворот теперь устойчив в обе стороны, остаток: обратный доворот CCW ~10 град после остановки и недобор скорости CW. Goals на карте D1 с P20 ещё не отправлялись.

#### Текущий рабочий конфиг (на ~22:20; проверять живые PID)

| Компонент | Значение |
| --- | --- |
| Карта | `ws/src/maps/d1_qazbot_20261029` (D1 slam_toolbox `Qazbot_29.10`, 673x903 @ 0.05 м, origin -30.01/-22.41, pgm sha 83d05baf..., map.yaml sha f747664c... с `free_thresh: 0.196`; оригинал D1 `map.yaml.orig_from_d1` c8dc4169... с 0.25). Скрипты `start.sh`/`start_loc.sh`/`start_nav.sh`/`restart_gc.sh`/`restart_trust.sh` переключены (бэкапы `*.pre_d1map`) |
| localization + Trust | load_map 21:47 (AMCL получил 673x903), Trust перезапущен после правки free_thresh (21:5x); T5 `jump_xy_max` **1.0 м**, `jump_yaw_max` **0.5 рад**. Последний initialpose: READY, sigma 0.41, second_mode 0.53, scan_score **0.47** (`seed_loop2.log`) |
| Motion adapter | **P20 sha256 9c05bb34...** (PID 405674 с 21:56), `yaw_loop=True`, `yaw_u_min_ccw` 0.6, `yaw_u_min_cw` 0.45, `yaw_u_max` 1.0, `yaw_ki` 0.4; MC source `sdk_qazbot_nav2_1029a` prio 30, перерегистрирован через DELETE (code 0) + ADD. Откат: `-p yaw_loop:=false` или `x2_motion_adapter.py.bak_20261029_pre_p20` (71db8ebd) |
| Goal checker | xy 0.20 м, **yaw 0.35 рад** (`yaw_tol:=0.35`) |
| Costmaps | полигон footprint (P18, вендорский app2.yaml), global inflation `inflation_radius` **0.60**, `cost_scaling_factor` 1.5; local `inflation_radius` **0.50**; Smac2D `cost_travel_multiplier` 4.0; global `static_layer.footprint_clearing_enabled` True |
| BT / replanning | keep-path Fallback + RecoveryNode x4 с ClearEntireCostmap GLOBAL; failure_tolerance 10 с |
| RPP / smoother | без изменений с 10-28: linear 0.3 м/с, angular 0.6 рад/с |
| Прочее | nav_full, recorder (лёгкий mcap), raw_scan_bridge (A/B, не в контуре), vendor apps остановлены, HAL nice 0 |

#### Результаты вечера

| Время | Что | Результат |
| --- | --- | --- |
| 20:5x | демо A (-7.38,6.02)->(-5.20,2.82), старая карта, старый adapter | **SUCCEEDED** 9.2 с, ~3.9 м |
| 20:5x | демо B ->(-5.85,-2.27) | **SUCCEEDED** 11 с, ~4.9 м (2/2 подряд) |
| 20:3x | MC ушёл в DAMPING -> leg odom origin reset dp 1.563 м | native_odom_bridge залип, рестарт localization+Trust (блокер a) |
| 21:47-21:5x | переключение на карту D1 + исправление free_thresh | load_map 0; initialpose #2: READY, sigma 0.41, scan_score 0.47 |
| 21:56 | P20 deploy (DELETE+ADD, adapter 405674) | registered, ready после arm |
| 21:56 | P20 тест 1: CCW 0.5 | 0.563 рад/с устойчиво, 180 град; прерван по критерию drift 0.30 м (поворот на месте «шагает») |
| 21:59 | P20 тест 2 (MAX_DRIFT 0.5 м, оператор OK) | см. таблицу ниже, abort нет |

P20 live (`calib/p20_yaw_test_20261029_215906_summary.json`, от STAND_DEFAULT, источник `sdk_qazbot_nav2_1029a`):

| Шаг | Цель | Факт | u (MC) | Дрейф xy |
| --- | --- | --- | --- | --- |
| CCW 0.5 рад/с, 6 с | 0.5 | 0-1.5 с 0.385, 1.5-6 с **0.534 рад/с**, 170 град | 0.60-0.77 (ср. 0.63) | 0.23 м |
| CW 0.5 рад/с, 6 с | -0.5 | 0-1.5 с -0.398, 1.5-6 с **-0.354 рад/с**, -125 град | -0.45..-0.85 (ср. -0.58) | 0.19 м |
| 90 град CCW | 90 | до 5 град за **3.3 с**, после 2 с settle ошибка **+11.5 град** (обратный доворот), пик 1.36 рад/с | 0..0.68 | 0.16 м |
| 90 град CW | -90 | до 5 град за **4.1 с**, финал **-4.9 град**, пик 1.27 рад/с | -0.57..0 | 0.17 м |

Для сравнения без P20 (P19): CCW 0.5-0.6 после ходьбы 0.00-0.37 рад/с, в goals 96 % времени стоял. Финальное 11.5 град укладывается в yaw tolerance 0.35 рад (20 град), но запас мал.

#### ТЕКУЩИЕ БЛОКЕРЫ (вечер 2026-10-29; заменяет таблицу 21:45 ниже)

| # | Блокер | Статус / доказательства | Что дальше |
| --- | --- | --- | --- |
| a | native_odom_bridge залипает при каждой смене режима MC (stand <-> damping): origin leg odom сбрасывается (dp 0.467 м в 14:41, **1.563 м в 20:3x**) | не исправлено; каждый раз рестарт localization+Trust + новый initialpose | исправить bridge: авто-переякорение при скачке origin (re-anchor), затем live-тест stand->damp->stand |
| b | MC: in-place поворот нелинеен/залипает | **смягчено P20 (live)**: CCW 0.5 -> 0.534 рад/с, CW 0.5 -> 0.354 рад/с. Остаток: (1) обратный доворот CCW ~10 град после стопа (90 град CCW: финал +11.5 град); (2) CW недобор скорости (u до 0.85, 0.354 вместо 0.5); (3) дрейф 0.16-0.23 м за поворот; (4) после ходьбы (LOCOMOTION) P20 живьём не проверен, только офлайн | goals с финальным разворотом на карте D1; при необходимости поднять `yaw_u_min_cw`/ki, учесть coast/обратный доворот (ранний стоп по направлению) |
| c | Карта | **переключено на D1** (scan_score 0.47 против 0.35-0.40); высота среза скана 0.26-1.71 м против LD14 D1 ~0.2-0.3 м остаётся | проверить AMCL/Trust/costmap в движении на D1; решение по low slice / raw_scan_bridge |
| d | Smac2D игнорирует heading (путь позади робота) | без изменений | SmacHybrid/Lattice или очистка global marks при accept |
| e | RPP проецирует footprint прямо к carrot на кривом пути | смягчено inflation 0.5/0.6 | MPPI ([MPPI_MIGRATION.md](MPPI_MIGRATION.md)) |
| f | native odom twist невалиден (angular.z 1.78 рад/с стоя); дрейф позы стоя ~0.01 м/мин (0.17 м/15 мин) | P20 использует только pose | без изменений |
| g | HAL nice 0 (против -19 на .89) | без изменений | решение оператора (sudo) |
| h | **NEW: D1 map_saver пишет `free_thresh: 0.25`**: в trinary-режиме неизвестные клетки (pgm 205, p=(255-205)/255=0.196) < 0.25 -> загружаются как FREE (планирование через неизвестность, AMCL likelihood по пустоте) | исправлено в нашей копии (0.196, оригинал `map.yaml.orig_from_d1`) | при каждом новом экспорте карты с D1 проверять/править free_thresh (лучше <0.196, напр. 0.19); сообщить владельцу D1 |

Блокеры 2026-10-28 без изменений: 3, 5, 6 (adapter не делает DELETE при выходе), 7, 9.

#### Продолжение (шаги)

1. Проверить живые PID (adapter 9c05bb34 / PID 405674, nav_full, localization, Trust 397013+, recorder) и режим MC.
2. После любой смены режима MC: оператор ставит робота -> `start_loc.sh` -> `restart_gc.sh` -> initialpose в кадре **карты D1** -> `seed_loop`.
3. Goals на D1 с P20: серия 3 подряд вокруг стола с финальным разворотом CCW и CW; записывать финальную ошибку yaw и обратный доворот.
4. Исправление native_odom_bridge (re-anchor), затем решение по SmacHybrid/MPPI.
5. Сценарии для цифрового двойника: [DIGITAL_TWIN_FINDINGS.md](DIGITAL_TWIN_FINDINGS.md) §g.

---

### 2026-10-29 день (до ~21:45)

Обновлено 2026-10-29 ~21:45 CST (+08), X2 agi@192.168.1.72. Источники: `.orchestrator/PLAN.md` ("2026-10-29 session"), `DECISIONS.md`, отчёты P15/P16/P18/P19/P20 (копии в [evidence/nav_session_20261029](evidence/nav_session_20261029/reports/)), логи/скрипты в [evidence/nav_session_20261029](evidence/nav_session_20261029/) (manifest.json: sha256 + исходные пути; bags только перечислены). Ветка `nav/first-goals-20261028`, repo `ws/src` = deployed `ws/src` (sha256 сверены на 21:45, кроме кандидата P20; вечером P20 развёрнут и стал основным файлом, см. «вечер» выше). Секреты/пароли роботов в репозиторий не пишутся.

**Итог дня:** после перезагрузки робота (14:24) стек поднят заново (P15). Демо текущей навигации: 2 из 2 goals подряд SUCCEEDED (20:5x, A 9.2 с, B 11 с, конфиг ниже, старый adapter, карта v3 despeckled). Сегодня найдены и частично устранены причины отказов (footprint, BT, inflation); главный нерешённый блок: in-place разворот в вендорском MC нелинеен/залипает (P19), исправление P20 (yaw-rate loop) написано и проверено офлайн, но живьём НЕ проверено и НЕ развёрнуто.

### Текущее состояние (что работает, конфиг)

Проверять живые PID, а не этот список. Состояние на ~20:37-21:30, после последнего рестарта (идёт простой, goals не отправлялись 30+ мин).

| Компонент | Состояние |
| --- | --- |
| localization (FAST-LIO, level_bridge, native_odom_bridge, map_server, AMCL OmniMotionModel) + Trust | перезапущены 20:33-20:37 (`start_loc.sh`), карта `first_map_nav2_v3_despeckled` (НЕ новая D1), Trust T5 jump_xy_max **1.0 м**, jump_yaw_max **0.5 рад** (оператор 17:58) |
| Motion adapter, MC source `sdk_qazbot_nav2_1029a`, prio 30 | **старый код sha256 71db8ebd...** (pre-P19/P20), registered; PID 17061 (с 14:28). Новый yaw-loop НЕ запущен |
| nav_full (controller/planner/smoother/bt_navigator + Goal Coordinator) | перезапущен 20:34, ждёт/имеет map->odom; конфиг ниже |
| `seed_loop2` + monitor | следит за initialpose оператора, после каждого seed делает no-motion updates |
| recorder | лёгкий (mcap, без глобального /tf, IMU, joints), `record_goal.sh`; старый recorder PID 18033 (107 % CPU) заменён |
| `raw_scan_bridge` | работает параллельно на `/qazbot_raw/*` (не влияет на навигацию), 18 % ядра |
| Vendor apps | perception/orbbec_camera/hal_camera/drp/lx_webrtc_app остановлены (разрешение оператора 2026-10-28, повторено после перезагрузки) |
| HAL | em-запуск PID 2810, **nice 0** (см. блокер g) |

### Изменения 2026-10-29 (значения, причина)

| Изменение | Значения | Причина / результат |
| --- | --- | --- |
| Bring-up после перезагрузки (P15) | `start.sh`, `start_nav.sh` (источник `sdk_qazbot_nav2_1029a`), MC после reboot PASSIVE_DEFAULT: ADD только после ручного stand оператором (RC) | stale регистрации нет; ADD gated на `mc_source rc/STAND_DEFAULT` |
| Recorder облегчён | без глобального /tf, IMU, joints (`record_goal.sh`, прежний `.pre_light`) | прежний recorder 107 % CPU |
| restart_gc.sh | `ros2 topic echo` заменён rclpy `gc_status.py` | CLI нестабилен (см. 2026-10-28 блокер 7) |
| BT keep-path (P17) | `Fallback{ComputePathToPose; AlwaysSuccess}` (оставляет последний путь) + `RecoveryNode` x**4** с `ClearEntireCostmap` GLOBAL; добавлен `clear_costmap_service_bt_node` | GOAL2 17:1x: 1 Гц replan «no valid path» -> PipelineSequence abort -> goal failed |
| static_layer.footprint_clearing | global `static_layer.footprint_clearing_enabled: True` | GOAL3: «Starting point in lethal space» (устаревшая карта, статическое препятствие 0.35 м от центра при радиусе .454) |
| Полигон footprint (P18) | вендорский 12-точечный полигон из `/agibot/software/nav/config/vectorflux_common/a2/app2.yaml` -> base_footprint с запасом; в base_footprint: `[[0.17,0.229],[0.008,0.34],[-0.182,0.34],[-0.305,0.229],[-0.305,-0.261],[-0.182,-0.372],[0.008,-0.372],[0.17,-0.261]]`; local padding 0.01, global padding 0.01+cell margin; вместо круга r=0.454 | RPP 1.1.18 `isCollisionImminent` растеризует контур круга и давал collision на 72/72 курсах при 37 см свободного места; с полигоном 0/72 (`p18_fpcheck_*.txt`). Размах рук при ходьбе не измерен (+5 см спереди) |
| failure_tolerance | 10 с (с 28.10, сохранено) | время на replanning |
| Smac2D / global inflation (18:3x-18:4x) | `cost_travel_multiplier` **4.0**; global inflation `cost_scaling_factor` 3.0 -> **1.5**, `inflation_radius` **0.60** (было 0.889); local inflation_radius **0.50** (было 0.464) | around-table GOAL2: путь прижимался к столу/стульям (~1 м), RPP проецирует footprint прямо к carrot (0.5-0.9 м) и упирается в метки. Оператор: радиус 0.5-0.6. Persist в launch (bak `pre_infl`) |
| goal_checker.yaw_goal_tolerance | 0.17 -> 0.35 рад (временно 3.15 для диагностики, возвращено 0.35 в P19) | финальный разворот залипает в MC (блокер b) |
| Trust T5 | jump_xy_max 0.25 -> **1.0 м**, jump_yaw_max 0.25 -> **0.5 рад** (`start.sh`, `restart_trust.sh`) | оператор 17:58: разрешить коррекции AMCL до 1 м (карта устарела). Ослабление правила T5 по решению оператора |
| raw_scan_bridge A/B (P16) | сырое облако HAL + roll/pitch из leg_odometry + extrinsic x2.yaml на `/qazbot_raw/*`; оператор одобрил, влияния на живую nav нет | стоя: 18 % ядра против 44 % у FAST-LIO+level_bridge; медиана разницы дальности 2.8 см; map-match (0.1 м): raw .550 vs lio .519 (low slice .432); за 900 с стоя raw .337/.689 vs lio .324/.633, low .301/.539. A/B в движении не получился (робот не ехал в P16b); `ab_eval_motion.py` запускался 17:08 на 1200 с, результат `raw_scan_ab/ab_motion_1708.json`. Решение о переключении — после новой карты |
| Карта D1 | загружена `ws/src/maps/d1_qazbot_20261029` (slam_toolbox, 673x903 @ 0.05, origin -30.01/-22.41, pgm sha 83d05baf...) | карта office стара (ящики переставлены) -> низкий scan_score. **Не включена** (оператор 20:3x: показать текущую nav) |
| P20 yaw-rate loop | кандидат `ws/src/navigation/candidates/x2_motion_adapter_yawloop_p20.py` (sha256 9c05bb34...), **не развёрнут**; deployed `x2_motion_adapter.py` = 71db8ebd | см. блокер b |
| P19 turn-stall kick | отвергнут и откачен (`x2_motion_adapter.py.p19_kick_rejected` на X2) | данные не поддерживают |
| Процедура `seed_loop.py` | `scripts/seed_loop.py`: при КАЖДОМ `/qazbot_d1_map/initialpose` ждёт 0.3 с, делает 12 AMCL no-motion updates (сервис `request_nomotion_update`, 2 Гц; прерывается новым initialpose), через 2.5 с печатает метрики Trust (ready, motion_ok, sigma, second_mode, scan_score). Запускать после каждого рестарта localization/Trust (seed_loop2); resume GC/adapter делает Trust сам при READY | после рестарта localization нужен новый initialpose оператора (RViz) |

### Результаты goals 2026-10-29

| Время | Goal | Результат | Причина / комментарий |
| --- | --- | --- | --- |
| 17:1x | GOAL1 (~2.8 м) | **SUCCEEDED** 8.6 с | |
| 17:1x | GOAL2 (-5.02,3.49)->(-5.94,-0.81) | FAILED | replan «no valid path» -> abort; исправлено BT keep-path (P17) |
| 17:13 | GOAL3 (-4.54,0.11)->(-6.36,0.37) | FAILED | start in lethal space (static_layer); footprint_clearing |
| 17:16 | GOAL4 (-4.50,0.12)->(-6.39,0.21) | FAILED | RPP collision ahead 10 с, локальные lethal ~0.5 м, затем T1 pause + TF extrapolation, patience exceeded; робот зажат у препятствия |
| 18:0x | (-4.11,0.04)->(-6.42,0.22) | xy достигнут (0.02 м, 8 с), затем отмена | поворот на месте ~180 град работал (полигон), финальный yaw залип -2.16 -> progress checker -> GC `goal_alignment_not_converging` на 39 с |
| 18:1x | (-6.61,0.58)->(-6.42,0.23) | **SUCCEEDED** 8.5 с | включая T1 pause с авто-resume (P8c работает живьём) |
| 18:2x | around-table GOAL2 (-2.59,-3.45)->(-5.59,3.18) | FAILED | RPP collision ahead со старта (метки стола/стульев ~1 м), patience x4, T1 pause; после clear метки появились снова = реальные |
| 18:5x | around-table GOAL2 retry | xy достигнут (0.02 м, 38 с), затем отмена | 16 с стоял (collision ahead; первый путь шёл назад), 7.2 м пройдено, финальный разворот залип yaw 2.25 30 с -> progress checker + GC alignment cancel |
| 20:5x | demo A (-7.38,6.02)->(-5.20,2.82) | **SUCCEEDED** 9.2 с | текущая конфигурация, старый adapter |
| 20:5x | demo B ->(-5.85,-2.27) | **SUCCEEDED** 11 с | 2/2 подряд |

Итого сегодня: 5 SUCCEEDED (из них 2 подряд на демо), 2 по xy с отменой из-за финального разворота, остальные FAILED (причины выше).

### БЛОКЕРЫ на 21:45 2026-10-29 (заменены вечерней таблицей выше; заменяли таблицу ниже, пункты 2026-10-28 остаются в истории)

| # | Блокер | Доказательства | Решение / нужно решение |
| --- | --- | --- | --- |
| a | **native_odom_bridge залипает при каждой смене режима MC** (stand/damping): origin leg odom сбрасывается (dp .467 м, gap 39.7 с в 14:41; dp 1.563 м в 20:3x) -> нужен рестарт localization+Trust | PLAN 14:45, 20:3x | Процедура: оператор поднимает робота -> `start_loc.sh` -> рестарт nav_full -> initialpose -> goals. Код: bridge должен пережить reset origin (не сделано) |
| b | **MC: in-place разворот CCW нелинеен/залипает**, особенно после ходьбы; от adapter/Nav2 не зависит (прямой MC такой же или хуже) | P19: CCW с места 0.6 -> 0.35-0.53 рад/с, 0.8 -> 1.25, 1.0 -> 1.87 (перерегулирование 1.6-1.9x); CW 0.4 -> -0.27, 0.6 -> -0.54; после ходьбы 0.6 -> 0.2-0.37 рад/с, stall 23-38 %; в goals CCW 0.6 после ходьбы 0.00 рад/с 96 % времени. Прямой MC (`sdk_qazbot_calib_direct`): CCW <=0.5 нет устойчивого поворота. Таблицы: `reports/20261029-p19-rotation.md`, `calib/*.json`. Pelvis/chest расхождение <5 град | Кандидат **P20 yaw-rate loop** в adapter (замкнутый контур по yaw из native odom pose, не twist; u 0.6/0.45..1.0, I-term ki 0.4, overspeed cut); офлайн replay: 0.50-0.62 рад/с против 0.10-0.40, 90 град за 3-4.4 с, перерегулирование 0-8 град (26 град после stall). **Живой тест не выполнен** (MC был в DAMPING 19:35-19:44). Шаги: робот стоит -> `CONFIRM=1 p20_restart_adapter.sh` -> reseed -> `CONFIRM=1 p20_yaw_test.py`. Откат: `-p yaw_loop:=false` или `.bak_20261029_pre_p20` |
| c | **Старая карта устарела; новая D1 скачана, но не переключена**; высота среза скана 0.26-1.71 м, а D1 LD14 строил карту на ~0.2-0.3 м | оператор: ящики переставлены; P16: низкий слой map-match .432 vs .550 | Решение оператора: переключить карту (`ws/src/maps/d1_qazbot_20261029`), тогда AMCL/Trust/costmap перепроверить, и пересмотреть slice (low slice или raw_scan_bridge). Потребует новый initialpose и тюнинг T5 |
| d | Планировщик может выбрать путь **позади робота** (Smac2D игнорирует heading) | goal 18:5x: путь 136 град назад, 10 м, 16 с стоял (CCW stall); после global clear путь вперёд 7 м | Опции: SmacHybrid / SmacLattice (heading, штраф за разворот); очищать global marks при accept goal. Больше изменение, решение нужно |
| e | **RPP проецирует footprint прямо к carrot**, а путь криволинейный -> ложные collision ahead у стола | P18/18:3x: collision со старта, метки ~1 м | Смягчено inflation/Smac cost. Опция: **MPPI** (см. [MPPI_MIGRATION.md](MPPI_MIGRATION.md), предложение, не реализовано) |
| f | **Native odom twist невалиден** (angular.z 1.78 рад/с, linear.y 0.30 стоя) | P20 live-замер | Использовать только pose; P20 так и делает. Дрейф позы стоя .17 м/15 мин |
| g | **HAL nice 0** (em-конфиг без приоритета; PID 2810 после reboot) против nice -19 на .89 | PLAN 14:24; ранее P8d | Нужен `sudo renice`/конфиг em; решение оператора (отложено). Ранее ~5-10 % T1_scan_stale |

Блокеры 2026-10-28 без изменений: 3 (метки вне FOV, смягчено BT global clear), 5, 6 (adapter не делает DELETE при выходе: для рестарта нужен ручной DELETE), 7, 9 (RC takeover latch не доказан).

### Продолжение (шаги)

1. Проверить живые PID/состояние (adapter 71db8ebd, nav_full, localization, Trust, recorder, raw_scan_bridge); MC режим и источник.
2. Если оператор перезапускал/опускал робота: оператор ставит робота на ноги (RC) -> `start_loc.sh` (localization+Trust) -> рестарт nav_full (`restart_gc.sh`) -> оператор initialpose -> `seed_loop` -> goals.
3. По разрешению оператора: P20 live (restart adapter DELETE+ADD, `p20_yaw_test.py` 0.5 рад/с и 90 град), затем goals с финальным разворотом. При успехе: `cp candidates/x2_motion_adapter_yawloop_p20.py` -> `navigation/x2_motion_adapter.py`, перепроверить sha256, закоммитить.
4. Решение по карте D1 + slice скана + raw_scan_bridge (A/B в движении `ab_eval_motion.py`).
5. Решение по планировщику (SmacHybrid/Lattice) и MPPI (см. MPPI_MIGRATION.md).
6. Не запускать unit-suites (правило оператора 28.10); не трогать работающие ROS-процессы без разрешения.

---

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
