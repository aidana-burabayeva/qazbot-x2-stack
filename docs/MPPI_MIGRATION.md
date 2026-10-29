# Предложение: миграция RPP -> MPPI (Nav2 Humble) для QazBot X2

**Статус: ПРЕДЛОЖЕНИЕ (2026-10-29). Ничего не реализовано, не развёрнуто и не проверено на роботе.** Решение о начале работ принимает оператор. Контекст: [HANDOFF.md](HANDOFF.md) раздел 2026-10-29, блокер (e); DECISIONS 2026-10-29 п.2.

## 1. Зачем

Сегодняшние отказы (HANDOFF 2026-10-29, goals 17:16, 18:2x, 18:5x):
- **Проекция RPP «прямо к carrot».** Regulated Pure Pursuit (Humble 1.1.18) проверяет столкновение, проецируя footprint по прямой на 0.5-0.9 м вперёд к carrot, хотя планируемый путь криволинеен. Метки стола/стульев на расстоянии ~1 м вдоль пути давали «collision ahead» со старта, хотя сам footprint свободен при любом yaw (P18: 72/72 курсов у круга, 0/72 у полигона). Смягчено inflation/Smac cost, но не устранено.
- **Тесные места.** Проход определяется шириной корпуса (~0.75 м), а не радиусом; RPP не умеет выбрать траекторию внутри прохода.
- **Скорость с учётом стоимости.** У RPP cost-regulated scaling выключен (`use_cost_regulated_linear_velocity_scaling False`); в MPPI замедление у препятствий получается из критиков.
- **Сэмплирование траекторий.** MPPI оценивает тысячи траекторий по costmap (footprint, а не точечная проекция), даёт плавный обход и устойчивость к локальным меткам.

Чего MPPI НЕ исправит: нелинейность in-place поворота MC (блокер b: стоит в вендорском MC, не в контроллере), planner-heading (d), залипание native_odom_bridge (a).

## 2. Доступность на Humble

- `nav2_mppi_controller` в Humble доступен начиная с бэкпорта (1.1.x); сегодняшний RPP = 1.1.18, значит ту же версию 1.1.18 нужно проверять.
- **Не проверено на X2:** в `runtime/qazbot_nav_deps/debs` есть `ros-humble-nav2-smac-planner_1.1.18`, пакета `nav2_mppi_controller` там нет. Первый шаг: поискать `ros-humble-nav2-mppi-controller_1.1.18` для arm64 (apt/зеркало), добавить deb в `qazbot_nav_deps`, зарегистрировать plugin в `controller_plugins`. Если версия 1.1.18 отсутствует, брать совместимую и проверять ABI.
- API Humble-версии отличается от Jazzy/Rolling (набор параметров, `motion_model`, критики): параметры ниже сверять с `nav2_mppi_controller/README` установленной версии.

## 3. Предлагаемые начальные параметры (черновик, подлежит калибровке)

Исходные данные X2: max linear `max_linear` (сегодня vmax, 0.3 м/с по сессии), max_angular сегодня 0.6, vy=0 в `velocity_smoother`, `allow_reversing False` (оператор: задом не ездить), AMCL использует `nav2_amcl::OmniMotionModel`. Робот гуманоидный, MC принимает (vx, vy, wz); в нашей nav vy зафиксирован 0.

| Параметр | Предложение | Комментарий |
| --- | --- | --- |
| controller_frequency | 20 Гц (сейчас controller_hz launch) | CPU, см. раздел 5 |
| motion_model | **DiffDrive** на старте (vy=0), затем опционально **Omni** | X2 физически омнидирекционален (AMCL Omni, MC принимает vy), но vy в nav отключён; Omni MPPI открыл бы боковой обход у препятствий, но требует разрешить vy в smoother/adapter и калибровки: отдельный этап |
| time_steps / model_dt | 40 / 0.05 (горизонт 2.0 с) | ориентир: 1.5-2 с при vx 0.3 даёт 0.45-0.6 м, как lookahead RPP |
| batch_size | 1000 на старте (штатно 2000) | экономия CPU |
| iteration_count | 1 | |
| temperature / gamma | 0.3 / 0.015 | штатные |
| vx_max / vx_min | 0.30 / **0.0** (без реверса) | как сегодня |
| vy_max | 0.0 (DiffDrive) | |
| wz_max | 0.6 как сейчас; 0.8-1.0 только вместе с P20 yaw-loop | нелинейность MC: ниже 0.6 CCW залипает, 0.8 даёт ~1.25 рад/с, 1.0 ~1.87 рад/с (P19). Ограничение wz снизу (минимальный ненулевой wz) MPPI не поддерживает штатно: нужен P20 в adapter |
| ax_max / az_max | согласовать с velocity_smoother (сегодня RPP max_angular_accel `rpp_angular_accel`) | иначе рывки на MC |
| vx_std / wz_std | 0.1 / 0.3 | |
| footprint | тот же полигон X2 (`X2_FOOTPRINT`, P18) | `consider_footprint: true` (если есть в версии) |
| transform_tolerance | 0.2 | как RPP |

**Критики (порядок и смысл):** `ConstraintCritic` (лимиты скоростей), `CostCritic` (costmap + footprint; `consider_footprint true`, `critical_cost` ~300, `cost_weight` ~3.8), `GoalCritic` и `GoalAngleCritic` (финальная поза; `threshold_to_consider` ~0.5 м, слабый вес на угол, пока MC не умеет повороты), `PathAlignCritic`, `PathFollowCritic`, `PathAngleCritic` (мягкий, чтобы не провоцировать большие развороты), `PreferForwardCritic` (не реверс). `TwistCritic` — по необходимости. Минимизировать вклад углового критика: любой тяжёлый `wz` упирается в MC.

## 4. Бюджет CPU на Jetson (риск)

- X2 (Orin, `nproc`=8): FAST-LIO (cpus 5,6), AMCL, Trust, Nav2, adapter, recorder, vendor HAL. Сегодня recorder занимал 107 % ядра и пришлось облегчать; raw_scan_bridge 18 % против 44 % у FAST-LIO.
- MPPI (batch 1000-2000 x 40 шагов x 20 Гц) может занять 0.5-1.5 ядра на CPU; в Humble нет GPU-версии. Риск: рост T1_scan_stale (уже 5-10 % из-за HAL nice 0) и задержки TF.
- Меры: batch 500-1000, 10-15 Гц, привязка контроллера к cpus (отдельно от 5,6), замер `top`/`schedstat` в простое и при goal; цель: суммарная добавка <= 0.7 ядра, без роста T1 событий. Если не укладывается, остаться на RPP.

## 5. План миграции и тестов (по этапам, каждый с разрешением оператора)

1. **Офлайн/пакет.** Найти/собрать `nav2_mppi_controller` 1.1.x arm64, положить в `qazbot_nav_deps`. Ничего не запускать на роботе.
2. **Параллельный launch.** Новый `nav_control_mppi.launch.py` и BT; RPP остаётся по умолчанию (`controller_plugins: ['FollowPath']` -> флаг `controller:=mppi`). Dry-run в отдельном namespace без MC (live=false), измерение CPU.
3. **Replay.** Прогон на записанных bags (`goal_bag_mcap_143120`, around-table goals) через controller_server в изоляции: сравнить cmd_vel и минимальные расстояния до lethal с RPP. Только чтение.
4. **Живой тест, робот стоит/оператор рядом:** a) пустое место, прямая 2 м; b) goal у стола (проблемный around-table GOAL2); c) тесный проход; критерии: 3 goals подряд, ни одного ложного collision, wz не выше лимита, нет T1 pause из-за CPU.
5. **Переключение по умолчанию** только после 3-х подряд успехов и согласия оператора; обновить HANDOFF/TEST_STATUS.

**Откат:** launch-флаг `controller:=rpp` (по умолчанию) и рестарт `nav_full` (`restart_gc.sh`); файлы RPP-конфига не удаляются; бэкапы `*.bak_*` как обычно. Adapter и MC-контур не меняются.

## 6. Открытые вопросы к оператору

- Нужен ли vy (Omni MPPI) для обхода, или остаёмся на DiffDrive?
- Допустимая добавка CPU и можно ли выделить cpus контроллеру?
- Решение по SmacHybrid/Lattice (блокер d) делать вместе с MPPI или раньше?
