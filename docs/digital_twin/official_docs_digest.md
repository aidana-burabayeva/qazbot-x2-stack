# Официальная документация AimDK X2 → требования к цифровому двойнику

Источники (все локальные, получены от оператора 2026-10-23): `aimdk-sdk-v1_1_0-aimdk-aarch64-faf4c945-artifacts` (docs `html/en/dev`), `mc-x86-v1.0.0-20260522`, `sim_mujoco-x86-v1.0.0-20260522`, `aimdk-extra-x2_rl_deploy-v1.1`. Онлайн: https://x2-aimdk.agibot.com (README SDK: «up-to-date online docs are always preferred»; онлайн-сайт этой сессией не открывался — только локальная копия).
Полный список всех 167 блоков Warning / Caution / Attention / Important / Note: `official_admonitions.md` (автоизвлечение, текст как в источнике). Заголовки «Best practices» в доках: только раздел 2.1.1 (запуск на подвесе, рекомендуется для SDK) и 4.2.2 (контейнерная разработка, рекомендуется для X2 Ultra); практики по движению — в блоках «Note: Best Practices» на страницах locomotion и modeswitch.

## 1. Главное для архитектуры

Официальная глава «X2 MuJoCo Motion-Control Simulation» (`sim_contents`, `sim_index`): *«drive the MuJoCo simulation with the official motion-control module mc (the same one used on the real robot) and validate your own high-level control logic on an x86 host»*. Схема: `SDK → mc → sim_mujoco` по `/aima/hal/joint/*/command|state`, IMU. Запуск: `start_sim.sh -s` (MuJoCo), `em_run.sh` (mc), проверка `ros2 run py_examples get_mc_action`, стойка `set_mc_action SD`, ходьба `set_mc_action LD` + `mc_locomotion_velocity`. Модель робота в симе — `lx2501_3_t2d5`, то же семейство, что прошивка на `.66` (`release-lx2501_3_t2d5-soc1-v1.1.5`).

Вывод: **настоящий `mc` заменяет самописный Virtual MC** как источник поведения MC (арбитраж, таймауты, стойка, походка, пороги). Мой `tools/twin/virtual_mc.py` остаётся только как лёгкий резерв без MuJoCo (быстрые регрессии), и должен быть честно помечен как неполный (см. §3).

## 2. Нормативные ограничения (обязательны для twin)

| # | Требование (страница) | Следствие |
|---|---|---|
| 1 | Locomotion — **открытый контур**, «no trajectory or endpoint accuracy», фактическая скорость отклоняется от команды (`locomotion`, Attention) | Перелёт/выбег — свойство MC; симулятор покажет только то, что даёт sim-MC. Точные цели — не задача этого интерфейса |
| 2 | Команда **обнуляется через 200 мс**; при ходьбе без команд 2000 мс → стабильная стойка (`locomotion`) | Адаптер обязан слать непрерывно (рекомендовано 50 Гц); тест «пропали команды» |
| 3 | Нулевые команды **не сбрасывают таймер источника**, таймаут считается по последней **ненулевой** (`MC_control`, Arbitration) | Модель MC и тесты must-have |
| 4 | Источник регистрируется через `SetMcInputSource` (ADD: name, priority, timeout; ENABLE отдельно), имя `sdk_*`, приоритет 20–39 по умолчанию; незарегистрированные/выключенные — отбрасываются (`MC_control`) | Уже так в адаптере; MC-часть должна быть официальной |
| 5 | Приоритеты: rc 80, app_proxy 60, interaction 50, pnc 40, vr 70, linkswarm 90 (эксклюзив). **Не использовать системные источники** (Warning) | Twin должен уметь тест «rc перехватывает» через настоящий mc |
| 6 | Локомоция только после перехода в Stable Stand; режимы — `SetMcAction`, защищены safety protection (SecureLevelForbid) (`modeswitch`) | Запускать twin: стойка → LD → скорости |
| 7 | Пороги страгивания: forward ~0.1 м/с, lateral 0.5–0.7, **yaw ~0.03–0.05 рад/с**, «learned by base model, not constants … always refer to real device testing» (`locomotion`, Important) | Док-значения ≠ робот (измерено 0.6 рад/с в S3b). Пороги в twin — только из измерений, не из docs |
| 8 | Границы скоростей по SpeedMode: LOW fwd −1.0…+0.8, ang ±1.0 (по умолчанию) и т.д.; читаются из `speed_status` в `/aima/mc/common/state` | Адаптер читает bounds из state — оставить |
| 9 | «Test locomotion control code in a simulation environment first»; «Ensure velocity commands are smooth and continuous»; мониторинг состояния, safety checks (`locomotion`, Note: Best Practices) | Обоснование twin; тест на гладкость команд |
| 10 | **Не запускать >2 ROS-узлов/с** в Stable Standing/Locomotion — перегрузка DDS discovery, потеря баланса (`warnings`, `MC_control`, `modeswitch`) | Уже есть `launch_stagger.py`; в SIM — соблюдать тоже (воспроизводимость), в REAL — обязательно |
| 11 | Cross-host service ненадёжен: нужны exception safety и retransmission (`MC_control`, Caution) | Тесты потери ответа регистрации; не «ретраи ради маскировки» — по документированному паттерну |
| 12 | **PC1 (10.0.1.40, Motion Control) нельзя использовать для сборки/запуска вторичной разработки**; не менять DDS-конфиги, параметры встроенных алгоритмов (`onboard_computer`, `warnings`) | Нельзя катать tools/twin/эксперименты на PC1; DDS-профиль робота не править |
| 13 | IMU: `header.stamp` = время приёма, не время измерения; ковариации нулевые, не использовать; frame_id `base_link` у chest и torso одинаково; потеря IMU = робот не ходит (`hal/sensor`, Note) | FAST-LIO по IMU: учесть задержку; sim IMU ≠ реальный по этим полям |
| 14 | Factory calibration (extrinsics) — **не изменять**, привязана к конкретному роботу (`coordinate_system`, Warning) | Extrinsics для twin брать только чтением с `.66` |
| 15 | Сырые видеопотоки не подписывать между вычислительными блоками (Attention, `hal/sensor`) | Для сбора данных лидара/камеры — запись на Orin локально |
| 16 | Sim/RL примеры «validated only in simulation»; KP/KD — симуляционные рецепты (`sim_rl`) | Результаты sim не доказывают поведение реального MC |
| 17 | Navigation/SLAM/Perception — опциональные модули по запросу в поддержку; задачи могут прерываться при отказе (`perception/*`) | Собственный Nav2 — отдельный стек; встроенный pnc приоритет 40 может конкурировать за MC |

## 3. Расхождения текущего `tools/twin/virtual_mc.py` с официальным контрактом

Реализовано: ADD/ENABLE с проверкой `sdk_`+20..39; приём velocity только от зарегистрированного источника; `McCommonState` 10 Гц; отдача vx/wz в модель.
Не реализовано (есть у официального `mc`): `GetCurrentInputSource`; таймаут источника 1000 мс по последней ненулевой команде; нулевые команды не сбрасывают таймер; арбитраж по приоритетам и перехват `rc`; возврат в стойку через 2000 мс; `SetMcAction`/режимы/safety protection; реальные границы speed mode; походка. Мои пороги/лаг/0.2 с остаются в `LocSim` (`tests/controller_dry/sim_robot.py`) как приближения.
Следствие: Virtual MC нельзя использовать как доказательство соответствия MC; только резерв.

## 4. Что официальный sim НЕ даёт

В `simulator/default.yaml` публикуются: `/aima/hal/imu/torso/state`, `/aima/hal/imu/chest/state` (500 Гц), `/aima/hal/joint/{arm,leg,waist,head}/state` (1000 Гц), `/aima/hal/odom/state` (500 Гц). **Нет LiDAR (PointCloud2), нет камер.** В MJCF есть меш `lidar_link`, но публикации облака нет. Следствия: FAST-LIO в этом sim не проверяется (Mode C требует собственного лидарного плагина MuJoCo — отдельная работа); 2D-скан для AMCL по-прежнему строится ray-cast по карте от истинной позы sim. Что именно публикует `/aima/hal/odom/state` (истина или оценка) — не проверено, требует чтения `mujoco_plugin`/запуска.

## 5. Риски изоляции при следовании официальной инструкции

Официальный `run_docker.sh` использует `--network host`, а `mc`/sim читаются по DDS в домене по умолчанию. На одной сети с роботом (`192.168.1.66`) это значит: DDS-discovery может связать sim/адаптер с реальным MC. Для twin обязательно: контейнер без внешней сети (`--network none`, loopback), `ROS_DOMAIN_ID=87`, `ROS_LOCALHOST_ONLY=1`; GUI MuJoCo — через сокет X11 (работает без сети). Также официально требуется `/agibot/data/var` (создаётся `sudo`) — в контейнере примонтировать каталог пользователя вместо создания `/agibot` на хосте. Состав `mc` (в т.ч. плагины net/grpc/mqtt в `lib/`) не аудирован на предмет сетевых портов — проверить до первого запуска.

## 6. Открытые вопросы (нужны данные/проверки)

1. Версия MC на роботе — **v1.1.0-r9-rc9** (mc_param v1.1.0-r9-rc3; снимок `vendor/pc1_mc/`, см. `20261023-twin-handoff.md`), в sim — `mc-x86-v1.0.0-20260522`: версии различаются; параметры локомоции идентичны (`20261023-twin-mcparam-diff.md`), код раннеров не сверен.
2. Совпадает ли MuJoCo-модель/параметры sim с реальной динамикой (масса, трение, задержки) — не гарантировано; нужна валидация на отдельных данных.
3. `/aima/hal/odom/state` — истина или оценка.
