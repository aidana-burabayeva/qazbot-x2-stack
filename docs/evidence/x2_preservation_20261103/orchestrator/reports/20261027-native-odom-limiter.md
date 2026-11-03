# Ограничитель остановки по base_link из штатной одометрии (native_base) и состояние после перезагрузки

Дата: 2026-10-27. Хост `agi` (X2, Jetson), uptime ~1 ч 08 мин на начало работы. wifi0 = 192.168.1.72 (ранее .66), develop0 = 10.0.1.41.
Робот не двигался. Источники в MC не регистрировались, arm не выполнялся, ненулевых скоростей не было, режим/action/баланс MC не менялись.
Использовались только подписки, метаданные графа, офлайн-чтение bag и изолированные тесты (домены 87/88, localhost, без MC).

## 1. Пассивное состояние после перезагрузки

Снимок: подписка на `/aima/mc/common/state`, `/aima/mc/leg_odometry`, `/tf`, `/tf_static` и метаданные графа (скрипт в scratchpad,
по образцу `tests/nav_e2e/passive_hardware.py`, с `env.sh` и `aimdk_env.sh`). Сервисы не вызывались.

| Что | Значение |
|---|---|
| MC action_desc / status | `STAND_DEFAULT` / 100 (81 сообщение за 8 с, одно и то же состояние) |
| body_status | 1 (STAND) |
| fsm_state | 4 (STABLE по актуальной zh-таблице, см. отчёт 20261023) |
| input_source | `rc`, priority 80. SDK-источников qazbot нет |
| Издатели `/aima/mc/locomotion/velocity` | 4 вендорских: aimrt_planning, app_proxy, soc0_rc, soc2_interaction. Наших нет |
| `/cmd_vel` | 1 вендорский controller_server_ros_node |
| Узлы qazbot | нет (только сам зонд) |
| `/aima/mc/leg_odometry` | 168 Гц, frame `leg_odom`, child `lidar_imu_chest_front`, возраст штампа p50 6 мс, max 39 мс. После перезагрузки начало обнулено (x≈0,20, y≈−1,01) |
| Вендорский `/tf` | 31 динамическая связь, около 46 Гц. Издатели mc_ros2_node, aimrt_perception_node, camera (RELIABLE/VOLATILE) |
| `/tf_static` | 22 связи, есть `base_link->pelvis` и `lidar_chest_front->lidar_imu_chest_front` |
| Часы | ptp4l/phc2sys в s2, offset сотни нс, без `jumped`/`SYNCHRONIZATION`. Лидар 10 Гц, age p50 0,103 с; IMU age max 9 мс; n_future 0 |

Свежесть цепочки TF на 20-Гц тиках за 30 с в покое: torso_link (best-effort) p50 14 мс, max 32 мс; reliable max 35 мс; leg odom max 39 мс.
Ни одного тика старше 0,15 с. В первом 6-секундном зонде были одиночные задержки TF до 0,34 с. Вероятная причина: начальная доставка
после discovery, но на долгом окне это не воспроизвелось. Зонд стоит повторить под нагрузкой (FAST-LIO + запись).

### Состояние стека

Не запущены: локализация (FAST-LIO, level_bridge, map_server, AMCL), trust, адаптер, Nav2/goal_coordinator. Проверено через `pgrep` по шаблону из quickstart.

Документированный запуск в dry-run: `docs/X2_QUICKSTART_BRINGUP_RU.md`, разделы 0–4.
1. Предпроверка (раздел 0): робот стоит; `pgrep` пуст; PTP в s2; `tools/stack/age_probe.py`. Сегодня всё в норме.
2. `bash ~/control_backend/tools/stack/stack_up.sh [TAG]`. Запускает по порядку, каждый этап ждёт свою строку в журнале:
   - localization (`run_localization.sh`: FAST-LIO portable на CPU 5,6, карта `first_map_nav2_v3`, seed выключен);
   - trust (`localization_trust_node.py`, sha карты v3, политика `QAZBOT_TRUST_POLICY`, по умолчанию strict; в прошлой сессии использовалась go2);
   - adapter (`run_x2_driver.sh -p live:=false`; ждёт `live=False`; при `live=True` запуск прерывается);
   - nav_full (`run_nav_full.sh allow_dry_run_driver:=true`).
3. `bash tools/stack/check.sh 30`: проверка здоровья только на чтение.
4. RViz на ПК, 2D Pose Estimate, затем nomotion/baseline/confirm_anchor (RVIZ_LOCAL.md, шаг 3).
5. Остановка: `tools/stack/stack_down.sh [TAG]`.

**Стек не запускал, хотя скрипт допустим** (адаптер dry-run, сервиса register нет, arm скрипт не вызывает). Причина в блокере ниже:
без него оператор не поставит позу из RViz, а после правки DDS-профиля процессы всё равно придётся перезапускать.

**Блокер (IP).** `ws/src/navigation/dds_qazbot.xml` (его задаёт `env.sh` для всех процессов стека) содержит
`interfaceWhiteList` = 10.0.1.41 и **192.168.1.66**. Адреса .66 больше нет. Локально стек будет работать через SHM и 10.0.1.41,
мои зонды с этим профилем вендорские топики видели. Но через wifi 192.168.1.72 участник не слушает, и RViz на ПК
(`tools/rviz/dds_pc_x2.xml`, `ws/src/navigation/rviz/dds_pc_x2.xml` тоже содержат .66) стек не обнаружит.
Нужно решение оператора: заменить .66 на .72 в трёх XML (с бэкапом) или вернуть .66 через резервацию DHCP.
Адрес .66 также зашит в `tests/motion_adapter/real_register_observe.py` и в документации (`ssh agi@192.168.1.66`).

## 2. Реализация: `--limit-source native_base` в tests/s3/step_cmd.py

Бэкап: `tests/s3/step_cmd.py.bak_20261027_pre_nativebase` (215 изменённых строк в diff).

Поведение по умолчанию не меняется (`--limit-source lio`): тот же LIO-лимит и проверка `odom_stale`, тот же CSV-заголовок и summary,
новых файлов нет. Это проверено тестом и модельным прогоном (раздел 3).

### Новые опции

- `--limit-source {lio,native_base}`, по умолчанию `lio`.
- `--native-odom-topic`, по умолчанию `/aima/mc/leg_odometry`.
- `--native-max-age`, по умолчанию 0,2 с, допустимо 0,05..0,5.
- `native_base` требует хотя бы один из `--stop-after-yaw-deg` / `--stop-after-dist`.

### Метод (класс `NativeBase`)

- `T_odom_base = T_odom_imu · T_base_imu⁻¹`. Здесь T_odom_imu = поза из leg_odometry (child `lidar_imu_chest_front`),
  T_base_imu = `tf2 BufferCore.lookup_transform_core('base_link', 'lidar_imu_chest_front', stamp)` по **штампу источника** одометрии.
  Цепочка: base_link→pelvis (static) → waist_yaw → waist_pitch → torso (динамический /tf) → lidar_chest_front → lidar_imu_chest_front (static).
  Это тот же способ, что в офлайн-аудите trial06.
- Подписки только в режиме native: leg odom (sensor-data QoS); `/tf` (BEST_EFFORT, в буфер идут только три динамические связи цепочки);
  `/tf_static` (RELIABLE + TRANSIENT_LOCAL).
- Без экстраполяции. Сэмпл одометрии новее последнего общего штампа TF ждёт в очереди (deque 400 ≈ 2,4 с).
  Вычисляется самый новый покрытый сэмпл, вызов идёт с 20-Гц такта инструмента.
- Yaw = атан проекции оси x base_link на плоскость leg_odom, разворачивается непрерывно. Дистанция = планарное смещение начала base_link
  от позы на первом шаге. Начальная поза снимается на том же такте, где `fault()` уже прошёл.
- Отказы, по штампам относительно `time.time()`; часы PTP-синхронизированы:
  - `native_odom_stale`: возраст одометрии больше max_age;
  - `native_tf_stale`: возраст самого старого из трёх «последних» штампов цепочки больше max_age;
  - `native_stamp_future`: штамп в будущем больше чем на max_age (скачок часов);
  - `native_base_stale[:ошибка tf2]`: нет оценки base или она старше max_age.

  Отказ проверяется в `fault()` во всех фазах (preconditions, arm, step, pause). Он идёт по штатному пути abort: нули `--pause` с,
  затем `motion_adapter/stop`, адаптер остаётся защёлкнутым, exit 2.
- LIO в режиме native только диагностика: подписка и CSV-колонки `dist_m`/`dyaw_deg` сохраняются; возраст LIO не прерывает запуск
  (trust по-прежнему его проверяет).
- Выход в режиме native:
  - CSV получает колонки `native_x, native_y, native_yaw_unwrapped, native_age_s, native_dist_m, native_dyaw_deg`;
  - `<csv>.native.csv` содержит все вычисленные сэмплы base и IMU;
  - summary получает `native{start, final_dist_m, final_dyaw_deg, evaluated, last_error, lio_final_*}` и `per_step_native`;
  - в `limit_hit` добавляется суффикс ` (native_base)`.

Пример (НЕ выполнялся; только по отдельному «go» оператора на каждое движение):
`python3 tests/s3/step_cmd.py --expect-domain 0 --expect-mode live --rot 0.5 --rot-dur 20 --stop-after-yaw-deg 90 --stop-after-dist 0.3 --limit-source native_base --csv ...`

### Новые файлы

- `tests/s3/extract_native_series.py`: офлайн-выгрузка из bag (sqlite `?mode=ro`) в компактную фикстуру.
- `tests/s3/data/trial06_native_series.json.gz` (280 КБ):
  - 3598 leg odom;
  - 3×1009 TF цепочки;
  - 22 static;
  - 196 LIO odom_level;
  - 187 cmd_vel_authorized.
- `tests/s3/test_native_limiter.py`: 17 тестов.

## 3. Тесты

### Новые: `tests/s3/test_native_limiter.py`, 17/17 OK

Запуск: `bash -c 'source ws/src/navigation/env.sh && python3 -m unittest -v tests/s3/test_native_limiter.py'`, около 37 с.

**NativeLimiterReplay (12 тестов).** `NativeBase` проигрывается на 20-Гц тиках во времени приёма recorder. Независимая эталонная серия:
`kinematic_series.json native_base` прежнего аудита.

| Проверка | Результат |
|---|---|
| Макс. планарное смещение base от старта (вся запись) | **0,0318 м** (аудит 0,0319). Порог 30 см не срабатывает. LIO в той же записи > 0,30 м |
| Итоговый поворот base | 67,42° (аудит 67,42°). Пик 70,5° < 90° (лимит trial06 не достигался бы) |
| Совпадение серии с аудитом | p95 \|Δyaw\| < 3° (рассогласование штампов ≤ 30 мс) |
| Время срабатывания по yaw (от первой команды) | 15°: 1,65 с (ожидание по аудиту 1,58 с, +0,07); 30°: 3,75 с (3,71, +0,04); 60°: 5,25 с (5,21, +0,05) |
| Ложные отказы свежести в окне команды (−1 с … +3 с после нуля) | 0 |
| Обрыв odom / обрыв TF / задержка TF 0,35 с через 1 с после старта | отказ через 0,20 с: `native_odom_stale` / `native_tf_stale` / `native_tf_stale` |
| Нет данных; штамп в будущем | `native_odom_stale`; `native_stamp_future` |
| Композиция (синтетика: поворот на месте, IMU вынесена на 0,3 м) | base неподвижен до 1e-5, yaw точный |
| `parse()`: умолчания, отказ native без лимита, отказ max_age=2 | OK |

**ToolReplay (5 тестов).** Реальный цикл `step_cmd.Tool.run()` в реальном времени. Узел rclpy работает в изолированном домене 88 (localhost).
Подменены только `spin()` (подача записанных сообщений через собственные колбэки Tool со сдвигом штампов к стене), статусы
adapter/trust/coordinator, запросы графа и Trigger-клиенты.

| Кейс | Результат |
|---|---|
| native, `--stop-after-dist 0.3`, шаг на всю записанную команду + 3,5 с выбега | exit 0 (не сработал). native max < 0,06 м, final 0,0058 м / 67,4°. LIO в CSV 1,09 м. В конце stop+resume |
| native, `--stop-after-yaw-deg 30` | exit 3, `yaw 30.2 deg >= 30.0 (native_base)`. Последний ненулевой тик 3,69 с после записанной команды (ожидание 3,71). Далее нули ≥ 1 с |
| native, обрыв leg odom через 1 с | exit 2 `native_odom_stale`. Последний ненулевой тик через ≤ 0,3 с после обрыва, затем нули ≥ 1 с, `motion_adapter/stop` без resume |
| native, обрыв TF через 1 с | exit 2 `native_tf_stale`, то же поведение |
| по умолчанию (LIO), `--stop-after-dist 0.3` | exit 3 `dist 0.301 m` через 5,82 с после записанной команды (в trial06 было 5,866 с, 0,306 м). CSV без native-колонок, без native.csv и native в summary |

### Существующие наборы

- Адаптер, `tests/motion_adapter`: test_adapter_core, test_dry_run_shadow, test_early_takeover, test_new_faults. **43/43 OK.**
  Три последних модуля импортируют `test_adapter_core` как соседний модуль, поэтому их надо запускать из `tests/motion_adapter`.
  Из корня ImportError; это не регрессия.
- S3, модель: `bash tests/s3/run_model_s3.sh s3_step_cmd_model_03_20261027` (домен 87, localhost; реальные адаптер dry-run, trust
  и coordinator). **35/38.** Не прошли `c1_adapter_zero_burst_then_silence_at_end`, `c2_adapter_silent_after_limit`, `c5_adapter_zero_then_silence`.
- **Базовая линия:** тот же харнесс с файлом `step_cmd.py.bak_20261027_pre_nativebase` (копия корня в scratchpad, символьные ссылки на ws/tools/nav_e2e)
  тоже дал **35/38 с теми же тремя отказами**. Значит, это не регрессия изменения. Вероятная причина: проверки окна харнесса,
  `t_off + 0.01` против задержки пересылки адаптером последнего ненулевого шага. Харнесс не трогал.
  Прежние прогоны 01/02 (20261021) останавливались на C1 из-за нагрузки; полный прогон C0–C7 получен впервые.

## 4. Ограничения и что не доказано

- Native-оценка совпадает с прежним аудитом и с наблюдением оператора (поворот на месте), но это не независимая ground truth.
  Leg odometry и TF идут от одного вендорского источника MC.
- Порог 0,2 с проверен на записи trial06 (в окне команды 0 ложных срабатываний) и на 30 с в покое (max 39 мс).
  Во время записи trial06 recorder под нагрузкой давал задержки приёма до 0,47 с за ~3 с до команды. Если такое случится в движении,
  ограничитель прервёт запуск (безопасная сторона), но возможны ложные abort. Перед первым живым применением нужен пассивный
  20-Гц зонд свежести под реальной нагрузкой (FAST-LIO + recorder).
- Время срабатывания ограничено 20-Гц тактом и задержкой TF: +0,04…0,07 с к моменту фактического пересечения. Выбег после нуля
  (в trial06 около −3° обратно при 0,5 рад/с) ограничитель не учитывает.
- Yaw считается проекцией в плоскость leg_odom. Наклон base в trial06 составлял 0,1–3,8°, ошибка от него мала.
- Нативный режим не прогонялся на живом графе (это потребовало бы запуска инструмента против адаптера). Проверены подписки и QoS:
  leg odom BEST_EFFORT/TRANSIENT_LOCAL ↔ sensor-data sub; /tf RELIABLE ↔ BEST_EFFORT sub; /tf_static RELIABLE+TL ↔ RELIABLE+TL sub.
  Совместимость QoS подтверждена пассивными зондами с тем же QoS.

## 5. Файлы

- Изменён: `/home/agi/control_backend/tests/s3/step_cmd.py`. Бэкап `step_cmd.py.bak_20261027_pre_nativebase`.
- Новые:
  - `/home/agi/control_backend/tests/s3/test_native_limiter.py`;
  - `/home/agi/control_backend/tests/s3/extract_native_series.py`;
  - `/home/agi/control_backend/tests/s3/data/trial06_native_series.json.gz`.
- Результаты модели: `/home/agi/control_backend/tests/integration_results/s3_step_cmd_model_03_20261027/result.json`.
- Скрипты пассивных зондов (не в репозитории):
  - `/tmp/claude-1001/-home-agi/064b986d-c606-4601-b9e1-dc10e302f794/scratchpad/mc_snapshot.py`;
  - `/tmp/claude-1001/-home-agi/064b986d-c606-4601-b9e1-dc10e302f794/scratchpad/tf_gap.py`.

## 6. Следующие шаги (каждый по отдельному решению оператора)

1. Решить вопрос IP: обновить .66→.72 в `dds_qazbot.xml` и обоих `dds_pc_x2.xml` (с бэкапом) или вернуть .66.
2. `stack_up.sh` в dry-run, затем `check.sh 30`, затем поза в RViz.
3. Пассивный зонд свежести leg odom и TF-цепочки на 20 Гц под рабочей нагрузкой (стек + recorder), 60 с.
4. Только по «go»: повтор поворота +0,5 рад/с с `--limit-source native_base --stop-after-yaw-deg <N> --stop-after-dist 0.3` и полной записью.
   Сверить `<csv>.native.csv` с офлайн-аудитом.

## 7. Исправление D1 по замечанию верификатора (20261027-native-odom-limiter-verify.md)

Робота по-прежнему не трогал: ни движения, ни регистрации, ни arm. Бэкап перед правкой: `tests/s3/step_cmd.py.bak_20261027_pre_nonfinite`, в diff 44 строки.

Суть дефекта D1: один нечисловой сэмпл leg odometry навсегда делал развёрнутый yaw равным NaN. Сравнение `abs(nan) >= limit` всегда ложно,
поэтому лимит не срабатывал, а `fault()` возвращал `''`, то есть ограничитель молча отключался.

### Что изменено в step_cmd.py

- `finite_pose(stamp, t7)`: штамп, позиция и кватернион должны быть конечными, а норма кватерниона в пределах 1 ± 0,01.
  В trial06 нормы 0,99999999…1,0000000000000002.
- `NativeBase.non_finite` — **защёлка** на первый отвергнутый вход, сама она не сбрасывается. Отвергаются:
  - `add_odom`: нечисловая или нулевая/ненормированная поза или кватернион, плюс нечисловой штамп. Сэмпл в очередь не попадает, `odom_stamp` не обновляется;
  - `add_tf`: то же для звена цепочки. Звено не передаётся в tf2, и **штамп звена в `tf_edge` не продвигается**;
  - `add_static`: то же для статических звеньев цепочки (pelvis, lidar_chest_front, lidar_imu_chest_front). Нечисловые звенья вне цепочки молча игнорируются;
  - `update`: нечисловой результат композиции. Поза не обновляется, защёлка срабатывает.
- `fault(now, start=None)` первым делом возвращает `native_non_finite:<odom|tf parent->child|tf_static …|pose>`.
  Если нечисловая стартовая поза, возвращает `native_non_finite:start`; если нечисловая текущая поза, `native_non_finite:pose`.
  `Tool.fault()` передаёт `self.native_start`. Прерывание идёт штатным путём: нули `--pause`, затем `motion_adapter/stop`, exit 2.
  Работает во всех фазах, включая preconditions (там отказ с exit 4).
- В summary добавлено `native.non_finite`.
- LIO `_on_odom` (путь по умолчанию). Исправление тривиальное и безопасное: нечисловой сэмпл (x, y или yaw) **отбрасывается** до развёртки yaw,
  `odom`/`odom_at` при этом не обновляются. Если NaN идут постоянно, сработает прежний `odom_stale` через 0,5 с.
  Счётчик пишется в summary как `odom_non_finite_dropped`, только при ненулевом значении, так что для нормальных данных вывод не меняется.
- O1 (проверка `header.frame_id == leg_odom`) не вносил: это вне требования, а в bag все 3598 сэмплов имеют `leg_odom`. Остаётся как наблюдение.

### Новые тесты: test_native_limiter.py, всего **25/25 OK** (53 с)

| Тест | Результат |
|---|---|
| NaN qw в leg odom через 1 с после команды | `native_non_finite:odom` на ближайшем тике (≤ 0,05 с). Защёлка держится до конца записи |
| NaN x в leg odom | `native_non_finite:odom` |
| Нулевой кватернион в leg odom | `native_non_finite:odom` |
| NaN в TF (waist_yaw→waist_pitch) | `native_non_finite:tf waist_yaw_link->waist_pitch_link`. `tf_edge` не продвинулся. При replay то же |
| Нечисловая стартовая поза | `native_non_finite:start` |
| Tool.run(), arm-фаза: arm_motion висит 1 с, на вызове arm обрывается leg odom | exit 2 `native_odom_stale`, ни одной строки step, все строки нулевые, ≥ 19 end_zeros, последний вызов `motion_adapter/stop` |
| Tool.run(), NaN qw во время шага | exit 2 `native_non_finite:odom`. Последний ненулевой тик через ≤ 0,1 с после NaN (фактически на 1,001 с при NaN на 1,0 с), затем нули ≥ 1 с, stop |
| Tool.run(), по умолчанию LIO, один NaN в LIO | exit 3 `dist 0.301 m` (как без NaN), `odom_non_finite_dropped=1`, в CSV нет nan |
| Прежние 17 тестов | без изменений: native 30 см не срабатывает (0,0058 м / 67,4°); yaw 30° на 3,695 с; обрывы odom/TF дают stale; LIO по умолчанию 0,301 м на 5,82 с |

### Существующие наборы после правки

- `tests/motion_adapter`, 4 модуля из каталога: **43/43 OK.**
- Модель S3 `s3_step_cmd_model_04_20261027_nonfinite` (домен 87): **35/38**, те же три отказа существовали до изменения
  (c1/c2/c5 «нули, затем тишина» в харнессе). Совпадает с базовой линией бэкапа и с проверкой верификатора.

### Замечания верификатора, оставшиеся открытыми (не блокируют)

- O1: header.frame_id не проверяется.
- O2: сброс начала leg odometry при непрерывных штампах не детектируется.
- O3: живая проверка синхронизации штампов с time.time() под нагрузкой ещё не сделана.
- O4: исключение в колбэке, поднятое внутри spin_once, обходит путь «нули + stop». Паттерн старый; страхует cmd_timeout адаптера 0,15 с.
- O6: нагрузка колбэков 168 + 46 Гц на живом графе не проверялась.
- Отдельно стоит разобрать отказ модельной проверки c5 (+0,2 с) в харнессе или адаптере.
