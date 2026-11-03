# IP .66→.72, сухой стек nav72_20261027, проект native-odom для Nav2

Дата: 2026-10-27, хост `agi` (X2). Движения не было. Источники MC не регистрировались, arm не выполнялся, ненулевых скоростей не было,
режим и action MC не менялись. Адаптер запущен только с `live:=false`. Конфигурацию Nav2 и level_bridge не трогал.

## 1. IP 192.168.1.66 → 192.168.1.72

Перед правкой сделаны бэкапы `*.bak_20261027_pre_ip72`. Затем `sed s/192.168.1.66/192.168.1.72/`. Все три файла проходят `xmllint`.

| Файл | Что было | Что стало |
|---|---|---|
| `ws/src/navigation/dds_qazbot.xml` | interfaceWhiteList 10.0.1.41 + .66 | 10.0.1.41 + **.72** |
| `tools/rviz/dds_pc_x2.xml` | initialPeer .66 | initialPeer **.72** |
| `ws/src/navigation/rviz/dds_pc_x2.xml` | то же, файл идентичен предыдущему | initialPeer **.72** |

**Что скопировать на ПК.** Нужен файл `ws/src/navigation/rviz/dds_pc_x2.xml`, положить его в `rviz_live/dds_pc_x2.xml` на ПК.
Его читает `run_local_viewer.sh` из quickstart. Файл `tools/rviz/dds_pc_x2.xml` идентичен, его использует `tools/rviz/run_rviz.sh`.

**Внимание: адрес ПК в этом профиле.** В профиле прописано `interfaceWhiteList` = **192.168.1.64**. Это адрес ПК, его я не менял.
Сейчас SSH к роботу открыт с **192.168.1.70**. Если это ПК оператора, его адрес тоже сменился. Тогда RViz с этим профилем
не увидит стек, пока в профиле на ПК не будет `.70` или актуальный адрес ПК.

Оставшиеся живые ссылки на .66 (не менял, не бэкапы и не логи):
- `tests/motion_adapter/real_register_observe.py:9`: требует `'192.168.1.66' in profile`. Теперь этот скрипт откажется запускаться.
  Это скрипт register, поэтому отказ на безопасной стороне, но проверку надо обновить.
- `docs/X2_QUICKSTART_BRINGUP_RU.md:1,3,8` (`ssh agi@192.168.1.66`, «X2 .66»).
- `docs/X2_NAV_STACK_RUNBOOK.md:4,41,44`.
- `docs/RVIZ_LOCAL.md:7` («PC 192.168.1.64, X2 192.168.1.66»).
- `docs/digital_twin/official_docs_digest.md:46`, `docs/digital_twin/architecture_audit.md:65`, `docs/digital_twin/official_sim_results.md:3`.
  Это исторические описания.
- `tools/twin/make_robot66_model.py`: «66» только в имени файла.

## 2. Сухой стек (тег `nav72_20261027`), оставлен запущенным для 2D Pose Estimate

Предпроверка:
- `pgrep` пуст;
- PTP в s2, offset −148…−157 нс, нет `jumped`/`SYNCHRONIZATION`;
- age_probe: облако 10 Гц, age p50 0,104 с, n_future 0; IMU 200 Гц, p50 0,9 мс, max 61 мс.

Политика trust: **go2**. `stack_up.sh` читает `QAZBOT_TRUST_POLICY` (strict|go2), в quickstart этого нет. Запуск:
`QAZBOT_TRUST_POLICY=go2 bash tools/stack/stack_up.sh nav72_20261027`. Строка в журнале trust: `policy=go2`.

| Компонент | PID | Состояние |
|---|---|---|
| localization (FAST-LIO portable sha 63903a5c, CPU 5,6; level_bridge levelled; map_server; AMCL) | 86415 | active; map_server/amcl `active`; taskset 5,6 |
| trust (`localization_trust_node.py`, карта v3, sha f6032f75…, go2) | 86641 | started; до позы reasons `T1_tf_stale`, `not_ready:awaiting_initialpose_and_amcl` (норма) |
| adapter (`run_x2_driver.sh -p live:=false`) | 86794 | `live=False`; status: `state idle`, `live false`, `mode dry_run`, `registration dry_run`, `mc_source rc`, **`motion_armed false`**, **`published 0`**, `foreign_publishers []` |
| nav_full (`allow_dry_run_driver:=true`, ROBOT_RADIUS 0.454) | 92489 | controller_server и velocity_smoother `active`; planner_server `timeout` и bt_navigator `inactive`: ждут map→base_footprint (норма до позы). SmacPlanner2D загружен |

`check.sh 30`:
- `/aima/mc/locomotion/velocity`: только 4 вендорских издателя;
- `/cmd_vel`: 1 вендорский;
- `duplicate_node_names []`;
- `cmd_vel_authorized`: издатель coordinator, подписчик adapter;
- `/qazbot_lio/odometry`: 10 Гц, p50 37 мс;
- `odom->base_footprint`: 10 Гц, p50 40 мс, p99 53 мс;
- scan: 5 Гц, p50 46 мс;
- map→odom отсутствует (до позы).

Во втором прогоне check.sh у odometry p99 был 0,45 с, в первом 0,046 с. Вероятно, это всплеск CPU от самого check.sh: load average был 17.
`T1_odom_stale` под нагрузкой стоит перепроверить.

Остановка: `bash tools/stack/stack_down.sh nav72_20261027` (`logs/stack_last_tag` = nav72_20261027).

### Блокер, найденный при запуске: дерево `candidates/` повреждено частичным восстановлением

- **15:36–15:39** перезаписано всё `candidates/` (около 3800 файлов). Кроме него `docs/*`, `docs/evidence`, `docs/digital_twin`.
- **16:13–16:14** перезаписаны `tests/*` и `tools/twin`.
- **16:16:12–14** перезаписаны `ws/src/navigation/{localization_trust_node.py, loc_trust.py, goal_coordinator.py, nav_full.launch.py, nav_control.launch.py, run_trust_go2_shadow.sh, profiles/go2_admission.yaml, __pycache__}`.

Похоже на копирование извне. В `candidates/` при этом:
- у всех файлов снят бит x;
- 527 файлов стали пустыми (в nav2_smac_bounds в том числе `src/…/amcl_node.cpp`, plugin XML, ament index);
- часть файлов обрезана до 262144 байт: `install/…/libnav2_smac_planner*.so` («missing section headers»), тестовые бинарники;
- в `tests/integration_results` 944 пустых файла и 8 файлов ровно по 262144 байт.

В ws/src и tools пустых или обрезанных файлов не нашёл. py_compile стековых .py проходит. Источник копирования не установлен.

Из-за этого `stack_up` сначала упал:
1. `fastlio_mapping` без бита x. Содержимое совпадает с задокументированным sha `63903a5c…`. Сделал `chmod ug+x`, больше ничего не трогал.
2. `nav_full` тихо выходил: `env.sh` проверяет `share/ament_index/…/packages/nav2_smac_planner` под `set -e`, а этого файла нет.

Как починил (обратимо, с проверкой хешем):
1. Выполнил `cmake --install` из неповреждённого `build/nav2_smac_planner` во временный prefix. Получилась
   `libnav2_smac_planner_2d.so` с sha `56b34510…`, это **побайтно совпадает** с `library_sha256` из `qazbot_bounds_fix.json`.
   Тот же хеш проверяет `nav_full.launch.py`.
2. Plugin XML из cmake-install равны upstream. ament-маркеры (`packages`, `nav2_core__pluginlib__plugin`, `package_run_dependencies`)
   взял из `runtime/qazbot_nav_deps`, `package.xml` взял из src кандидата.
3. Повреждённый каталог перенесён без изменений в `candidates/nav2_smac_bounds/install_corrupt_restore_20261027/nav2_smac_planner`.
   Восстановленный положен в `candidates/nav2_smac_bounds/install/nav2_smac_planner`.
4. Этап nav_full запущен вручную командой из quickstart, шаг 4, с тем же тегом.

**Нужно решение оператора и оркестратора:** кто и откуда копировал файлы; нужно ли восстановить остальное в `candidates/`
(body_frame_v1, nav2_lifecycle_shutdown, fast_lio_portable/src, …) и `tests/integration_results` из исходной копии;
те ли версии trust/coordinator/launch лежат сейчас в `ws/src/navigation`, которые ожидались.

## 3. Проект: native-odom режим для Nav2 (ничего не изменено)

### 3.1 Что сейчас публикует level_bridge (`ws/src/lio/level_bridge.py`)

- Вход `/qazbot_lio/odometry` (FAST-LIO, 10 Гц).
- Уровень: `R_lv` из среднего ускорения LiDAR-IMU за 1 с при старте.
- **base_footprint по определению = проекция позиции `lidar_imu_chest_front` на пол.** Yaw = курс оси вперёд LiDAR
  (`LIDAR_FWD_IN_IMU`), спроецированной на пол. Это НЕ base_link.
- TF `odom→base_footprint` в `/qazbot_d1_map/tf`: штамп = штамп FAST-LIO, 10 Гц, только x, y, yaw.
- `/qazbot_lio/odom_level` (`Odometry` odom/base_footprint): то же плюс z. Twist считается разностью поз на базе 0,3 с
  в осях base_footprint. Ковариация FAST-LIO повёрнута в уровень, по yaw первый порядок.
- `/qazbot_lio/scan`: срез `cloud_registered` в кадре **base_footprint**. Поза LIO берётся по штампу облака, 5 Гц (scan_every 2).
- При откате штампа чистит кэш поз и базу twist.

### 3.2 Кто что потребляет

| Потребитель | Что | Кадры / параметры |
|---|---|---|
| AMCL (`localization.launch.py`) | scan `/qazbot_lio/scan`; TF odom→base_footprint на **штамп скана** | odom/base_footprint/map, `OmniMotionModel`, alpha и transform_tolerance по умолчанию (1,0 с, поэтому map→odom со штампом +1 с). Ковариацию одометрии не читает. Позу лазера относительно base (base_footprint→кадр скана) **кэширует один раз** на frame_id |
| local costmap | TF odom→base_footprint; scan в obstacle_layer | global_frame odom, robot_base_frame base_footprint, радиус 0,454 вокруг base_footprint, 5 Гц |
| global costmap / planner | TF map→base_footprint | |
| controller_server (RPP) | `odom_topic` = `/qazbot_lio/odom_level` (twist: скорость и `max_angular_accel` от измеренного yaw-rate); TF | transform_tolerance 0,2 |
| velocity_smoother | `odom_topic`, но OPEN_LOOP, то есть фактически не используется | |
| bt_navigator | `odom_topic` (обратная связь по скорости) | |
| trust (`localization_trust_node.py`) | `/qazbot_d1_map/tf` (map→odom, odom→base_footprint), `/qazbot_lio/odom_level` (**жёстко в коде**, требует frame odom/base_footprint), `/qazbot_lio/scan` (требует frame base_footprint) | T1_odom_stale и T1_odom_message_stale ≤ 0,5 с; T1_tf_stale (map→odom) ≤ 1,0 с; T1_scan_stale ≤ 0,5 с; T1_tf_frozen (TF не меняется, а message движется); `_moving` по TF за 0,6 с (0,02 м/с, 0,03 рад/с); T5 скачок map→odom между обновлениями AMCL ≤ 0,10 м / 0,10 рад; `odom_src` за 4 с для сопоставления по штампам |
| adapter / coordinator | только map→odom (свежесть) и trusted | |

### 3.3 Измерения для проекта (только пассивно и офлайн)

- **Живой leg odometry, 3 с подписки:**
  - frame `leg_odom`, child `lidar_imu_chest_front`;
  - **139 Гц** под нагрузкой стека (в покое было 168), age p50 6,5 мс;
  - ось z `leg_odom` расходится с вектором гравитации от level_bridge на **2,2°**, то есть leg_odom практически выровнен по гравитации;
  - `twist.angular` = **0 всегда**, `twist.linear` ненулевой, но в неизвестном кадре;
  - pose covariance задана: var xy 7,1e-4 (σ 2,7 см), var z 1,4e-4, var rot 7,3e-5 (σ 0,49°);
  - twist covariance = 0;
  - начало сейчас в (−0,10, −1,45). Предыдущий отчёт после перезагрузки давал (0,20, −1,01). Двигали ли робот, неизвестно.
    Дрейф в покое надо измерить в режиме сравнения.
- **Офлайн по trial06 (фикстура, та же математика, что в `NativeBase`):**
  - горизонтальный рычаг base_link→IMU **0,107 м** (0,092…0,111);
  - waist_yaw во время поворота **−3,9…+10,2°**, waist_pitch ±1,1°. Значит, yaw IMU-footprint − yaw base_link = −3,9…+10,3°;
  - поворот на месте: base_link сместился на 0,004 м, IMU-footprint на 0,116 м; yaw native IMU 67,5°, base 67,4°;
  - LIO в том же окне: 61,6° и **1,77 м** смещения при развороте на месте. Это дрейф LIO, его разбирает отдельный отчёт lio-drift.

### 3.4 Главный вывод для проекта

Прямая замена «odom→base_footprint := base_link из NativeBase» **несовместима** со сканом.

Скан и trust привязаны к base_footprint = IMU-footprint по курсу LiDAR. Если base_footprint станет проекцией base_link,
появится смещение 0,107 м и разворот до 10° (waist). Перевести скан в отдельный кадр с динамическим TF нельзя:
- AMCL кэширует позу лазера один раз;
- trust отвергает скан не в `base_footprint`.

Поэтому два шага:

- **Режим `native_imu` (шаг 1, рекомендую первым).** base_footprint по определению не меняется. Поза из leg odometry
  `lidar_imu_chest_front` напрямую, вендорский TF не нужен:
  - (x, y) = позиция IMU в `leg_odom`;
  - yaw = atan2 проекции `R_odom_imu·LIDAR_FWD_IN_IMU` на плоскость xy `leg_odom`. Та же формула, что в level_bridge, но
    `R_lv` = I, потому что leg_odom уже выровнен (2,2°, это отдельный тест);
  - z = z IMU (только в Odometry, в TF его нет).

  Это сравнение один к одному с LIO: тот же физический кадр. Скан, AMCL, trust и радиус не меняются. Нет зависимости
  от цепочки TF талии.
- **Режим `native_base` (шаг 2, по отдельному решению).** base_footprint = проекция base_link. Используется ровно математика
  `NativeBase`: `T_odom_base = T_odom_imu·T_base_imu⁻¹`, `T_base_imu` из `tf2 BufferCore` по **штампу источника** odom,
  без экстраполяции, ожидание покрытия цепочкой. Yaw = проекция оси x base_link на плоскость `leg_odom`.

  Обязательная пара к нему: level_bridge публикует скан в base_link-footprint. Для этого в `on_cloud` нужен
  `T_base_imu(t_cloud)` из той же цепочки: сдвиг начала на рычаг и поворот на waist yaw. Если цепочка устарела, скана нет,
  срабатывает T1_scan_stale (безопасная сторона).

  Плюсы: это кадр команды MC; центр поворота на месте почти неподвижен (0,004 м против 0,116 м).
  Требование: перепроверить центр круга ROBOT_RADIUS 0,454, сейчас он стоит на IMU-footprint.

### 3.5 Узел `native_odom_bridge` (общий для обоих режимов)

- **Подписки:**
  - `/aima/mc/leg_odometry`, sensor-data QoS (BEST_EFFORT; издатель BE/TRANSIENT_LOCAL, совместимо, проверено ранее);
  - в режиме `native_base` также `/tf` (BEST_EFFORT, в буфер только 3 звена талии) и `/tf_static` (RELIABLE + TL).
    Это вендорские глобальные топики, а наш TF живёт в `/qazbot_d1_map/tf`, коллизии нет.
- **Частота.** Каждый сэмпл (139–168 Гц) идёт в оценку twist, но публикация децимирована до `publish_rate_hz` = **50**
  (допустимо 20–100): берётся самый новый оценённый сэмпл со **штампом источника**. Причины:
  - Python-подписчики TF (trust под `input_lock`, Nav2 listeners) на 168 Гц;
  - `odom_src` в trust пересобирается списком O(n) на каждое TF-сообщение: за 4 с при 168 Гц это около 670 элементов на сообщение;
  - AMCL ищет TF на штамп скана (возраст около 45 мс), при 50 Гц интерполяция идёт между точками через 20 мс.
- **Планарная проекция.** TF несёт только x, y, yaw (z = 0, roll = pitch = 0), как сейчас. В Odometry `position.z` = z IMU.
- **Twist.** Разность поз на базе 0,3 с в осях base_footprint, как в level_bridge. Пока база не набралась, Odometry не публикуется.
  `twist.angular` вендора равен 0, поэтому вендорский twist не используется.
- **Ковариация.** Её никто из потребителей не читает: AMCL использует alpha-модель, Nav2 ковариацию игнорирует, T2 берёт ковариацию AMCL.
  - pose: σ²x, σ²y из leg-cov (изотропна), σ²yaw из rot var;
  - z, roll, pitch: копия leg-значений;
  - twist: фиксированная номинальная диагональ (документировать как номинал, не измерение).
- **Защита. Если сработала, публикация TF и Odometry прекращается, и trust видит T1_odom_stale через 0,5 с:**
  - неконечные или ненормированные входы: защёлка, как в `NativeBase.non_finite`;
  - frame_id ≠ `leg_odom` или child ≠ `lidar_imu_chest_front`: отказ (закрывает O1);
  - возраст leg odom > 0,2 с, в `native_base` также возраст цепочки TF > 0,2 с: пауза публикации;
  - **сброс начала leg odom (O2):** |Δp| между соседними сэмплами > 0,05 м + 1,5 м/с·dt или |Δyaw| > 0,3 рад: защёлка до перезапуска.
    Тихий сдвиг odom прошёл бы мимо AMCL как телепорт;
  - штамп назад: очистить базу twist, как в level_bridge;
  - штамп из будущего > 0,2 с: отказ.
- **Публикация в рабочем режиме:**
  - TF `odom→base_footprint` в `/qazbot_d1_map/tf`;
  - `Odometry` в `/qazbot_native/odom` (frame odom, child base_footprint);
  - `/qazbot_native/status`: std_msgs/String JSON 1 Гц (режим, частоты, возрасты, причина паузы или защёлки).

### 3.6 Переключение (LIO по умолчанию остаётся)

Параметр `odom_source` ∈ {`lio` (по умолчанию), `native_imu`, `native_base`} плюс `odom_compare` ∈ {false (по умолчанию), true}.
- `lio`: всё как сейчас, байт в байт. Native-узел не запускается, если `odom_compare` выключен.
- `native_*`: level_bridge с `publish_tf:=false`. Он продолжает публиковать скан и `/qazbot_lio/odom_level` для сравнения.
  native-узел с `publish_tf:=true`. Nav2 и trust получают `odom_topic:=/qazbot_native/odom`.
  Ровно **один** издатель odom→base_footprint; проверяется в check.sh и в тесте.
- `native_base` отклоняется в launch, пока не реализован скан в base-кадре (шаг 2).
- trust пишет `odom_source` в строку старта и в status, чтобы READY и anchor, полученные на одном источнике,
  не переносились молча на другой. После смены источника нужны новая поза и новый anchor.

### 3.7 Пассивный режим сравнения (`odom_compare:=true` при `odom_source:=lio`)

TF не трогается. Native-узел работает с `publish_tf:=false` и публикует отдельно:
- `/qazbot_native/compare/odom_imu` (Odometry, frame `native_odom`, child `native_imu_footprint`);
- `/qazbot_native/compare/odom_base` (frame `native_odom`, child `native_base_footprint`; если доступна цепочка TF, это только диагностика);
- `/qazbot_native/compare/status`, JSON 1 Гц:
  - native IMU интерполируется на штамп каждого LIO-сэмпла `/qazbot_lio/odom_level`;
  - на скользящих окнах 1 с и 10 с считаются невязки SE2-приращений (Δd, Δyaw) native−LIO;
  - накопленные Δyaw и путь от старта сравнения;
  - частоты и возрасты;
  - флаг «стоит» и шум покоя (std x, y, yaw за 5 с) по обоим источникам.

Сравниваются только приращения: начала и курсы двух кадров разные.

Для записи достаточно добавить три топика в recorder. Офлайн-скрипт `tools/stack/compare_odom.py` по bag выдаёт
то же самое, что `/compare/status`.

### 3.8 Влияние на проверки trust

- **T1_odom_stale / T1_odom_message_stale (0,5 с):** в native-режиме меряют свежесть MC-одометрии (p50 6 мс, max 39 мс в покое,
  до 0,47 с приёма под recorder) и запас растёт. Но **отказ LIO ими больше не ловится**. Его ловят:
  - T1_scan_stale (скан строится из облака LIO; порог 0,5 с при 5 Гц, запас мал, это известно);
  - и по желанию новая диагностика `T1_lio_stale` по `/qazbot_lio/odom_level` (только в native-режиме, причина motion-блока).
  Без неё замороженный LIO, который продолжает публиковать облако, но с застывшей позой, даст «свежий» скан: тот же риск, что сегодня.
- **T1_tf_frozen:** TF и message из одного узла, поэтому проверка остаётся проверкой ретрансляции. Семантика та же.
- **`_moving`** (0,02 м/с, 0,03 рад/с за 0,6 с): IMU-footprint качается в покое. Перед включением измерить шум покоя
  native против LIO в режиме сравнения. Ложное «движение» сдвигает только логику «AMCL overdue».
- **T5 (0,10 м / 0,10 рад между обновлениями AMCL):** коррекции map→odom станут отражать дрейф leg odom вместо дрейфа LIO.
  По trial06 расхождение yaw native и LIO было 5,9° за поворот 62–67°, LIO при этом «уехал» на 1,77 м.
  Ожидается меньше скачков, но порог не менять, пока не будет данных сравнения.
- **T2, T3, T4, T6, T8:** без изменений, это AMCL и скан.
- **Нагрузка:** при 50 Гц TF у trust около 5× больше TF-колбэков, чем сейчас. Перевести `odom_src` на deque с отсечкой головы.

### 3.9 Файлы к изменению (шаг 1)

1. **Новый** `ws/src/lio/native_odom.py`: чистая математика без ROS. Функции `imu_footprint(p7)`, класс `BaseFromChain`
   (перенос `NativeBase`), `Twist0p3`, `JumpGuard`, `cov_planar`.
2. **Новый** `ws/src/lio/native_odom_bridge.py`: узел; параметры `mode`, `publish_tf`, `publish_rate_hz`, `max_age`, `compare`.
3. `ws/src/lio/level_bridge.py`: параметр `publish_tf` (по умолчанию true; при false нет `sendTransform`, остальное без изменений).
   Шаг 2: `scan_frame:=base` с цепочкой TF.
4. `ws/src/navigation/localization.launch.py`: аргументы `odom_source`, `odom_compare`; запуск узла; shutdown-guard как у level_bridge;
   отказ на `native_base`.
5. `ws/src/navigation/localization_trust_node.py`: параметры `odom_topic` (по умолчанию `/qazbot_lio/odom_level`) и `odom_source`;
   по желанию `lio_topic` для T1_lio_stale; `odom_src` на deque.
6. `ws/src/navigation/run_nav_full.sh`: `odom_topic:=${QAZBOT_ODOM_TOPIC:-/qazbot_lio/odom_level}`.
7. `tools/stack/stack_up.sh`: `QAZBOT_ODOM_SOURCE` (lio|native_imu) и `QAZBOT_ODOM_COMPARE`, передаются в 4 этапа;
   выводит режим; по умолчанию lio.
8. `tools/stack/check.sh`:
   - зонд активного odom-топика и `/qazbot_native/*`;
   - проверка «ровно один издатель odom→base_footprint»;
   - строка trust `odom_source`.
9. Документация: `docs/X2_QUICKSTART_BRINGUP_RU.md` (режимы, go2 через `QAZBOT_TRUST_POLICY`, IP .72), `docs/DECISIONS.md`.

### 3.10 Тесты к добавлению

- `tests/lio/test_native_odom.py` (без ROS, на фикстуре trial06):
  - IMU-footprint совпадает с формулой level_bridge при `R_lv` = I;
  - `BaseFromChain` побайтно (1e-9) совпадает со строками `step_cmd.NativeBase` (повтор 0,107 м и 67,4°);
  - развёртка yaw;
  - проекция при наклоне 4°;
  - twist на синтетике (прямая 0,3 м/с, поворот 0,5 рад/с);
  - децимация сохраняет штампы источника и монотонность;
  - нет экстраполяции;
  - NaN, нулевой кватернион и чужой frame_id: защёлка;
  - сброс начала (скачок 0,5 м при непрерывных штампах): защёлка;
  - штамп назад: сброс twist;
  - возраст > 0,2 с: пауза.
- `tests/lio/test_level_bridge_publish_tf.py` (домен 87, localhost; синтетические `/Odometry`, облако, IMU): при `publish_tf:=false`
  нет TF, а `odom_level` и скан идентичны режиму true.
- `tests/localization/test_odom_source_launch.py` (домен 87, replay bag mc_execution23 trial06 через `replay_lio.py`):
  - lio: граф идентичен текущему;
  - native_imu: ровно 1 издатель odom→base_footprint (native), AMCL принимает сканы (нет «Failed to compute odom pose»),
    у trust нет T1_odom_stale;
  - compare: издатель TF только level_bridge, compare-топики есть, TF не меняется.
- `tests/loc_trust`:
  - параметр `odom_topic` соблюдается;
  - T1_tf_frozen и T1_odom_stale на 50-Гц входе;
  - T1_lio_stale, если будет добавлена;
  - нагрузочный тест `_tf` при 50 Гц за 10 с (время колбэка).
- Регрессия: `tests/motion_adapter` 43/43, `tests/loc_trust`, `nav_e2e` (домен 87) в режиме lio без изменений.

### 3.11 Порядок внедрения

1. Шаг 1 в коде и тесты.
2. **Пассивное сравнение на роботе** (стоя 5 мин, затем записи поворотов из trial-серии по «go»).
3. Решение оператора о `native_imu` в dry-run.
4. Шаг 2 (`native_base` и скан в base-кадре) только после этого.

## 4. Блокеры и открытые вопросы

1. **Повреждённое `candidates/` и неизвестный источник перезаписи** (раздел 2). Исправлено только то, что нужно стеку
   (`fastlio_mapping` +x; smac install пересобран из build, хеш 56b34510 совпал). Остальное повреждено.
   Файлы в `ws/src/navigation` перезаписаны в 16:16, и их происхождение надо подтвердить.
2. **Адрес ПК в `dds_pc_x2.xml` = 192.168.1.64, а SSH сейчас идёт с 192.168.1.70.** Если ПК теперь .70, RViz не увидит стек,
   пока на ПК не обновят whitelist.
3. Пока нет позы оператора, стек в dry-run и READY нет.
4. Leg odom под нагрузкой 139 Гц против 168 в покое; начало odom сместилось на 0,53 м с прошлого снимка. Причина неизвестна.

## 5. Файлы

- Изменены:
  - `/home/agi/control_backend/ws/src/navigation/dds_qazbot.xml`;
  - `/home/agi/control_backend/tools/rviz/dds_pc_x2.xml`;
  - `/home/agi/control_backend/ws/src/navigation/rviz/dds_pc_x2.xml`.

  Бэкапы `*.bak_20261027_pre_ip72`.
- Права: `/home/agi/control_backend/candidates/fast_lio_portable/install/fast_lio/lib/fast_lio/fastlio_mapping` (+x).
- Восстановлен: `/home/agi/control_backend/candidates/nav2_smac_bounds/install/nav2_smac_planner`.
  Повреждённый оригинал в `/home/agi/control_backend/candidates/nav2_smac_bounds/install_corrupt_restore_20261027/`.
- Журналы стека: `/home/agi/control_backend/logs/{localization,trust,adapter,nav_full}_nav72_20261027.{log,pid}`.
- Скрипты зондов и анализа (не в репозитории):
  - `/tmp/claude-1001/-home-agi/064b986d-c606-4601-b9e1-dc10e302f794/scratchpad/leg_probe.py`;
  - `/tmp/claude-1001/-home-agi/064b986d-c606-4601-b9e1-dc10e302f794/scratchpad/t06_footprint.py`;
  - вывод check.sh: `/tmp/claude-1001/-home-agi/064b986d-c606-4601-b9e1-dc10e302f794/scratchpad/check_nav72.txt`.
