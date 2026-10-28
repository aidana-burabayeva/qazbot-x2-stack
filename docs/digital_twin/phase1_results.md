# Phase 1 — воспроизводимое окружение (x86), результаты 2026-10-23

Сделано (в `~/QazBot/control_backend`, `.66` не менялся, только чтение):
- Стянуто с `.66`: `candidates/{nav2_smac_bounds,fast_lio_portable,nav2_lifecycle_shutdown}` (источники, без ARM64-сборок), bag `liolat_lidar_imu_01`, `g_motion_02`, `g_motion_03`.
- Собрано (`runtime_x86/build_ws/`, colcon): `aimdk_msgs` (из `/agibot/software/aimdk/src`), `nav2_smac_planner` 1.1.20 + `bounds.patch` (применён чисто, в .so есть "outside the costmap"). Манифест: `tools/twin/make_smac_manifest.sh`.
- `tools/twin/env_sim.sh`: домен 87, `ROS_LOCALHOST_ONLY=1`, loopback-DDS-профиль, `QAZBOT_SIM=1`. `tools/twin/run_model_x86.sh` — запуск существующего `tests/nav_e2e/run_suite_real.py`.
- `run_suite_real.py`: добавлен `max_angular:=${QAZBOT_MAX_ANGULAR:-0.5}` (раньше launch брал 0.6). Исходники стека не менялись.

Факт запуска: `tools/twin/run_model_x86.sh twin_baseline_01` → rc=0, **167 проверок, 0 провалов**; в сгенерированном yaml контроллера `rotate_to_heading_angular_vel: 0.5`; остаточных процессов нет. ERROR в `nav2.log` — от намеренных fault-сценариев (driver loss, operator stop).

Ограничения: адаптер здесь dry-run (`live:=false`), AMCL эмулирован, модель — кинематический стенд-ин без измеренных порогов MC; Nav2 1.1.20 против 1.1.18 на роботе; версия Smac пересобрана из 1.1.20 (не побитно как на роботе).

## Phase 3 (первый срез): Virtual MC + adapter live:=true — 2026-10-23

Файлы: `tools/twin/virtual_mc.py` (нода на интерфейсе `aimdk_msgs`: сервис `SetMcInputSource`, приём `/aima/mc/locomotion/velocity` только от зарегистрированного `sdk_*` источника, `McCommonState` 10 Гц, проброс vx/wz в существующую кинематическую модель `LocSim`), `tests/twin/test_live_chain.py` + `tools/twin/run_live_chain.sh`, `tools/twin/dds_sim_loopback.xml`.
Изменено в тестовом коде: `tests/nav_e2e/loc_model_fixture.py` — при `QAZBOT_VIRTUAL_MC=1` эмулятор не создаёт издателя `/aima/mc/common/state` (иначе два издателя); `run_suite_real.py` — `max_angular` из `QAZBOT_MAX_ANGULAR` (0.5). Код стека (`ws/src`) не менялся.

Находка: профиль `tests/nav_e2e/dds_domain87_unicast.xml` содержит initial peers только на participant id 0–7. Процессы с id ≥ 8 не находят друг друга (поздний `bt_navigator` и `lifecycle_manager` — «Waiting for service bt_navigator/get_state» бесконечно). Поведение зависит от числа процессов. Для twin — профиль на 40 участников.

Результаты (факт):
| Прогон | Проверок | Провалов |
|---|---|---|
| `live_chain_05`, `live_chain_06` (adapter live, Virtual MC, START→GOAL→START) | 23 | 0 |
| `twin_baseline_02` (старый suite с новым DDS-профилем) | 167 | 0 |
Live: 1249 команд на MC, 0 отклонений; |v|≤0.3, |w|≤0.5, lateral=0, без реверса; единственный издатель `locomotion/velocity` — `motion_adapter`; нет глобального `/cmd_vel`; регистрация только по явному вызову. Остаточных процессов нет. Virtual MC отказывается стартовать вне `QAZBOT_SIM=1` + домен 87 + localhost.

Ограничения: AMCL по-прежнему эмулирован, динамика — стенд-ин без калибровки (пороги страгивания/лаг приблизительные), дефект остановки `lifecycle_manager` (Блокер 6) в успешных прогонах не проявился (`abnormal_children: 0`), но замечен в прогоне с зависшей активацией (SIGINT→SIGKILL); не маскируется, не исправлялся.

## Реальный AMCL в twin — 2026-10-23

Что подключено: `tools/twin/amcl_sim.launch.py` — настоящие `nav2_map_server` + `nav2_amcl` с **продакшн-параметрами** (`OmniMotionModel`, `max_beams:=1440`, остальное по умолчанию; различается только `scan_topic`). Паритет с `navigation/localization.launch.py` проверяет `tests/twin/test_amcl_parity.py` (1 passed; мутация `max_beams` ловится). Эмулятор AMCL отключается `QAZBOT_REAL_AMCL=1` (нет второго издателя `map`/`amcl_pose`/`particle_cloud`/`map->odom`). Запуск: `QAZBOT_AMCL=real tools/twin/run_live_chain.sh NAME`.
Процедура оператора воспроизведена как в `X2_LOCALIZATION_TRUST.md`: RViz-поза START с ковариацией по умолчанию (0,25 м², 0,0685 рад²) → `request_nomotion_update` до σ ≤ порогов Trust → baseline → `confirm_anchor` → READY → регистрация → цели.

Измерения (модель: идеальные одометрия и скан, 360°, карта v2; поза истины известна):
- Старт: σ_xy после RViz-позы 0,46 м (порог 0,20). Сходится за **7** no-motion обновлений (воспроизводит «на стоящем роботе σ сходится только через nomotion», Блокер 3).
- При AMCL по умолчанию (alpha1–5 = 0,2), 9 полных прогонов START→GOAL→START: **2 успешных, 6 прерваны `T5_map_odom_jump`, 1 промах по yaw (0,117 рад > допуск 0,10)**. Скачки положения между соседними обновлениями 0,110 и 0,147 м (порог 0,10) при движении 0,3 м/с.
- Диагностический эксперимент (только в twin, прод не менялся): alpha1–5 = 0,01 (в twin одометрия идеальна, шум движения ≈ 0): 3 прогона — **0 срабатываний T5**, 2 успешных, 1 промах `goal_out_pose` (истинная ошибка выше допуска 0,10; ошибка AMCL p95 ≈ 0,08, max ≈ 0,10 м).

Вывод и границы:
- Подтверждено на модели: раздувание облака при alpha = 0,2 приводит к скачкам map→odom > 0,10 м, и это даёт ложные аборты по T5. Это согласуется с Блокером 3, но **не доказывает значение alpha для реального робота**: у twin идеальная одометрия, реальный шум FAST-LIO в нём не смоделирован. Калибровка alpha — только по измерениям (3 сессии, `reports/20261023-calib-design.md`); число 0,01 — не рекомендация.
- Ошибка AMCL (~0,08–0,10 м) сопоставима с допуском цели 0,10 м: цель считается достигнутой по оценке AMCL, а истинная поза может выйти за допуск (связано с Блокером 2). Выборка малая (3 прогона), статистику нужно набрать.
- Скан twin — идеальный 360° ray-cast, у робота ~90–100° и неизвестная ошибка: результаты оптимистичны по входу и пессимистичны по alpha; это не одно и то же с поведением на роботе.
- Тест с реальным AMCL сейчас детерминированно не проходит при продакшн-параметрах — это воспроизводимый сценарий бага, а не флак. В регрессионный набор его надо добавить как «ожидаемо падает до калибровки alpha» (xfail), не ослабляя порог T5.

## Реплей AMCL на записях робота (уровень 2) — 2026-10-23
`tools/twin/replay_amcl.py`: записанные `scan` + `odom→base_footprint` (записанные `map→odom`, `amcl_pose`, Trust не публикуются) → настоящий AMCL с продакшн-параметрами; карта v3; сдвиг штампов на константу; 1× темп. Сид — поза записанного AMCL, ковариация RViz по умолчанию.
| bag | длит. | обновлений AMCL (запись/реплей) | max шаг xy запись → реплей | T5-срабатывания запись → реплей | расхождение поз реплей vs запись |
|---|---|---|---|---|---|
| short_nav_01 | 177 с | 8 / 8 | 0,066 → 0,107 | 0 → 1 (на 132,6 с) | xy med 0,112, max 0,172 м |
| cycle2_01 | 290 с | 3 / 3 | 0,026 → 0,023 | 0 → 0 | xy max 0,040 м |
| turn_nav_01 | 383 с | 2 / 2 | 0,023 → 0,103 | 0 → 1 (на 268,1 с) | xy max 0,080 м |
Ограничения: по одному прогону на bag; обновлений AMCL мало (2–8, робот в основном стоял); повтор того же входа даёт иной результат (AMCL недетерминирован по тайминг/случайности) — разброс не измерен. Вывод только такой: на реальных входах шаги map→odom порядка 0,02–0,11 м, т.е. порог T5 (0,10 м) достижим и на реальных данных; сравнение с идеальным 360° сканом twin (там скачки 0,11–0,15 м) показывает, что реальный узкий скан не хуже, но выборка слишком мала для вывода.

## Перенос в `control_backend` и проверка (2026-10-26)
Папка `~/qazbot-twin` ликвидирована; всё внутри `control_backend/` (карта — `README.md` рядом). Дубли `ws/src`, `tests`, `tools`, `candidates`, `docs/robot_docs`, bag (9 шт., побайтно идентичны) удалены после проверки; две файла Trust в моей старой копии были старше (Go2-правки 16:12 в `control_backend`) — использована новая версия. Три мои правки тестов перенесены (`loc_trust/test_loc_trust.py` — относительный путь карты; `nav_e2e/loc_model_fixture.py` — `QAZBOT_REAL_AMCL`/`QAZBOT_VIRTUAL_MC`; `nav_e2e/run_suite_real.py` — `max_angular` из `QAZBOT_MAX_ANGULAR`, по умолчанию 0,5). x86-сборка пересобрана на новом месте `tools/twin/build_x86.sh` (в `install/` зашиты абсолютные пути).
Проверка на актуальном коде (с Go2-правками):
| Что | Результат |
|---|---|
| pytest: parity AMCL, adapter (43), loc_trust | 146 passed, 1 skipped, **1 failed**: `test_trust_gaps::NodeStampReject::test_duplicate_and_old_amcl_stamp_keep_reference` — `_amcl` теперь читает `self.cfg.admission_policy`, а фикстура теста строит `TrustNode` без `cfg` (дефект теста относительно Go2-правки, не связан с переносом) |
| `run_model_x86.sh` (базовый suite) | прогон 1: 158 проверок без провалов, затем перезапущенная группа Nav2 не активировалась за 90 с (`controller_server: failed to send response to …/change_state (timeout)`), rc=1; прогон 2: **167/167**, rc=0, `abnormal_children: 0`. Итого по двум месторасположениям: 3 из 4 прогонов целиком успешны; срыв — нестабильность DDS-ответа сервиса lifecycle при перезапуске группы, причина не установлена |
| `run_live_chain.sh` (адаптер live ↔ Virtual MC, эмулированный AMCL) | 23/23 |
| `QAZBOT_AMCL=real run_live_chain.sh` | падает на `goal_back_succeeded` (T5), как и ранее — ожидаемый воспроизводимый сценарий (см. раздел «Реальный AMCL»), не регрессия |
