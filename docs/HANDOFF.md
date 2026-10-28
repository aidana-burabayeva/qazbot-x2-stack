# QazBot X2: продолжение к первому RViz NavigateToPose

Обновлено 2026-10-28. Читать вместе с [AGENTS.md](../AGENTS.md), [CLAUDE.md](../CLAUDE.md) и [TEST_STATUS.md](TEST_STATUS.md). Первый успешный физический NavigateToPose после исправлений **не подтверждён**.

## Активная задача и точные версии

Приоритет: T5 LIVE → I1 E2E → I1 REVIEW → разрешённый DEPLOYMENT → разрешённый короткий RViz Goal → фактическая причина ABORTED → SUCCEEDED. Небольшие безопасные ошибки движения допустимы; collision detection, реальный footprint и freshness/T5 не ослаблять.

- GitHub main: **6e70c0f90665c06dfc704162c9ddc84efce977c1**, документация и T5 evidence поверх production baseline.
- Production/T5 baseline: **c15dd6c6c8fe8d5e6cc1014c68bb9da51dd8a3fa**.
- GitHub fix/i1-rpp-collision: **a223c454f21ca85b4a1a4572a10d6d88fe2c9813**. Bounded-clearing production diff из c23ce839; последующие коммиты добавляют passive capture и подготовку scoped HAL recovery.
- [Draft PR #1](https://github.com/aidana-burabayeva/qazbot-x2-stack/pull/1) не готов к финальному review: A/B и actual cloud validation не завершены.
- Эта передача публикуется в отдельной ветке handoff/first-rviz-goal-20261028. Продолжать с её опубликованного HEAD, затем проверить актуальные refs; не восстанавливать код по памяти.

На X2 **T5 установлен, I1 не установлен**. Production-файлы в этой сессии не изменялись.

## Пути и Git на X2

Git checkout на ПК: /home/adn/QazBot/control_backend; I1 worktree: /home/adn/QazBot/control_backend_i1. Локальные HEAD остаются c15dd6c: GitHub API публикация не двигает их автоматически. Четыре ранее изменённых файла main сохранены: docs/X2_QUICKSTART_BRINGUP_RU.md, tests/loc_trust/run_ros_synthetic.py, tests/nav_e2e/{loc_model_fixture.py,run_suite_real.py}. Они не включены в новые коммиты.

На X2 /home/agi/control_backend — deployed-каталог **без .git**. Поэтому git status там закономерно не работает. Не делать git init поверх production/runtime. Для Git нужен отдельный checkout; для установленной версии — проверенные SHA256. Доступ: agi@192.168.1.72, ROS 2 Humble.

Digital Twin доступен локально: /home/adn/QazBot/digital_twin, вне основного checkout. Если каталог отсутствует в новой среде, сообщить и не восстанавливать по догадкам. Старые I1 A/B из digital_twin/docs/I1_rpp_collision_20261027.md относятся к прежней реализации.

Проверенные production SHA256 X2:
- level_bridge.py: 86c434ce0ee5a48de1a324f7a3eb5e2136f8d9d3f1b91b582f6fa522707be686.
- nav_control.launch.py: ebea6637578530f0b30be6dd5d720ef1011270ce2a4df902d2bd1e1b5c266bec.
- loc_trust.py: 4b81f0721b13e510f4da05b7494177ad8a2dbdd104d56bf12d3ce6cd786f59e8.
- localization_trust_node.py: 6b36ea0efdba0ca16344e62771500527b2e2b580f53122535b0461564bb35a46.

## T5 LIVE: стационарный этап выполнен

Оператор подтвердил фактическую позу и передал t5live1. Четыре initialpose заданы оператором через RViz. AMCL принял seed, появился map → odom; native_imu odometry и единственность владельцев namespace TF подтверждены. Измеренная коррекция 0.0327435 м / 0.0193354 рад не создала T5; ложных T5=0.

Последний непрерывный READY/TRUSTED/MOTION_OK интервал после reseed reset: **124.9717 с / 523 статуса**. Max ages: scan 0.398181 с, odom 0.092942 с при неизменённых лимитах 0.5 с. Два переходных scan stale 0.631636/0.596968 с корректно сняли MOTION_OK. Seed-time AMCL warnings future extrapolation 8–34 мс сохранены; прежний строгий критерий полного отсутствия temporal warnings не выполнен. По новому приоритету это не повод повторять закрытую стационарную T5 проверку без новых свидетельств. Перед движением нужен новый live freshness preflight.

Covariance/particles/scan score/no baseline/no anchor diagnostics Go2 не скрыты. READY не доказывает точность AMCL или готовность всего маршрута. Большой скачок в LIVE не вводился; блокировку подтверждают прежние real TF replay и synthetic E2E (125 passed/4 skipped на X2; BAD=0 OK=10). Потерянный хвост First_RViz_goal_1135 не восстанавливать.

[Summary, SHA256 и данные T5](evidence/t5live1_summary_20261028.json). t5live1 корректно остановлена: Trust, затем launch; launch children clean exit. Никакого MC/автономного движения агент не запускал.

## Новый фактический блокер: падение vendor HAL LiDAR

Для I1 оператор разрешил один FAST-LIO и passive recorder, запись **только внутри /home/agi/control_backend/docs**. Затем отдельно потребовал коммитить сами bags и логи в GitHub. Выполнены две попытки, обе FAIL: **0 cloud_registered / 0 LIO odometry**. Первая содержит 437 LiDAR IMU и 6306 native leg odometry; вторая — 16487 leg odometry.

Установлено новое свидетельство: vendor hal_sensor_orin PID 2623 создал crash в **16:50:58 +08:00**, во время первого capture startup. Trace: LidarParser::publish → rcl_publish → Fast DDS 2.14.6 SharedMemTransport → interprocess_mutex::lock. HAL log перед падением: publish buffer is not released. Точная причинность/race не установлена; не выдавать это за доказанный баг I1 (I1 на роботе отсутствует).

20-секундная passive probe штатного DDS profile также не увидела raw cloud/IMU publishers. Оператор подтвердил отсутствие выключения/перезапуска робота. На snapshot отсутствует /soc1_hal_sensor_orin2623; камера /soc1_hal_camera3361 продолжает работать. PTP s2, последние offset порядка десятков микросекунд, не секундный скачок.

[Bags, логи, counts и восстановление](evidence/I1_DATA_README.md), [manifest / SHA256 / provenance](evidence/I1_capture_manifest_20261028.json), [crash text](evidence/hal_sensor_orin_20261028_165058.crash). Сами lossless .db3.gz + metadata.yaml сохраняются в Git; raw .db3 производные игнорируются. SQLite quick_check и побайтная проверка gzip пройдены. Core 1052823552 bytes остаётся на X2; путь/SHA256 в manifest, текст trace в Git.

Последняя проверка 17:05:05 +08:00: FAST-LIO 168922, recorder 172036 и passive probe 178843 завершены; действующих QazBot localization/capture процессов нет. HAL sensor не восстановлен, камера не тронута. Перед любым новым действием проверить актуальные процессы.

### Следующий конкретный шаг

**Ожидается отдельное разрешение на восстановление HAL.** Запрос оператору уже отправлен. Прежнее согласие на FAST-LIO/recorder не подменяет его.

Подготовлено в fix/i1-rpp-collision:
- tools/restore_hal_sensor.sh — refuse duplicate, восстановление штатного hal_sensor_orin через em_run.sh, исходный cfg/hal_sensor_orin_t2d5.yaml, owner run и исходные NET_RAW/SYS_NICE capabilities. Использует vendor environment действующей камеры, не печатает его и не трогает camera/MC. Запускается с sudo после разрешения.
- tools/i1_capture_udp_only.xml — только process-local профиль новых FAST-LIO/recorder; глобальные DDS файлы не меняются.
- tools/capture_i1_cloud.py — subscriber-only, quota 100 непустых clouds с same-stamp odometry, без runtime retries/магического таймаута.

После разрешения: повторно проверить отсутствие HAL, сохранить SHA исходных vendor файлов, перенести проверенные helper/profile в /tmp, восстановить один HAL. Сначала подтвердить raw cloud/IMU data. Затем запустить один recorder и один FAST-LIO с process-local UDP-only profile. Данные — только control_backend/docs/evidence/<новый tag>. Не повторять blind restarts и не удалять чужие SHM files.

При успешно набранной квоте остановить owned FAST-LIO/recorder, проверить SQLite/counts/SHA, коммитить данные и выполнить геометрическую проверку на ПК. При повторном HAL crash остановить тест и сохранить trace; не маскировать sensor loss. Восстановленный штатный HAL учитывать отдельно от тестовых процессов.

## I1 и путь к Goal

Кандидат сохраняет production TF/native code. Отдельные bounded PointCloud2 rays заканчиваются на фактическом возврате/выходе из height slice; floor (1,0,-1) даёт 0.628019 м, не 3.5 м. Scan только marks, bounded source только clears. **14 Python tests / 21 assertions фактического ObstacleLayer 1.1.20 проходят**. Это не full lifecycle/TF integration и не A/B.

Docker socket в текущей среде недоступен; sudo ограничен no-new-privileges, автоматическое разрешение выхода из sandbox отклонено. Полный новый A/B пока не запускался; старый scan-only plant не валидирует новый bounded source. После actual cloud проверки адаптировать A/B к настоящему bridge conversion, подтвердить obstacle preservation, sensor-loss behavior и устранение исходного RPP false collision.

После I1 E2E: review → отдельное разрешение deployment → baseline SHA/backup/minimal diff/hashes/config check → один стек → obstacle/clearing/TF/AMCL проверки. При неуспехе остановить тест и откатить изменённые файлы.

Затем подготовить короткий свободный маршрут по D1 first_map_nav2_v3 без движения. Только после отдельного разрешения, присутствия оператора и проверки RC/останова — одна RViz Goal. Скорость выбрать по существующим измерениям торможения и реакции, не по догадке. Сохранить start/end pose, Nav2/actual velocity, AMCL/T5, Costmap, action result и фактическую остановку. Далее исправлять измеренную причину ABORTED до SUCCEEDED, не отключая защит.
