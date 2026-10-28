# Передача состояния QazBot X2

Обновлено: 2026-10-28, после пассивного T5 LIVE и остановки t5live1.
Правила: [AGENTS.md](../AGENTS.md). Результаты: [TEST_STATUS.md](TEST_STATUS.md).

## Краткий итог

- Установленное исправление T5 подтвердило стационарный reseed и измеренную живую коррекцию без ложного отказа. Полный критерий отсутствия T1/ошибок времени во всём прогоне НЕ выполнен.
- t5live1 передана оператором этой сессии и остановлена после теста. Повторный старт не выполнен.
- I1: минимальный экспериментальный diff на production baseline; unit и проверки фактического ObstacleLayer проходят. Полные A/B, интеграция источников и реальные cloud_registered ещё не проверены. На X2 не устанавливалось.
- Запись облака не разрешалась и не выполнялась.

## Git и файлы

Production baseline: c15dd6c6c8fe8d5e6cc1014c68bb9da51dd8a3fa. На старте он подтверждён через GitHub API как main и как HEAD обоих локальных worktree. Коммит передачи в main содержит документацию и доказательства, не production-код.

- /home/adn/QazBot не является Git-репозиторием.
- /home/adn/QazBot/control_backend: локальный main на baseline; имел изменения docs/X2_QUICKSTART_BRINGUP_RU.md, tests/loc_trust/run_ros_synthetic.py, tests/nav_e2e/loc_model_fixture.py, tests/nav_e2e/run_suite_real.py. Они сохранены и не включены в публикацию.
- /home/adn/QazBot/control_backend_i1: fix/i1-rpp-collision; локальная база c15dd6c, новый diff находится в рабочем дереве. Кандидат публикуется отдельно в одноимённой GitHub-ветке.
- GitHub операции выполнялись подключённым GitHub API: shell git ls-remote был недоступен из sandbox. Публикация API не передвигает локальные HEAD; не делать reset поверх незакоммиченных изменений.
- Digital Twin доступен в /home/adn/QazBot/digital_twin; это соседний каталог вне репозитория. Не восстанавливать его по догадкам в другой среде.
- Существующие внешние отчёты: digital_twin/docs/T5_geometry_20261028.md и I1_rpp_collision_20261027.md. Старый вердикт I1 «готов к ревью» больше не является допуском.

## T5 LIVE: результаты и границы

Полные измерения: [summary JSON](evidence/t5live1_summary_20261028.json),
[observer JSONL gzip](evidence/t5live1_observer_20261028.jsonl.gz),
[Trust log gzip](evidence/t5live1_trust_20261028.log.gz),
[localization log gzip](evidence/t5live1_localization_20261028.log.gz).
В summary сохранены SHA256 архивов и исходных данных.
[Первый graph snapshot](evidence/t5live1_graph_20261028.json) относится к состоянию ДО seed.

### Что подтверждено

Оператор подтвердил присутствие у X2, знание фактической позы и передачу t5live1. Initialpose задавал оператор через RViz; координаты агент не выбирал. Получены 4 seed, 6 AMCL poses, 723 map → odom TF и 1492 статуса Trust. Принятие seed видно в логах и сообщениях.

Максимальная измеренная T5-коррекция: 0.0327434893 м / 0.0193353509 рад, допуск сохранён. Ложных T5 в записанном прогоне: 0. Порог остаётся 0.10 м / 0.10 рад. После последнего reset непрерывный READY/TRUSTED/MOTION_OK: 124.9717 с, 523 статуса без причин блокировки. Этот интервал начинается по событию восстановления READY после reset; полный переходный поток сохранён.

На namespace TF зарегистрированы только AMCL и native_odom_bridge; level_bridge с publish_tf:=false, FAST-LIO публикует TF отдельно в /qazbot_lio/tf. /qazbot_native/odom имеет одного publisher native_odom_bridge; /qazbot_lio/scan — одного level_bridge. Native bridge сообщает mode=native_imu, 30 Hz и отсутствие текущего fault. Humble rclpy callback не предоставляет sender GID: владельцы установлены по graph endpoints, процессам и их параметрам, не по атрибуции каждого TF-пакета.

SHA256 повторно прочитаны с X2, совпали; переустановки не было:
- loc_trust.py: 4b81f0721b13e510f4da05b7494177ad8a2dbdd104d56bf12d3ce6cd786f59e8.
- localization_trust_node.py: 6b36ea0efdba0ca16344e62771500527b2e2b580f53122535b0461564bb35a46.

### Что не прошло / не доказано

- Во всём прогоне после первого seed есть 2 T1_scan_stale. На входах reseed есть переходные not_ready/T1_tf_stale; после последнего reset они исчезли.
- Все 4 initialpose сопровождались предупреждением AMCL о future extrapolation: требуемое время опережало последний odom TF примерно на 8–34 мс. Это не замаскировано увеличением tolerance/таймаута.
- Исторический счётчик rejected scan stamps=140 оставался постоянным во время seed-проверки; содержит старый source stamp около 0.095 с. Новых stamp rejection в проверенных seed-эпохах не замечено; происхождение прежних событий не установлено.
- Go2 допускает READY при diagnostics T2_covariance, T3_particles, T4_no_baseline/T4_scan_score, T6_no_anchor. Последние sigma_xy≈0.478 м, sigma_yaw≈0.232 рад; READY не является доказательством точной локализации или успешной навигации.
- Реальный глобальный скачок специально в живую систему не вводился. Его блокировку подтверждают прежние реальные TF replay и synthetic E2E, а не данный stationary LIVE.
- Полный discovery показал штатный /controller_server_ros_node на /cmd_vel и четыре штатных publishers locomotion/velocity (RC, app_proxy, interaction slave, planning). QazBot Motion Adapter/Goal Coordinator и их command publishers отсутствовали. Velocity-сообщений в наблюдателе не получено; это не доказательство нулевых значений. MC state: input_source=rc, STAND_DEFAULT, Trust moving=false. Команды/цели агент не отправлял.

Итог: **T5 stationary/reseed PASS; полный temporal acceptance FAIL / остаётся открытым.** Не объявлять T5 LIVE полностью закрытым и не переходить автоматически к физической записи облака.

### Остановка и текущее состояние робота

До теста существовали процессы agi: launch 76790; FAST-LIO 76841; level_bridge 76880; map_server 76886; AMCL 76904; lifecycle manager 76922; native bridge 76956; Trust 78593.

После передачи владения тест завершён: SIGINT Trust, затем SIGINT launch. Launch подтвердил clean exit дочерних процессов. Собственный observer остановлен отдельно; при зависшем DDS teardown потребовалось адресное завершение только своих наблюдателей. Ошибка первого варианта observer (callback с лишним аргументом, несовместимым с Humble) исправлена ДО seed; финальная запись не содержит error events.

На 2026-10-28 16:25:05 +08:00 все перечисленные PID и финальный observer 135989 отсутствуют. Последний graph summary показывает отсутствие publishers /qazbot_d1_map/tf, /qazbot_native/odom, /qazbot_lio/scan. Штатные процессы робота не останавливались. В следующей сессии актуальность проверить заново; не считать этот snapshot вечным.

## I1: экспериментальный bounded-clearing diff

Ветка: fix/i1-rpp-collision; GitHub commit c23ce839d98ba7a83ee39ef8f3b5ec6dad5dc0d1. [Draft PR #1](https://github.com/aidana-burabayeva/qazbot-x2-stack/pull/1) не готов к финальному review. [Техническая записка кандидата](https://github.com/aidana-burabayeva/qazbot-x2-stack/blob/fix/i1-rpp-collision/docs/I1_BOUNDED_CLEARING.md).

Старый candidate превращает floor return на 1 м в +inf своего и шести соседних бинов. Все 10 старых тестов проходят; реальный Nav2 ObstacleLayer стирает дальний lethal при дальнем endpoint.

Новый diff добавлен к актуальному production level_bridge, без копирования старого файла:
- /scan продолжает давать реальные obstacle returns и вход AMCL.
- Отдельный PointCloud2 topic resolved_scan_topic + "_clearing" передаёт только измеренные лучи с конечным endpoint.
- Endpoint ограничен фактическим возвратом и первым выходом луча из существующего высотного среза. Для точки (1,0,-1) это 0.628019 м, не 3.5 м.
- Ближайший endpoint в бине ограничивает более дальний луч; dilation/interpolation/+inf для clearing нет.
- В Nav2 scan только marks; bounded_rays только clears. Collision detection, footprint, радиус и range thresholds сохранены. TF/native odometry код не заменён.

Проверено на ПК: 14 Python tests и 21 assertion реального установленного Nav2 ObstacleLayer 1.1.20. C++ harness вызывает raytraceFreespace и updateBounds без DDS; это не полная source/TF/lifecycle интеграция.

Открыто:
1. Полный regression pipeline и повтор A/B именно нового кода и источника; Docker socket сейчас permission denied.
2. Реальные /qazbot_lio/cloud_registered отсутствуют. Проверить IMU origin/deskew и геометрию лучей на actual bridge input; raw lidar_pointcloud не заменяет его.
3. Измерить, устраняет ли консервативный clearing исходный RPP blockage: ближайшая точка может уменьшить clearing.
4. Сопоставить версию Nav2 ПК с production и повторить при необходимости.
5. Финальный review-ready PR только после результатов. На физический X2 ничего из I1 не устанавливать.

## Следующее действие

Сначала разобраться с измеренными временными предупреждениями/T1 и согласовать дополнительную физическую проверку, если она нужна. Не менять production-код или AMCL параметры без отдельного разрешения.

Пока T5 полностью не принят, I1 продолжать только на ПК. После закрытия T5 и освобождения робота запросить отдельное разрешение на пассивную запись cloud_registered. Сенсорный стек сейчас остановлен: если для записи понадобится запуск, сначала проверить процессы и согласовать конкретный запуск. MC и движение для записи не нужны.

Хвост First_RViz_goal_1135 физически потерян; не требовать восстановления. Использовать существующие реальные TF/reseed replay с явным ограничением исторического покрытия.
