# QazBot X2: продолжение к первому физическому RViz Goal

Обновлено 2026-10-28, snapshot X2 около 18:48 +08:00. Начать с этой передачи и AGENTS.md, не с нового аудита. Первый физический NavigateToPose SUCCEEDED **пока не подтверждён**.

## Активная задача и Git

Оператор одобрил diff PR #1 и **временную пассивную установку I1 2d240dbd**. Установка выполнена, один localization/Trust и один пассивный Costmap работают. **Ожидается новая initialpose оператора** через RViz; запрос отправлен. MC, Motion Adapter, Goal Coordinator, goals и движение НЕ разрешены и агентом не запускались.

- main: 6e70c0f90665c06dfc704162c9ddc84efce977c1 (документация поверх production baseline c15dd6c6c8fe8d5e6cc1014c68bb9da51dd8a3fa).
- I1 / PR #1: **2d240dbd09ca69de87e9ac4b1acde735293ad6b2**, https://github.com/aidana-burabayeva/qazbot-x2-stack/pull/1. Код I1 не merged в main.
- Передача и данные: ветка handoff/first-rviz-goal-20261028; начинать с её опубликованного HEAD. Точный SHA будущего docs-коммита определяет Git, а не самоссылка внутри документа.
- Локальные checkout HEAD c15dd6c; GitHub API-коммиты не перемещают локальные refs. Чужие изменения сохранены: docs/X2_QUICKSTART_BRINGUP_RU.md, tests/loc_trust/run_ros_synthetic.py, tests/nav_e2e/{loc_model_fixture.py,run_suite_real.py}.
- ПК: /home/adn/QazBot/control_backend и control_backend_i1. Digital Twin доступен /home/adn/QazBot/digital_twin, пока вне Git. Не восстанавливать отсутствующие исходники по догадке.
- X2 agi@192.168.1.72: /home/agi/control_backend — deployed-каталог без .git; не git init/reset поверх runtime.

## Установлено на X2

Только два production-файла из 2d240dbd, исходный backup проверен. Manifest: `/home/agi/control_backend/docs/evidence/i1_deploy_2d240dbd/deployment.json` (копия в evidence). Manifest INSTALLED_NOT_STARTED фиксирует именно транзакцию установки; последующий запуск фиксируется отдельным passive_snapshot.json.

| Файл | Установленный SHA256 |
| --- | --- |
| ws/src/lio/level_bridge.py | c817a5ed503386be742fa7b6fdab2b3209bdefaf42be799d2a130cd198f9369b |
| ws/src/navigation/nav_control.launch.py | c4e46506069481f3cebd3fef38af7e83bac858f9d6d4e344edeb657a137de38e |
| ws/src/navigation/loc_trust.py (без переустановки) | 4b81f0721b13e510f4da05b7494177ad8a2dbdd104d56bf12d3ce6cd786f59e8 |
| ws/src/navigation/localization_trust_node.py (без изменения) | 6b36ea0efdba0ca16344e62771500527b2e2b580f53122535b0461564bb35a46 |

UDP-only process-local profile dds_udp_only.xml: 29d4c736a364691689ccc8f27c62f566c65af946df1b6fc9dd5ec03ae239ec63; включает develop0 и wifi0 для RViz. Vendor/global DDS XML не изменялись. После `source ws/src/navigation/env.sh` обязательно переопределять FASTRTPS_DEFAULT_PROFILES_FILE, ROS_DOMAIN_ID=0, ROS_LOCALHOST_ONLY=0.

## Последние процессы и пассивные данные

Сессия i1_deploy_2d240dbd, PID snapshot: launch 287375, FAST-LIO 287381, level_bridge 287406 publish_tf=false, AMCL 287436, native bridge 287488 native_imu publish_tf=true, Trust 287803, observer 288603, standalone Costmap 288900. Все принадлежат этой сессии; **PID не считать актуальными без новой проверки**. map_server/lifecycle также входят в launch.

Проверено: свежие cloud_registered, scan, scan_obstacles, scan_clearing; по одному publisher. Оба новых PointCloud2 в base_footprint. Native status fault/latched пусты; odom→base_footprint от native bridge. Namespace TF endpoints — native bridge и AMCL, разных рёбер. До initialpose map→odom ещё не появился, Trust корректно NOT_READY/awaiting_initialpose_and_amcl/T1_tf_stale. Это не отказ I1.

Пассивный Costmap /costmap/costmap configured/active, реальные источники scan_obstacles marking и scan_clearing clearing; 4×4 м, 0.05 м, radius .454 + padding .01, штатные 3/3.5 м, inflation .464. Это standalone ObstacleLayer, без controller/RPP action. Летальные клетки видны; полноценная проверка операторским препятствием ещё не выполнена.

У QazBot namespace command publishers нет. Штатные /cmd_vel и MC velocity endpoints существуют, чужие процессы не останавливались; отсутствие полученных velocity samples не доказывает нулевые команды всех штатных publishers. MC state получен rc/STAND_DEFAULT. Никаких команд агент не отправлял.

## T5 LIVE: стационарный этап выполнен

Оператор подтвердил фактическую позу и передал t5live1. Четыре initialpose заданы оператором через RViz. AMCL принял seed, появился map → odom; native_imu odometry и единственность владельцев namespace TF подтверждены. Измеренная коррекция 0.0327435 м / 0.0193354 рад не создала T5; ложных T5=0.

Последний непрерывный READY/TRUSTED/MOTION_OK интервал после reseed reset: **124.9717 с / 523 статуса**. Max ages: scan 0.398181 с, odom 0.092942 с при неизменённых лимитах 0.5 с. Два переходных scan stale 0.631636/0.596968 с корректно сняли MOTION_OK. Seed-time AMCL warnings future extrapolation 8–34 мс сохранены; прежний строгий критерий полного отсутствия temporal warnings не выполнен. По новому приоритету это не повод повторять закрытую стационарную T5 проверку без новых свидетельств. Перед движением нужен новый live freshness preflight.

Covariance/particles/scan score/no baseline/no anchor diagnostics Go2 не скрыты. READY не доказывает точность AMCL или готовность всего маршрута. Большой скачок в LIVE не вводился; блокировку подтверждают прежние real TF replay и synthetic E2E (125 passed/4 skipped на X2; BAD=0 OK=10). Потерянный хвост First_RViz_goal_1135 не восстанавливать.

[Summary, SHA256 и данные T5](evidence/t5live1_summary_20261028.json). t5live1 корректно остановлена: Trust, затем launch; launch children clean exit. Никакого MC/автономного движения агент не запускал.

## I1: новые результаты, не повторять закрытые проверки

Финальный diff: 69 insertions / 4 deletions, два production-файла. LaserScan AMCL/Trust не изменён. Marking и clearing используют парные реальные PointCloud2 точки: это устраняет обнаруженный float/угловой mismatch LaserScan на границе costmap клетки. Clearing ограничен реально измеренным лучом/выходом из height slice/первым препятствием, без dilation и выдуманных свободных дальностей. Пол на 1 м сам по себе даёт только 0.628019 м clearing. Дальний реально наблюдённый луч может подтвердить большую дальность; близкое настоящее препятствие ограничивает её.

- 14 bounded-clearing Python tests PASS; 5 deployment/rollback tests PASS.
- Фактический ObstacleLayer 1.1.20 на ПК: 21 assertions PASS.
- 29 existing native-odom/clock tests PASS, 5 ROS Node cases deselected. TF/native callback AST сохранён относительно baseline.
- Реальный cloud replay: 100 clouds записано, 71 пригодны после штатных level/cache; 0 changed LaserScans, lost marks, rays outside observed FOV или unsupported endpoint witnesses.
- Native ROS DDS/TF/lifecycle A/B на X2, изолированный domain 88, Nav2 1.1.18: 11 phases × 2 modes × 2 widths (4/8 м) = **44 функциональные проверки PASS**. Sensor loss/empty/partial сохраняют препятствия; близкий floor не очищает дальний lethal; подтверждённые дальние лучи очищают старые клетки; реальные и вне-FOV препятствия сохраняются; unknown остаётся unknown.
- **Известный FAIL завершения отдельного Costmap присутствует и в baseline, и candidate**: active SIGINT abort; даже deactivate/cleanup → SIGINT дал exit -11. Не скрывать как PASS всего runtime. Оператор явно одобрил временную пассивную установку с этим ограничением. Docker sandbox больше не блокирует функциональный A/B: он выполнен native на X2.
- Полный MuJoCo/реальный RPP action A/B и физический NavigateToPose не выполнены; unit/ObstacleLayer PASS их не заменяют.

Результаты: I1_real_cloud_final_20261028.json, I1_costmap_ab_points4_20261028.json, I1_costmap_ab_points8_navudp_20261028.json, I1_costmap_ab_managed_shutdown_20261028.json. Код/воспроизведение — tools/i1_costmap_ab.py и docs/I1_BOUNDED_CLEARING.md в 2d240dbd.

## Реальные данные и HAL

Ранее HAL PID2623 упал внутри FastDDS SharedMemTransport во время capture startup; crash text и две неуспешные bags сохранены. Точная причинность не доказана. По отдельному разрешению восстановлен **один штатный HAL PID204720**, owner run, через исходный em_run.sh/hal_sensor_orin_t2d5.yaml, без изменения vendor файлов. Камера PID3361 не тронута. HAL оставлен как восстановленный штатный компонент; не останавливать его при завершении нашей localization-сессии и не запускать второй.

После восстановления UDP probe: 200 raw clouds/3986 IMU, по одному publisher. Затем записаны 100 cloud_registered + 100 same-stamp LIO odometry, 9363 raw IMU, 7726 leg odometry. Quota met, FAST-LIO/recorder capture завершены. Данные: `/home/agi/control_backend/docs/evidence/i1_cloud_registered_udp_20261028_092700Z/`, копия на ПК в таком же repo-relative пути.

Сами данные публикуются в GitHub по прямому указанию оператора: lossless gzip разбит на parts/*.part000..007; concat в порядке имён восстанавливает .db3.gz, затем gzip -d. Проверить archive_manifest.json. Gzip SHA256 182bd4a292f9947804412c1f121a0a9582cc8e2fbe138558874472e0a111380c; raw db3 SHA256 76e6f510427ff98ae690fbf538dfb61ee5309e11213a238e8df8e7c8f99191a1. Core 1 GB не публикуется: проверенный путь/SHA в прежнем capture manifest; текст trace в Git.

## Следующие действия и разрешения

1. Не запускать второй стек. Проверить свежие PID/логи и passive_observer.jsonl в deployment docs directory.
2. Оператор должен отправить **новую** RViz 2D Pose Estimate фактической позы в frame map на /qazbot_d1_map/initialpose. Запрос уже отправлен. Старые initialpose из t5live1 не заменяют новый seed.
3. Измерить принятие seed, map→odom, READY/TRUSTED/MOTION_OK, T5 и свежесть. Попросить оператора поставить/убрать реальное препятствие перед неподвижным X2; проверить lethal сохраняется при наличии и очищается после подтверждённых лучей. При фактическом отказе остановить тест и rollback.
4. После пассивной проверки подготовить путь без движения. **Отдельное разрешение** необходимо для запуска MC/Motion Adapter/Goal Coordinator и физического Goal. Перед ним оператор рядом, RC/физический останов проверены, свежая initialpose/AMCL/Trust, карта D1 v3, один владелец TF, реальные препятствия видны. Скорость — по существующим измерениям торможения, не наугад.
5. После сессии остановить только owned процессы (Trust, launch, passive Costmap/observer), проверить освобождение. Известный Costmap teardown может быть не clean: сохранить exit/log, не затрагивать vendor HAL/camera. Новые данные сохранять только внутри control_backend/docs и публиковать в отдельной ветке.

Rollback **после остановки owned ROS процессов**:

```sh
python3 /tmp/i1_stage_2d240dbd/tools/deploy_i1.py --backup /home/agi/control_backend/docs/evidence/i1_deploy_2d240dbd --restore
```

Проверка воспроизводимости установки и процесса: tools/deploy_i1.py в Git 2d240dbd, deployment.json содержит before/after SHA; original/ws/src/ содержит проверенный backup на X2. Не переустанавливать T5.
