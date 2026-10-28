# QazBot X2: продолжение к первому физическому RViz Goal

Обновлено 2026-10-28, финальная проверка X2 18:56:05 +08:00. Начать с этой передачи и AGENTS.md, не с нового аудита. Первый физический NavigateToPose SUCCEEDED **пока не подтверждён**.

## Активная задача и Git

Оператор одобрил diff PR #1 и **временную пассивную установку I1 2d240dbd**. Установка и пассивный запуск выполнены, затем **LIVE freshness FAIL → проверенный rollback**. Наши localization/Trust/Costmap и наблюдатели остановлены. Оператор задал новую initialpose, AMCL принял её; READY/TRUSTED/MOTION_OK достигнуты, но допуск прерывался из-за T1. Стульчик оператор поставил, запрос убрать ещё НЕ давался; удаление не проверено. **Сейчас действий оператора не требуется. Ждать подтверждения каждого запрошенного действия прежде следующего шага.** MC, Motion Adapter, Goal Coordinator, goals и движение НЕ разрешены и агентом не запускались.

- main: 6e70c0f90665c06dfc704162c9ddc84efce977c1 (документация поверх production baseline c15dd6c6c8fe8d5e6cc1014c68bb9da51dd8a3fa).
- I1 / PR #1: **2d240dbd09ca69de87e9ac4b1acde735293ad6b2**, https://github.com/aidana-burabayeva/qazbot-x2-stack/pull/1. Код I1 не merged в main.
- Передача и данные: ветка handoff/first-rviz-goal-20261028; начинать с её опубликованного HEAD. Точный SHA будущего docs-коммита определяет Git, а не самоссылка внутри документа.
- Локальные checkout HEAD c15dd6c; GitHub API-коммиты не перемещают локальные refs. Чужие изменения сохранены: docs/X2_QUICKSTART_BRINGUP_RU.md, tests/loc_trust/run_ros_synthetic.py, tests/nav_e2e/{loc_model_fixture.py,run_suite_real.py}.
- ПК: /home/adn/QazBot/control_backend и control_backend_i1. Digital Twin доступен /home/adn/QazBot/digital_twin, пока вне Git. Не восстанавливать отсутствующие исходники по догадке.
- X2 agi@192.168.1.72: /home/agi/control_backend — deployed-каталог без .git; не git init/reset поверх runtime.

## Испытанная версия и текущий rollback X2

Временно ставились только два production-файла из 2d240dbd, исходный backup проверен. После отказа свежести возвращены baseline SHA256: level_bridge.py 86c434ce0ee5a48de1a324f7a3eb5e2136f8d9d3f1b91b582f6fa522707be686; nav_control.launch.py ebea6637578530f0b30be6dd5d720ef1011270ce2a4df902d2bd1e1b5c266bec. Повторная SHA-проверка PASS. **I1 сейчас не установлен.** Manifest: `/home/agi/control_backend/docs/evidence/i1_deploy_2d240dbd/deployment.json` (копия в evidence). Manifest теперь ROLLED_BACK; пассивный результат FAIL_LIVE_FRESHNESS_ROLLED_BACK фиксирует passive_snapshot.json.

| Файл | SHA256 испытанной версии (I1 впоследствии откачен) |
| --- | --- |
| ws/src/lio/level_bridge.py | c817a5ed503386be742fa7b6fdab2b3209bdefaf42be799d2a130cd198f9369b |
| ws/src/navigation/nav_control.launch.py | c4e46506069481f3cebd3fef38af7e83bac858f9d6d4e344edeb657a137de38e |
| ws/src/navigation/loc_trust.py (без переустановки) | 4b81f0721b13e510f4da05b7494177ad8a2dbdd104d56bf12d3ce6cd786f59e8 |
| ws/src/navigation/localization_trust_node.py (без изменения) | 6b36ea0efdba0ca16344e62771500527b2e2b580f53122535b0461564bb35a46 |

UDP-only process-local profile dds_udp_only.xml: 29d4c736a364691689ccc8f27c62f566c65af946df1b6fc9dd5ec03ae239ec63; включает develop0 и wifi0 для RViz. Vendor/global DDS XML не изменялись. После `source ws/src/navigation/env.sh` обязательно переопределять FASTRTPS_DEFAULT_PROFILES_FILE, ROS_DOMAIN_ID=0, ROS_LOCALHOST_ONLY=0.

## Последние процессы и пассивные данные (запуск остановлен)

Сессия i1_deploy_2d240dbd, PID snapshot: launch 287375, FAST-LIO 287381, level_bridge 287406 publish_tf=false, AMCL 287436, native bridge 287488 native_imu publish_tf=true, Trust 287803, observer 288603, standalone Costmap 288900. Позднее observer заменён облегчённым PID295474, отдельный obstacle probe PID293649. **Все эти owned процессы уже остановлены**; финальная проверка 18:56:05: нет QazBot FAST-LIO/bridge/native/AMCL/Trust/Costmap/probes. Штатный HAL204720 оставлен работающим, камера не тронута. Не считать старые PID текущим состоянием. map_server/lifecycle также входят в launch.

Проверено: свежие cloud_registered, scan, scan_obstacles, scan_clearing; по одному publisher. Оба новых PointCloud2 в base_footprint. Native status fault/latched пусты; odom→base_footprint от native bridge. Namespace TF endpoints — native bridge и AMCL, разных рёбер. Новая initialpose принята AMCL, map→odom появился; Go2 Trust достиг READY/TRUSTED/MOTION_OK. Seed-time future-TF warning 15.635 мс сохранился. **После seed 1356 статусов, 1301 допущенный и 55 блокированных**: T1_scan_stale 42, T1_tf_stale 8, T1_odom_message_stale 7, T1_odom_stale 7 (причины могут совпадать). Max input ages scan 1.207120 с, odom TF .650090 с, odom message .731287 с. T5=0; непрерывный READY на всю эпоху НЕ заявлять. Смена observer на облегчённый не убрала повторение scan stale.

Пассивный Costmap /costmap/costmap configured/active, реальные источники scan_obstacles marking и scan_clearing clearing; 4×4 м, 0.05 м, radius .454 + padding .01, штатные 3/3.5 м, inflation .464. Это standalone ObstacleLayer, без controller/RPP action. Оператор поставил стульчик в 1 м от самой передней части ног/робота. В probe ROI x=.55…1.45, |y|<.3 м относительно base_footprint количество lethal выросло с 0 до 55 в последней записи (max99). Новые marking returns присутствуют. Это наблюдение согласуется с постановкой препятствия; классификация стул/оператор не выполнялась. Удаление/clearing после удаления НЕ проверено: агент остановил тест из-за freshness, пока оператор выполнял постановку. Оператор указал впредь обязательно ждать выполнения каждого запрошенного действия; не выдавать саму задержку ответа за отказ проверки.

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

1. Не запускать второй стек. Сначала прочитать новые freshness_events.json и freshness_same_stamp_analysis.json. В одном совпадающем header stamp cloud_registered уже пришёл observer с age .447639 с, scan с .474864 с; разница callback receipts .027405 с. В других примерах cloud тоже задержан. Это **не доказывает причину HAL/LIO/DDS**, но не позволяет приписать весь отказ bounded-clearing математике.
2. Следующий узкий эксперимент: одной существующей age_probe.py измерить raw cloud/IMU → cloud_registered/odometry → scan → native odom/Trust при одинаковой конфигурации и минимуме observers; сравнить single-interface capture UDP profile с двухинтерфейсным navigation profile. Предыдущий capture имел cloud age p95 .066521/max .092492 с, но нагрузка/профиль отличались, поэтому это не контролируемый A/B. Не ослаблять T1 и не менять clearing по догадке. Новая production-правка требует review и разрешения; не переустанавливать откаченный кандидат автоматически.
3. После устранения измеренной задержки повторить пассивную интеграцию: новая операторская initialpose, препятствие сохранить/убрать по отдельным последовательным запросам. **После каждого запроса дождаться подтверждения оператора**, затем измерить. Сейчас оператору ничего выполнять не нужно. При отказе — остановить испытание и rollback, объяснив причину отдельно от невыполненного операторского шага.
4. После пассивной проверки подготовить путь без движения. **Отдельное разрешение** необходимо для запуска MC/Motion Adapter/Goal Coordinator и физического Goal. Перед ним оператор рядом, RC/физический останов проверены, свежая initialpose/AMCL/Trust, карта D1 v3, один владелец TF, реальные препятствия видны. Скорость — по существующим измерениям торможения, не наугад.
5. В этой сессии owned процессы уже остановлены. При следующей сессии остановить только owned процессы (Trust, launch, passive Costmap/observer), проверить освобождение. Известный Costmap teardown может быть не clean: сохранить exit/log, не затрагивать vendor HAL/camera. Новые данные сохранять только внутри control_backend/docs и публиковать в отдельной ветке.

Rollback **после остановки owned ROS процессов**:

```sh
python3 /tmp/i1_stage_2d240dbd/tools/deploy_i1.py --backup /home/agi/control_backend/docs/evidence/i1_deploy_2d240dbd --restore
```

Проверка воспроизводимости установки и процесса: tools/deploy_i1.py в Git 2d240dbd, deployment.json содержит before/after SHA; original/ws/src/ содержит проверенный backup на X2. Не переустанавливать T5.

Новые итоговые архивы и SHA256: evidence/i1_deploy_2d240dbd/archive_manifest.json. SIGINT для nohup launch не завершил два Python child; после проверки собственных PID завершены через TERM. Standalone Costmap снова выдал terminate called without active exception; журнал сохранён. Ни один чужой/vendor процесс не остановлен. Записи сохранены в Git, а не только в dirty файлах.
