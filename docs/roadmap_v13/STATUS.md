# Roadmap v1.3 — журнал исполнения
Дата: 27.10.2026. Цель активна: исполнить физическую ветку PLAN_v1.3_20261027.md по acceptance gates. Последнее указание владельца: цифровой двойник полностью исключён из текущего scope.
Все новые инженерные отчёты этого цикла находятся на физическом X2 в control_backend/docs/roadmap_v13/.
Нельзя считать наличие файла внедрением или текущий READY доказательством правильной физической позы.

## Очередь и статусы
| Этап | Статус цикла | Результат / следующий проверяемый шаг |
|---|---|---|
| B0 / P0-C | IN PROGRESS | As-built snapshot создан: 5583 файлов, 12 процессов, 704 mapped path; далее независимые тесты frozen tree и классификация повреждений |
| A0 | OUT OF SCOPE | Исключён владельцем: работы цифрового двойника прекращены |
| B1 / physical scheduler A/B | NOT STARTED | Требует завершённого release freeze и отдельно согласованного controlled restart; MC не затрагивать |
| B2 / native_imu M0a | IN PROGRESS (review) | Отличать IMU-footprint нового owner от native-to-pelvis shadow; runtime по-прежнему LIO |
| A1 / x86 P2-T | OUT OF SCOPE | Исключён владельцем: работы цифрового двойника прекращены |
| A2/P2 | OUT OF SCOPE | Исключён владельцем: работы цифрового двойника прекращены |
| A3/P6 | OUT OF SCOPE | Исключён владельцем: работы цифрового двойника прекращены |
| A4/P3/P5 | OUT OF SCOPE | Исключён владельцем: работы цифрового двойника прекращены |
| B3 | NEEDS_OPERATOR | Известный физический ориентир + RViz initialpose; новый goal не отправлять |
| B4/S4 | NEEDS_SEPARATE_APPROVAL | Подтверждение RC takeover и физического stop |
| B5/M1/M2/M3 | NEEDS_SEPARATE_APPROVAL | 1 м, затем повороты, затем отдельные A→B и B→A |
| P7 | GATED | Django integration после navigation MVP, без создания второго backend |
| P8 | PARTIAL | Использовать прежние bags; полный stack replay/command sink ещё требуется |
| P10 | NEEDS_HARDWARE | Второй D1 и тот же map_id/version |
| P11 | AFTER_MVP | Terrain/semantics/новые модели — отдельная квалификация |

## Инварианты цикла
- Физические ненулевые команды, registration/arm/goals и перестановка TF-owner не разрешены общей командой /goal автоматически.
- Новая документация пишется только здесь, на роботе.
- Snapshot не доказывает Python loaded-code version; файлы disk vs процесс различаются.
- Все исходы, включая неудачи, сохраняются отдельно. Никаких массовых повторных копирований в рабочее дерево.
- Прежняя квалификация native-to-pelvis (источник: PC reports/20261027-native-odometry-qualification.md) поддерживает кандидатуру native, но зафиксировала gaps 0.65–0.79 с, gravity discrepancy ~2.33°, CPU ~42–45% ядра; это не проверка нового native_imu owner.


## Уточнение scope владельцем
Работаем только с физическим роботом и связанным с ним кодом, данными, документацией и изолированными regression tests. Twin-контейнер, запущенный в этом цикле, остановлен. Исторические результаты не удаляются и не считаются выполнением физической ветки. Sim A0/A1 больше не являются задачами текущей цели.

## Приоритет владельца: первая физическая цель RViz
Поток B1 длительного CPU A/B отложен, прежний вопрос о нём не является следующим действием. Следующая карточка — RVIZ-FIRST-GOAL.md: R0 controlled dry-run bringup native_imu с новым операторским seed, затем R1/R2/S4 и одна M1 цель. Для R0 ожидается отдельное разрешение; MC и движение пока не включены.
C2 native64 PASS; Trust113 (1 skip), adapter43, native limiter25, stop7 PASS. 11 восстановленных ELF прошли loader/list-tests проверку.

## R0 применён по явному разрешению
Новый tag rviz_native_v13_20261027_110741Z. Native owner probe PASS:30Hz, native match100%, LIO0%, без второго odomTF owner. C1/C2 теперь применены в ws/src и загружены новыми процессами; старые строки о STAGED выше относятся к истории до R0. Adapter live=false, MC registration/arm/movement не выполнялись. Следующий шаг: оператор повторяет2DPoseEstimate, затем проверка READY; подробности R0-native-dry-run.md.

## R0 поза принята
Пассивный монитор подтвердил AMCL и initialized/ready/motion_ok=true на native_imu; native30Hz/fault empty; RC/STAND/FSM4; adapterdry_run published0, coordinatorbusyfalse/gatefalse. Initialpose event мог прийти до подписки монитора; принятие доказано состоянием нового Trust и сообщением AMCL. Следующий шаг R1-S4-takeover-card.md, ожидает отдельного разрешения на live registration и одну ограниченную пробу.

### Подготовка записи S4 — 2026-10-27 11:27 UTC
Подготовлен `/tmp/qazbot_roadmap_v13/record_s4.sh` (bash -n PASS; SHA256 34698ab0c0e6a3d086f8810b1d9f31f3ae2c36808f855a130ce05fe34ddae5a4). Не запущен. Включает обе одометрии, native leg, vendor/private TF, MC velocity/state, Trust/adapter/coordinator, команды и RViz goal/seed. Для команд установлен best_effort/volatile, static TF — transient_local depth100. Без service calls и control publishers. Recorder необходимо подключить до движения и дождаться стабилизации DDS. Readiness monitor PID242279 подтверждён живым на 09:10 из лимита600с. Отдельное разрешение R1/S4 пока не получено; live registration/arm/команды не выполнялись. Scope остаётся только физический робот.

S4 result saved in S4-result-and-first-goal.md. Operator confirmed physical stop. Operator explicitly skips further RC takeover testing and requests RViz navigation trial; S4 remains NOT VERIFIED. Both controller latches retained pending new initialpose. First_RViz_goal_1135 recorder PID261795 started passively; no autonomous goal or new motion sent. New readiness_first_goal_1142 monitor added before any movement because previous 900s observer is near scheduled expiry. Existing T5 thresholds and MC bounds unchanged.

### RViz goal 02 (27.10): принята, частичное движение, Trust отменил
Nav2 linear0.3m/s, angular сначала+, затем-. База прошла~1.21м, native yaw дал разброс≈-9.5°/+12.1°, net~-1.6°. Линия к XY была практически прямо, терминальный yaw был≈41° влево; до terminal pose Nav2 не дошёл. T1_scan_stale и Coordinator отмена через~3.2с команды; оператор сообщил, что корпус не довернул. После отмены остановка подтверждена пользователем, adapter/coordinator latched. После stop анализировался bag, результаты RViz-goal02-result.md. Сейчас MC STAND, Trust fresh, adapter unarmed; coordinator снова разрешён для новой цели без cached goal replay. Навигация вокруг стола пока не квалифицирована.

Table bypass check: D1 v3 pixels на прежней линии start-goal=300/300 gray254 free; global costmap plugins static_layer+inflation only. Local scan obstacle layer служит контроллеру для проверки collision, не входит в глобальный planner. Поэтому стол, отсутствующий в карте, сам по себе не создаёт глобальный объезд: RPP может остановиться на прямой линии. Не посылать повторно ту же цель. Для автономного объезда нужно либо наличие столов в D1, либо dynamic local-to-global planning (вне текущей квалификации). После check coordinator operator_stop latched, adapter stop_requested/unarmed, MC stand.

Clarification after operator review: gray unknown quadrilateral at the office center is the table; intended objective is a static-map global route around it. Earlier goal was in a free northern corridor and did not cross table, so prior «table not mapped / planner cannot avoid it» conclusion was withdrawn. SmacPlanner2D allow_unknown=false on D1 static+inflation should route around unknown if clearance for the body exists. Need a fresh goal on far side of the unknown region to verify the route. Current preparation: MC STAND, Trust fresh/ready, adapter resumed but motion_armed=false, coordinator resumed for new goal, passive recorder PID301007 and observer PID301005 started. No new goal yet.

Static-map trial after operator requested retry: global ComputePathToPose status SUCCEEDED, 153 samples; all D1 pixel gray254/free, path goes west around table unknown. Actual RPP local collision checker blocked at goal initiation, repeated `collision ahead`; path/global planner not the refusal source. Need investigate local_costmap and plan/odom alignment. Later new initialpose hit T5 and trust is not initialized; adapter and coordinator locked. Ask operator to localize again before any next goal.

## RPP collision ahead — диагностика 27.10 (вечер)
Причина Goal01/trial03: устаревшие lethal-отметки local costmap, не очищаемые inf-лучами (inf_is_valid=false); checker не «слишком строгий». Исправление предложено, НЕ применено; частичный кандидат candidates_v13/I1_rpp_collision не проверен. Подробно: RPP-local-collision-review.md.
