# P20 / TWIN-02 — передача Claude, 2026-10-29

Рабочая ветка: `test/p20-twin02-sim-20261029`. Worktree: `/home/adn/QazBot/control_backend_p20_twin02`. База: `ab837253` (предыдущие offline-тесты поверх актуальной ветки `nav/first-goals-20261028`). Не переключать dirty worktree `/home/adn/QazBot/control_backend` и `control_backend_i1`. В `main` ничего не переносилось. Production-файлы `ws/src/navigation` и `ws/src/lio` в этой работе не изменялись; новый recovery — только кандидат для симуляции.

## Что выполнено

- Прочитан пользовательский pasted-text, правила AGENTS и Henki ROS 2 best practices. Робот не использовался. Прежние проблемы доступа Docker устранены настройкой среды; ROS работает после `source /opt/ros/humble/setup.bash`.
- P20: **18** специальных тестов `tests/motion_adapter/test_yaw_rate_loop.py`; полный выбранный набор adapter/native/TWIN-02 — **122 PASS без skip**. Ранее пропущенные ROS/native node tests теперь выполнены. Дополнительно **62 PASS** T5/READY/Trust gap regressions (2 существующих warning `invalid value encountered in cast`, сохранены).
- C++ recovery-core: 16 сочетаний 0.467/0.5/1.563/1.6 м × 0.1/40 с × подтверждённый/необъяснимый сброс; дополнительные проверки SE(2), реального перемещения за разрыв, неверных evidence/epoch/MC-state, повторной инъекции, NaN; UBSan PASS.
- TWIN-02 через **реальные DDS-сообщения**: **8 PASS**, включая 40-секундные разрывы wall clock, C++ source и C++ recovery-node. Тесты используют настоящие TrustEvaluator/AdapterCore, проверяют ноль до подтверждения сброса, до повторного Trust, до resume адаптера и свежей команды. Переякоривание без перезапуска recovery-node; максимальный скачок опубликованной позы после восстановления <3e-16 м. Необъяснимый сброс и NaN сохраняют latch.
- P20 после ходьбы: настоящий ROS adapter → официальный MC x86 v1.0.0 → MuJoCo, карта `main_office`, оба направления и baseline/P20. Результаты ниже.

| Вариант | Устойчивая скорость после ходьбы, рад/с | 90°: время до допуска, с | Ошибка после остановки, рад |
|---|---:|---:|---:|
| CCW baseline | 0.00607 (stall воспроизведён) | 8.503 | +0.16277 |
| CCW P20 | 0.50088 | 3.470 | +0.12516 |
| CW baseline | -0.20162 | не достигнуто за 10 с | -1.61967 |
| CW P20 | -0.45073 | 4.547 | -0.08026 |

Калибровочные gates P20 (≤0.35 рад, ≤10 с, CW ≥0.35 рад/с) выполнены. Это **не** равно полной квалификации S2/S3 через Nav2.

## Полная цепочка Nav2: что действительно доказано

На той же карте подняты planner/RPP/smoother/Goal Coordinator/реальный P20 adapter/MC/MuJoCo. Локализация была **truth**, AMCL/T5 здесь не квалифицировались. Две цели с ходьбой и финальным CCW/CW получили NavigateToPose **SUCCEEDED**:

- CCW: 31.765 с целиком, XY=0.09792 м, yaw=+0.01526 рад.
- CW: 6.734 с целиком, XY=0.09407 м, yaw=+0.06891 рад.
- `goal_alignment_not_converging` в полном сохранённом stack log отсутствует.

**Открытый gate:** CCW-цель сделала несколько кругов/подходов к XY до SUCCEEDED. По pose trace робот выходит из XY-допуска при повороте и возвращается. Отдельное время финального выравнивания в полной цепочке не инструментировано; **не объявлять S2/S3 full PASS или доказанные ≤10 с** по этим данным. Проверить последний XY-подход и финальный rotate отдельно, с `cmd_vel_authorized`/MC command trace. Не менять P20 и параметры контроллера/локализации/геометрию одновременно.

Первый Nav2 harness отправил вторую цель слишком рано, до `source_zero`/ready после первого SUCCEEDED: получен ожидаемый precondition reject. Это сохранённый **FAIL harness**, а не отмена первой цели. Harness исправлен: ждать свежий coordinator ready. CW затем выполнена отдельным клиентом с явно переиспользованной регистрацией той же нашей simulation session. Это **не** серия трёх последовательных goals.

## Ограничения и критерии достоверности

1. MC v1.0.0 в 5-секундном наблюдении не выдал native leg odometry (665 HAL odom, 164 TF, 0 native). Поэтому первый probe с настоящим native bridge завершился timeout. Не фабриковать native stream и не выдавать это за аппаратный native PASS.
2. Для P20 измерение — **yaw торса** из MuJoCo pelvis + суставных TF. `tests/p20_sim/torso_odom.cpp` — C++ callbacks, 30 Гц наружу, исходные stamps, намеренно мусорный twist. Это заменитель для теста регулятора, не квалификация дрейфа/позы физической грудной IMU.
3. Recovery использует независимую известную позу симуляции и явное подтверждение сброса. Переход MC сам по себе доказательством не является. **Реальный поставщик reset evidence / независимой позы и production admission wiring не реализованы.** Bool-ack в экспериментальном сервисе — граница теста, не аутентификация.
4. Экспериментальный recovery transport публикует Odometry и diagnostic gate, **не TF**; twist помечен неизвестным большой covariance. Его gate должен объединяться с настоящим свежим Trust. DDS-тесты соединяют его с настоящими decision cores, не запускают ROS Trust node/AMCL/GC. Не устанавливать кандидат на робот.
5. При 4 CPU sim был throttled ~77% периодов; первый P20 run остановлен по неправильному MC body (сохранён FAIL). Сравнение выполнено при одинаковых **8 CPU, 320×180 Xvfb, 1 llvmpipe thread**, без изменения P20 gains/guard thresholds. В начальных результатах есть failure GLFW из-за повторного start того же контейнера со stale X-состоянием; runner исправлен на `--force-recreate`, это не retry алгоритма.

## Где файлы

- `tests/motion_adapter/test_yaw_rate_loop.py`: P20 regressions; `tests/p20_sim/`: C++ yaw relay, ROS calibration runner, full-Nav2 follow-up, команды A/B.
- `candidates/native_odom_reanchor/`: C++ core, два ROS-пакета, semantic service, README. Каталог обычно ignored, нужные исходники явно включены в коммит.
- `tests/lio/test_reanchor_core.cpp`, `reanchor_ffi.cpp`, `test_twin02_trust.py`, `twin02_ros_scenario.py`: проверки core, Trust и DDS.
- `ws/src/maps/main_office/{map.pgm,map.yaml,manifest.json}`: исправленная карта; source PGM пользователя не менялся. 413052 unknown, 13712 occupied, 180955 free; `free_thresh=.196`. Nav2 map_server дал **0 unknown→free**. Карта на ПК также находится в исходном `/home/adn/QazBot/control_backend/ws/src/maps/main_office`.
- `digital_twin/`: snapshot **существующего** исходного twin-кода из локального c1880c3, синхронизированный с фактически использованными файлами. Не восстановление по догадкам и не merge production-изменений этой ветки. Из старого tracked tree изменены только compose/.env.example, добавлены map helper scripts; provenance в `twin_snapshot.json`. Vendor binaries, results/build/runtime не копировались. Старые исследовательские symlinks сохранены как reference и могут требовать отдельных локальных checkout; новые тесты от них не зависят.
- Рабочий runtime twin остаётся `/home/adn/QazBot/digital_twin`, `.env`: `TWIN_MAP=main_office`, `TWIN_MAP_DIR=../../control_backend/ws/src/maps/main_office`. Не вернуть устаревшее имя `t2_nav2`.
- `docs/evidence/p20_twin02_20261029/`: все результаты, включая failures, command/pose traces, source YAML, service/MC events, Nav2 logs, container/world manifests и SHA256 manifest.

## Команды продолжения

```bash
cd /home/adn/QazBot/control_backend_p20_twin02
git status --short --branch
source /opt/ros/humble/setup.bash
python3 -m pytest -q -p no:cacheprovider tests/motion_adapter tests/lio/test_native_odom.py tests/lio/test_native_odom_clock_gaps.py tests/lio/test_reset_scenario.py tests/lio/test_twin02_trust.py
cmake -S tests/p20_sim -B /tmp/qazbot-p20-build
cmake --build /tmp/qazbot-p20-build -j2
```

Calibration A/B: `P20_RUN_SUFFIX=$(date +%Y%m%d-%H%M%S) bash tests/p20_sim/run_ab.sh` (runner не перезаписывает evidence). В `run_ab.sh` все параметры инфраструктуры одинаковы; каждый случай стартует с новой MC session. `P20_CASES` позволяет выбрать оставшиеся случаи.

ROS recovery build/scenario: точные команды и API в `candidates/native_odom_reanchor/README.md`; существующий image `x2-twin-stack:dev` содержит compiler/colcon, сборка в `/tmp/qazbot-twin02-build`, без rebuild image.

Full Nav2 overlay: `tests/p20_sim/nav2.compose.yaml`, `P20_BACKEND` — абсолютный путь текущего worktree. Обязательно использовать этот source bind и torso relay: обычный baked image содержит старый adapter. Нужна новая simulation session, затем `tests/p20_sim/nav2_goals.py`. Папка evidence новая; `--reuse-registration` допустим только для подтверждённой нашей текущей session.

## Следующий узкий шаг

Измерить отдельно время последнего XY-подхода и final rotate полной CCW-цели, воспроизвести выход из XY-допуска при yaw. После этого решить отдельным экспериментом, относится ли оставшееся время к controller XY/stateful goal-checking, линейному coast или P20. Без новых оснований не повторять уже закрытые I1 geometry/ObstacleLayer проверки. MPPI не внедрять. T1/T5, footprint и collision detection не ослаблять. На физический X2 ничего не устанавливать и не отправлять команды.
