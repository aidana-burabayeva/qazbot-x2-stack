# Статус проверок QazBot X2

Обновлено: 2026-10-28. Production baseline: c15dd6c.
Передача: [HANDOFF.md](HANDOFF.md). PASS действует только для указанной проверки.

## Новые результаты

| Проверка | Среда | Результат | Доказательство / граница |
| --- | --- | --- | --- |
| GitHub main и два локальных HEAD на старте | GitHub API / ПК | PASS | c15dd6c6c8fe8d5e6cc1014c68bb9da51dd8a3fa |
| Digital Twin доступен | ПК | PASS | /home/adn/QazBot/digital_twin |
| Повторная SHA256 T5 | X2 | PASS | Оба полных SHA256 в HANDOFF, переустановки не было |
| Процессы и TF ownership | X2 | PASS в проверенном snapshot | Один FAST-LIO/bridge/AMCL/native bridge/Trust; на namespace TF AMCL+native bridge, два разных TF ребра |
| Операторские initialpose | X2 LIVE | PASS | 4 seed получены AMCL/Trust; 6 AMCL poses; map → odom появился |
| Измеренные T5 corrections | X2 LIVE | PASS в stationary/reseed условиях | max XY=0.0327434893 м, yaw=0.0193353509 рад; 0 T5 failures |
| Устойчивый допуск после последнего reset | X2 LIVE | PASS | 124.9717 с / 523 статуса ready/trusted/motion_ok, reasons=[] |
| Нет T1 во всём прогоне | X2 LIVE | FAIL | 2 T1_scan_stale после первого seed; переходный T1_tf_stale на reseed |
| Нет предупреждений времени | X2 LIVE | FAIL | 4 AMCL initialpose future-extrapolation warnings, опережение последнего odom TF на 8–34 мс |
| Корректная точная локализация для навигации | X2 LIVE | НЕ ДОКАЗАНО | Go2 diagnostics covariance/particles/scan score/no baseline/no anchor сохраняются |
| Блокировка специально введённого большого скачка | X2 LIVE | НЕ ВЫПОЛНЯЛАСЬ | Не вводилась ложная физическая поза; есть прежние replay/synthetic проверки |
| Отсутствие QazBot motion stack/его command publishers | X2 | PASS в пределах graph | Motion Adapter/Coordinator отсутствовали; агент команд не отправлял |
| Все штатные velocity-команды равны нулю | X2 | НЕ ДОКАЗАНО | Native publishers существуют; velocity samples не получены; MC state=rc/STAND_DEFAULT |
| Освобождение t5live1 | X2 | PASS | Clean exit launch children, все owned PID отсутствуют; namespace publishers исчезли |
| Старые I1 unit tests | ПК / ROS Humble | PASS: 10 | Их прохождение не закрывает floor/far-lethal risk |
| Старый I1 floor coverage | ПК | FAIL модели, воспроизведён | 1 м floor → +inf в 7 бинах |
| Новый bounded I1: unit tests | ПК | PASS: 14 | tests/lio/test_i1_bounded_clearing.py, включая настоящий on_cloud callback |
| Фактический ObstacleLayer | ПК / Nav2 Humble 1.1.20 | PASS: 21 assertions / 1 CTest | raytraceFreespace + updateBounds; distant lethal, missing observation, unknown, range cap, real obstacle |
| Полная I1 source/TF/lifecycle интеграция | ПК | PENDING | C++ harness не покрывает DDS/TF/ObservationBuffer pipeline |
| Новый I1 повтор A/B | Twin | PENDING / среда BLOCKED | Docker socket permission denied; старый plant не выдаёт новый bounded source |
| Новый I1 реальный cloud_registered | X2 bag / ПК | PENDING | Actual bridge input не записан; пассивная запись требует отдельного разрешения |
| I1 на физическом X2 | X2 | НЕ УСТАНАВЛИВАЛСЯ | Только PC worktree и отдельная GitHub-ветка |

Итог T5: **stationary/reseed PASS, полный temporal acceptance FAIL**.
Итог I1: **экспериментальный кандидат; не допущен к установке и финальному review**. [Draft PR #1](https://github.com/aidana-burabayeva/qazbot-x2-stack/pull/1), отдельный commit c23ce83.

## Доказательства LIVE

[Summary и SHA256 артефактов](evidence/t5live1_summary_20261028.json),
[полный observer JSONL gzip](evidence/t5live1_observer_20261028.jsonl.gz),
[Trust log gzip](evidence/t5live1_trust_20261028.log.gz),
[localization log gzip](evidence/t5live1_localization_20261028.log.gz).
[Первый graph snapshot](evidence/t5live1_graph_20261028.json) снят до seed и до полного DDS discovery: позднее найден штатный /controller_server_ros_node на /cmd_vel. Не использовать ранний пустой publisher list как вечное доказательство отсутствия команд.

В финальном observer: 1492 Trust статуса, 723 map → odom сообщения, 4 initialpose, 6 AMCL poses, 0 error events. После последней initialpose полная эпоха содержит переходный reset verdict T1_tf_stale/not_ready; непрерывный допуск посчитан после него по событию READY, без изменения runtime thresholds.

В первом варианте собственного observer был несовместимый callback signature; он остановился до seed. Исправлено, не является дефектом production Trust. DDS discovery был медленным и teardown наблюдателей задерживался. Полное отсутствие влияния наблюдателя на timing не доказано.

## Воспроизведение I1 на ПК

Старый candidate: после source /opt/ros/humble/setup.bash, из /home/adn/QazBot:

```python
import importlib.util
import numpy as np
spec = importlib.util.spec_from_file_location(
    "i1", "digital_twin/candidates/i1_rpp_collision/level_bridge.py")
lb = importlib.util.module_from_spec(spec)
spec.loader.exec_module(lb)
covered = lb.coverage_bins(np.array([0.0]))  # floor return, distance discarded
scan = lb.build_ranges(np.array([]), np.array([]), covered)
assert np.isposinf(scan[lb.N_BINS // 2])
assert np.isinf(scan).sum() == 7
```

Новый candidate, из /home/adn/QazBot/control_backend_i1:

```sh
source /opt/ros/humble/setup.bash
python3 -m pytest -q -p no:cacheprovider tests/lio/test_i1_bounded_clearing.py
cmake -S tests/lio -B /tmp/qazbot_i1_observation_build
cmake --build /tmp/qazbot_i1_observation_build -j2
ctest --test-dir /tmp/qazbot_i1_observation_build --output-on-failure
```

14 Python tests и CTest с 21 проверкой прошли. py_compile и git diff --check прошли. Проверяется установленная библиотека ObstacleLayer, не переписанный raytracer. Наличие конечного endpoint ещё не доказывает правильность origin/deskew на реальном FAST-LIO.

## Исторические результаты — не новые прогоны

| Проверка | Ранее подтверждено | Ограничение |
| --- | --- | --- |
| T5 unit на физическом X2 | 125 passed, 4 skipped | Предыдущий регресс |
| T5 synthetic ROS E2E на X2 | BAD=0, OK=10 | Синтетический вход |
| T5 сохранённые реальные TF | 16 шагов; S4: translation 0.1019 м → robot correction 0.0396 м | Скачки 0.168–1.99 м продолжают блокироваться |
| T5 повторные initialpose на реальных TF | Проверены офлайн | Не новое LIVE свидетельство |
| T5 B2 | Принято ужесточение: yaw 0.06 рад при плече 6 м → correction 0.36 м | Пороги не изменены |
| Старые I1 twin A/B | r1/r2b/r3/f92: старый баг воспроизведён, старый кандидат устранял RPP false collision | Результаты не переносятся на новый bounded diff |
| Старые I1 сырые физические облака | 75 standing / 72 walking | lidar_pointcloud не заменяет /qazbot_lio/cloud_registered |

Источник исторических результатов: передача оператора, существующие отчёты Digital Twin и история c15dd6c. Числа разных сред не суммировать. Потерянный шаг 0.174 м из First_RViz_goal_1135 не восстанавливается.

## Открытые критерии

1. Разобрать переходные T1 и предупреждения времени; при необходимости отдельно согласовать повтор LIVE.
2. Не считать Go2 READY доказательством точности AMCL или NavigateToPose.
3. Повторить новый I1 A/B и полную интеграцию с bounded source.
4. После полного закрытия T5 согласовать пассивную запись cloud_registered и необходимый запуск сенсорного стека; проверить actual geometry на ПК.
5. Финальный review-ready PR только после этих результатов. I1 автоматически на X2 не устанавливать.
