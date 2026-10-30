# Lab → Robot handoff

Сгенерировано `tools/nightly/lab.py stop` 2026-10-30T11:52:15+0500. Ручные данные: `docs/hardware/lab_robot_state.json`. Сначала verify SHA ниже через `git ls-remote`.

## Что доступно через Git

- Ветка: `test/p20-twin02-sim-20261029`, SHA на момент остановки: `33c10a1228cac7f2b005a49e60d44c1051fb55b5`
- Интеграция с production snapshot (`origin/main`): `6e70c0f90665c06dfc704162c9ddc84efce977c1`
- Тег остановки ставится после push: `lab-stop/<UTC>` (точный SHA в `docs/hardware/LAB_STATE.json` нельзя записать в тот же коммит; он в теге).

## Тесты на момент остановки

- Regression: **PASS** — 96 passed in 41.70s
- PASS: P20 yaw-rate loop + motion adapter + nightly tooling (pytest) — см. строку Regression выше; симуляция, не железо
- PASS (sim): L2 CCW-петля, допуски 0.20/0.35 (как на роботе) — 3/3 за 3.9–4.3 с; при 0.10/0.10 было 2/4. Truth-localization, AMCL не квалифицирован
- FAIL (sim): L2 s1_0046 / s1_0005 stall — NavigateToPose ABORTED, 'Failed to make progress'; контроллер командует wz≈-0.42, vx≈0.03, робот не поворачивается. Причина НЕ установлена
- UNVERIFIED: L2 выводы о контакте/зазоре по карте — frame-contract: мир симулятора и карта Nav2 расходятся до 0.7 м; все verdict по карте понижены
- UNVERIFIED: doorway_crossing_in — 6/6 SUCCEEDED, но 2/6 контакт по карте и 2/6 зазор <=0.075 м (под вопросом из-за frame-contract)
- NEEDS_HARDWARE: Проём ~0.80 м (кандидат из анализа карты) — UNVALIDATED; нужен замер рулеткой, автономно не заходить
- FAIL: make nightly end-to-end — сборка падает (vendor в worktree, DNS docker.io); не расследовано до конца

## Затронутые Nav2 / ROS 2 параметры

- goal checker робота: xy_tol=0.20, yaw_tol=0.35 (twin по умолчанию 0.10/0.10) — в лаборатории выровнено под робота, на робота НЕ переносится (там уже так)
- RPP rotate-to-heading срабатывает при dist<xy_tol; progress_checker radius 0.1 м / 15 с
- x2_motion_adapter: P20 YawRateLoop активен только при |vx| < yaw_lin_eps=0.02; sim-only guarded QAZBOT_SIM_YAW_LIN_EPS (SIM domain 87), прод-дефолт не менялся
- footprint X2 (8-угольник), T1/T5, collision detection — НЕ менялись

## Известные ошибки (открыты)

- L2 stall: контроллер командует поворот, отклика по yaw нет (s1_0046, s1_0005). Корень не найден; H2 (windup P20) опровергнута кодом; yaw_lin_eps A/B confounded frame-contract
- Недетерминизм: повтор s1_0048 даёт min-зазор 0.244 → 0.225 м
- Расхождение world/plant ↔ карта Nav2 до 0.69–0.73 м (frame-contract)
- CCW-петля: возврат в XY-допуск при повороте (дрейф 0.16–0.23 м)

## НЕЛЬЗЯ переносить на X2

- любые изменения twin-стека, QAZBOT_SIM_YAW_LIN_EPS, run/nightly, l1/L2 раннеры, tournament-результаты (L1 follower упрощён, hard-fail в нём — артефакт модели)
- любые выводы L2 по зазорам до закрытия frame-contract
- образ x2-twin-stack:p20 и overlay nav2.compose.yaml — только симуляция

## Проверки на физическом роботе (только по разрешению оператора)

- NEEDS_HARDWARE: CCW-петля при xy_tol 0.20 / yaw_tol 0.35 — замерить дрейф и число подходов
- NEEDS_HARDWARE: стопор «команда поворота без отклика» — записать cmd_vel, MC-статус, одометрию одновременно
- NEEDS_HARDWARE: ширина проёма рулеткой (кандидат ~0.80 м; координаты в docs/nightly/map/main_office_analysis.json)
- обычный порядок: T5 / I1 / отдельные разрешения оператора; движение, SSH, deploy — только по явному слову оператора

## Диагностика (команды; read-only на роботе)

    ros2 topic echo --once /tf_static  # read-only: фреймы и привязка
    ros2 param get /controller_server goal_checker.xy_goal_tolerance
    ros2 param get /controller_server goal_checker.yaw_goal_tolerance
    ros2 topic hz /cmd_vel_nav  # частота команд контроллера
    ros2 doctor --report | head -40

## Риски (TF, локализация, Nav2, MC, сенсоры)

- TF: расхождение привязки plant/world в симуляции может скрывать реальные ошибки frame; на роботе сверять map→odom→base
- Локализация: в L2 использован truth, AMCL/T5 не квалифицированы
- Nav2: допуски goal checker меняют поведение RPP rotate-to-heading
- MC: native leg odometry в MC v1.0.0 в симуляции не выдавался; поведение на железе может отличаться
- Сенсоры/LiDAR: не моделировались как реальные; freshness guards не менялись

## Ограничения

- Лаборатория не деплоит и не шлёт команд на X2. Любой результат симуляции — кандидат, не квалификация. Движение, SSH, правка конфигурации и deploy — только по отдельному разрешению оператора.
