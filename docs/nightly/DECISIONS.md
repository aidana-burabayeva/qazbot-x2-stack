# Nightly — решения

## D1 (2026-10-30) CCW gate: корень — расхождение goal-checker допусков sim vs robot, усиленное дрейфом MC
Доказательства (docs/evidence/nightly_20261030/ccw_gate, phases.json):
- Twin запускал `nav_full` с дефолтами xy_tol=yaw_tol=0.10; робот — 0.20 / 0.35 (HANDOFF 2026-10-29). Контракт sim↔robot допусков не содержал.
- Центр уходит на 0.22–0.26 м при развороте на месте (`max_xy_dev_while_turning_m`), больше xy-допуска 0.10.
- RPP humble: `shouldRotateToGoalHeading` = `dist_to_goal < goal_dist_tol_`, а `goal_dist_tol_` берётся у goal checker (`getTolerances`, regulated_pure_pursuit_controller.cpp:291-294, 383). Пока XY вне допуска, RPP вращает корпус к направлению пути (wz=+0.5 насыщено, пока ошибка yaw проходит через 0), центр снова уходит — цикл (4–6 выходов из XY, 6–9 пересечений yaw).
- goal checker `stateful=false` (читается из controller_server) → вход в XY не фиксируется.
- Коэффициент выбега мал (0.005–0.12 м после последней ненулевой команды) → линейный coast НЕ причина. P20 не причина (wz отрабатывается ~0.5–0.6 рад/с).
A/B (одинаковые карта, старт, P20, 8/4 CPU, свежие контейнеры; меняется только допуск goal checker):
| условие | прогонов | SUCCEEDED+критерий | время цели |
| 0.10/0.10 | 4 (1 ранний 31.8 с + 3 новых) | 2/4 (1 FAIL goal_alignment_not_converging ×2) | 31.8–38.4 с |
| 0.20/0.35 (как на роботе) | 3 | 3/3 | 3.9–4.3 с, XY 0.11 м, yaw 0.11–0.20 рад |
Решение: twin обязан запускаться с допусками робота (ROBOT_XY_TOL/ROBOT_YAW_TOL в scripts/contract/robot.env, args xy_tol/yaw_tol в twin.launch.py). Это выравнивание конфигурации, не ослабление: измеряемая точность (XY ≤0.20, yaw ≤0.35 по S1/S2) оценивается независимо от Nav2 checker.
Риск для робота (OPEN, NEEDS_HARDWARE): измеренный дрейф при развороте на реальном X2 0.16–0.23 м при xy_tol 0.20 — на границе; та же петля возможна. Решения для кандидатов (не внедрять без отдельного A/B): stateful goal checker, поворот к goal yaw до XY-подхода, расширение xy_tol только по решению оператора.
Фиксированная регрессия: сценарий CCW (start −7.885,12.966,0.0 → +0.5 м, yaw π/2, карта main_office) в nightly как `REG_CCW_DRIFT_LOOP`; ожидание при робот-допусках: SUCCEEDED ≤10 с, 0 выходов из XY.
