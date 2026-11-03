# RPP «collision ahead» — диагностика (27.10.2026), исправление НЕ применено
Scope: только физический X2, офлайн-анализ bags/costmap snapshots. Движения, цели, рестарты, service calls не выполнялись.
Evidence: evidence/RPP_local_collision_20261027/ (отчёты D1, D2, офлайн-чекер RPP, скрипты replay).

## Вердикт
Гипотеза «collision checker слишком строгий» НЕ подтверждена. RPP 1.1.18 срабатывает только на cost254 (lethal) по контуру footprint; inflation/inscribed253 его не вызывает.
Причина обоих отказов — устаревшие lethal-отметки local costmap, которые скан не очищает.
- level_bridge.py выдаёт inf для лучей без отражения; obstacle_layer.scan.inf_is_valid не задан (false) → такие лучи не raytrace-clear, отметки в этих bearings живут бесконечно.
- Goal01 (класс b, ДОКАЗАНО replay): первый цикл контроллера, робот неподвижен, 7 нулевых команд. Офлайн-чекер на local_costmap_before_clear.npy при v=0.3 попадает в lethal 0.82м впереди (-3.6°/-11.3°). Bins -14..-2° дали 0/1208 возвратов за 30с (все inf). Ближайший реальный возврат ≥1.5м.
- Trial03 (класс b, сильно поддержано, происхождение отметки не доказуемо — пробел записи 3856–4200): при старте шага реальный доворот корпуса ~10° вправо (native odom и LIO yaw совпадают в 0.4° — это не ошибка odom). Проекция footprint RPP попала в отметку ~0.6м на bearing -50..-62°, сразу за правым краем FOV. Добавление одной stale-клетки на -50° воспроизводит live onset в 301.84.

## Безопасность предлагаемого стирания
- Слепой зоны впереди за RANGE_MIN 0.45м нет (0.5–0.7м на ±45–50°); препятствие в высоте среза не выпадает из скана раньше, чем войдёт в радиус робота.
- Препятствия ниже ~0.34+0.035·r м скан не видит вообще — существующий gap, исправление его не меняет.
- Replay: inf-clear только внутри FOV (-46..+44.5°) + guard ±2 bins у реальных hits → все 5 проверок v=0.3 для Goal01 свободны; синтетическое препятствие 0.6–0.8м остаётся lethal 48/48.
- Inf-clear по всем bins (без FOV-маски) стирает lethal 239→15, включая ненаблюдаемый тыл/бока — НЕБЕЗОПАСНО, отвергнуто.

## Предложенное исправление (не применено)
1. level_bridge.py: bins = NaN по умолчанию; inf только для покрытых облаком bearings внутри FOV; np.fmin.at вместо np.minimum.at; NaN для inf в ±2 bins от реального hit. Маска и guard — параметры.
2. local costmap: obstacle_layer.scan.inf_is_valid: true; raytrace_max_range 3.5 и obstacle_max_range 3.0 без изменений.
3. BT: FollowPath в RecoveryNode (1–2 retries) с ClearEntireCostmap local_costmap — нужен для случая trial03 (отметка вне FOV).
Не менять: robot_radius 0.454, padding 0.01, use_collision_detection, failure_tolerance.

## Состояние реализации
Реализация I1 остановлена владельцем на середине. Частичные кандидаты в control_backend/candidates_v13/I1_rpp_collision/ (level_bridge.py, BT xml, nav launch, тест) — НЕ проверены, manifest/тесты не завершены, к ws/src не применялись. Live-дерево не менялось.

## Открытые вопросы
- Проверить по bag /cloud_registered, что маска «покрытых bearings» охватывает FOV; иначе фиксированная -46..+44.5°.
- Replay приближённый (raytrace, поведение Nav2 inf→range_max−ε по памяти, исходников Nav2 на диске нет).
- Слепая зона измерена стоя, не в ходьбе.
- Отдельно: ~10° рывок вправо при старте шага MC.
