# Codex: один A/B s1_0046, 2026-10-30 (SIM ONLY)

Ветка `nightly/codex-stall-20261030`, worktree `/home/adn/QazBot/control_backend_codex_nightly`, база `b6b2b93`. Source-файлы соседнего worktree не редактировались. Ни SSH, ни physical X2, ни deployment не использовались.

## Эксперимент

Сценарий сгенерирован ровно указанной командой `scenario_gen.py --seed 1 --per-category 20`: `s1_0046`, corridor_to_wing, старт `(-8.535,-15.434,0.002)`, цель `(-13.385,-4.434,-2.657)`. Оба запуска — существующий `run_l2.sh`, свежие контейнеры, одна очередь и `resource=sim`. Между A и B очередь выполняла другие задания Claude; вмешательства в них не было. Два arm-а выполнились по одному разу, без retries; времена не перекрываются (`queue_receipt.json`).

Единственный изменяемый **параметр** — `yaw_lin_eps` (A 0.02; B 0.10). Экспериментальная переменная `P20_YAW_LIN_EPS` передаётся через test compose как `QAZBOT_SIM_YAW_LIN_EPS`; loader требует QAZBOT_SIM=1, TWIN_SANDBOX=1, ROS_DOMAIN_ID=87 и конечное значение 0.005..0.10. Без fixture default остаётся 0.02. `_yaw_loop` не изменён: vx не обнуляется и не масштабируется; меняется только условие включения yaw-loop. Значение подтверждено в реальных adapter status обоих arm-ов.

Исходники, сценарий и controller readback совпадают (автоматические assertions в `analyze_stall_ab.py`): XY=0.20, yaw=0.35, stateful=false, rotate-to-heading=true, angular cap=0.5, collision detection=true. Footprint, costmap, Trust, MC bounds, gains и anti-windup не менялись. SHA исходников, карты и бинарников инструментов сохранены; бинарники были собраны до обоих запусков.

| Arm | yaw_lin_eps | NavigateToPose | Длительность action, с | XY после settle, м | yaw после settle, рад | `Failed to make progress` |
|---|---:|---|---:|---:|---:|---:|
| A | 0.02 | ABORTED | 107.937 | 4.4116 | -1.8483 | 5 |
| B | 0.10 | SUCCEEDED | 37.597 | 0.0484 | -0.0412 | 0 |

A: остановка произошла около `(-10.06,-7.30,1.67)`, а не в исходной позе `(-9.93,-8.73,2.3)`. В первом 10-секундном stationary window MC получал vx=0.30, wz=-0.202, но pose yaw rate был +0.0049 рад/с и dxy=0.0033 м. С 60 с до конца yaw rate +0.00039 рад/с. Это другой stall, не точное воспроизведение эпизода с малым vx.

B: 10-секундных commanded-negative-rotation/stationary окон по объявленному критерию не найдено. Доля `yaw_loop/published` за всю цель A 5.79%, B 12.66%; эти величины включают всю ходьбу и не являются локальной причинной оценкой.

## Что установлено, а что нет

1. Командная цепочка передаёт wz: в исходной записи в окне 60–130 с Nav2 даёт средний wz=-0.410, authorized=-0.409, MC=-0.491, но pose-derived yaw rate=-0.0027 рад/с. Это не потеря команд на пути до MC. P20 anti-windup уже ограничен кодом (D2), H2 «неограниченный интегратор» не подтверждается.
2. Условие H3 действительно существует: при |vx|>=0.02 yaw-loop обходится и интегратор сбрасывается; при 0.10 остаётся активным для части малых смешанных команд. Но увеличение порога **не делает поворот чистым** — vx сохранён. В исходном окне около 70% authorized samples уже имеют |vx|<0.02, поэтому «yaw-loop всегда выключен» тоже неверно.
3. Успех B при единственном изменённом параметре — полезное наблюдение, **не доказательство исправления MC**. В A не воспроизвёлся тот же локальный stall; кроме того, есть крупный неконтролируемый frame confound, ниже. Один A/B не даёт повторяемости.

## Подтверждённое расхождение map ↔ generated world

`world.py` размещает стены с фиксированным преобразованием `map_xy = R(start_yaw) * sim_xy + start_xy`. В то же время `HalPlantNode` использует `FrameAligner(start)`: `start * inverse(first_HAL_pose) * current_HAL_pose`. Если первая HAL-поза после stand имеет ненулевой yaw/XY, эти преобразования различаются. Это исходное поведение; в эксперименте оно не менялось.

Независимый offline fit из raw MuJoCo/FK torso XY (`kind=torso`) в published plant/pelvis XY (`kind=truth`) даёт:

| Данные | Разность угла fit и world-контракта, рад | Fit RMSE, м | Расхождение точек в конце, м | Distance to occupied: nav / raw torso в world, м |
|---|---:|---:|---:|---:|
| исходный stall s1_0046 | -0.10581 | 0.00466 | 0.6919 | 0.873 / 0.206 |
| A | -0.09101 | 0.00497 | 0.7289 | 0.632 / 0.150 |
| B | -0.00523 | 0.00542 | 0.0838 | 1.600 / 1.550 |

Это **разные body points** (torso против pelvis), поэтому RMSE и ограничение явно сохранены; значения не названы точной позой native IMU. Однако систематическое расхождение, растущее с пройденным расстоянием, согласуется с двумя разными преобразованиями из кода. Диагностические тесты подтверждают: начальный HAL yaw=0.1 рад даёт расхождение ~0.70 м после 7 м; при нулевой первой позе оба преобразования совпадают.

**Следствие:** свободное место вокруг published nav pose НЕ исключает контакт физического тела со стеной MuJoCo. Нулевая реакция plant может быть контактной блокировкой; реальных contact forces/пар geom в записи нет, поэтому контакт как физическая причина **не доказан**. В B существенно меньше frame error, чем в A, — нельзя отделить эффект epsilon от различий фактической привязки тела/мира по одной паре.

## Решение и следующий шаг для Claude

Оставить `eng_l2_doorway_stall` в NEEDS_ENGINEERING. Не повышать production default yaw_lin_eps до 0.10 по этому результату. Сначала отдельной задачей проверить frame contract по прямой raw HAL pose и данным контактов; затем согласовать map/world преобразования и только после этого сравнивать MC/P20 на одинаковой физической позе. Здесь ни alignment, ни геометрия не исправлялись: это нарушило бы условие одного A/B.

Ровно два sim job-а уже израсходованы. Дополнительных симуляций этот task не запускает. Runtime queue остаётся под управлением Claude; чужие процессы не остановлены.

## Файлы и воспроизведение

Evidence: `docs/evidence/nightly_20261030/codex_stall_ab/` — A/B result, raw motion gzip, status, stack/runner logs, hashes, manifests, queue receipt, comparison и три frame audit. Старые данные не перезаписывались.

- `tools/nightly/run_stall_ab_case.py` — wrapper существующего L2 runner, только через `nq enqueue ... --resource sim`.
- `tools/nightly/analyze_stall_ab.py` — read-only сравнение с проверками инвариантов.
- `tools/nightly/audit_stall_frames.py` — offline fit/контракт мировых координат, без управления ROS/симулятором.
- `tests/motion_adapter/test_sim_yaw_lin_eps.py` — fixture guard и сохранение vx; adapter suite: **79 PASS**.
- `tests/nightly/test_stall_frame_audit.py` — fit и алгебра frame contract: **3 PASS**.

```bash
python3 tools/nightly/analyze_stall_ab.py docs/evidence/nightly_20261030/codex_stall_ab --out /tmp/stall_comparison.json
python3 tools/nightly/audit_stall_frames.py docs/evidence/nightly_20261030/l2_stall_s1_0046 --start=-8.535,-15.434,0.002 --out /tmp/historical_frame_audit.json
python3 -m pytest -q -p no:cacheprovider tests/motion_adapter tests/nightly/test_stall_frame_audit.py
```
