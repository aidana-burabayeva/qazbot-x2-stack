# P26 20261030-loc-integrity — целостность локализации (пункты 1–2 ревью, одобрены оператором)

Статус: РАЗВЁРНУТО, робот стоит. **Нужен initialpose от оператора.** Цели и initialpose агент не отправлял.
Юнит-тесты не писались (правило оператора). Проверка: py_compile, синтетическая проверка `_verdict` в одну команду, live read-back.

## Изменения (каждое значение — ROS-параметр)
A) Trust (`loc_trust.py`, `localization_trust_node.py`; параметры заданы в start.sh, start_loc.sh, restart_trust.sh)
- T5 `jump_xy_max` 1.0→**0.30**, `jump_yaw_max` 0.5→**0.25**. Реакция не изменилась: в go2 T5 сбрасывает READY, GC/адаптер отменяют цель и ставят latch.
  Ревью предлагало вместо этого PAUSE, но в задаче это не просили, поэтому не реализовано.
- `amcl_update_min_d/a` 0.20/0.15 синхронизированы с AMCL. В go2 это влияет только на диагностику.
- Новая **Q-пауза** (только go2, только при READY): `sigma_xy > q_sigma_xy_max(0.35)` ИЛИ `second_mode_weight > q_second_mode_max(0.30)`
  ИЛИ `scan_score < q_scan_score_min(0.20)` дольше `q_persist_s(1.0)`. Тогда причина `Q_localization_quality`, motion_ok=False,
  `t1_transient=True` и новое поле `quality_pause=True`. В метриках появились `quality_bad` и `quality_bad_s`. Если какой-то метрики нет, по ней решение не принимается.
  Возобновление происходит сразу, как только качество вернулось. Выключатель: `quality_pause`.
- **Seed gate для READY** (`ready_gate`): после initialpose READY выставляется только при одновременном выполнении условий `sigma_xy<=0.25`, `2nd<=0.15`, `score>=0.35`,
  при наличии всех трёх метрик и без блокирующих причин. Пока условия не выполнены, `ready_cause=awaiting_seed_quality`.
  Флаг latched до следующего initialpose или T5/T8. Узел сбрасывает старое облако частиц на initialpose, чтобы gate оценивал облако после seed.
- GC (`goal_coordinator.py`): новый параметр `quality_pause_max_s`=10.0. Если статус свежий, `t1_transient` и `quality_pause` равны True, применяется своя граница,
  после неё trip `localization_quality_prolonged` (cancel+latch). Для T1 осталась граница 3.0 с.
- Адаптер (`x2_motion_adapter.py`): `quality_pause_max_s`=10.0. `_pause_max()` передаёт границу в `core.step(pause_max_s=)`.
- Совместимость: если GC или адаптер не знает про `quality_pause`, Q для него выглядит как T1 и он отменяет цель через 3 с. Такой вариант консервативен.

B) `seed_loop.py`: `NOMOTION_N=2` (было 12). Перезапущен.
   Trust по-прежнему делает свой nomotion (`go2_nomotion_on_initialpose`, обычно 1 попытка, максимум 3).

C) AMCL (`localization.launch.py`): likelihood_field_prob, do_beamskip true, 0.5/0.3/0.9, update_min_d 0.20, update_min_a 0.15,
   resample_interval 2. Без изменений: max_beams 180, alpha .3/.2/.15/.15/.15, recovery 0, z_hit/z_rand 0.5/0.5. Локализация перезапущена.

D) Адаптер: строки 352/354, `abs(wz) > yaw_w_min + 1e-6` (и `stopped` с тем же eps). Перезапуск выполнен через p20_restart_adapter.sh:
   DELETE code 0, затем ADD, статус registered.

## Развёртывание (CST)
1. 18:37 `restart_loc.sh` (новый): seed_loop + Trust + launch локализации. INT, затем TERM; nav_full и адаптер не трогались.
   Ожидание пайпа из-за фонового seed_loop исправлено в скрипте.
2. 18:45 `restart_gc.sh`: перезапуск nav_full (иначе GC не перезапустить), xy_tol 0.35 как раньше. Скрипт вернул rc=6 «not active after 120 s».
   Причина: costmap ждёт TF map→base_footprint, а AMCL после рестарта ещё без позы. Все процессы живы.
   controller_server и velocity_smoother уже **active**, planner_server и bt_navigator активируются после initialpose. owned_nav_pids.txt записан вручную.
3. 18:48 `p20_restart_adapter.sh`: новый pid 393827, sha 772c0396.

## Live-проверка (18:50)
- AMCL read-back: все значения C подтверждены. Trust read-back: T5 0.3/0.25, все q_* и ready_*, policy go2.
- GC: quality_pause_max_s=10.0, busy=false, latched=false. Адаптер: quality_pause_max_s=10.0, t1 3.0, idle, latch "", registered.
  motion_armed=false, GC взводит его на цели.
- Trust status: поля `quality_pause` и `ready_cause` присутствуют. Сейчас статус `not_ready:awaiting_initialpose_and_amcl` (ожидаемо).
- В trust, adapter, nav_full и localization логах нет Traceback. seed_loop, status_logger и goal_watch живы. standstill_refine не запускался.
- **Не проверено live** (нужен initialpose или движение): переход awaiting_seed_quality→READY, фактическое срабатывание Q-паузы,
  активация planner/bt («Managed nodes are active» в nav_full.log), поведение beamskip.

## После initialpose оператора
- Если seed_loop печатает `ready False ... awaiting_seed_quality`, поза плохая. Нужен новый initialpose (или короткий проезд, по ревью).
- Пороги взяты из одной сессии. Если Q-пауза срабатывает часто, крутить q_* в start_loc.sh/restart_trust.sh. Это требует перезапуска Trust (параметры читаются на старте).

## Файлы и откат
- Бэкапы `*.bak_20261030_pre_p26`: ws/src/navigation {loc_trust, localization_trust_node, goal_coordinator, x2_motion_adapter, localization.launch}.py;
  $EV {seed_loop.py, start.sh, start_loc.sh, restart_trust.sh}; зеркало qazbot-x2-stack {loc_trust, localization_trust_node, goal_coordinator, localization.launch}.py.
- В зеркало скопированы 4 файла: их версии до изменений были байт-идентичны. **x2_motion_adapter.py НЕ зеркалирован**: он уже отличался до P26.
- ОТКАТ: `CONFIRM=1 $EV/rollback_p26.sh`. Скрипт восстанавливает 13 файлов и перезапускает restart_loc.sh, restart_gc.sh, p20_restart_adapter.sh.
  Есть режим `--files-only`. Прогон `--dry-run` выполнен, действия распечатаны.
- Новые скрипты: $EV/restart_loc.sh, $EV/rollback_p26.sh. Логи: $EV/p26_restart_gc.out, $EV/p26_restart_adapter.out, start_run.txt, operator_actions.txt.
