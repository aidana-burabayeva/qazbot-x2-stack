# Codex → Claude: s1_0046 one A/B завершён, 2026-10-30

Работа велась только в `/home/adn/QazBot/control_backend_codex_nightly`, ветка `nightly/codex-stall-20261030`, база b6b2b93. Прочитать [CODEX_STALL_AB.md](CODEX_STALL_AB.md), затем D3 и STATUS. Source-файлы `control_backend_p20_twin02` не редактировались. Shared runtime DB использовалась только через `nq enqueue --resource sim`; физический X2/SSH/deploy не использовались.

Ровно один A/B, epsilon=.02/.10, прочие настройки одинаковы: A ABORTED 107.94 с; B SUCCEEDED 37.60 с, XY .048 м, yaw -.041 рад. Оба задания COMPLETED (механическое выполнение), attempts=1; это не означает PASS навигации A. B не доказывает причину: A остановился в другой позе, а frame registration world↔plant существенно различалась между попытками. Fit mismatch -.091 рад в A против -.005 рад в B; в исходной записи -.106 рад. До исправления/проверки этого контракта map clearance не исключает физический контакт.

**Не брать epsilon=.10 в production.** Следующая отдельная задача — direct raw HAL/world/contact evidence и согласованное преобразование мира/plant. Ничего из геометрии/локализации здесь не менялось; дополнительных sim job-ов не ставить в рамках этого A/B. Все результаты, hashes и очередь: `docs/evidence/nightly_20261030/codex_stall_ab/`. Тесты 79 adapter + 3 audit PASS. Незакоммиченный `run/nightly/` — локальный runtime/сценарии, не часть передачи исходников.

---

# Nightly — handoff

Ветка `test/p20-twin02-sim-20261029`, worktree `/home/adn/QazBot/control_backend_p20_twin02`. Читать: SPEC.md → DECISIONS.md → STATUS.md. Робот не трогать.

Воспроизвести CCW A/B (SIM only, новые имена результатов):
```
cmake сборка инструментов: см. tests/p20_sim/README.md (torso_odom + recorder → /tmp/qazbot-p20-build, recorder в timing/)
P20_RUN=ccw_<stamp> GOAL_ARGS="--only-ccw --record-phases --xy-tol 0.20 --yaw-tol 0.35" tests/p20_sim/run_nav2_trace.sh
python3 tools/nightly/phase_analysis.py /home/adn/QazBot/digital_twin/results/<run>
```
Каждый прогон — свежие контейнеры (runner делает --force-recreate и down). Результаты не перезаписываются.
Следующий шаг: фаза B (анализ main_office: помещения, граф связности, кандидаты дверей — пометить UNVALIDATED), затем C.

## P0 Continuous Execution — состояние на 2026-10-30 ~01:40
- Оркестратор: `systemctl --user status qazbot-nightly` (Restart=always, linger=yes). Очередь: `python3 tools/nightly/nq.py status`; checkpoint: `run/nightly/checkpoint.json` (локально, не в git). Один владелец: flock `run/nightly/orchestrator.lock`.
- Агенты: Codex сессия 01a18872 держится открытыми TUI (`codex resume` pid 5123/5958) -> `exec resume` невозможен ("active writer"); канал = `codex queue --thread` (сообщение 01a1894e-… поставлено 01:16, ACK UNVERIFIED). Не создавать дублирующую сессию; не убивать TUI пользователя.
- Фазы: A, B, C, D готовы (см. STATUS). E: `tools/nightly/run_l2.sh <id> <tsv>` — один сценарий через настоящий Nav2+P20+MC; первый запуск упал на парсинге `--goals` с минусом (исправлено `--goals=`), r2 в очереди.
- Следующее: L2 doorway-серия (resource=sim, строго по одному), сравнение L1 vs L2, пересборка twin-образа (OPEN), fitness/Парето-оптимизатор, HTML-отчёт, GPU-замеры, пилот G.
- Запрещено: команды на физический X2, ssh/deploy, ослабление допусков/safety ради PASS.

## ОСТАНОВКА ПО УКАЗАНИЮ ОПЕРАТОРА — 2026-10-30 ~10:45 (физический робот снова доступен)
**Состояние: лаборатория остановлена, ничего не выполняется.**
- systemd: `qazbot-nightly.service` и `qazbot-nightly-enqueue.timer` отключены и остановлены (symlink в ~/.config/systemd/user удалены; unit-файлы лежат в `tools/nightly/systemd/`). Cron `/loop` (1ч, 2ч) удалены. Docker-проект x2-twin остановлен (`docker ps | grep x2-twin` пусто).
- Очередь SQLite (`run/nightly/queue.db`, локально, не в git) сохранена; незавершённые задачи припаркованы `NEEDS_APPROVAL` (reason `OPERATOR STOP`). `make_nightly_20261030` и `_r2` упали на сборке (нет vendor в worktree / временный DNS-отказ docker.io) — NEEDS_ENGINEERING, не успешный nightly.
- Codex 01a18872: послано сообщение `OPERATOR STOP` (queue 01a18b53-7a77…, ответ `STOPPED <sha>` ожидается; его последний принятый результат — коммит e54067f, смёржен). Его TUI не завершались мной. Задача FRAME CONTRACT (queue 01a18b51-7d86…) поставлена ДО стопа — Codex её по указанию оператора выполнять не должен.
- Ветка: `test/p20-twin02-sim-20261029`; `run/` убран из git и добавлен в .gitignore (по ошибке был закоммичен merge-коммитом 003e8be; секретов в нём не найдено; baseline сохранён в docs/evidence/nightly_20261030/baseline).

### Возобновление (только по просьбе оператора)
```
cd /home/adn/QazBot/control_backend_p20_twin02
ln -sf $PWD/tools/nightly/systemd/* ~/.config/systemd/user/ && systemctl --user daemon-reload
systemctl --user enable --now qazbot-nightly.service qazbot-nightly-enqueue.timer
python3 tools/nightly/nq.py status        # припаркованные задачи вернуть: nq.py mark <name> PENDING
```
Для `make nightly` в worktree нужен `digital_twin/vendor/*` (локальные symlink на /home/adn/QazBot/digital_twin/vendor) и сеть к docker.io.

### Подготовка к работе с физическим роботом (ТОЛЬКО оператор; агенты X2 не трогают)
Всё ниже — NEEDS_HARDWARE, ничего из симуляции не доказывает поведение робота:
1. **CCW-петля финального выравнивания.** В sim корень найден (допуски 0.10/0.10 vs робот 0.20/0.35, центр уходит 0.22–0.26 м при развороте). На роботе дрейф 0.16–0.23 м у границы xy_tol 0.20 — петля возможна. Проверка: тот же CCW-сценарий (+0.5 м, yaw π/2) с записью cmd_vel_nav/authorized/MC; критерий — SUCCEEDED ≤10 с без повторных выходов из XY. Регрессия `REG_CCW_DRIFT_LOOP` (DECISIONS D1).
2. **Проём офиса ~0.80 м** (кандидат UNVALIDATED, карта (-6.49,-6.93)): замерить рулеткой реальную ширину до любых испытаний; ширина робота 0.712 м, запас ≈0.04 м/сторона. Не заходить в проём автономно, пока ширина и зазор не подтверждены.
3. **Стопор «команда поворота без отклика»** (L2 s1_0046/s1_0005): причина не установлена; возможное расхождение карта↔мир sim до 0.7 м (Codex D3). Не переносить выводы L2 на робота.
4. Допуски goal checker: на роботе `xy_tol:=0.20 yaw_tol:=0.35` (контракт `scripts/contract/robot.env`); ослаблять нельзя.
5. Обычный порядок из CLAUDE.md/HANDOFF.md: T5 установлен, I1 E2E и review отдельно, любой старт сенсорного стека/движение/установка — только с разрешением оператора.
