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
