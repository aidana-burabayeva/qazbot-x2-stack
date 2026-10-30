# Roadmap V2 / V2.1 (статус реализации)

Порядок по указанию оператора: **1) сохранность результатов → 2) автоматический handoff и восстановление после лимитов → 3) оптимизация квоты.**
Метрика эффективности: подтверждённые инженерные результаты на единицу лимита без ухудшения безопасности (воспроизведённые баги, подтверждённые первопричины, исправления, прошедшие A/B + регрессию) — НЕ процент использованной квоты.
Принцип: дорогая модель — на поиск первопричины и контракт/тест; дешёвая — на реализацию найденного исправления и прогон тестов; провал регрессии → задача возвращается сильной модели.

| Блок | Статус |
|---|---|
| V2 P0: `lab.py stop/resume/status`, skills `/nightly-stop|resume|status`, STOP→GIT→RESUME | **VERIFIED** (реальный цикл 2026-10-30, stop `5a15b62`, tag `lab-stop/20261030T065218Z`; 9 тестов `tests/nightly/test_lab.py`) |
| Остановка Codex | **STOP_UNVERIFIED** (сообщение доставлено, ack-файл не получен) |
| P0 hardening: причина `timer=failed` (symlink удалялся раньше остановки таймера) исправлена; stop не зависит от наличия codex/claude; остаточные enqueue после STOP не запускаются | **VERIFIED** (тесты) |
| V2.1 §4 Zero-loss (`tools/nightly/store.py`): evidence публикуется атомарно (tmp→fsync→rename) ДО транзакции SQLite (results + task state), adopt/quarantine после аварии, EVIDENCE_MISSING при порче, защита от ложного PASS, Execution vs Engineering checkpoint, root_causes (VERIFIED не понижается и попадает в промпт агента) | **VERIFIED** — 21 тест, аварии в 4 точках публикации + падение агента + оба агента недоступны (`tests/nightly/test_zero_loss.py`) |
| V2 P1 минимум: `agents.py` (ClaudeAdapter/CodexAdapter), владелец задачи (`tasks.owner`), один писатель на worktree (`resource wt:<cwd>`), авто-handoff при limit/lost по фактам (лог провайдера, сигнал, тишина) с контекстом из SQLite+Git, без финального сообщения агента; настоящая ошибка задачи НЕ передаётся вслепую | **VERIFIED на имитации** (стабы CLI, `tests/nightly/test_agent_handoff.py`, оба направления, оба агента потеряны). Реальный лимит провайдера не наступал: UNVERIFIED. Codex через `codex queue` — завершение наблюдается по Git/очереди, не по коду возврата: UNVERIFIED |
| Автопродолжение после завершения AI-сессии (периодический планировщик, не /loop) | основа есть (демон + таймер + PAUSED_AGENT/revive); проверка реального reset — UNVERIFIED |
| V2.1 §1–2: usage-aware пороги (80/90/95/97/99), reset ≤60 мин ждать / >60 мин передавать, `PAUSED_LIMIT`; нужны реальные quota metrics — источник НЕ подтверждён, без данных квоту не предполагать | OPEN / UNVERIFIED |
| V2.1 §3: Model Router (task_type, complexity, risk, estimated_cost; выбор model/effort; эскалация после провала), метрика «подтверждённые результаты на лимит» | OPEN |
| V2 P3 merge queue, P4 twin fidelity (FIDELITY-запросы уже генерируются из `lab_robot_state.json`) | OPEN |
| Тесты §6 V2.1 (потеря доступа при 95/97/98 %, kill без handoff, reset <60/>60, оба лимита, восстановление, model/effort, крах SQLite) | OPEN, на симулированных quota states |

Ограничения без изменений: нет deploy/SSH/движения X2, safety и допуски не ослаблять.

## P1.1 — Agent Completion Gate (2026-10-30)
| Пункт | Статус |
|---|---|
| Codex completion contract (`CODEX_COMPLETION_CONTRACT.md`): доставка ≠ выполнение, DELEGATED + фазы, результат привязан к task/attempt/attempt_id/SHA/hash | **VERIFIED на имитации** (23 теста `test_agent_handoff.py`, стабы CLI/observer) |
| Lease на worktree до подтверждённого конца, блок чужого TUI, STOP при DELEGATED | **VERIFIED на имитации** |
| Реальные источники квоты: Codex `account/rateLimits/read` (подтверждён живым вызовом), Claude — нет интерфейса (UNKNOWN) | Codex **VERIFIED (read-only)**; Claude **UNVERIFIED/UNKNOWN** |
| Допущение сброса через 3600 с | **удалено** (`reset_known=false`, только ограниченный интервал повторной пробы `NIGHTLY_LIMIT_PROBE_S`) |
| Пороги 95–99 % | **не включены** |
| Реальный E2E Claude↔Codex (настоящая сессия Codex исполняет делегированную задачу) | **OPEN**: очередь Codex не потребляется, пока сессия не активирована оператором |

## P1.2 — Real Agent Execution (2026-10-30)
| Пункт | Статус |
|---|---|
| P0 реальный E2E Claude → Codex → Claude на настоящих `claude -p` и `codex exec` (scratch-репо, worktree на попытку, orchestrator re-run тестов, независимый clone-check) | **PASS** (2 прогона по 90–94 с; `docs/evidence/nightly_20261030/p1_2_real_e2e/`). Это *плановая последовательная* передача через Git+SQLite |
| Передача, вызванная настоящим лимитом провайдера | **UNVERIFIED** (лимит не исчерпывался намеренно; проверено на имитации) |
| P1 старая очередь Codex (12 сообщений) | разобрана read-only: `CODEX_QUEUE_AUDIT.md`; не активирована, не удалена. Review Codex: **REVIEW_PENDING** |
| P2 изоляция попыток, single writer, fencing, reap | **VERIFIED** (стабы + реальные агенты) |
| P3 неблокирующая верификация (`VERIFYING`, отдельный niced процесс) | **VERIFIED** |
| P4 usage-aware: официальный `resetsAt` Codex, ожидание ≤60 мин, длиннее — handoff, критичные не ждут, оба лимитированы — `PAUSED_LIMIT` без блокировки очереди, Claude UNKNOWN | **VERIFIED на имитации квоты**; реальный сброс лимита UNVERIFIED |
| P5 Router: класс задачи → provider/model/effort, валидация по каталогу Codex, эскалация после провала gate, статистика `router_stats` | **VERIFIED** (логика, 2 реальных прогона с pin); измерение качества/стоимости на репрезентативных задачах — **OPEN** (данных мало: 6 реальных попыток, все PASS) |
| Claude модели | список недоступен в CLI → `unverified` до первого реального успеха (`claude-haiku-5-5/low` подтверждён реально; sonnet/opus не проверялись) |
