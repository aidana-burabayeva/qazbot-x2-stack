# Roadmap V2 / V2.1 (статус реализации)

Порядок по указанию оператора: **1) сохранность результатов → 2) автоматический handoff и восстановление после лимитов → 3) оптимизация квоты.**
Метрика эффективности: подтверждённые инженерные результаты на единицу лимита без ухудшения безопасности (воспроизведённые баги, подтверждённые первопричины, исправления, прошедшие A/B + регрессию) — НЕ процент использованной квоты.
Принцип: дорогая модель — на поиск первопричины и контракт/тест; дешёвая — на реализацию найденного исправления и прогон тестов; провал регрессии → задача возвращается сильной модели.

| Блок | Статус |
|---|---|
| V2 P0: `lab.py stop/resume/status`, skills `/nightly-stop|resume|status`, STOP→GIT→RESUME | **VERIFIED** (реальный цикл 2026-10-30, stop `5a15b62`, tag `lab-stop/20261030T065218Z`; 9 тестов `tests/nightly/test_lab.py`) |
| Остановка Codex | **STOP_UNVERIFIED** (сообщение доставлено, ack-файл не получен) |
| V2.1 §4 Zero-loss: таблица results (run_id, seed, SHA, map/config sha, метрики, evidence hash), запись артефакта ДО COMPLETED, agent checkpoint | **OPEN** — следующий шаг (P1) |
| V2 P1 / V2.1 §5: ClaudeAdapter/CodexAdapter, auto-handoff из SQLite+Git без последнего сообщения агента | OPEN |
| V2.1 §1–2: usage-aware пороги (80/90/95/97/99), reset ≤60 мин ждать / >60 мин передавать, `PAUSED_LIMIT`; нужны реальные quota metrics — источник НЕ подтверждён, без данных квоту не предполагать | OPEN / UNVERIFIED |
| V2.1 §3: Model Router (task_type, complexity, risk, estimated_cost; выбор model/effort; эскалация после провала), метрика «подтверждённые результаты на лимит» | OPEN |
| V2 P3 merge queue, P4 twin fidelity (FIDELITY-запросы уже генерируются из `lab_robot_state.json`) | OPEN |
| Тесты §6 V2.1 (потеря доступа при 95/97/98 %, kill без handoff, reset <60/>60, оба лимита, восстановление, model/effort, крах SQLite) | OPEN, на симулированных quota states |

Ограничения без изменений: нет deploy/SSH/движения X2, safety и допуски не ослаблять.
