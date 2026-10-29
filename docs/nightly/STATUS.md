# Nightly — статус (обновлено 2026-10-30)

| Фаза | Статус |
|---|---|
| A. baseline, dirty-state, Henki | DONE: run/nightly/baseline (локально), Henki SHA256 1bc1cd6b…, путь /home/adn/henki_ros2_best_practices.md |
| A. CCW trace gate | DONE (sim): корень найден, A/B 3/3 vs 2/4, см. DECISIONS D1. Робот: NEEDS_HARDWARE |
| B. карта/мир, двери | OPEN |
| C. генератор сценариев | OPEN |
| D. L1 C++ полигон | OPEN |
| E. L2/L3 выборочно | OPEN (инструмент run_nav2_trace.sh готов) |
| F. очередь SQLite, make nightly, Claude/Codex handoff | OPEN (существующий `make nightly` = lite-тир + I1 A/B, расширять, не заменять) |
| G. пилот, восстановление | OPEN |
| GPU/RL/Парето/HTML-отчёт | OPEN (после end-to-end цикла) |

Факты: путь карты `control_backend/ws/src/navigation/maps/main_office` не существует; актуальная карта — ws/src/maps/main_office (pgm sha 72ee4094…, идентична в worktree и control_backend). Henki-требования (SingleThreaded executor, C++ для hot path, ограниченные очереди, throttled logs, без sleep в тестах) учитывать в новом коде. Версии CLI: claude 2.1.312, codex-cli 0.165.0 (флаги автоматизации не проверены — OPEN).

## P0 Continuous Execution (2026-10-30)
- `tools/nightly/nq.py`: SQLite WAL очередь + оркестратор (flock-lease, heartbeat/timeout, сигнатуры отказов, без слепых повторов, checkpoint.json после каждого события, `recover` после рестарта).
- systemd --user: `qazbot-nightly.service` (Restart=always), `qazbot-nightly-enqueue.timer` (01:00, `make nightly`). `loginctl enable-linger` = yes (без sudo).
- VERIFIED: kill -9 демона посреди задачи -> systemd рестарт -> INTERRUPTED -> PENDING -> COMPLETED; NEEDS_APPROVAL не блокирует независимые; hang по heartbeat; PAUSED_AGENT при usage_limited.
- Codex 01a18872: `exec resume` отказан ("already has an active writer" — открыты 2 TUI `codex resume`, не трогаем). Канал передачи: `codex queue --thread <id> --message` (флаг проверен по --help; доставка UNVERIFIED до сброса лимита 2026-10-30 01:14).
- /loop: CronCreate job a56ac490, каждый час :17, session-only, живёт 7 дней (не заменяет systemd).
