# Nightly — статус (обновлено 2026-10-30)

| Фаза | Статус |
|---|---|
| A. baseline, dirty-state, Henki | DONE: run/nightly/baseline (локально), Henki SHA256 1bc1cd6b…, путь /home/adn/henki_ros2_best_practices.md |
| A. CCW trace gate | DONE (sim): корень найден, A/B 3/3 vs 2/4, см. DECISIONS D1. Робот: NEEDS_HARDWARE |
| B. карта/мир, двери | DONE (static): `tools/nightly/map_analysis.py`, `docs/nightly/map/main_office_analysis.json` + overlay. 4 региона, 449 м² main free (unknown=obstacle). Дверные кандидаты UNVALIDATED: (-11.54,-4.48) w=1.6 м; (-6.49,-6.93) w=0.80 м — при ширине робота 0.712 м запас ≈0.04 м/сторона (NARROW). Мир симулятора ≠ карта Nav2: валидация в мире OPEN |
| C. генератор сценариев | DONE: `tools/nightly/scenario_gen.py` (seed, 10 позитивных + 4 негативных категории, независимые yaw, геом. достижимость по диску half-width 0.356 м, детерминизм проверен), `tests/nightly/test_scenario_gen.py` 4 passed. Кинодинамика/doorway-безопасность: L2/L3 |
| D. L1 C++ полигон | DONE (MODEL): `tools/nightly/l1/l1_runner.cpp` (A* по сетке с инфляцией, unicycle, swept footprint по реальному 8-угольнику, unknown=obstacle), `run_l1.sh`, тест. Пропускная способность: 1100 сц/с на 12 потоках (1100 сценариев за 1.0 с). Находка (MODEL, не Nav2): проём офиса w=0.80 м почти непроходим в strict-режиме; narrow-режим (half-width 0.356) даёт коллизии 14/20 + 19/20; ниже разрешения карты (0.05) — вердикт требует L2/L3 |
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

- Регрессия A: adapter/native/TWIN-02 набор = 122 passed (очередь, regress_a_adapter_native). Пересборка twin-образа с новым twin.launch.py: OPEN.
