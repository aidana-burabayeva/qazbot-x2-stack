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
