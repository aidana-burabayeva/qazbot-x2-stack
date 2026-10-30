# Codex completion contract (P1.1) — установленные факты и правила

Проверено на установленных `codex-cli 0.165.0` / app-server daemon `0.166.0` (2026-10-30). Всё ниже получено инспекцией, не предположением.

## Факты
| Вопрос | Факт | Источник |
|---|---|---|
| Что подтверждает `codex queue --thread` | **Только доставку**: строка в `~/.codex/queue_1.sqlite` → `queued_items`, вывод `Queued message <id> for thread <id>`. Исполнение не подтверждается | схема БД; 12 сообщений лежат неконсумированными, последний ход сессии `interrupted` |
| Когда сообщение «потреблено» | строка исчезает из `queued_items` (только когда сессия его берёт); простаивающая/прерванная сессия не берёт | БД очереди (read-only) |
| Как увидеть исполнение | app-server (unix socket + websocket, JSON-RPC): `thread/turns/list` → ходы со `status` `inProgress/completed/failed/interrupted` и текстом пользовательского сообщения | `codex app-server generate-json-schema`, живой read-only вызов |
| Квота/сброс Codex | `account/rateLimits/read`: окна `primary` (300 мин) и `secondary` (10080 мин), `usedPercent`, `resetsAt` (эпоха), `rateLimitReachedType` | живой read-only вызов (55%/43% на момент проверки) |
| Квота/сброс Claude | **в установленном CLI нет документированного интерфейса** → `UNKNOWN`; лимит подтверждается только текстом провайдера в логе задачи, время сброса не выводится | `claude --help` |
| Опасные методы | `account/rateLimitResetCredit/consume` (тратит кредит), `thread/delete`, `account/logout` — клиент запрещает их вызов в коде | `agents.NEVER_CALL` |

## Конечный автомат делегированной задачи
`DISPATCHED` (доставлено, в очереди) → `ACKNOWLEDGED` (строка потреблена или есть ход с токеном) → `RUNNING` (ход `inProgress`) → `RESULT_RECORDED` (валидный `engineering_result.json` принят, evidence опубликован) → `VERIFIED` → `COMPLETED`.
- Код возврата `codex queue` **никогда** не даёт `COMPLETED`.
- Токен `QAZBOT-TASK:<task>:<attempt>:<attempt_id>` в тексте сообщения связывает ход с попыткой.
- Результат принимается только если совпали `task`, `attempt`, `attempt_id`, `git_sha` (коммит существует и потомок стартового SHA), `tests{passed,failed}` и sha256 всех `files[]`. Устаревшая попытка → `RESULT_REJECTED STALE_ATTEMPT`.
- Evidence публикуется атомарно до транзакции SQLite (см. `store.py`).
- Нет ответа ≠ успех и ≠ безопасное прекращение: состояние остаётся `DELEGATED`, при долгом молчании только событие `NO_ACK_ALERT`.
- Повторная отправка запрещена, пока попытка может быть активна. `NIGHTLY_SUPERSEDE_AFTER_S` (по умолчанию выключено) допускает новую попытку только если сообщение доказуемо не потреблено; поздний результат старой попытки всё равно отвергается.
- Lease (`resource wt:<cwd>`) держится в `DELEGATED`; снимается при `COMPLETED`/доказанном конце хода без результата (`NEEDS_ENGINEERING`). Процессы `claude`/`codex` в этом worktree, не запущенные оркестратором (TUI оператора), блокируют старт нового писателя.
- `lab.py stop` при открытой делегированной попытке: lease сохраняется, `DELEGATED_OPEN`, `STOP_UNVERIFIED`.

## Ограничения (UNVERIFIED)
- Сопоставление «строка очереди исчезла = потреблено» выведено из схемы и поведения, не из документации Codex.
- Запись результата Codex'ом в `run/nightly/work/...` может быть вне его песочницы; запасной путь `<worktree>/.qazbot/results/<task>.<attempt>.json` (в `.gitignore` не добавлен).
- Версии CLI (0.165) и daemon (0.166) различаются; протокол experimental.
- Пороги 95–99 % **не включены**: измерение квоты Codex подтверждено, Claude — нет, истории расхода на задачу нет.

## Дополнения после независимого review (см. REVIEW_P1_1.md)
- Write-ahead намерение `DISPATCHING` до `codex queue`; таймаут = `DISPATCH_UNKNOWN`, lease сохранён, повторной отправки нет.
- Принятие результата дополнительно требует: непустой `files[]` внутри worktree без symlink, и успешный **перезапуск оркестратором** `tests.cmd` (только allow-list `NIGHTLY_VERIFY_ALLOW`, по умолчанию `python3 -m pytest;pytest`, без shell). Самоотчёт агента о тестах не доверяется.
- Контракт одинаков для Claude и Codex: код возврата агента 0 без проверенного результата ⇒ `NEEDS_ENGINEERING`.
- Автоматического supersede нет; ручной: `nq.py supersede <task> --force-lease-release`.
- Допустимые методы app-server: `initialize`, `account/rateLimits/read`, `thread/turns/list` (allowlist).
