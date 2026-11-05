# /robot-sync-validate: синхронизация лаборатории с физическим X2

Skill Claude Code для приёма результатов лаборатории/digital twin на физический X2, проверки кандидатов под шлюзами безопасности и публикации обратной связи ROBOT → LAB.
Источник: [.claude/skills/robot-sync-validate/SKILL.md](../.claude/skills/robot-sync-validate/SKILL.md).

## Кто и как запускает

- Только оператор: `disable-model-invocation: true`, модель не вызывает skill сама и не видит его в списке skills.
- Запуск: открыть Claude Code в checkout `control_backend` (бывш. `qazbot-x2-stack`) (ветка, содержащая `.claude/skills/robot-sync-validate/`) и ввести `/robot-sync-validate`.
- Команда видна только в сессии, запущенной из этого checkout; в `~/control_backend` (deployed, без `.git`) её нет.

## Что команда делает и чего не делает

- Сама по себе: read-only проверка состояния, `git fetch`, разбор входящих коммитов в изолированном worktree, классификация, read-only диагностика, отчёты.
- Вызов команды **не является** разрешением на движение, `cmd_vel`/Nav2 goals, включение моторов, деплой, перезапуск ROS 2 или изменение конфигурации. Для каждого такого шага skill обязан показать план и получить отдельное явное подтверждение оператора в текущей сессии; без него результат `NEEDS_APPROVAL`.
- Не выполняет `git pull`, checkout, merge, reset или clean в рабочем checkout; правила [AGENTS.md](../AGENTS.md) (T5, collision detection, footprint, freshness, единственный владелец робота) имеют приоритет.

## Результаты

- `docs/hardware/ROBOT_TO_LAB_FEEDBACK.md`, датированный отчёт валидации, `docs/hardware/TWIN_FIDELITY_REQUESTS.md`.
- Коммиты только в отдельную ветку обратной связи с проверкой remote SHA; при ошибке push — `PUBLISH_PENDING`.
- Контрольные точки `last_lab_reviewed_sha` и `last_lab_integrated_sha` (последняя только для реально проверенных изменений).
- Статусы: `PASS`, `FAIL`, `UNVERIFIED`, `NEEDS_APPROVAL`, `NEEDS_HARDWARE`, `TWIN_MISMATCH`.

## Проверка установки (2026-10-30)

Новая headless-сессия Claude Code 2.1.312 в `qazbot-x2-stack`: `robot-sync-validate` присутствует в `slash_commands`; в списке skills, доступных модели, отсутствует (контроль: `robotics-task` присутствует). Workflow при проверке не запускался.
