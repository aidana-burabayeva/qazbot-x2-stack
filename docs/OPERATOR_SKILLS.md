# QazBot Operator Skills (v1.0.0, contract v1)

Один пакет, один репозиторий. Skills лежат в `.claude/skills/` каждого checkout и обновляются только через Git. Ручного копирования нет.

| Skill | Роль | Назначение |
|---|---|---|
| `/nightly-stop` | LAB | FREEZE→STOP→CHECKPOINT→DOCUMENT→VALIDATE→COMMIT→PUSH→VERIFY, документы передачи роботу |
| `/nightly-resume` | LAB | fetch, безопасный ff-only pull, diff по checkpoint, triage блокеров робота, запуск оркестратора |
| `/nightly-status` | LAB | read-only статус |
| `/robot-sync-validate` | ROBOT | получить изменения лаборатории, read-only анализ, проверки на X2 только с разрешением оператора, публикация `ROBOT_TO_LAB_FEEDBACK.md` |

Все четыре — `disable-model-invocation: true`: запускает только оператор. `qazbot-nightly` — справочный (не управляющий), вне пакета (`unmanaged`).

## Единый источник истины
- `.claude/skills/OPERATOR_SKILLS.json`: версия пакета, `contract_version`, роль и версия каждого skill, sha256 `SKILL.md`.
- `ws/tools/operator_skills/opskills.py`: `validate` (целостность, operator-only, дрейф, конфликты имён с `~/.claude`, версия контракта), `sync` (есть ли новая совместимая версия; сам ничего не тянет), `hashes --update` (переподпись после осознанной правки), `role`.
- Логика Git/handoff не дублируется: LAB-skills вызывают `lab.py`, ROBOT-skill — работает по тому же контракту документов. Существующий `robot-sync-validate` (`ce4537e`) включён без изменений.
- Роль: `QAZBOT_ROLE=LAB|ROBOT` или файл `~/.config/qazbot/role`. `lab.py stop|resume` отказывается работать в роли ROBOT. При UNKNOWN skill не выполняет действий над железом.

## Git-контракт LAB ↔ X2 (`docs/hardware/`)
- `LAB_TO_ROBOT_HANDOFF.md` — пишет lab stop; шапка `<!-- QAZBOT-CONTRACT {...contract_version, lab_branch, lab_sha, package_version} -->`; классы верификации `SIM_VERIFIED / SIM_FAILED / UNVERIFIED / NEEDS_HARDWARE / HARDWARE_VERIFIED` (последний — только из замеров робота).
- `ROBOT_TO_LAB_FEEDBACK.md` — пишет `/robot-sync-validate`; блок содержит `last_lab_integrated_sha` и `last_lab_reviewed_sha`; `/nightly-resume` читает их и ставит блокеры в приоритет (`triage_robot_findings`).
- `TWIN_FIDELITY_REQUESTS.md` — запросы данных с X2 (генерируется лабораторией, дополняется роботом).
- Совместимость: старые документы без блока допускаются (WARN); несовпадение `contract_version` — ошибка.

## Установка и обновление (оба checkout)
```bash
git fetch --prune
python3 ws/tools/operator_skills/opskills.py sync        # UP_TO_DATE | UPDATE_AVAILABLE | INCOMPATIBLE_MAJOR
git merge --ff-only origin/<ветка>                      # только если sync сказал так и дерево чистое
python3 ws/tools/operator_skills/opskills.py validate --role lab   # или robot
```
Dirty или diverged checkout: `sync` пишет `do not pull blindly` — интегрировать в чистом worktree, не перезаписывать. Новая сессия Claude Code подхватывает skills из `.claude/skills/` рабочего каталога.
Выпуск новой версии: правка skill → поднять версию в манифесте → `opskills.py hashes --update` → тесты → commit.

## План безопасной интеграции (ничего не выполнено автоматически)
1. Ветка `tooling/operator-skills-20261030` (эта) → ревью Codex в отдельном worktree.
2. LAB: после ревью — merge в лабораторную ветку `test/p20-twin02-sim-20261029` (содержит все `lab.py`/`store.py` изменения).
3. ROBOT: оператор решает сам; ветка робота `nav/first-goals-20261028` уже содержит `ce4537e`. Перенос: merge tooling-ветки в robot-feedback ветку, не в production и не в `main`. Производственный каталог X2 (`~/control_backend`, не Git) пакетом не затрагивается.
4. Ничего не удалять, пока ветки не аудированы.

## Handoff для Claude на X2
Прочитать это + `docs/hardware/LAB_TO_ROBOT_HANDOFF.md` (SHA `lab_sha` из шапки). Выполнить `opskills.py validate --role robot`, `sync`. `/robot-sync-validate` читает документы и диффы read-only. Движение, `cmd_vel`/Nav2-цели, SSH, deploy, перезапуск сервисов — только после явного разрешения оператора в текущей сессии. `last_lab_integrated_sha` записывать только за реально интегрированное и проверенное. UNVERIFIED не называть PASS.
Статус на момент выпуска: **SIM_VERIFIED/UNVERIFIED только по симуляции; HARDWARE_VERIFIED пусто.**
