# Структура репозитория и развёртывания X2

Дата аудита: 2026-10-30 (read-only; без ROS-команд, без DDS-участников, без sudo). Источник: отчёты A1 (рантайм) и A2 (Git против live). Документ описывает фактическое состояние, а не желаемое; целевая схема вынесена в раздел 7.

## 1. Два дерева на X2

| Путь | Что это | Git |
|---|---|---|
| `/home/agi/control_backend` | **Развёртывание (deployment root)**: то, что реально запускается | нет `.git` (не делать `git init`) |
| `/home/agi/control_backend/qazbot-x2-stack` | Вложенный Git checkout `aidana-burabayeva/qazbot-x2-stack` (ветка `nav/first-goals-20261028`, 982f618) | да |

Из checkout ничего не исполняется: ни один процесс не имеет cmdline/cwd/exe в нём, ни один симлинк не ведёт в него. Прямых зависимостей от пути checkout нет (единственное вхождение, комментарий в `ws/src/navigation/nav_control.launch.py:27`).

## 2. Что лежит в развёртывании

- `ws/src/{lio,navigation,maps}`: код запускается **напрямую** (python, launch, sh). Каталогов `ws/build`, `ws/install`, `ws/log` нет; это не colcon-workspace.
- `runtime/` (122M, git-ignored): готовые overlay-префиксы (deb-extract) `nav2_bt_deps`, `qazbot_nav_deps`, `user_pkgs`, `fast_lio`. Nav2 (amcl, controller, planner, bt_navigator, map_server, lifecycle, smac, RPP) есть **только** здесь, в `/opt/ros/humble` их нет. Рядом `runtime_arm64.tar.gz` (43M).
- `candidates/` (269M, git-ignored): `fast_lio_portable/install` (реальный бинарник `fastlio_mapping`), `nav2_smac_bounds/install`, `nav2_lifecycle_shutdown`; `candidates_v13/`.
- `docs/evidence/nav_20261029_1425/{start.sh,start_loc.sh,start_nav.sh,restart_gc.sh,restart_trust.sh,p20_restart_adapter.sh}`: ручные скрипты запуска с абсолютными путями развёртывания. Копии в Git: `docs/evidence/nav_session_20261029/scripts/`. `tools/stack/stack_up.sh` тоже жёстко прописывает пути развёртывания.
- DDS-профиль: `docs/evidence/nav_20261029_1425/dds_nav_two_iface_log.xml` (sha256 661979e2...), передаётся через `FASTRTPS_DEFAULT_PROFILES_FILE` скриптами запуска. Дефолт `ws/src/navigation/dds_qazbot.xml` из `env.sh` ими переопределяется.
- `runtime/fast_lio`: symlink-install в `/home/agi/ws_lio/...`, которого нет; 79 висячих ссылок во всём дереве. Бинарник берётся из `candidates/fast_lio_portable`, поэтому сейчас безвредно, но префикс остаётся в `AMENT_PREFIX_PATH`.
- Прочее: `backups/`, `logs/` (6.9G), `vendor_snapshots/` (356M), `tests/`, `tools/`, `run/` (в т.ч. `x2_motion_owner.lock`), `passive_probe.py`, `README.md`.

## 3. Порядок overlay (`env.sh` + `aimdk_env.sh`, проверено в чистом `env -i`)

`AMENT_PREFIX_PATH`, первый побеждает:
1. `/agibot/software/common` (aimdk_env.sh, сообщения вендора; только nav/adapter)
2. `candidates/nav2_smac_bounds/install/nav2_smac_planner` (если задан `QAZBOT_SMAC_PREFIX`)
3. `runtime/nav2_bt_deps/root/opt/ros/humble`
4. `runtime/qazbot_nav_deps/root/opt/ros/humble`
5. `runtime/user_pkgs/opt/ros/humble`
6. `runtime/fast_lio` (висячие ссылки, см. выше)
7. `/opt/ros/humble`

`env.sh` вычисляет `QAZBOT_BACKEND = <ws/src/navigation>/../../..`; это неявный контракт: `ws/src` обязан лежать рядом с `../../runtime` и `../../candidates`. **Из checkout запуск не работает**: `runtime/` и `candidates/` там нет, overlay указывают в пустоту, Nav2 не найден, запуск упадёт (молчаливого fallback нет). `env.sh` в checkout побайтно равен развёрнутому.

## 4. Сеть и автозапуск

- Запуск скриптов: `ROS_DOMAIN_ID=0`, `ROS_LOCALHOST_ONLY=0`, `RMW_IMPLEMENTATION` не задан (Humble по умолчанию `rmw_fastrtps_cpp`). В `env.sh` `ROS_DOMAIN_ID` не задаётся. Для процессов вендора значения не проверены (нужен sudo); предположение: домен 0, FastDDS, профиль `~/.aima/env/ros_dds_configuration.xml`.
- Автозапуска нашего стека **нет**: ни systemd (system/user), ни timer, ни cron (`crontab -l` пуст), ни docker (dockerd inactive), ни `~/.bashrc`/`.profile` (там только `~/.aima/env/bashrc` вендора). Стек всегда стартуется вручную.
- Вне control_backend, но наше: user-юниты `qazbot-speech.service`, `lingxi-off.service`, `tailscaled.service`. `qazbot-speech` в crash-loop (~6.5 с; `KeyError: 'SONIOX_API_KEY'`, нет `~/.config/qazbot_speech/env`); каждый запуск успевает создать ROS 2 узел, то есть DDS-участник входит и выходит из домена 0. Гипотеза (не проверена): это добавляет churn обнаружения. Не часть этой работы, сервис не менялся.

## 5. Какой код выполняется

Состояние на 2026-10-30 16:15: **наш стек не запущен** (перезагрузка робота 13:40, после неё не стартовал). Процессы: только вендор (`/agibot/software`, `/opt/ros/humble`, пользователи `run`/`root`) и прочие user-юниты. Последний запуск (2026-10-29) использовал **только пути развёртывания**: `ws/src/...`, `runtime/...`, `candidates/...`, DDS-профиль из `docs/evidence`. Чего нет в проверке: что именно загрузил процесс (файлы могли быть изменены после старта) и граф ROS/TF (пропущено намеренно, чтобы не создавать DDS-участника).

## 6. Git против live (SHA256, A2)

Сравнивались развёртывание, worktree checkout и HEAD-блобы (982f618).

- `ws/src`: **86 идентичны, 0 различаются**, 63 только в live (все `*.bak*`, `.p19_kick_rejected`, `.p20_yawloop_candidate`; не отслеживаются, игнорируются), 1 только в Git (`ws/src/navigation/candidates/x2_motion_adapter_yawloop_p20.py`, игнорируется, sha 9c05bb34... = развёрнутый адаптер P20). Дрейфа кода нет: развёрнутое = `nav/first-goals-20261028 @ 982f618` (запушено в origin).
- Прочий дрейф: `run/x2_motion_owner.lock` (в Git устаревший pid=133021 от 2026-10-22, в live pid=405674 от 2026-10-29; runtime-артефакт в Git, live считается истиной); `README.md` (в checkout новее на одну строку); `docs/evidence/i1_deploy_2d240dbd/{archive_manifest.json,process_snapshot.txt}` отличаются от HEAD (детально не разбирались); устаревшая строка 109 в `docs/HANDOFF.md` («P20 not deployed»).
- Только в Git: `AGENTS.md`, `CLAUDE.md`, `.gitignore`, `.claude/`, часть `docs/*.md`, шесть `tools/roadmap_v13/*.py`, три `test_*_t1_pause.py`.
- Только в live: резервные копии, `tests/integration_results` (1474 файла, игнорируется), несколько тестов без коммита (loc_trust, nav_e2e, planner_clearance, controller_dry), `docs/evidence` (352) и `docs/roadmap_v13/evidence` (161).
- Счётчики по каталогам (IDENT/DIFF/ONLY_LIVE/ONLY_GIT): tools 109/0/6/6; tests 82/0/1508/11; docs (верхний уровень) 47/0/18/6; docs/evidence 74/2/352/296; docs/roadmap_v13 16/0/161/0; candidates_v13 5/0/0/0.
- **Не сравнивалось**: `backups/`, `candidates/`, `logs/` (6.9G, не обходился), `runtime/`, `vendor_snapshots/`, tarball, `.pytest_cache`; 21 файл >20 MB (около 17.3 GB: bags, `fastdds.log`, jsonl, `asbuilt.tar.gz`) не хэшировались; `ws/build|install|log` отсутствуют; сами `.git`, `__pycache__`, `*.pyc` исключены.
- Манифесты сессий в Git: 241 совпадение, 25 расхождений (растущие логи и `.orchestrator/*.md`), 2 пропущено (>20 MB). Расхождений в коде `ws/src` нет.

## 7. Целевая схема (предложение, не выполнено)

- `/home/agi/control_backend`: **единственный production-корень** развёртывания; без вложенного checkout; рядом (под корнем) deployment manifests (см. `DEPLOYMENT_CONTRACT.md`).
- `/home/agi/qazbot-x2-stack`: **единственный исходный checkout** `github.com/aidana-burabayeva/qazbot-x2-stack`; общий для X2 и digital twin.
- Развёртывание только копированием из чистого checkout на записанном SHA по шагу, одобренному оператором. Порядок миграции: `MIGRATION_PLAN.md`.

## 8. Claude Skills

- Навыки хранятся в Git: `.claude/skills/` (сейчас `robot-sync-validate`, команда оператора, см. `docs/ROBOT_SYNC_VALIDATE.md`).
- Проектные навыки Claude Code загружаются из `.claude/` текущего каталога. В развёртывании (`/home/agi/control_backend`) такого каталога нет, а checkout лежит в подкаталоге, поэтому из развёртывания команда не видна.
- Решение (2026-10-30): пользовательский симлинк, без копий и без записи в production-корень:
  `~/.claude/skills/robot-sync-validate -> /home/agi/control_backend/qazbot-x2-stack/.claude/skills/robot-sync-validate`.
  При переносе checkout симлинк нужно перенаправить (`MIGRATION_PLAN.md`).
- Проверка 2026-10-30, Claude Code 2.1.312 (headless `claude -p`, stream-json, событие `system/init`, модель haiku, workflow не запускался):
  - cwd `/home/agi/control_backend`: `slash_commands` содержит `robot-sync-validate` ровно один раз.
  - cwd `/home/agi/control_backend/qazbot-x2-stack`: тоже один раз, дубликата или конфликта нет.
  - Видимость для модели (`--tools Skill`): `robotics-task: YES`, `robot-sync-validate: NO` (как задумано: команда только для оператора).
  - Ограничение: интерактивное окно VS Code отсюда открыть нельзя; сессия VS Code использует то же обнаружение навыков (тот же бинарник и каталоги), но фактически в окне не проверялось.
- Будущий вариант: plugin marketplace (больше подвижных частей; пока не нужен).
