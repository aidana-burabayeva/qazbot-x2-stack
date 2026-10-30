# План переноса checkout в /home/agi/qazbot-x2-stack

**Статус: НЕ ВЫПОЛНЕНО.** Это план для одобрения оператором. Ни один шаг не применён, ничего не удалялось. Цель и обоснование: `REPOSITORY_LAYOUT.md`, раздел 7.

## 1. Предусловия

1. **Нет зависимости от старого пути** (аудит A1, 2026-10-30). Метод: `grep -rIlF` по фиксированным строкам `/home/agi/control_backend/qazbot-x2-stack`, `qazbot-x2-stack`, `/home/adn` по `/home/agi/control_backend` (`*.sh *.bash *.py *.yaml *.yml *.xml *.launch *.json *.cfg *.toml *.service *.env *.ini *.rviz *.md`; исключены build, log, logs, install, .git, `__pycache__`, `.pytest_cache`, сам checkout), а также `~/.bashrc`, `~/.profile`, `/etc/profile.d`, `/etc/bash.bashrc`, юниты systemd (system и user), crontab. Результат: 0 файлов по полному пути; 1 по `qazbot-x2-stack`: комментарий `ws/src/navigation/nav_control.launch.py:27`; `/home/adn` только в 5 документах и результатах тестов; systemd, cron, docker, shell init: 0. Симлинков в checkout и из него в развёртывание нет, кроме `tests/twin` (висячий, относительный). Повторить grep непосредственно перед переносом.
2. **`env.sh` и относительный `runtime/`**: скрипт вычисляет `QAZBOT_BACKEND` относительно своего расположения и ожидает `runtime/`, `candidates/` рядом с `ws/`. Из checkout запуск не работает. До переноса исправить (явный `QAZBOT_BACKEND`, понятная ошибка при отсутствии `runtime/`) либо задокументировать запрет запуска из checkout. Этот план `env.sh` не меняет.
3. Стек остановлен, либо оператор подтвердил, что он не запущен (на аудите 2026-10-30 16:15 не был запущен; с 16:24-16:25 наш стек (localization, FAST-LIO, level_bridge, AMCL, Trust, bt_navigator, x2_motion_adapter с `live:=true`) запущен из путей развёртывания отдельной сессией оператора/Claude; непосредственно перед переносом требуется свежая проверка запущенных процессов и подтверждение оператора). Чужие процессы не трогать.
4. Checkout чист, все нужные ветки запушены (`git status`, `git branch -vv`, `git ls-remote origin`). Локальные ветки без upstream (`nav/first-goals-20261028`) учесть.
5. Зафиксировано «до»-состояние (раздел 4).

## 2. Шаги (каждый по одобрению оператора)

1. Снимок «до»: `snapshot.sh` (раздел 4).
2. Создать `/home/agi/qazbot-x2-stack`: предпочтительно `git clone` из origin (с чистой историей) и добавить локальные ветки/ремоут; либо `git worktree move`/`mv` существующего checkout с сохранением remotes и веток. Выбор записать в DECISIONS.
3. Сверить в новом checkout: `git remote -v`, `git branch -vv`, HEAD = 982f618 (или записанный SHA), `git status` чист.
4. Перенаправить симлинк навыка: `ln -sfn /home/agi/qazbot-x2-stack/.claude/skills/robot-sync-validate ~/.claude/skills/robot-sync-validate`.
5. Обновить ссылки на пути в `docs/`, `AGENTS.md`, `CLAUDE.md`, `docs/HANDOFF.md` (и комментарий `nav_control.launch.py:27`, если нужно) отдельным коммитом на отдельной ветке.
6. Старый путь **сохранить** до окончания проверок. Удаление вложенного checkout: отдельный шаг с отдельным одобрением, вне этого плана.
7. Проверки «после» (раздел 4); при расхождении откат (раздел 5).

## 3. Риски

- Окна VS Code и workspace-файлы, открытые на старом пути; `.orchestrator`-отчёты и сохранённые команды со старым путём.
- Вложенный checkout внутри развёртывания путает grep, rsync, find и инструменты, обходящие `/home/agi/control_backend` (после переноса это исчезает; до того исключать его явно).
- Висячие `runtime/fast_lio` (68 ссылок; 79 во всём дереве, включая 10 в `runtime/user_pkgs` и `tests/twin`) не связаны с переносом, но могут сбить проверки целостности.
- Скрипты `tools/stack/*` в Git жёстко прописывают пути развёртывания (`/home/agi/control_backend/...`); перенос checkout их не затрагивает и не должен.
- Не часть миграции, но учесть: `qazbot-speech.service` в crash-loop (`KeyError: 'SONIOX_API_KEY'`, ~6.5 с) создаёт DDS-участника при каждом запуске; возможный источник churn обнаружения. Решение за оператором.
- Симлинк навыка станет висячим, если забыть шаг 4: команда `/robot-sync-validate` пропадёт из сессий.

## 4. Проверки до и после

Снимок: `/home/agi/.orchestrator/reports/20261030-p0-before-snapshot/snapshot.sh [OUT_DIR]` (read-only, без DDS и sudo, `git --no-optional-locks`). База: `before_20261030T161912/`. После: запустить в новый каталог и `diff -r before_20261030T161912 <new>`; ожидаемо меняется только `32_volatile_speech_restarts.txt`, плюс сведения о пути checkout.

Сравнить до и после:
- процессы и их cwd/exe (вендор недоступен без sudo);
- граф ROS **только если стек запущен**, одним коротким наблюдателем (новый DDS-участник ранее задерживал FAST-LIO);
- издатели TF: `odom -> base_footprint` у `native_odom_bridge`, `map -> odom` у AMCL;
- `ROS_DOMAIN_ID=0`, RMW, `FASTRTPS_DEFAULT_PROFILES_FILE`;
- запущенные пакеты и конфиги (пути в cmdline указывают на развёртывание);
- SHA256 всего `ws/src` развёртывания до и после: должны совпасть (перенос его не затрагивает);
- Claude: свежая headless-сессия в `/home/agi/control_backend` и в новом checkout, `slash_commands` содержит `robot-sync-validate` ровно один раз; `--tools Skill`: `robot-sync-validate: NO`.

## 5. Откат

Старый путь не удалялся, поэтому откат: вернуть симлинк `ln -sfn /home/agi/control_backend/qazbot-x2-stack/.claude/skills/robot-sync-validate ~/.claude/skills/robot-sync-validate`, откатить коммит со ссылками, при необходимости убрать новый каталог (с одобрения оператора). Развёртывание на всём протяжении не менялось. Повторить проверки раздела 4.
