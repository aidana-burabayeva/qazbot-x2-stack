# control_backend — навигационный стек QazBot для AgiBot X2

Репозиторий: `git@github.com:aidana-burabayeva/control_backend.git` (бывш. `qazbot-x2-stack`; GitHub перенаправляет старый
адрес). ROS 2 Humble, X2 Ultra (Jetson, AimDK 1.1.x). Состояние до консолидации: тег `pre-consolidation-fb18dc1`.

## Структура
```
ws/src/                      ROS 2 пакеты (colcon)
  qazbot_localization        FAST-LIO + C++ проектор + map_server + AMCL (launch), localization trust, seed/pose store
  qazbot_navigation          Nav2 (Smac 2D, RPP, smoother, bt_navigator), goal_coordinator, BT XML, RViz
  qazbot_motion_control      x2_motion_adapter (Nav2 → MC /aima/mc/locomotion/velocity)
  qazbot_lidar_projector     C++ проектор FAST-LIO → 2D scan/odom_level/TF (узел qazbot_level_bridge)
  qazbot_maps                карты (map.yaml/pgm + manifest sha256)
  qazbot_bringup             единая точка входа x2_navigation.launch.xml, env.sh, профили, DDS
ws/third_party.repos         сторонние исходники с точными коммитами (FAST-LIO, navigation2 1.1.18)
ws/third_party/              патчи, runtime_debs.lock (sha256 debs Nav2), скрипты сборки
ws/tools/, ws/tests/         лабораторные инструменты и стенды (в обоих каталогах COLCON_IGNORE: `cd ws && colcon build` собирает только ws/src; см. их README)
.claude/                     служебный каталог инструментов Claude Code: project skills оператора (`skills/robot-sync-validate`, `OPERATOR_SKILLS.json`); не код робота
docker/                      LAB Dockerfile/compose, deploy/ (релизы, откат), tools/ (очистка), twin/
docs/                        лёгкая документация (архитектура, runbooks, сводки результатов)
```
Доказательства (bags, логи, результаты прогонов) в репозитории **не хранятся**: `/home/agi/qazbot_archive/<sha>/`
(дерево до консолидации: `repo_tree_fb18dc1/INDEX.md`).

## Сборка (всегда вне дерева)
```bash
source /opt/ros/humble/setup.bash
B=/tmp/qazbot_build                                  # любой каталог вне репозитория
ws/third_party/fetch_runtime_debs.sh $B/deps --cache /home/agi/qazbot_archive/third_party_debs
nice -n 19 ws/third_party/build_third_party.sh --src $B/tp_src --build-base $B/build_tp --install-base $B/install --deps $B/deps --jobs 2
nice -n 19 colcon --log-base $B/log build --base-paths ws/src --build-base $B/build --install-base $B/install --parallel-workers 2
ROS_DOMAIN_ID=88 ROS_LOCALHOST_ONLY=1 colcon --log-base $B/log test --base-paths ws/src --build-base $B/build --install-base $B/install
```
Или контейнер: `docker/README.md` (сеть отключена, домен 88, без устройств, лимиты CPU/памяти).

## Запуск
```bash
QAZBOT_RUNTIME=$B/deps source $B/install/qazbot_bringup/lib/qazbot_bringup/env.sh
ROS_DOMAIN_ID=88 ROS_LOCALHOST_ONLY=1 ros2 launch qazbot_bringup x2_navigation.launch.xml      # lab: адаптер dry-run
ros2 launch qazbot_bringup x2_navigation.launch.xml mode:=production   # робот, только с разрешения оператора
```
Первая строка лога: `QAZBOT BRINGUP mode=… git_sha=… odom_source=lio lidar_source=… tf_owner=… control_channel=…`.
Отдельные компоненты: `ros2 launch qazbot_localization localization.launch.py`, `qazbot_navigation nav_full.launch.py`
(см. README пакетов).

## Какой SHA работает на роботе
```bash
docker/deploy/status.sh            # current/previous + SHA по путям запущенных процессов (release <sha> или legacy)
```
Также: строка `QAZBOT BRINGUP … git_sha=` в логе запуска и `<prefix>/current/manifest.yaml`.

## LAB → PRODUCTION
1. Ветка → PR → проверка в LAB (сборка/тесты/replay, домен 88, вне дерева) → merge; независимая верификация.
2. На роботе: `docker/deploy/build_release.sh --prefix <prefix> --sha <sha>` (`--prefix` или явный `QAZBOT_PROD_PREFIX` обязателен, умолчания нет) → `<prefix>/releases/<sha>/` (чистый checkout ровно этого
   коммита, грязные деревья отклоняются, `manifest.yaml`: sha, дата, профиль, версии ros-humble-* dpkg, пины third-party).
3. С разрешения оператора: `docker/deploy/deploy.sh --prefix <prefix> --sha <sha>` (только симлинки `current`/`previous`), перезапуск
   стека из `<prefix>/current`, `docker/deploy/status.sh`.
## Откат
`docker/deploy/rollback.sh --prefix <prefix>` (меняет местами `current` и `previous`), перезапуск, `status.sh`. Откат на состояние до
консолидации (каталог `/home/agi/control_backend`, fb18dc1) — `docs/RELEASE.md`, раздел «Legacy».

Правила работы агентов и DoD: [AGENTS.md](AGENTS.md).
