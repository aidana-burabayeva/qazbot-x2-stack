# control_backend — навигационный стек QazBot для AgiBot X2

Репозиторий: `git@github.com:aidana-burabayeva/control_backend.git` (бывш. `qazbot-x2-stack`; GitHub перенаправляет старый
адрес). ROS 2 Humble, X2 Ultra (Jetson, AimDK 1.1.x). Состояние до консолидации: тег `pre-consolidation-fb18dc1`.

## Структура (один каталог = git-репозиторий = рабочее пространство)
```
<repo>/                      на роботе: /home/agi/control_backend (корень git, .git внутри); лаборатория: любой клон
  ws/src/                    ROS 2 пакеты (colcon)
    qazbot_localization      FAST-LIO + C++ проектор + map_server + AMCL (launch), localization trust, seed/pose store
    qazbot_navigation        Nav2 (Smac 2D, RPP, smoother, bt_navigator), goal_coordinator, BT XML, RViz
    qazbot_motion_control    x2_motion_adapter (Nav2 → MC /aima/mc/locomotion/velocity)
    qazbot_lidar_projector   C++ проектор FAST-LIO → 2D scan/odom_level/TF (узел qazbot_level_bridge)
    qazbot_maps              карты (map.yaml/pgm + manifest sha256)
    qazbot_bringup           единая точка входа x2_navigation.launch.xml, env.sh, профили, DDS
    third_party/             (gitignored, COLCON_IGNORE) FAST-LIO + ikd-Tree, navigation2 1.1.18 — импортирует build.sh
  ws/build ws/install ws/log (gitignored) результат ws/tools/build.sh; навигация запускается из ws/install
  ws/install/_runtime_deps   pinned runtime debs Nav2 (ws/third_party/runtime_debs.lock), env.sh берёт их по умолчанию
  ws/third_party.repos       сторонние исходники с точными коммитами; ws/third_party/ — патчи, lock debs, импорт
  ws/tools/                  build.sh, status.sh, task_*.sh, systemd/ (host-юниты) + лабораторные инструменты (COLCON_IGNORE)
  ws/tests/                  лабораторные стенды (COLCON_IGNORE)
  run/                       (gitignored) lock владельца движения адаптера (lock_path ~/control_backend/run/…)
  docker/                    только Docker: LAB Dockerfile/compose, twin/
  docs/                      лёгкая документация; .claude/ — skills оператора (не код робота)
```
Доказательства (bags, логи, результаты прогонов) в репозитории **не хранятся**: `/home/agi/qazbot_archive/<sha>/`.

## Сборка (одна команда, на месте, из корня репозитория)
```bash
ws/tools/build.sh [--deb-cache DIR] [--clean]     # nice 19, 2 workers; падает при любой ошибке
cd ws && source install/setup.bash && ROS_DOMAIN_ID=88 ROS_LOCALHOST_ONLY=1 colcon test --base-paths src && colcon test-result
```
`build.sh`: отказ, если что-то запущено из `ws/install` → runtime debs в `ws/install/_runtime_deps` (офлайн-кэш:
`--deb-cache DIR` или `QAZBOT_DEB_CACHE`) → `ws/third_party.repos` + патчи в `ws/src/third_party` (идемпотентно, изменённый
checkout → отказ) → colcon: `fast_lio`, `nav2_smac_planner`, затем `ws/src` в `ws/build` + `ws/install` →
`ws/install/qazbot_build_manifest.yaml` (sha, dirty, describe, release_tag, дата, пины/sha256 патчей, sha256 lock debs,
версии ros-humble dpkg). `--clean` удаляет `ws/build ws/install ws/log`. Обычный `cd ws && colcon build` тоже работает
(без third-party и манифеста; production его не примет).

## Запуск / остановка / статус (на роботе; LAB — так же, но домен 88)
```bash
cd /home/agi/control_backend
QAZBOT_MODE=production source ws/install/qazbot_bringup/lib/qazbot_bringup/env.sh
ros2 launch qazbot_bringup x2_navigation.launch.xml mode:=production     # только с разрешения оператора
```
* На переднем плане: остановка — **Ctrl-C** (SIGINT) в этом терминале. В tmux: `tmux new -s qazbot`, те же две
  команды, отсоединиться `Ctrl-b d`; остановка `tmux send-keys -t qazbot C-c` (или attach + Ctrl-C).
* LAB: `source ws/install/qazbot_bringup/lib/qazbot_bringup/env.sh && ROS_DOMAIN_ID=88 ros2 launch qazbot_bringup
  x2_navigation.launch.xml` (QAZBOT_MODE=lab по умолчанию: loopback DDS + ROS_LOCALHOST_ONLY=1, адаптер dry-run).
* Статус: `ws/tools/status.sh` — HEAD, тег, dirty, sha манифеста, вердикт production-гейта и для каждого ROS-процесса
  OK (из `<repo>/ws/install`) или FOREIGN (candidates, worktree, /tmp, legacy, другой install); код 1 при FOREIGN.
* Первая строка лога запуска: `QAZBOT BRINGUP mode=… git_sha=… dirty=… release_tag=… odom_source=lio …`.
Отдельные компоненты: `ros2 launch qazbot_localization localization.launch.py`, `qazbot_navigation nav_full.launch.py`.

## LAB → PRODUCTION, откат
Production = чистое дерево на коммите с аннотированным тегом `x2-release-YYYYMMDD-N`, собранное `ws/tools/build.sh` в
`/home/agi/control_backend`. `mode:=production` отказывает иначе. Порядок, откат на предыдущий тег и на legacy:
[docs/RELEASE.md](docs/RELEASE.md).

## Host-юниты (production)
`ws/tools/systemd/`: `qazbot-vendor-trim.service` (после загрузки останавливает hal_camera/orbbec_camera/perception/drp/
lx_webrtc_app через `aima em stop-app`; лог `~/.local/state/qazbot/vendor_trim.log`). Установка:
`ws/tools/systemd/install_user_units.sh [--dry-run]`; откат: `... --uninstall`. Подробности: `ws/tools/systemd/README.md`.

Правила работы агентов и DoD: [AGENTS.md](AGENTS.md). Docker (LAB-контейнер): `docker/README.md`.
