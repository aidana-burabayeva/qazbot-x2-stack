# Релиз, запуск, откат (control_backend)

Решение оператора 2026-11-06: **один** активный каталог на роботе — `/home/agi/control_backend` = корень git-репозитория;
сборка на месте в `ws/build`, `ws/install`, `ws/log`; навигация запускается из `ws/install`. Нет `/home/agi/prod`,
`releases/`, `current`/`previous`, отдельных префиксов и копирования install. Лабораторный клон
(`/home/agi/lab/control_backend`) — отдельный рабочий checkout.

## Что такое production-сборка
Чистое дерево (`git status` пуст) на коммите с **аннотированным** тегом `x2-release-YYYYMMDD-N`, собранное
`ws/tools/build.sh` в этом же каталоге. `ros2 launch qazbot_bringup x2_navigation.launch.xml mode:=production`
(`bringup_info.py`) отказывает, если не выполнено хотя бы одно:
* запущенный install — это `<repo>/ws/install` git-checkout'а; есть `ws/install/qazbot_build_manifest.yaml`;
* `sha` манифеста == `git rev-parse HEAD`, `dirty: false`, `workspace` манифеста == `<repo>` (собрано здесь);
* дерево чистое сейчас; на HEAD есть аннотированный тег `x2-release-YYYYMMDD-N`;
* `env.sh` подключён с `QAZBOT_MODE=production` из этого install; не заданы `QAZBOT_SMAC_PREFIX`/`QAZBOT_LIFECYCLE_PREFIX`;
* `ROS_DOMAIN_ID` не задан или 0.
`ws/tools/status.sh` печатает ту же проверку (строка `gate`) без запуска.

## Выпуск (LAB)
1. Ветка → PR → LAB-проверка в клоне (сборка `ws/tools/build.sh`, `colcon test`, launch в домене 88) → merge;
   независимая верификация.
2. `git tag -a x2-release-YYYYMMDD-N -m "<что внутри>" <sha> && git push origin x2-release-YYYYMMDD-N`.

## Установка релиза на роботе (с разрешения оператора)
```bash
cd /home/agi/control_backend
ws/tools/status.sh                        # что запущено; остановить стек (Ctrl-C в его терминале) до сборки
git fetch --tags origin && git checkout x2-release-YYYYMMDD-N
ws/tools/build.sh --deb-cache /home/agi/qazbot_archive/third_party_debs   # на месте; отказ, если стек запущен
QAZBOT_MODE=production source ws/install/qazbot_bringup/lib/qazbot_bringup/env.sh
ros2 launch qazbot_bringup x2_navigation.launch.xml mode:=production     # передний план или tmux (README)
ws/tools/status.sh                        # в другом терминале: все ROS-процессы OK, gate: release: yes
```
Остановка: Ctrl-C (SIGINT) процессу `ros2 launch` (в tmux: `tmux send-keys -t qazbot C-c`).

## Переход с legacy (один раз)
Сейчас `/home/agi/control_backend` — legacy-каталог без `.git` (fb18dc1 + candidates/ + runtime/). Остановить legacy-стек,
`mv /home/agi/control_backend /home/agi/control_backend_legacy_fb18dc1`,
`git clone git@github.com:aidana-burabayeva/control_backend.git /home/agi/control_backend`, затем «Установка релиза».

## Откат
**(a) на предыдущий релиз:** остановить стек → `git checkout x2-release-<предыдущий>` → `ws/tools/build.sh` → запуск →
`ws/tools/status.sh`. (`--clean` не нужен: colcon пересобирает изменённое; при сомнении — `ws/tools/build.sh --clean`.)

**(b) на legacy (fb18dc1):** остановить стек, затем
```bash
mv /home/agi/control_backend /home/agi/control_backend_x2_<sha>
mv /home/agi/control_backend_legacy_fb18dc1 /home/agi/control_backend
```
и запустить прежние команды из `/home/agi/qazbot_archive/20261106_legacy_baseline_fb18dc1/cmdlines/*.txt` (порядок:
`localization.launch.py` → `localization_trust_node.py` → `nav_full.launch.py` → `x2_motion_adapter.py`). Legacy-пути
абсолютные (`/home/agi/control_backend/...`), поэтому работают только после второго `mv`. Возврат — обратные `mv`.
`ws/tools/status.sh` из `control_backend_x2_<sha>` покажет legacy-процессы как FOREIGN legacy.

Адаптер берёт lock `~/control_backend/run/x2_motion_owner.lock` (параметр `lock_path`, умолчание не менялось; каталог
создаётся сам, `run/` в .gitignore), так что legacy и новый стек не могут одновременно владеть движением.

## Host-юниты
| Юнит | Назначение |
|---|---|
| `qazbot-vendor-trim.service` (user, `ws/tools/systemd/`) | после boot останавливает vendor-приложения, не нужные навигации (`aima em stop-app`); лог `~/.local/state/qazbot/vendor_trim.log` |

Установка: `ws/tools/systemd/install_user_units.sh --dry-run`, затем без флага (идемпотентно, без sudo). Откат:
`install_user_units.sh --uninstall` (`systemctl --user disable qazbot-vendor-trim.service`). Вручную вернуть приложения:
`aima em start-app hal_camera|orbbec_camera|perception`.
