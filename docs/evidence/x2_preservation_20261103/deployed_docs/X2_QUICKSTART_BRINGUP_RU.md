# X2 .66: быстрый запуск сухого стека навигации (без движения)

Проверено 2026-10-23 на X2 .66 после перезагрузки (запуски bringup23t и bringup23f). Состав стека: локализация (FAST-LIO portable с loop-back fix,
CPU 5,6, level_bridge с clock-loopback fix, карта `first_map_nav2_v3`, AMCL), trust (pose_store выключен, seed выключен), адаптер в
dry-run (`live=false`, без регистрации в MC), Nav2 `nav_full` и goal_coordinator с `allow_dry_run_driver`.
**Робот не движется**: в `/aima/mc/locomotion/velocity` публикуют только 4 вендорских узла, наших издателей нет.

Все команды выполняются на X2 (`ssh agi@192.168.1.66`). Журналы и PID: `~/control_backend/logs/<этап>_<TAG>.{log,pid}`.

## 0. Перед запуском

Робот **стоит** (не сидит) и неподвижен: level_bridge определяет уровень при старте.

1. Проверить, что стек ещё не запущен (вывод должен быть пустым):
   ```bash
   pgrep -af "fastlio_mapping|level_bridge|nav2_amcl/amcl|map_server|planner_server|controller_server|bt_navigator|velocity_smoother|lifecycle_manager|goal_coordinator|x2_motion_adapter|localization_trust"
   ```
2. Часы и PTP. Последние строки должны быть в состоянии `s2`, offset должен быть порядка сотен наносекунд, без `clock jumped` и `SYNCHRONIZATION_FAULT`:
   ```bash
   timeout 10 journalctl -b --no-pager -u 'agibot_ptp_from@develop0:1' -u 'agibot_ptp_to@sensor0' | tail -6
   timeout 20 journalctl -b --no-pager | grep -iE "jumped|SYNCHRONIZATION" | tail
   ```
3. Штампы лидара относительно часов системы (15 с, только подписка):
   ```bash
   bash -c 'source ~/control_backend/ws/src/navigation/env.sh >/dev/null 2>&1; set +e; timeout 40 python3 ~/control_backend/tools/stack/age_probe.py 15 /aima/hal/sensor/lidar_chest_front/lidar_pointcloud:pc2 /aima/hal/sensor/lidar_chest_front/imu:imu' 2>/dev/null
   ```
   Норма: облако 10 Гц, `age_p50` около 0,10 с (штамп ставится на начало скана), IMU 200 Гц с `age` около 1 мс, `n_future` 0.
   Если `n_future` > 0 или возраст ушёл на секунды, был скачок часов: стек не запускать, см. раздел 6.

## 1. Запуск (один скрипт)

```bash
bash ~/control_backend/tools/stack/stack_up.sh            # TAG = дата-время; можно задать свой: stack_up.sh mytag
```
Скрипт выполняет те же четыре команды, что и ручной запуск ниже, по порядку. Каждый этап ждёт свою строку в журнале (это не слепая пауза),
затем делает паузу 2 с. При ошибке скрипт останавливает последовательность и печатает команду остановки. Успех выглядит так:
```
localization active
trust started
adapter started (dry_run)
nav_full up; planner waits for map->base_footprint: operator sets 2D Pose Estimate in RViz
...
STACK_UP OK tag=<TAG>  (stop: tools/stack/stack_down.sh <TAG>)
```
Время запуска около 35 с.

### То же вручную (команды скрипта; cwd `~/control_backend/ws/src/navigation`)
```bash
cd ~/control_backend/ws/src/navigation
# 1) локализация; ждать в logs/localization_<TAG>.log строки "Managed nodes are active" и "levelled"
python3 detach.py localization_<TAG> bash run_localization.sh lio_binary:=/home/agi/control_backend/candidates/fast_lio_portable/install/fast_lio/lib/fast_lio/fastlio_mapping map_yaml:=/home/agi/control_backend/ws/src/maps/first_map_nav2_v3/map.yaml lio_cpus:=5,6 seed_pose_file:=/nonexistent/qazbot_seed_disabled.json
# 2) trust (pose_store выключен); ждать "localization_trust started"
python3 detach.py trust_<TAG> bash -c 'source ./env.sh >/dev/null 2>&1; exec python3 localization_trust_node.py --ros-args -p map_yaml:=/home/agi/control_backend/ws/src/maps/first_map_nav2_v3/map.yaml -p expected_map_sha256:=f6032f75ffd386361993a4b4864cf0816713fc27f6d037c88d929ae5d48562fb'
# 3) адаптер dry-run; ждать "x2_motion_adapter started: live=False"
python3 detach.py adapter_<TAG> bash run_x2_driver.sh -p live:=false
# 4) Nav2 + координатор; ждать "Managed nodes are active" или "Timed out waiting for transform from base_footprint to map" (до позы)
ROBOT_RADIUS=0.454 python3 detach.py nav_full_<TAG> bash run_nav_full.sh allow_dry_run_driver:=true map_yaml:=/home/agi/control_backend/ws/src/maps/first_map_nav2_v3/map.yaml
```
Пример ожидания строки, без цикла опроса:
`timeout 60 tail -n +1 -f ~/control_backend/logs/localization_<TAG>.log | grep -m1 -E "Managed nodes are active|malformed|exited with"`.

Важно:
- `seed_pose_file:=` с пустым значением `ros2 launch` отвергает (`malformed launch argument`). Чтобы выключить seed, нужен путь к
  несуществующему файлу. В журнале при этом появится `"nothing seeded … reason": "no_file"`.
- `run_trust.sh` здесь **не** используется: он включает pose_store и закрепляет sha карты v2.
- **Не** запускать адаптер с `-p live:=true` и **не** вызывать `motion_adapter/register`. В dry-run этого сервиса и нет.

## 2. Проверка здоровья (одна команда, только чтение)

```bash
bash ~/control_backend/tools/stack/check.sh 30
```
Скрипт сам задаёт `FASTRTPS_DEFAULT_PROFILES_FILE=…/dds_qazbot.xml` (через `env.sh`) и ограничивает время выполнения. Норма до позы оператора:

| Что | Норма |
|---|---|
| `/aima/mc/locomotion/velocity` pub | 4 вендорских (`aimrt_planning`, `soc0_rc`, `app_proxy`, `soc2_interaction`), наших нет |
| `/cmd_vel` pub | 1 вендорский `controller_server_ros_node` |
| `duplicate_node_names` | `[]` |
| lifecycle | map_server, amcl, controller_server, velocity_smoother = `active`; planner_server `timeout` (ждёт TF), bt_navigator `inactive`, пока нет позы |
| `motion_adapter/status` | `state idle`, `live false`, `mode dry_run`, `registration dry_run`, `mc_source rc`, `published 0` |
| `/qazbot_lio/odometry` | 10 Гц, age p50 около 0,03 с, p99 меньше 0,1 с |
| `/qazbot_lio/odom_level`, `odom->base_footprint` | 10 Гц, age p99 меньше 0,1 с |
| `/qazbot_lio/scan` | 5 Гц, age p99 меньше 0,1 с |
| `map->odom` | **отсутствует до 2D Pose Estimate**; после позы 5 Гц с `age` около −0,96 с (AMCL ставит штамп скан + 1,0 с, это норма) |
| trust reasons до позы | `T1_tf_stale`, `T1_amcl_pose_stale`, `not_ready:no_anchor` на каждом такте: у AMCL ещё нет позы, это норма |

Привязка FAST-LIO к ядрам (должно быть `5,6`):
`taskset -acp $(pgrep -x fastlio_mapping) | awk '{print $NF}' | sort -u`

## 3. RViz (на ПК оператора)

1. Запуск: `bash rviz_live/run_local_viewer.sh` (подробности в `RVIZ_LOCAL.md`; профиль `dds_pc_x2.xml`, обнаружение около 25 с).
   Fixed Frame: `map` (так в `localization.rviz`).
2. **2D Pose Estimate** в истинной позе робота: стрелка из точки base_footprint (проекция грудного IMU) по курсу робота. Робот стоит.
   Поза должна быть в пределах примерно 0,7 м и 20° от истины. Лидар видит около ±52°, вдоль одной прямой стены положение не определено.
3. После позы: в `logs/nav_full_<TAG>.log` появится `Managed nodes are active` (bt_navigator активен), в RViz скан ляжет на карту,
   у trust пропадут `T1_tf_stale` и `T1_amcl_pose_stale`.

Дальнейшие шаги (nomotion-обновления AMCL, baseline, confirm_anchor) в этом сеансе **не выполнялись**: они требуют позы оператора,
их выполняет оркестратор. Имена сервисов проверены на живом графе:
`/qazbot_d1_map/request_nomotion_update` (std_srvs/Empty; несколько вызовов, без циклов),
`/qazbot_d1_map/localization_trust/baseline_start`, `…/baseline_finish`, `…/confirm_anchor` (std_srvs/Trigger),
`/qazbot_d1_map/goal_coordinator/stop` (std_srvs/Trigger). Порядок описан в `RVIZ_LOCAL.md`, шаг 3.
Подтверждать anchor можно только после визуальной проверки совпадения скана с картой.

## 4. Остановка (обратный порядок)

```bash
bash ~/control_backend/tools/stack/stack_down.sh            # TAG по умолчанию берётся из logs/stack_last_tag
```
Скрипт посылает SIGINT каждому launch-PID в порядке nav_full → adapter → trust → localization, ждёт выхода каждого (до 30 с) и затем
проверяет, что не осталось процессов стека. Успех: `STACK_DOWN OK`. Вручную то же самое: `kill -INT $(cat ~/control_backend/logs/<этап>_<TAG>.pid)`
в том же порядке, затем команда `pgrep` из раздела 0. Не использовать `pkill`.
Известно: если остановить стек до позы оператора, planner_server выходит с кодом −6 (SIGINT во время ожидания TF). Это косметика,
сирот не остаётся.

## 5. Новая поза после перезапуска

Сохранение позы (pose_store) в этой конфигурации выключено. После **любого** перезапуска локализации поза потеряна,
и оператор заново ставит 2D Pose Estimate.

## 6. Частые проблемы

- **CLI `ros2 …` висит или не видит узлы стека.** Не задан профиль DDS стека. Все команды запускать через
  `bash -c 'source ~/control_backend/ws/src/navigation/env.sh >/dev/null 2>&1; set +e; timeout 30 ros2 …'`
  (`env.sh` задаёт `FASTRTPS_DEFAULT_PROFILES_FILE=…/dds_qazbot.xml`). Всегда ставить `timeout`. Не запускать CLI в цикле: каждый старт
  `ros2` нагружает CPU, а это задерживает FAST-LIO и trust.
- **Скачок часов (PTP).** Признаки: `clock jumped` в журнале ptp, `n_future` > 0 или возраст в секундах в age_probe, level_bridge перестаёт
  публиковать скан. Что делать: дождаться, когда штампы вернутся в норму (раздел 0, п. 3), затем `stack_down.sh` и `stack_up.sh`,
  затем новая поза. Сервис `agibot_net_time_sync` пишет только RTC, системные часы он не трогает.
- **FAST-LIO не стартует или нет `Managed nodes are active`.** Смотреть `logs/localization_<TAG>.log`. Нет `levelled` значит робот не стоит или нет
  лидара (проверка из раздела 0, п. 3). `lidar loop back, clear buffer` в первые секунды допустим: патченный бинарник восстанавливается.
- **Скрипт упал на «stack processes already running».** Остался прежний стек: `stack_down.sh <старый TAG>`; если остались сироты,
  `kill -TERM <pid>` по каждому.
- **T1_stale в trust.**
  - `T1_tf_stale` вместе с `T1_amcl_pose_stale` постоянно, а одометрия и скан свежие: у AMCL нет позы (до 2D Pose Estimate). Это норма.
  - Редкие одиночные `T1_scan_stale` или `T1_odom_stale` бывают при всплесках CPU (старт `ros2` CLI, тяжёлые офлайн-расчёты): скан идёт
    с частотой 5 Гц, поэтому запас до порога 0,5 с мал.
  - Постоянные `T1_odom_stale` или `T1_scan_stale`: проверить часы (раздел 0) и `check.sh`. Для разбора запустить
    `bash -c 'source ~/control_backend/ws/src/navigation/env.sh >/dev/null 2>&1; set +e; timeout 140 python3 ~/control_backend/tools/stack/t1_probe.py 120' 2>/dev/null`.
    Probe записывает каждый такт с T1, имена причин и что в тот момент видел независимый подписчик. Пороги не менять.
