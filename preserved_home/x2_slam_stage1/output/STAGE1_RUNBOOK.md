# Этап 1 — картографирование AimDK X2 (mapping → save → verify → RViz)

Источники: https://x2-aimdk.agibot.com/en/latest/Interface/perception/SLAM.html ; локальный SDK `/home/agi/aimdk` (`src/py_examples/py_examples/slam.py`, `get_map.py`); отчёт `.orchestrator/reports/20261102-slam-stage1-T1-discovery.md`.

## 1. Чек-лист предусловий
- [ ] Модуль SLAM (опциональный) активирован после-продажной службой (SLAM.html: "optional component, contact after-sales").
- [ ] `/integrated_command` имеет >=1 подписчика (сейчас 0 — B2).
- [ ] Сервис `/aimdk_5Fmsgs/srv/GetStoredMapByName` присутствует (сейчас нет — B1).
- [ ] LiDAR `/aima/hal/sensor/lidar_chest_front/lidar_pointcloud` ~10 Гц, IMU `/aima/hal/sensor/lidar_chest_front/imu` ~199 Гц (на момент T1 OK).
- [ ] Оператор готов вести/проводить робота сам (мы НЕ отправляем команды движения). Явное "go" оператора перед start_mapping.
- [ ] Нет конфликтующих стеков (AMCL/Nav2/map_server): в T1 не запущены.
- [ ] Права на чтение map.db (сейчас Permission denied — B3).

## 2. Команды проверки ROS 2 (только чтение)
```bash
source /opt/ros/humble/setup.bash
source /home/agi/aimdk/install/setup.bash
./preflight.sh            # GO/NO-GO, код возврата 0/1
ros2 topic info -v /integrated_command --no-daemon
ros2 service list --no-daemon | grep GetStoredMapByName
ros2 interface show aimdk_msgs/srv/GetStoredMapByName   # (вывод обрывается ошибкой парсера QRCode; полный .srv: /home/agi/aimdk/src/aimdk_msgs/prebuilt_aarch64/share/aimdk_msgs/srv/GetStoredMapByName.srv)
```

## 3. Код (reference files)
| Файл | Назначение |
|---|---|
| `slam_mapping_client.py` | узел `slam_mapping_client`: start_mapping → Enter/SIGINT → `stop_mapping:<map_name>` → опрос GetStoredMapByName ≤120 с → JSON |
| `check_stored_map.py` | отдельная проверка сохранённой карты, `--db` — read-only sqlite |
| `preflight.sh` | предполётные проверки |
| `stage1_common.py` | общая логика (payload, JSON, ожидание) |
| `tests/test_stage1.py` | unit-тесты с моками |
| `output/stage1_rviz_config.rviz` | конфиг RViz2 |

Интерфейсы (все из официальной документации/SDK): `/integrated_command` std_msgs/String RELIABLE+TRANSIENT_LOCAL (SLAM.html, slam.py); `/aimdk_5Fmsgs/srv/GetStoredMapByName` (SLAM.html, get_map.py). Недокументированные GetStoredMap, GetMapVersion, MappingService НЕ используются. Официального топика подтверждения start/stop НЕТ — успех определяется только по `code==0`.

Поля ответа GetStoredMapByName (.srv): `code, header, map_info{resolution,width,height,origin}, map_version, map_path, data[], navi_points, ..., map_id`. Поле `data[]` помечено в .srv как зарезервированное — `grid_data_present` может быть false даже при валидной карте.

## 4. Команды запуска
```bash
source /opt/ros/humble/setup.bash
source /home/agi/aimdk/install/setup.bash        # порядок: сначала ROS, затем aimdk
cd /home/agi/x2_slam_stage1
python3 slam_mapping_client.py --ros-args -p map_name:=office_test_01
```
Предупреждение: не запускать более 2 ROS-узлов в секунду (риск для баланса в Stable Standing/Locomotion). Клиент — один узел. Коды выхода: 0 ок; 2 нет подписчиков /integrated_command (публикации НЕ было); 3 пустой map_name; 4 карта не подтверждена за 120 с.
Остановка: Enter или Ctrl+C → `stop_mapping:office_test_01`.

## 5. Проверка сохранённой карты
```bash
python3 check_stored_map.py office_test_01 [--db /agibot/data/var/MapManagerModule/map.db]
ros2 service call /aimdk_5Fmsgs/srv/GetStoredMapByName aimdk_msgs/srv/GetStoredMapByName "{map_name: 'office_test_01'}"
sqlite3 /agibot/data/var/MapManagerModule/map.db "SELECT * FROM map;"   # команда из SLAM.html
```
Скрипт открывает БД только `mode=ro`. Результат — `output/stage1_map_check.json` (`rviz_visible` оператор ставит вручную после проверки в RViz). Если map.db недоступна: "Permission denied" (B3) — путь на этом ПК отсутствует (T1), на PC1/nfs прав нет.

## 6. Инструкция RViz
Запускать на внешнем ПК (10.0.1.2, проводное подключение) с тем же `ROS_DOMAIN_ID` (по документации AimDK, раздел RViz Visualization; по умолчанию 0, RMW fastrtps). На роботе RViz не запускаем (нет DISPLAY).
```bash
rviz2 -d output/stage1_rviz_config.rviz
```
Fixed Frame `map`; Map на `/map` (Transient Local); TF; PointCloud2 лидара. Карта из `map_path` (PNG) отображается только через `/map` либо документированным загрузчиком — собственный загрузчик не изобретаем. Издатель `/map` в T1 неизвестен; наш узел `/map` и TF `map` не публикует.

## 7. Таблица
| Ожидаемый результат | Как проверить | Что делать при ошибке |
|---|---|---|
| preflight GO | `./preflight.sh` | см. раздел G |
| start_mapping опубликован при >=1 подписчике | лог "Publishing: [start_mapping]" | exit 2 -> B2 |
| stop_mapping:<name> отправлен | лог "Publishing: [stop_mapping:...]" | повторить после проверки подписчика |
| Карта сохранена | GetStoredMapByName code==0, лог "Map saved" | exit 4: смотреть code, ждать, обратиться в поддержку |
| JSON заполнен | `cat output/stage1_map_check.json` | запустить `check_stored_map.py` |
| map.db согласован | `check_stored_map.py --db ...` | B3: права/ПК |
| Карта видна в RViz | визуально, затем `rviz_visible=true` | проверить /map, TF, domain id |

## 8. Что НЕ входит в Этап 1
Relocalization, POI, navigation, PncTaskRequest, любые команды движения.

## Диагностика G — текущие блокеры (T1)
- B1: сервис `/aimdk_5Fmsgs/srv/GetStoredMapByName` отсутствует (есть только недокументированные GetStoredMap/GetMapVersion).
- B2: у `/integrated_command` 0 подписчиков (темы нет в графе); `robot_mode` nav=false dc=false.
- B3: map.db не читается пользователем agi (`/agibot/data/var/MapManagerModule` отсутствует на PC2; nfs — Permission denied).

Порядок запасных вариантов: a) официальный SDK / поддержка AGIBOT (активация SLAM для SN X2B0028C6Z0034); b) open-source `agibot-x2-navigation` — только если совместим с железом/Humble (ничего не устанавливаем); c) собственный узел по топикам LiDAR/IMU — только mapping/visualization.

## Итоговый вердикт
**STAGE 1 BLOCKED.** Причина: модуль SLAM не экспонирует официальный интерфейс — `/integrated_command` без подписчика (B2) и нет `GetStoredMapByName` (B1); map.db недоступна (B3). Следующее действие: запросить у AGIBOT after-sales активацию SLAM-модуля и подтверждение, зависит ли он от robot_mode nav=true; затем повторить `./preflight.sh` (после полной загрузки робота).
