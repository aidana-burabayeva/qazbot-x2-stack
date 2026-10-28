# RViz на рабочем компьютере

Рабочая конфигурация хранится на X2 в ws/src/navigation/rviz/.
Локальная копия: /home/adn/QazBot/X2-locomotion/rviz_live/.
Запуск на компьютере: bash rviz_live/run_local_viewer.sh (после копирования файлов).

Текущая сеть: PC 192.168.1.64, X2 192.168.1.66, domain 0, rmw_fastrtps_cpp.
dds_pc_x2.xml ограничивает интерфейс PC Wi-Fi и задаёт X2 initial peer.
Если адрес меняется, профиль нужно обновить по фактическому адресу интерфейса.
Глобальные профили ROS/DDS и вендорская конфигурация не менялись.
ROS-потоки передаются напрямую по Wi-Fi/DDS, не через SSH. SSH служит управлению
запуском и копированию файлов. Обычный TCP-forward SSH не туннелирует DDS/UDP.

RViz запускается локально на DISPLAY=:1, а не на процессоре робота.
Конфигурация `ws/src/navigation/rviz/localization.rviz` (резервная копия прежней: `localization.rviz.bak_20261021_pre_goal_tool`) показывает карту, LaserScan,
TF, AMCL covariance и ParticleCloud, а также (добавлено 2026-10-21) глобальный путь `/qazbot_d1_map/nav2/plan`, локальный план RPP
`/qazbot_d1_map/nav2/received_global_plan`, точку упреждения `nav2/lookahead_point`, footprint локальной costmap, позу `/qazbot_d1_map/goal_pose`
и локальную costmap (выключена по умолчанию: нагрузка на Wi-Fi). QoS каждого дисплея согласован с живыми издателями. Twist-дисплея в RViz Humble нет:
команду смотреть `ros2 topic echo /qazbot_d1_map/motion_adapter/dry_run_cmd`.
Инструменты: **2D Pose Estimate** (`/qazbot_d1_map/initialpose`, ковариация 0,25/0,25/0,0685) и **2D Goal Pose** (`rviz_default_plugins/SetGoal`, топик
`/qazbot_d1_map/goal_pose`, Reliable/Volatile). Цель из 2D Goal Pose принимает `goal_coordinator` (подписка `goal_pose`), отправляет в свой публичный `NavigateToPose` и далее в единственный
`bt_navigator`. **Не** использовать `/qazbot_d1_map/nav2/goal_pose`: это прямой вход `bt_navigator` в обход шлюза владения целью.
Начальная гипотеза не является подтверждением точности. Точка base_footprint соответствует проекции грудного IMU, не таза.
Файл проверен как YAML (11 дисплеев, 6 инструментов); в самом RViz с дисплеем на роботе не открывался, проверка оператором на ПК.

## Шаги оператора (сухой режим на аппаратном DDS; робот не движется: адаптер `live:=false`, издателя MC нет)
1. На ПК скопировать с X2 `ws/src/navigation/rviz/` и запустить `bash rviz_live/run_local_viewer.sh`.
2. **2D Pose Estimate** в истинной позе робота. Проверить, что Nav2 закончил активацию: `ros2 lifecycle get /qazbot_d1_map/nav2/bt_navigator` → `active [3]`
   (пока позы AMCL нет, `planner_server` ждёт `map→base_footprint`, `bt_navigator` неактивен).
3. Робот стоит в подтверждённой позе. На X2 (`source ws/src/navigation/env.sh`):
   `ros2 service call /qazbot_d1_map/localization_trust/baseline_start std_srvs/srv/Trigger`, ждать ≥ 20 отсчётов (~5 с), затем `…/baseline_finish` (должно вернуть success),
   затем `…/confirm_anchor`. Проверить: `ros2 topic echo --once /qazbot_d1_map/localization_trust/status` → `trusted: true`
   (после 5 с чистой оценки; пока причины `settling`/`T…` — доверия нет). Положение «подтверждено» должно быть измерено, а не «на глаз» (процедура F0, `X2_FIRST_PHYSICAL_TEST_PLAN.md`).
4. **2D Goal Pose** → цель. Наблюдать: глобальный и локальный планы в RViz; `ros2 topic echo /qazbot_d1_map/motion_adapter/dry_run_cmd`;
   `ros2 topic echo /qazbot_d1_map/goal_coordinator/status`; `ros2 topic echo /qazbot_d1_map/motion_adapter/status`. Робот не двигается.
   Цель может быть отклонена (`trust`, `driver_not_ready`; в журнале координатора `Goal rejected (<причина>): …` со всеми условиями готовности, например `Goal rejected (driver_arm: …)`; цель, отправленная сразу после восстановления доверия/resume, до того как `goal_coordinator/status` покажет `ready: true`, отклоняется — это безопасно) или завершиться `ABORTED` (например, таймаут подтверждения у
   `bt_navigator` при нагрузке, см. `X2_NAV2_CONTROLLER.md`, известные дефекты); это безопасные исходы.
5. **Остановка цели**: `ros2 service call /qazbot_d1_map/goal_coordinator/stop std_srvs/srv/Trigger` (закрывает шлюз, отменяет цель, защёлка). `…/goal_coordinator/resume` снимает
   защёлку, но старую цель не возобновляет: нужна новая цель. Остановка процессов: `X2_NAV2_CONTROLLER.md`, раздел 7 (адресно SIGINT, не `pkill`).

Состояние на 2026-10-21: на аппаратном DDS работают локализация (FAST-LIO портативный бинарник, карта `first_map_nav2_v2`), trust, адаптер `live:=false`, Nav2 и координатор (`logs/*.pid`:
`localization_v2_dryrun_01`, `trust_dryrun_01`, `adapter_dryrun_01`, `nav_full_dryrun_02`); PID перед остановкой проверять по `ps`, файлы PID не доказывают, что процесс жив.
Старый `localization_manual_01` заменён.
