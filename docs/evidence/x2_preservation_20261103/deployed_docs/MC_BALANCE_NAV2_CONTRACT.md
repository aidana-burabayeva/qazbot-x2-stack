# Штатный баланс MC и интеграция Nav2 — 2026-10-22

## Подтверждённый принцип
STAND_DEFAULT — активное удержание стойки штатным Motion Controller AGIBOT.
LOCOMOTION_DEFAULT — ходьба/бег того же контура. Начиная с v0.8 штатный MC
сам переключает эти режимы по командам скорости; Nav2 не должен отдельно
переключать режимы. См. официальные документы ниже.

Высокоуровневый путь:
RViz/goal → стандартный Nav2 → velocity smoother → goal coordinator →
x2_motion_adapter → McLocomotionVelocity → встроенный MC → команды суставам.
Nav2 решает, куда ехать и какую скорость запрашивать. Он не рассчитывает
моменты суставов и не заменяет регулятор равновесия/шага.

Документация X2 связывает пороги страгивания с внутренне обученной базовой
моделью. Это согласуется с обучаемым контроллером ходьбы, но не раскрывает
точную архитектуру заводской политики STAND_DEFAULT на конкретной прошивке.
Нельзя по наличию папки mc-rl утверждать, что именно её policy.onnx сейчас
держит реальный робот или что вся заводская стойка реализована только RL.

## Важное различие: штатный MC и собственный RL
На .66 существуют extra/mc-rl и extra/x2_rl_deploy. В локальном README mc-rl
описан отдельный путь развёртывания своей политики с остановкой штатного MC
и последующим восстановлением. Это замена нижнего контроллера, а не запуск
навигации поверх заводского баланса. Такой путь для Nav2 не используется.
Веб-документация sim_rl отдельно различает RL deployment и штатный MC в MuJoCo.
В разных пакетах/версиях примеров заявления о проверке на железе различаются;
параметры этих примеров нельзя считать параметрами действующего заводского MC.

## Пассивное подтверждение на .66
docs/evidence/mc_balance_readonly_20261022_01.json:
- action_info.action_desc=STAND_DEFAULT, status.value=100 (RUNNING);
- body_status=1 (STAND), FSM=4 (STABLE), source=rc;
- единственный видимый издатель leg/arm/waist/head command — mc_ros2_node3117;
- по leg command получено 774 сообщения, observed receive rate около 324 Гц.
  Это частота приёма BEST_EFFORT Python-наблюдателем в коротком окне,
  НЕ измеренная внутренняя частота MC и не повод менять таймауты/контроллер.
runtime_model в McCommonState сообщает доступность waist/hand, а не имя RL-файла.
Конкретная заводская нейросеть/контрольная сумма модели не установлены.
На PC1 не заходили, ничего там не запускали. Создан один пассивный ROS-наблюдатель.

## Проверенный программный путь QazBot
Аудит x2_motion_adapter.py, goal_coordinator.py, nav_control.launch.py,
nav_full.launch.py, run_x2_driver.sh, run_nav_full.sh:
- MC output: только McLocomotionVelocity на /aima/mc/locomotion/velocity;
- registration: SetMcInputSource для нашего sdk_источника (приоритет30),
  это выбор источника целевой скорости, не выключение управления суставами;
- нет SetMcAction, остановки mc, deploy_x2, прямой публикации в HAL/joint command;
- stop/cancel/loss: закрытый допуск и пачка нулевых скоростей, затем тишина;
  режим и процесс MC не выключаются;
- допуск требует body STAND и FSM STABLE/MOVING, свежие state/Trust/TF;
- takeover/latch/отсутствие автоматического возобновления остаются в нашем слое.

Вывод ограничен этим кодом и пассивным снимком. Другие приложения/пульт могут
отдельно переключить режим, а остановленный/вытесненный MC нельзя восстановить
просто нулём cmd_vel. Не запускать автоматический SetMcAction в ответ на отказ.

## Обязательная граница для навигации
1. Не останавливать/заменять MC и не переводить систему в Develop_MC.
2. Не писать в HAL-команды ног/пояса/рук параллельно MC.
3. Не использовать PASSIVE_DEFAULT, DAMPING_DEFAULT или JOINT_DEFAULT как
   обычный stop/cancel Nav2. PASSIVE — нулевой момент, DAMPING — демпфирование;
   это не тот же активный баланс STAND_DEFAULT.
4. Не запускать собственные RL deploy-процессы для исправления проблем Nav2.
5. Навигационные процессы — на PC2/.66, не на PC1 (10.0.1.40).
6. Не запускать ROS-узлы пачкой: официальная рекомендация не более 2 новых
   узлов/с при стоянии/ходьбе. Перегрузка DDS может нарушить realtime MC,
   даже когда сам процесс MC не выключен.
7. Сохранять обычный путь остановки через нулевую скорость и PS5 takeover;
   штатный аппаратный emergency stop — отдельная процедура по инструкции
   оператора, его нельзя приравнивать к безопасному продолжению балансировки.
8. Не увеличивать скорости/ускорения ради порогов страгивания без измерений.
   Активный MC не гарантирует отсутствие падения при любых командах/поверхности.

## Официальные источники (прочитаны 2026-10-22)
- Motion modes:
  https://x2-aimdk.agibot.com/en/dev/Interface/control_mod/modeswitch.html
- Locomotion (описаны целевые скорости, learned thresholds, timeout и стойка):
  https://x2-aimdk.agibot.com/zh-cn/latest/Interface/control_mod/locomotion.html
- MC arbitration, приоритеты и риски DDS:
  https://x2-aimdk.agibot.com/en/latest/Interface/control_mod/MC_control.html
- Control module, запрет разработки на PC1:
  https://x2-aimdk.agibot.com/en/latest/Interface/control_mod/index.html
- Native MC versus RL deployment:
  https://x2-aimdk.agibot.com/en/latest/sim_rl/index.html
- Low-level examples, требующие остановки MC (не наш путь):
  https://x2-aimdk.agibot.com/en/latest/Interface/control_mod/endeffector.html

Изменений кода, параметров, режимов и команд движения не выполнялось.
