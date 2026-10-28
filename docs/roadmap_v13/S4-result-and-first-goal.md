# S4 и переход к первой цели — 27.10.2026
Scope: только физический робот. Рабочая одометрия native_imu, один владелец odom->base_footprint.

## Проведено
R1 разрешён оператором. Coordinator остановлен до замены dry-run adapter. Остановлен только PID234319. Live PID249828, source sdk_qazbot_v13_s4a_1127, priority30; ADD отправлен один раз и подтверждён registration=registered. До пробы motion_armed=false, published0, MCrc/STAND. Канонический adapter PID-file обновлён. Лимиты/режим MC не менялись.
Первая попытка S4_takeover_1129: REFUSED, published0, admission увидел localization_untrusted. Запись подтверждает T1_scan_stale/T1_tf_stale. Связь с обнаружением человека сбоку не доказана.
Оператор явно разрешил повтор. S4_takeover_b_1132: +0.5рад/с, vx=vy=0, окно1.2с, native pelvis guards15°/.12м; выход2 localization_untrusted. Команда nonzero 1793100628.543600–1793100629.742725 (~1.199с), затем ноль. Native pelvis итог12.45°,31.3мм; это odometry-based, независимой ground truth нет. Оператор подтвердил стойку и отклонение/отпускание стика.
MC: rc -> SDK1793100628.627470 -> rc1793100631.948769. operator_takeover latch отсутствует; возврат RC после нуля. S4 takeover НЕ ДОКАЗАН.
После пробы adapter disarmed/stop_requested, coordinator operator_stop/gate=false; MCrc/STAND.

## Причина снятия допуска
Trust T5_map_odom_jump: изменение translation(map->odom)=0.101916м, yaw=0.0237749рад(1.3622°). initialized сброшен; нужна новая initialpose/AMCL.
В обеих поправках, применённых к одной ближайшей odom-base точке, разница положения робота=0.039614м. Ближайшая TF принята через20.65мс от поправки, поэтому это диагностика геометрии, не точное source-time доказательство ошибки локализации. Текущий translation-only T5 зависит от выбора начала odom при изменении угла. Требуется отдельная проверка координатной инвариантности; рабочие пороги/код защиты НЕ изменены. Меньшая поправка у робота не доказывает правильность AMCL.

## Доказательства
`evidence/S4_takeover_b_1132/`: bag/, step.log, step.csv*, state_transitions.json, takeover_timeline.json, map_correction_geometry.json.
`evidence/S4_takeover_1129/`: первая отказанная попытка, passive_bag_summary.json.
Recorder сначала отказал из-за nounset при source ROS setup; скрипт исправлен set+u на source; первый неуспешный recorder не двигал робота. В записи1129 tf_static и AMCL count0: bag не является самостоятельным полным replay этих состояний, baseline TF/AMCL сохранены отдельно. Это ограничение записи, не доказательство отсутствия TF/AMCL в рабочем стеке.

## Изменение процедуры оператором
Оператор: «давай без перехвата пультом будем тестировать навигацию». Отдельная проба takeover исключена по явному указанию; результат не засчитывается PASS. Защиты stale/jump и штатный stop сохраняются. Следующий шаг: новая операторская initialpose, проверка Trust и стойки, запись, явный resume только для НОВОЙ пользовательской RViz цели ~1м впереди в свободной зоне. Старые цели не воспроизводить. Ни одна автономная цель пока не отправлена.
