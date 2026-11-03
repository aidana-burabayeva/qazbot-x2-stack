# Первая RViz цель — отказ collision, без движения
Цель map[-6.143385,2.579215,-1.291669], получена1793103181.404, Nav2 старт[-6.48,4.42], расстояние~1.87м. Controller RPP detected collision ahead; через~0.3с patience exceeded, NavigateToPose ABORTED. Оператор подтвердил: стоит, впереди свободно.
Рабочая защита сработала до ненулевой команды: за1793103180–3183 на cmd_vel_nav7сообщений, authorized7, nonzero0; adapter MC published остался60 от прежней S4. После отказа motion_armed оставалсяtrue, gate=false/busy=false. Это известный недочёт terminal cleanup (нет ненулевой команды, watchdog действует), далее явно coordinator.stop и adapter.stop SUCCESS. Не засчитывать goal PASS.

## Подтверждённое
В сканах при цели ближайшие точки1.75–1.77м. После цели local costmap snapshot содержал cost>=253 в0.2206м от опорной native-base точки, radius0.454+padding0.01. Layers obstacle+inflation, frameodom, scan/qazbot_lio/scan. Опорная точка snapshot была из мониторинга, точная временная привязка не выполнялась; робот стоял по оператору.
При disarmed/latched штатно вызван только local clear_entirely_local_costmap, параметры/габарит/collision detection не менялись. После3с и повторного GetCostmap ближайший cost>=253=1.2856м. Lethal254 cells246->10; inscribed253 cells3433->287; free2471->6063. Свежие препятствия после очистки присутствуют: очистка не равна отключению слоя.

## Причина и гипотезы
Накопленные локальные отметки действительно устраняются очисткой и занимали свободную окрестность робота, объясняют отказ collision. Их происхождение пока не доказано: старые наблюдения, ограниченный FOV/неполное raytrace clearing, прежние движения/изменение привязки. Не объявлять причину установленной по одному снимку. Нужна повторная цель и наблюдение повторного накопления; не уменьшать радиус ради прохода.

## Артефакты
`evidence/First_RViz_goal_C3_1205/`: stopped bag сmetadata, collision_snapshot_before_clear.json/local_costmap_before_clear.npy, collision_snapshot.json/local_costmap_after.npy, goal_collision_before_clear_summary.json, goal_collision_bag_summary.json. Bag анализировался только после остановки PID278460 и проверки отсутствия процесса.
ObserverPID274355 завершился по900с; новыйPID287570 Goal2_readiness_1216(900с), recorderPID287572 RViz_goal2_1216. Подготовка повторной цели~1м; никакого replay старой цели. C3 runtime reseed recovery подтверждён, но полная навигация пока не подтверждена.
