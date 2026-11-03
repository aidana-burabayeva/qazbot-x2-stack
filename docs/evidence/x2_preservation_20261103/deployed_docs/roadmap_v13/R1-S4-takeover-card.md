# R1 + S4 — подготовка live и ограниченная проверка RC takeover (ожидает разрешения)
Дата27.10.2026. Scope: физический X2, native_imu; никакого цифрового двойника.
Readiness monitor R0_readiness_monitor01: AMCL pose получена послеR0, Trust initialized/ready/motion_ok=true, native30Hz безfault, MC rc/STAND/FSM4, adapter live=false/published0, coordinator busy=false/gate=false.

## Требуется явное разрешение
Общее R0 разрешение не включало MC registration и движение.
Разрешение S4 должно включать одну подготовку live source и ровно одну ограниченную пробу. Без него SDK ADD/arm/скорости не выполняются.

## R1 — до движения
1. Проверить fresh state и отсутствие активной цели. Защёлкнуть coordinator через штатный stop, пока старый adapter ещёdry_run, чтобы случайная2DGoalPose не могла запустить движение.
2. Остановить только dry-run adapter234319, проверить освобождение lock.
3. Запустить прежний x2_motion_adapter с live=true, fresh sdk_source_name и priority30, без изменения limits/режимовMC.
4. Один register, без retry/ENABLE fallback. Требуется registration=registered, motion_armed=false, published0, MC всё ещёrc/STAND.
5. Recorder и мониторы подключены до движения. Source timestamps, обеodom, native pelvis limiter TF, вход/выход команд, MC/status/Trust/coordinator сохраняются.
Если ADD ack неизвестен — не повторять ADD автоматически.

## S4 — ровно одна проба
Команда: vx=0,vy=0,wz=+0.5 рад/с (влево,REP103).
Программное окно около1.2с; это Linux/20Hz, не гарантированная realtime миллисекундная граница.
Ранний ноль при native_base yaw15° или перемещении0.12м; реальный выбег после нуля наблюдается отдельно.
Штатный step_cmd с --limit-source native_base; его NaN/TF/source-age guards проверены25/25.
Оператор держит пульт, заранее подтверждает свободную зону вокруг робота. При начале движения кратко отклоняет и отпускает стик: это команда пульта, после отпускания робот должен остановиться.
Ожидаем SDK acquisition, затем source=rc и adapter latch=operator_takeover, zeros/silence. Если SDK не был выбран — takeover не доказан.
Любой stale/jump/ошибка/предмет/неожиданное движение => zero/stop, без повторной пробы и автоматического resume.

## После
MC source=rc, стойка, фактическое прекращение движения подтверждены оператором и записью.
Координатор/adapter остаются latched для review. После подтверждения причины/остановки разрешить только NEW goal, не replay старой цели.
Первую RViz2DGoalPose — около1м впереди, в свободной зоне, направление примерно прежнее. Возврат отдельный.

