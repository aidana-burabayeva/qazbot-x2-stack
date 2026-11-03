# Краткий путь к первой физической 2D Goal Pose
Приоритет владельца: физическая навигация в RViz как можно скорее. Twin вне scope.
Предыдущий вопрос о 10-минутном CPU A/B отложен; его разрешение не подменяет разрешение этого шага.

## Шаг R0 — требуется согласование сейчас
Контролируемо остановить только наш dry-run navigation stack, не vendor/MC.
После проверки hashes применить reviewed C1/C2: T5 jump gate, отклонение nonpositive native stamps, очистка velocity baseline после gap. Тесты: Trust113 (1 existing skip), native64, adapter43, limiter25, stop7 PASS. Исходные файлы и библиотеки сохранены в B0 snapshot, новые bytes имеют manifest.
Запустить тот же стек в native_imu режиме, AMCL/D1 v3, adapter live=false. Никакой регистрации MC, arm, speeds, autonomous goals.
Новый native odom origin требует одного нового 2D Pose Estimate от оператора. Сохранение прежней map→odom не допускается.
Проверить single-owner TF, согласование odom/scan/Trust, healthy rates и source stamps; остановиться при reset/jump/stale.
Полный affinity stress A/B не выполняется вместе с миграцией; source gaps/latency продолжают фиксироваться. При stale — не разрешать goal.
Rollback: default lio на проверенном дисковом release и новый operator seed; точный старый Python memory image не восстанавливается.

## Затем, по отдельному объявлению
R1: подготовка live adapter, один ADD fresh SDK source priority30 без arm/скорости; проверка RC source.
R2/S4: отдельно объявленная короткая проба перехвата пультом, пределы и готовность оператора до любого движения.
R3/M1: одна цель около1м впереди в широкой свободной зоне. Настройка goal tolerance для MVP из карточки M0b рассматривается до Nav2 bringup; скорости MC не повышаются. Оператор с пультом, recorder уже подключён, не стартовать ROS tools во время движения.
Фактическая остановка измеряется по физической отметке/наблюдению, не только SUCCEEDED. Возврат — отдельная цель.

## Сейчас не блокирует первую пробу
Полный цифровой двойник, 8 sim targets, Django/backend, второй робот, VIO/камеры и сантиметровая точность.
Не снимаются блокеры: отсутствие проверенного stop/takeover, потеря необходимых данных, неясная физическая начальная область, двойной TF-owner или SDK source.

