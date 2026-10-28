# R0 — native_imu dry-run успешно запущен
27.10.2026. Отдельное разрешение оператора получено: «Разрешаю; робот стоит, повторю позу».
Runtime tag: rviz_native_v13_20261027_110741Z.
Evidence: evidence/R0_20261027_110741Z/.

## Выполнено
- Положительные SHA preconditions проверены по B0 freeze до stop/продвижения.
- Остановлены только четыре корня QazBot: localization86415, trust86641, dry-run adapter86794, nav_full92489.
- Применены11 проверенных file changes C1/C2, before bytes сохранены по каждому пути. Новые hashes/параметры — journal.json.
- Запущен native_imu; новый localization PID233530, Trust234158, dry-run adapter234319, nav_full234425, native bridge233701.
- Новая одометрия /qazbot_native/odom; Trust go2 читает её; Nav2 odom_topic согласован.
- Один owner одометрического TF: native_imu. Probe: native matches1.0, LIO0.0, 177TF samples, 30Hz; publishers /qazbot_native_odom_bridge и /qazbot_d1_map/amcl. Level bridge не публикует TF.
- AMCL/D1 v3, initialpose автопереноса нет. Ожидается новый операторский2DPoseEstimate.
- Первый MVP controller goal tolerance20см/10°, stateful=true как выбранный supervised M1 контракт. Это не изменение скоростей MC и не приёмка фактической остановки.

## Не выполнялось
Никакой регистрации/arm MC, ненулевых команд, автономных целей, изменения vendor processes или CPU affinity.
Адаптер live=false. RViz2DGoal до live допуска не приводит к физической навигации.

## Следующий шаг
Получить повторный initialpose, подтвердить native/scan/Trust health, затем отдельно подготовить R1 live-source и объявить S4 takeover.
Старый вопрос про длительный CPU A/B отложен по приоритету первого реального RViz goal.

## Rollback
Before bytes11файлов сохранены, старые runtime argv/params/libraries — B0.
Вернуться в lio возможно через controlled stop/start на выбранном release и новый RViz seed; точная старая Python память не выдаётся за восстановимую.

