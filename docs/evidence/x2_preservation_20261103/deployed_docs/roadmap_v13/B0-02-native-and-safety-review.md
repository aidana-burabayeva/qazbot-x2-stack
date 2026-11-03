# B0-02 / B2 — независимая проверка физического native_imu и кандидаты C1/C2
Дата: 27.10.2026. Scope: только физический X2 и его код/записи. Цифровой двойник исключён.
Снимок: evidence/B0_20261027_1030Z. Working runtime не перезапускался; новые source/policy patches НЕ применены к ws/src.

## Baseline
На frozen копии физического дерева, ROS Humble, domain88/localhost, без аппаратного DDS:
- native_imu: 61/61 PASS.
- Trust: 107 tests, 56 failures, 6 errors, 1 skipped. Одной ошибке способствует hardcoded ~/control_backend на PC; прочие включают stale test fixtures/default strict vs actual go2.
Это отдельный physical-code test, не симуляционный маршрут и не перенос цифрового двойника.

## C1 — Trust contract и восстановление тестов (STAGED, NOT LIVE)
1. Legacy strict tests теперь явно выбирают strict; default test проверяет go2. Починен repo-relative map path, callback fixtures получили обязательное cfg/helper state, добавлено 5 seed-refresh tests.
2. Найден отдельный safety дефект: go2 _verdict пропускает T5_map_odom_jump в diagnostics. Roadmap §0.5/P4.6 требует стоп при больших необъяснённых global jumps.
3. Кандидат включает T5 в blocking и отзывает initialization до нового seed. Пороги, covariance и scan-score gating не ужесточаются, источник odometry не переключается.
4. Новый safety regression FAIL на старом _verdict и PASS на кандидате; before/after сохранён.
Результат: 113 tests, 0 failures/errors, 1 existing skipped. 107 прежних test IDs сохранены, ни один не удалён; 6 добавлены.
Дополнительно native_imu 61/61 и motion adapter 43/43 PASS на этой копии.

## C2 — native source clock/gap defects (STAGED, NOT LIVE)
Три новые проверки воспроизвели 3 FAIL на прежнем коде:
- stamp=0 принимался как stamp_back и заменял последнюю корректную source pose;
- Twist0p3 усреднял скорость через необозреваемый многосекундный gap;
- после gap bridge выдавал Odometry до накопления нового окна скорости (существующий тест проверял только позднее восстановление и пропустил эту ошибку).
Минимальные поправки:
- source stamps <=0 отклоняются без изменения последней корректной pose/JumpGuard;
- velocity baseline очищается при нерастущем времени или source gap>NATIVE_MAX_AGE=0.2 с. После gap свежий TF возможен, Odometry отсутствует до заполнения нового 0.3-секундного окна. Это не подмена неизвестной скорости нулём.
Номинальные параметры, source stamp и default odom_source=lio не менялись.
Результат: native suite 64/64 PASS (прежние 61 + три регрессии).
Нулевой stamp был физически замечен у HAL cloud ранее, НЕ у native odometry; native дефект обнаружен анализом кода/инъекцией, не объявляется наблюдённым MC bug.

## Восстановление повреждённых ELF
11 обрезанных test executables точно совпали с префиксом полных AArch64 файлов на PC. Размер повреждений 262144 bytes; для каждого сохранены old/full hashes и exact-prefix proof.
Байты восстановлены по одному файлу с SHA precondition, backup и atomic replace; потом включён owner execute по роли test target (исторический полный mode неизвестен).
test_a_star --gtest_list_tests загрузился успешно в domain88/localhost. Это не полный C++ test run.
Рабочие библиотеки и процессы не менялись. 270 нулевых code/config в candidates и 3 повреждённых библиотеки quarantine install_corrupt_restore остаются в manifest; они не подменяются автоматически неизвестными версиями.

## Ограничения
Loaded Python source hash не установлен; frozen disk tree не равен содержимому памяти PID. Поэтому C1/C2 пока review candidates, а не новый live runtime.
Native_imu — IMU footprint level_bridge, а не native-to-pelvis shadow. Его совместимость не доказывается результатами другого footprint.
Covariance nominal, gravity axes discrepancy ~2.3°, физическая скорость/stop без нового внешнего эталона не принимаются автоматически.
Операторское движение, MC registration, arm, goals и TF-owner switches в этом цикле не выполнялись.

## Verdict
GO: offline math/guard regression кандидатов, snapshot/recovery 11 ELF.
PARTIAL: P0-C release/provenance и native physical qualification.
NO-GO: объявлять новый native owner или Trust patch внедрёнными; произвольные автономные цели.

## Next ONE step
Согласовать узкий B1 scheduler A/B на зафиксированном runtime без изменения odom owner/MC; предпочтительно сменить affinity того же PID вместо перезапуска локализации и потери seed. Подробная карточка B1-experiment-card.md.

