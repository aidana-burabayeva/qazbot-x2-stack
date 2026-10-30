# Nightly — статус (обновлено 2026-10-30)

| Фаза | Статус |
|---|---|
| A. baseline, dirty-state, Henki | DONE: run/nightly/baseline (локально), Henki SHA256 1bc1cd6b…, путь /home/adn/henki_ros2_best_practices.md |
| A. CCW trace gate | DONE (sim): корень найден, A/B 3/3 vs 2/4, см. DECISIONS D1. Робот: NEEDS_HARDWARE |
| B. карта/мир, двери | DONE (static): `tools/nightly/map_analysis.py`, `docs/nightly/map/main_office_analysis.json` + overlay. 4 региона, 449 м² main free (unknown=obstacle). Дверные кандидаты UNVALIDATED: (-11.54,-4.48) w=1.6 м; (-6.49,-6.93) w=0.80 м — при ширине робота 0.712 м запас ≈0.04 м/сторона (NARROW). Мир симулятора ≠ карта Nav2: валидация в мире OPEN |
| C. генератор сценариев | DONE: `tools/nightly/scenario_gen.py` (seed, 10 позитивных + 4 негативных категории, независимые yaw, геом. достижимость по диску half-width 0.356 м, детерминизм проверен), `tests/nightly/test_scenario_gen.py` 4 passed. Кинодинамика/doorway-безопасность: L2/L3 |
| D. L1 C++ полигон | DONE (MODEL): `tools/nightly/l1/l1_runner.cpp` (A* по сетке с инфляцией, unicycle, swept footprint по реальному 8-угольнику, unknown=obstacle), `run_l1.sh`, тест. Пропускная способность: 1100 сц/с на 12 потоках (1100 сценариев за 1.0 с). Находка (MODEL, не Nav2): проём офиса w=0.80 м почти непроходим в strict-режиме; narrow-режим (half-width 0.356) даёт коллизии 14/20 + 19/20; ниже разрешения карты (0.05) — вердикт требует L2/L3 |
| E. L2/L3 выборочно | IN PROGRESS: `run_l2.sh` работает (r1/r2 — баги harness: `--goals` с минусом, зашитый start_pose в overlay; исправлены). **Находка L2**: s1_0005 коридор->офис (старт -13.6,14.0,-2.3 -> цель -5.8,-7.2,-0.6): NavigateToPose ABORTED 153 с, controller `Failed to make progress` x5, робот встал в (-7.33,-4.13) за 2.9 м до проёма, дребезг vx≈0.07/wz≈±0.4, truth-скорость 0.014 м/с; зазор мин 0.225 м, коллизий нет. Причина НЕ установлена (гипотеза: RPP rotate-to-path / collision-detection dithering). Evidence: docs/evidence/nightly_20261030/l2_doorway_stall. Задача eng_l2_doorway_stall = NEEDS_ENGINEERING; контрольные L2 corridor_internal/wing/s1_0004 в очереди |
| F. очередь SQLite, make nightly, Claude/Codex handoff | OPEN (существующий `make nightly` = lite-тир + I1 A/B, расширять, не заменять) |
| G. пилот, восстановление | DONE (малый): конечный бюджет — L1 seed 401 (1100 сц.), 3 L2 (s1_0048 PASS, s1_0055 PASS, s1_0051 FAIL: Nav2 rc=1, min зазор 0.075 м), отчёт run/nightly/report.html; всё прогнано очередью без участия, параллельно Codex ставил свои задачи в общую очередь. Восстановление: kill -9 демона/воркера VERIFIED. Недетерминизм: повтор s1_0048 дал min зазор 0.244 -> 0.225 м (одна карта/seed) — учитывать в A/B (нужно N повторов). Перезагрузка ПК и первый реальный запуск 01:00 таймера — UNVERIFIED |
| GPU/RL/Парето/HTML-отчёт | OPEN (после end-to-end цикла) |

Факты: путь карты `control_backend/ws/src/navigation/maps/main_office` не существует; актуальная карта — ws/src/maps/main_office (pgm sha 72ee4094…, идентична в worktree и control_backend). Henki-требования (SingleThreaded executor, C++ для hot path, ограниченные очереди, throttled logs, без sleep в тестах) учитывать в новом коде. Версии CLI: claude 2.1.312, codex-cli 0.165.0 (флаги автоматизации не проверены — OPEN).

## P0 Continuous Execution (2026-10-30)
- `tools/nightly/nq.py`: SQLite WAL очередь + оркестратор (flock-lease, heartbeat/timeout, сигнатуры отказов, без слепых повторов, checkpoint.json после каждого события, `recover` после рестарта).
- systemd --user: `qazbot-nightly.service` (Restart=always), `qazbot-nightly-enqueue.timer` (01:00, `make nightly`). `loginctl enable-linger` = yes (без sudo).
- VERIFIED: kill -9 демона посреди задачи -> systemd рестарт -> INTERRUPTED -> PENDING -> COMPLETED; NEEDS_APPROVAL не блокирует независимые; hang по heartbeat; PAUSED_AGENT при usage_limited.
- Codex 01a18872: `exec resume` отказан ("already has an active writer" — открыты 2 TUI `codex resume`, не трогаем). Канал передачи: `codex queue --thread <id> --message` (флаг проверен по --help; доставка UNVERIFIED до сброса лимита 2026-10-30 01:14).
- /loop: CronCreate job a56ac490, каждый час :17, session-only, живёт 7 дней (не заменяет systemd).

- Регрессия A: adapter/native/TWIN-02 набор = 122 passed (очередь, regress_a_adapter_native). Пересборка twin-образа с новым twin.launch.py: OPEN.

## L2 stall: сигнатура `controller_no_progress_commanded_rotation` (2026-10-30, sim, truth-localisation)
- Воспроизведено 3 раза из 5 L2-прогонов (s1_0005 r3/r4, s1_0046 x2): NavigateToPose ABORTED, `Failed to make progress` x5. s1_0048 PASS (48.7 с), s1_0004 SUCCEEDED, но yaw 0.40>0.35 и footprint касается unknown/obstacle по карте (12 отсчётов, -0.025 м) — `SUCCEEDED` не доказательство безопасности.
- s1_0046 (evidence l2_stall_s1_0046, с cmd_vel_nav): с 60 с робот стоит в (-9.93,-8.73), контроллер Nav2 (nav_ns) 99% времени командует wz<0 (в среднем -0.42), vx≈0.03; MC получает wz -0.36 -> -0.59 (растёт монотонно), truth: dyaw≈0 (±0.08/10 с), dxy≈0. По карте в этой позе зазор >=0.378 м при любом yaw => НЕ упор в стену.
- Гипотезы: H1 MC не исполняет смесь малый vx + wz (stall, ср. P20 CW baseline); H2 P20 yaw-петля без ограничения интегратора (команда растёт при отсутствии отклика; saturated=0, rejected=0) — вопрос безопасности/anti-windup; H3 RPP вместо чистого rotate-to-heading даёт vx>0 при большом heading error. Не проверено: контакт в MuJoCo (мир != карта Nav2).
- Следующий эксперимент (один): тот же сценарий с одним изменением — A/B по (a) выключен P20 (baseline adapter) и (b) rotate_to_heading_min_angle; НЕ менять допуски/footprint/costmap. Статус: eng_l2_doorway_stall = NEEDS_ENGINEERING.

## P0 verification matrix (2026-10-30)
| # | Проверка | Результат |
|---|---|---|
| 1 | nightly стартует без меня | VERIFIED: transient systemd timer -> enqueue -> демон выполнил (`verify_timer_unattended`); реальный `qazbot-nightly-enqueue.timer` 01:00, ближайший срабатывает 2026-10-31 (ещё не срабатывал — UNVERIFIED на деле) |
| 2 | закрытие сессии не убивает очередь | VERIFIED косвенно: демон (systemd --user, linger=yes) пережил все сессии/терминалы; перезагрузка ПК UNVERIFIED |
| 3 | kill -9 worker сохраняет результаты | VERIFIED: partial-лог сохранён, задача -> NEEDS_ENGINEERING (без слепого повтора), следующая задача выполнена; kill -9 демона -> recover -> COMPLETED |
| 4 | NEEDS_APPROVAL не блокирует независимые | VERIFIED |
| 5 | передача Claude -> Codex | VERIFIED: `codex queue` -> сессия 01a18872 ответила `ACK d63f50c` (01:16); контекст = HEAD SHA + handoff в git |
| 6 | usage limit не останавливает механику | VERIFIED на тест-очереди (PAUSED_AGENT, механические продолжились); реальный лимит Claude не наблюдался |
| 7 | commit/handoff в Git | VERIFIED: ветка test/p20-twin02-sim-20261029 |
| 8 | /loop зарегистрирован | VERIFIED: CronCreate a56ac490 (1h) и 8c5b8731 (2h), session-only, 7 дней |

## Engineering extension: машина готова (2026-10-30)
- `tools/nightly/tournament.py`: сетка конфигураций (L1 MODEL), общие seed/карта, перепроверка победителей на unseen seeds, hard-fail = контакт footprint, Парето-набор (baseline/safest/most_accurate/fastest_among_safe + фронт). Первый прогон: **ни одна конфигурация не даёт 0 hard-fail** (baseline 246/~840 контактов), поэтому `fastest_among_safe = None` — правило соблюдено, сигнала нет: коллизии даёт сам упрощённый follower L1 (срезает углы), настраивать надо Nav2 в L2, не модель. Результаты: docs/nightly/tournament/l1_20261030.
- `tools/nightly/report.py` -> run/nightly/report.html (очередь, сигнатуры, L2, Парето, карта).
- Передача инженерной задачи L2-stall Codex: отдельный worktree `/home/adn/QazBot/control_backend_codex_nightly` (ветка nightly/codex-stall-20261030), общая очередь через NIGHTLY_RUN. Результат ожидается; OPEN.
- GPU/RL: OPEN/не начато — по замерам Nav2 и MuJoCo physics не GPU-ускоряются; RL только после стабильного baseline на L2 (его нет: stall).

- Итог L2 (7 прогонов, 4 сценария): PASS 2/4 сценариев стабильно (s1_0048,s1_0055), FAIL s1_0046/s1_0005/s1_0051 (stall/abort), s1_0004 SUCCEEDED с нарушением yaw и контактом по карте. Эти сигнатуры — основной вход для инженерной работы (Codex: ветка nightly/codex-stall-20261030).

## Закрытие хвостов P0/E (2026-10-30)
- **Реальный unit**: `systemctl --user start qazbot-nightly-enqueue.service` -> Result=success, в очередь встал `make_nightly_20261030` (его сейчас выполняет демон, resource=sim). Календарное срабатывание таймера 01:00 — всё ещё UNVERIFIED до 2026-10-31.
- **Перезагрузка ПК**: задача `verify_reboot_recovery` = NEEDS_APPROVAL (нужен оператор; шаги в reason). Линжер=yes, unit enabled.
- **Claude-limit handoff**: реального лимита Claude не было. DRILL: claude помечен usage_limited -> engineering-задача автоматически ушла в `codex queue` (msg 01a18b32-2d42). Это проверка маршрутизации, не реального лимита.
- **Twin image**: `x2-twin-stack:p20` собран (TWIN_VERSION=p20, :dev не тронут), в образе есть `xy_tol` в twin.launch.py. Правки twin.launch.py/robot.env синхронизированы в /home/adn/QazBot/digital_twin (оригиналы: run/nightly/backup). Квалификационные прогоны на p20 — следующий шаг (L2 пока идут на :dev + harness override).
- **L2 doorway (doorway_crossing_in, 6 сценариев)**: Nav2 SUCCEEDED 6/6, НО по swept footprint на карте: 2/6 контакт с unknown/obstacle (-0.025 м: 1 и 9 отсчётов; ≈ полклетки карты, вердикт на границе разрешения), 2/6 зазор <=0.075 м. По правилу ТЗ (SUCCEEDED не доказательство, контакт = hard fail) это 2 hard-fail; проверка контакта в MuJoCo-мире — OPEN. doorway_crossing_out не прогонялся.
- **Codex stall**: результат ещё не получен (A/B в очереди/на ветке nightly/codex-stall-20261030).

## Codex: единственный A/B s1_0046 (2026-10-30)

DONE (experiment), root cause still NEEDS_ENGINEERING. Ветка `nightly/codex-stall-20261030`, база b6b2b93. A yaw_lin_eps=.02: ABORTED 107.94 с; B .10: SUCCEEDED 37.60 с, XY .048 м/yaw -.041 рад. Два job-а через общую очередь `resource=sim`, attempts=1, без overlap/retry. Никакие допуски/footprint/costmap/Trust/MC limits не менялись; vx сохранён.

A воспроизвёл другой stall, а измеренная привязка plant/world различалась на .091 рад против .005 рад в B. В старом stall — .106 рад, ошибка точки .69 м. Поэтому causal PASS для threshold не заявляется; контакты MuJoCo не исключены. Приоритет следующей отдельной задачи — frame contract и raw HAL/contact evidence, а не retuning P20. См. [CODEX_STALL_AB.md](CODEX_STALL_AB.md), D3, evidence `codex_stall_ab`. 79 adapter + 3 offline audit tests PASS. Физический X2 не тронут; deployment нет.

## Принят результат Codex (merge origin/nightly/codex-stall-20261030, 2026-10-30)
- yaw_lin_eps 0.02 -> ABORTED 107.9 с, 0.10 -> SUCCEEDED 37.6 с (XY .048, yaw -.041): ровно один A/B, причинный вывод НЕ заявлен (A остановился в другой позе, привязка plant/world различалась на .091 против .005 рад).
- **Следствие для ВСЕХ моих L2-выводов**: map-clearance по nav-позе не равен зазору в MuJoCo-мире (расхождение позы до 0.69-0.73 м). Метки «контакт/зазор» в L2 doorway и s1_0004 = UNVERIFIED относительно физического мира; нужен frame-contract + raw contact evidence. Приоритет выше ретюнинга P20.
- Адаптерный фикстур `QAZBOT_SIM_YAW_LIN_EPS` защищён SIM-доменом 87, дефолт продакшна не меняется (ревью диффа выполнен; adapter+nightly тесты 87 passed).

## ОСТАНОВЛЕНО оператором (2026-10-30 ~10:45)
Лаборатория остановлена по указанию пользователя: демон и таймер отключены, cron удалены, sim-контейнеры остановлены, очередь припаркована, Codex получил OPERATOR STOP. Физический робот доступен; подготовка — docs/nightly/HANDOFF.md. Незакрыто: реальное срабатывание таймера 01:00 (UNVERIFIED), перезагрузка ПК (NEEDS_APPROVAL), реальный лимит Claude (не наступал; проверена только маршрутизация DRILL), успешный `make nightly` end-to-end (build падал: vendor/DNS), причина L2-stall, L2 doorway_crossing_out, квалификация на образе x2-twin-stack:p20, frame contract. Фазы B,C,D — готовы; E,F — частично; G — малый пилот; GPU/RL — не начаты.

<!-- BEGIN LAB_STOP -->
## LAB STOP 2026-10-30T12:04:12+0500
- HEAD `4c648e3e0ad1` (test/p20-twin02-sim-20261029); очередь {'COMPLETED': 32, 'NEEDS_APPROVAL': 2, 'NEEDS_ENGINEERING': 4}; запаркованных задач 0.
- Codex: STOP_UNVERIFIED. Units: {'qazbot-nightly.service': 'inactive/Failed to get unit file state for qazbot-nightly.service: No such file or directory', 'qazbot-nightly-enqueue.service': 'inactive/Failed to get unit file state for qazbot-nightly-enqueue.service: No such file or directory', 'qazbot-nightly-enqueue.timer': 'inactive/Failed to get unit file state for qazbot-nightly-enqueue.timer: No such file or directory'}.
- Regression: PASS (117 passed in 69.66s (0:01:09)).
- Не контролируется отсюда: открытые TUI Claude/Codex, session-only /loop cron и /goal hook открытых сессий.
- Возобновление: `python3 tools/nightly/lab.py resume` (`/nightly-resume`).
<!-- END LAB_STOP -->

## P1 integration gate (2026-10-30)
- Codex review of `5b64245`: **REVIEW_PENDING** (запрос в очереди `01a18ba5`, ответа и review-документа нет). Самопроверка выполнена, найдено и исправлено: (1) `verify-evidence` мог отправить в карантин чужую in-flight публикацию — теперь только если pid-владелец мёртв; (2) fsync вложенных каталогов перед rename; `synchronous=FULL`; (3) `lab stop/resume` теперь требуют роль LAB (UNKNOWN тоже отказ); (4) role-subset установка пакета (на роботе нужен только ROBOT skill).
- Имитационные crash-тесты (`os._exit` в 4 точках) доказывают логику восстановления, **не** устойчивость к потере питания: UNVERIFIED (нужен тест с реальным отключением/`dm-flakey`).
- Регресс: `tests/nightly tests/motion_adapter` 133 passed. Лаборатория остановлена, X2 не трогался.
