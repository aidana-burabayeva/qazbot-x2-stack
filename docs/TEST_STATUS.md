# 2026-11-01 — NIGHTLY-M1 preparation, not milestone completion

[Current evidence](evidence/nightly_m1_20261101/README.md): both ordinary-host CLI challenges passed. Added a minimal trusted-helper invocation correction so capability fingerprinting retains daemon PATH instead of login-shell PATH. Full regression **300 PASS**, existing Merge Queue gate **165 PASS**, robot source **73/73**; source integrated as `ef507af0d12c`. Queue remains **frozen / STOPPED_OK** after completed operator STOP. No daemon/agent resume and no actual accepted AMCL engineering result or independent-review cycle claimed. Older session failures below are historical, not ordinary-host execution status. Physical X2 untouched.

---

# 2026-11-01 — autonomous engineering execution repair

Read [engineering lead policy](nightly/ENGINEERING_LEAD.md) and [evidence](evidence/engineering_lead_20261101/README.md). Historical provider/adapter failures are separated from navigation defects; original attempts are retained. WAL cursor lock reproduced and corrected without increasing busy_timeout. AMCL effective P0, blocked until actual provider execution is qualified. Both real session probes failed; no genuine verified engineering CLI result or live daemon restart claimed. Stored RUNNING_ENGINEERING is historical; systemd access is denied and the observed lease was free. Physical X2 untouched.

---

# 2026-11-01 — capability-scoped Nightly bootstrap

The global L3-receipt bootstrap deadlock is replaced by ENGINEERING / L3_QUALIFICATION / FULL_NIGHTLY in the existing queue. Read [execution modes](nightly/EXECUTION_MODES.md) and [current evidence](evidence/nightly_modes_20261101/README.md). Engineering uses its own preflight; it cannot certify simulation. The first official MC scope is bounded/stationary and remains UNVERIFIED until real metrics exist. Actual systemd/Docker access is blocked here; no runtime start or real CLI engineering task is claimed. Saved STOPPED_OK/frozen checkpoint retained. Physical X2 untouched. Historical reports below are not current verification.

---

# 2026-11-01 — AIMDK-aligned sensor/runtime qualification

Read [AIMDK fidelity gates](nightly/AIMDK_FIDELITY.md) and [evidence](evidence/aimdk_fidelity_20261101/README.md). Recorded cloud fixtures and source sensor contract corrections implemented. Runtime E2E is NOT READY: Docker/systemd are inaccessible here. A PASS lite doctor is insufficient for unattended start; a current verified L3/E2E receipt is now required. No physical X2 access; existing queue remains frozen.

---

# 2026-10-31 — navigation optimization P0 continuation

[Current implementation and gates](nightly/NAV_OPTIMIZATION.md), [evidence](evidence/navopt_p0_20261031/README.md). Shared exact image transformation and native Humble DDS-GID probe implemented; compile/offline tests verified. Live doctor/DDS/official-MC qualification remains UNVERIFIED due restricted Docker/systemd access. Append-only archive implemented; automatic optimization disabled. Physical X2 untouched, queue remains frozen.

---

# 2026-10-31 — terminal LAB bootstrap continuation

`labctl.sh` implementation: [runbook](nightly/LABCTL.md), [qualification evidence](evidence/lab_bootstrap_20261031/README.md). Control checkout `lab/control`; integration ref `lab/stable` remains un-checked-out. Physical X2 untouched. Full runtime acceptance is INCOMPLETE because this environment blocks Docker, user systemd and GitHub DNS. The saved STOPPED_OK checkpoint is preserved; do not report a new successful start/stop.

---

# 2026-10-30 — Codex bounded s1_0046 A/B

79 adapter tests + 3 frame-audit tests PASS. Ровно два queued sim runs, `resource=sim`: eps=.02 ABORTED 107.94 с; eps=.10 SUCCEEDED 37.60 с. Causal conclusion OPEN из-за другой stall-позы A и world/plant frame confound. Ни hardware qualification, ни production fix не заявляются. Подробности: [nightly/CODEX_STALL_AB.md](nightly/CODEX_STALL_AB.md), D3 и [nightly/HANDOFF.md](nightly/HANDOFF.md).

---

# 2026-10-29 — P20 / TWIN-02: новые результаты симуляции

| Проверка | Результат / границы |
|---|---|
| P20 специальные регрессии | 18 PASS; wrap, gap/backstep, stale/non-finite feedback, MC limits, overspeed, direction, translation, zero/T1, anti-windup, dead-time, takeover |
| Adapter + native + deterministic TWIN-02 | 122 PASS, 0 skip, включая существующие ROS native node tests |
| T5 / READY / Trust gaps | 62 PASS, 2 существующих `invalid value encountered in cast` warnings |
| C++ recovery core + UBSan | 16 reset scenarios + дополнительные safety/increment checks PASS |
| Actual DDS recovery/source + TrustEvaluator/AdapterCore | 8 PASS, confirmed/unexplained, 0.467–1.6 м, реальные паузы до 40 с, NaN |
| Official MC/MuJoCo after-walk baseline | CCW 0.006 рад/с; CW -0.202 рад/с, CW 90° не достигнуты за 10 с |
| Official MC/MuJoCo after-walk P20 | CCW 0.501 рад/с, 3.47 с, error +0.125 рад; CW -0.451, 4.55 с, error -0.080 рад |
| Full Nav2/GC/P20 truth-localisation | CCW и CW SUCCEEDED; XY 0.098/0.094 м, yaw 0.015/0.069 рад |
| S2/S3 full final-alignment time | OPEN: CCW goal целиком 31.8 с с повторными XY-подходами; отдельное время final rotate не инструментировано |
| Native leg stream в official MC sim | Не наблюдался; первый native-bridge probe TIMEOUT. P20 sim uses explicitly labelled torso FK surrogate |
| Initial 4-CPU / GLFW runs | FAIL, сохранены; CPU throttling и stale X startup. A/B использует одинаковые 8 CPU/320×180, fresh containers |
| main_office trinary map / world | 413052 unknown, 13712 occupied, 180955 free; actual map_server 0 unknown→free; world coverage exact |
| Physical robot deployment / AMCL navigation acceptance | NOT RUN; simulation evidence их не заменяет |

Материалы, ограничения и воспроизведение: [P20_TWIN02_HANDOFF.md](P20_TWIN02_HANDOFF.md), [evidence](evidence/p20_twin02_20261029/). Production source guard/adapter/Trust, T1/T5 thresholds, I1 и main не менялись. Recovery работает только в экспериментальном стенде и не имеет квалифицированного физического evidence provider. Предыдущие записи ниже — история, а не новые прогоны.

---

# Статус проверок QazBot X2

Обновлено 2026-10-28 ~23:00 CST (вечерняя сессия nav, X2). Полные ограничения и текущие процессы: [HANDOFF.md](HANDOFF.md).

| Новая проверка | Среда | Результат |
| --- | --- | --- |
| I1 bounded geometry unit tests | ПК | 14 PASS |
| Verified deployment/rollback unit tests | ПК | 5 PASS |
| Actual ObstacleLayer assertions | ПК / Nav2 1.1.20 | 21 PASS |
| Existing native odometry/clock | ПК | 29 PASS, 5 Node cases deselected |
| Real capture | X2 UDP-only | 100 clouds + 100 matched odometry; quota met |
| Real bridge replay | ПК | 71 usable clouds; 0 changed scan/lost marks/outside-FOV/unsupported rays |
| Actual DDS/TF/ObstacleLayer A/B | X2 isolated domain 88, Nav2 1.1.18 | 44 functional checks PASS, 4/8 m windows |
| Standalone Costmap shutdown | X2 | FAIL in both baseline and candidate; exit -11 even after cleanup |
| Approved passive installation 2d240dbd | X2 | PASS transaction, then verified ROLLBACK after freshness FAIL; T5 untouched |
| Live I1 Scan/PointCloud2/native odom | X2 | Data present, single publishers, native_imu fault empty |
| Passive physical Costmap | X2 | Active during test; ROI lethal 0→55 after operator placement; removal NOT RUN |
| New AMCL seed/Trust after I1 installation | X2 | Seed accepted, map→odom, READY reached; 0 T5 failures |
| Live sensor/odom freshness | X2 | FAIL: 55/1356 post-seed verdicts blocked; max scan1.207120 s, odom msg .731287 s |
| Owned-process cleanup and rollback | X2 18:56:05 +08 | PASS; baseline hashes restored, native HAL left running |
| Physical NavigateToPose/RPP action (до 18:56) | X2 | NOT RUN тогда; см. строки 2026-10-28 вечер ниже |
| P1 latency breakdown age_probe (A 2-iface / B 1-iface, 19:20–19:47) | X2 | DONE: первая задержка = raw HAL cloud под CPU нагрузкой (max .376 с после scan end); level_bridge накопление ОПРОВЕРГНУТО; T1 scan_stale A 3/600, B 0/591 |
| Trust после seed 60 с (P5a, 20:0x) | X2 | 293/293 READY+MOTION_OK, 0 T1 |
| Trust с Nav2 (P5b) | X2 | MOTION_OK 83/101 (18 T1_scan_stale, load ~30) |
| T1 pause unit tests (P8c) | X2 | 198 passed / 4 skipped (loc_trust+motion_adapter+goal_coordinator+nav_e2e); новые 5+7+11 |
| Независимая верификация T1 pause (P8v) | X2 | PASS: 290 passed / 5 failed (env, не связаны) / 7 skipped; 8 adversarial scratch tests; F1 flapping, F2 resume step |
| GC P10 ordering tests | X2 | goal_coordinator 15 passed; loc_trust 130 passed/4 skipped; motion_adapter 50 passed |
| P11/P13 BT/launch | X2 | xml parse + py_compile + generate_launch_description OK; live НЕ проверено на заблокированном goal |
| failure_tolerance 10.0 / max_beams 180 live param | X2 22:48–22:52 | read-back OK; эффект не проверен |
| Физические goals 2026-10-28 | X2, оператор | 3 SUCCEEDED (21:05 ~2.2 м 12.5 с; 22:12 A 4.9 с; 22:42 #1 8.8 с); 2 ABORTED (20:54 goal occupied, 20:56 RPP collision); 22:42 #2 RPP collision abort; 3 CANCELLED (21:07 T1/latch, 22:12 B T5 0.1045 м, 22:45 T5 при 0.25). **3 подряд — НЕ достигнуто** |
| Unit-test suites после 22:28 | — | НЕ запускаются по указанию оператора; только live проверки |

**Текущие блокеры (подробно в HANDOFF):** (1) AMCL не сходится → T5 отменяет goals; (2) HAL nice 0 vs -19 (нужен sudo); (3) метки вне FOV; (4) RecoveryNode/failure_tolerance 10 с не проверены; (5) resume-скачок 0→0.3 м/с; (6) adapter не удаляет MC source; (7) ros2 CLI нестабилен; (8) recorder sqlite → mcap; (9) RC takeover latch не доказан на железе.

T5 stationary LIVE accepted in checked conditions: 4 operator seeds, max correction 0.0327434893 m / 0.0193353509 rad, 0 false T5, final READY interval 124.9717 s. Strict zero-time-warning criterion FAIL: two earlier scan stale and four seed future-TF warnings 8–34 ms; protections remained enabled. No false-pose jump injected live; earlier replay/synthetic evidence retained. No claim of full physical navigation readiness.

Evidence: [real cloud](evidence/I1_real_cloud_final_20261028.json), [4 m A/B](evidence/I1_costmap_ab_points4_20261028.json), [8 m A/B](evidence/I1_costmap_ab_points8_navudp_20261028.json), [shutdown limitation](evidence/I1_costmap_ab_managed_shutdown_20261028.json), [deployment snapshot](evidence/i1_deploy_2d240dbd/passive_snapshot.json). Code/reproduction commands are in I1 commit 2d240dbd docs/I1_BOUNDED_CLEARING.md. Historical failed capture/crash and T5 evidence remain versioned; do not repeat closed tests without new evidence.

READY spans include blocked intervals; do not report 275.594 s as continuous READY. Freshness events predate the added obstacle observer and persist after the observer was lightened. Upstream/cloud-vs-scan evidence narrows the next measurement, not a proven root cause. Operator placement completed while shutdown was underway; removal was not requested. Future operator steps must be sequential and await explicit completion.

| 2026-10-29 bring-up после reboot (P15) | X2 | стек поднят (loc, Trust, adapter 1029a, nav_full, recorder); MC после reboot PASSIVE_DEFAULT, ADD после ручного stand; native_odom_bridge залип при stand-up -> рестарт localization+Trust; после initialpose (14:57) Trust READY, sigma .32, second_mode .53, scan_score .38-.45 |
| 2026-10-29 raw_scan_bridge A/B (P16, стоя) | X2 | raw 18 % ядра против FAST-LIO+level_bridge 44 %; разница дальности 2.8 см; map-match 0.1 м raw .550 vs lio .519 (900 с: .337/.689 vs .324/.633); в движении A/B не получен (нет goals), `ab_motion_1708.json` |
| 2026-10-29 footprint check (P18, `p18_fpcheck.py`, read-only) | X2 | круг 0.464: lethal 72/72 курсов; полигон: 0/72; параметры local+global прочитаны (rclpy), `published_footprint` 8 точек; Humble RPP 1.1.18 |
| 2026-10-29 калибровка in-place поворота (P19, adapter path, оператор одобрил) | X2 | CCW с места: 0.4->+0.10, 0.6->+0.53, 0.8->1.25, 1.0->1.87 рад/с; CW 0.4->-0.27, 0.6->-0.54; после ходьбы 0.6 -> 0.2-0.37 рад/с (stall 23-38 %). Часть 1 прервана по критерию translation 0.3 м |
| 2026-10-29 прямой MC тест (P19, `p19_mc_direct.py`, обход adapter) | X2 | CCW <=0.5 нет устойчивого поворота, 0.6 -> 0.35 рад/с; прервано по translation 0.3 м, CW ramp не выполнен; вывод: причина в MC; DELETE source code 0, но MC sticky `sdk_qazbot_calib_direct` до касания RC |
| 2026-10-29 P20 yaw-rate loop | офлайн replay + симуляция | 0.50-0.62 рад/с вместо 0.10-0.40; 90 град 3-4.4 с, перерегулирование 0-8 град (26 град при stall); py_compile OK, dry-run OK; **живой тест НЕ выполнен**, не развёрнут |
| 2026-10-29 демо-goals | X2, оператор | SUCCEEDED: 17:1x GOAL1 8.6 с; 18:1x 8.5 с; 20:5x A 9.2 с и B 11 с (2 подряд на текущей конфигурации); FAILED/отменены: GOAL2-4 (17:1x, BT/lethal/RPP collision), 18:0x и 18:5x (xy достигнут, финальный разворот залип в MC), around-table 18:2x (RPP collision). Детали в HANDOFF |
| 2026-10-29 20:5x демо-goals (старая карта v3 despeckled, adapter 71db8ebd) | X2, оператор | **2/2 подряд SUCCEEDED**: A (-7.38,6.02)->(-5.20,2.82) 9.2 с ~3.9 м; B ->(-5.85,-2.27) 11 с ~4.9 м. Конфиг: T5 1.0/0.5, yaw tol 0.35, inflation 0.5/0.6, полигон footprint |
| 2026-10-29 21:47-21:5x переключение на карту D1 `Qazbot_29.10` | X2 | load_map 0, AMCL 673x903; ошибка D1 map_saver `free_thresh 0.25` (unknown 205 -> FREE) исправлена на 0.196 (оригинал `map.yaml.orig_from_d1`); initialpose: READY, sigma 0.41, second_mode 0.53, **scan_score 0.47** (старая карта 0.35-0.40). Goals на D1 ещё не выполнялись |
| 2026-10-29 21:56-22:00 P20 yaw-rate loop LIVE (adapter 9c05bb34, оператор одобрил) | X2, STAND_DEFAULT | **развёрнут** (DELETE+ADD `sdk_qazbot_nav2_1029a`). Тест 1 CCW 0.5 -> 0.563 рад/с, прерван по drift 0.30 м. Тест 2 (MAX_DRIFT 0.5): CCW 0.5 -> **0.534** рад/с (170 град, u 0.60-0.77); CW 0.5 -> **0.354** (u до 0.85); 90 град CCW: 3.3 с до 5 град, финал **+11.5 град** (обратный доворот); 90 град CW: 4.1 с, финал -4.9 град; дрейф 0.16-0.23 м на поворот. После ходьбы и в goals живьём не проверен. `evidence/nav_session_20261029/calib/p20_yaw_test_*` |

<!-- BEGIN LAB_STOP_TESTS -->
### Nightly lab regression at stop (2026-11-01T02:11:16+0500)
- `python3 -m pytest -q -p no:cacheprovider tests/nightly tests/motion_adapter`: **PASS** — 299 passed in 317.22s (0:05:17)
<!-- END LAB_STOP_TESTS -->
