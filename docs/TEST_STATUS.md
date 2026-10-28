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
