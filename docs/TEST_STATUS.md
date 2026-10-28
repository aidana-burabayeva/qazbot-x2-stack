# Статус проверок QazBot X2

Обновлено 2026-10-28. Полные ограничения и текущие процессы: [HANDOFF.md](HANDOFF.md).

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
| Approved passive installation 2d240dbd | X2 | PASS transaction; backup, SHA256, syntax verified; T5 untouched |
| Live I1 Scan/PointCloud2/native odom | X2 | Data present, single publishers, native_imu fault empty |
| Passive physical Costmap | X2 | Configured/active, real lethal cells; operator obstacle test pending |
| New AMCL seed/Trust after I1 installation | X2 | WAITING operator initialpose; NOT_READY expected before seed |
| Physical NavigateToPose/RPP action | X2 | NOT RUN; separate authorization required |

T5 stationary LIVE accepted in checked conditions: 4 operator seeds, max correction 0.0327434893 m / 0.0193353509 rad, 0 false T5, final READY interval 124.9717 s. Strict zero-time-warning criterion FAIL: two earlier scan stale and four seed future-TF warnings 8–34 ms; protections remained enabled. No false-pose jump injected live; earlier replay/synthetic evidence retained. No claim of full physical navigation readiness.

Evidence: [real cloud](evidence/I1_real_cloud_final_20261028.json), [4 m A/B](evidence/I1_costmap_ab_points4_20261028.json), [8 m A/B](evidence/I1_costmap_ab_points8_navudp_20261028.json), [shutdown limitation](evidence/I1_costmap_ab_managed_shutdown_20261028.json), [deployment snapshot](evidence/i1_deploy_2d240dbd/passive_snapshot.json). Code/reproduction commands are in I1 commit 2d240dbd docs/I1_BOUNDED_CLEARING.md. Historical failed capture/crash and T5 evidence remain versioned; do not repeat closed tests without new evidence.
