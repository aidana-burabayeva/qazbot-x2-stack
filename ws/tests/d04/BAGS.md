# D-04 bag manifest

Bags are NOT in the repo (3.7 GB each). Set `QAZBOT_D04_BAGS` to the directory that contains the bag folders
(default `/home/agi/control_backend/docs/evidence/nav_20261102_live` on the X2). Tests/replays that need a bag
skip (pytest) or exit (replay_d04.py, nav2_real_costmap.py) with the missing path; check files with
`sha256sum -c` against the table.

| name (registry key) | path under `$QAZBOT_D04_BAGS` | size (bytes) | sha256 of the mcap |
|---|---|---|---|
| d04_static_bag_204153 (`static_204153`) | `d04_static_bag_204153/d04_static_bag_204153_0.mcap` | 3700720716 | `2b7f7adf5204f0f47391095d85b8da94e80718e65b465abb0addd951ba76d4b7` |
| d04_static_bag_211835 (`static_211835`) | `d04_static_bag_211835/d04_static_bag_211835_0.mcap` | 3698473010 | `6c953804ad1c27804933aca31c781d8e60367d4de775c3e79dfe245e8d4c86bf` |
| d04_static_bag_183015 (`static_183015`) | `d04_static_bag_183015/d04_static_bag_183015_0.mcap` | 9039573139 | `bb90e41b65874adb14e01636e5f1cd18d75d130ba7b1d2a01eb930eadbc1546f` |

`static_183015` = P-D04 real-object test 2026-11-03 (719 s, also raw `/aima/hal/sensor/lidar_chest_front/lidar_pointcloud`, not in the default replay set; analysis: `.orchestrator/reports/20261103-pd04-analysis.md`).

Recorded with `tools/d04/record_d04_static.sh` (static robot, 300 s, includes live `scan_obstacles`/`scan_clearing`/`costmap_raw`).

Legacy bags (kept out of the default replay; `--bags <key>`; root via `QAZBOT_D04_LEGACY`, default `/home/agi/control_backend`):
`udp_20261028` (`docs/evidence/i1_cloud_registered_udp_20261028_092700Z/..._0.db3`, used by `test_replay_d04.py`),
`replay_dense_01`, `replay_portable_01`, `replay_dense_controlled_01` (`logs/<name>/bag/bag_0.db3`).

Repo inputs resolved from the repo root (never absolute): baseline A `ws/src/lio/level_bridge.py`
(sha256 `c817a5ed503386be742fa7b6fdab2b3209bdefaf42be799d2a130cd198f9369b`, identical to the running file),
candidate C `docs/engineering/d04_d05_20261102/candidate/level_bridge.py`, extrinsics `docs/evidence/x2_extrinsics.json`.
