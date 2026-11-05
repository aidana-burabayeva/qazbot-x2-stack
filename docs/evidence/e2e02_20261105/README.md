# E2E-02 free 3 m Goal (2026-11-05 17:07 CST): lossless bag

`bag_e2e02_20261105_1707_0.db3.gz` = `gzip -9` of the rosbag2 sqlite3 file (`gunzip` gives the original; SHA256 of the original in
SHA256SUMS), with `metadata.yaml`. Goal accepted at epoch 1793869680.34 s, SUCCEEDED at +60.2 s. Topics: Nav2 plan/cmd_vel chain,
`/aima/mc/locomotion/velocity` (all sources, field `source`), `/aima/mc/common/state`, `/aima/mc/leg_odometry`, `/qazbot_lio/odom_level`,
`/qazbot_lio/odometry`, `/qazbot_d1_map/{tf,tf_static,amcl_pose}`, adapter/GC/Trust status, costmap updates, `/rosout`.
Read it with `ws/src/qazbot_approach_governor/test/tools/extract_stop_events.py` (needs the aimdk_msgs overlay) or the scripts in
`docs/results/20261105_e2e02/`. Analysis: `docs/results/20261105_e2e02_free3m.md`, `docs/results/20261105_d05_governor.md`.
