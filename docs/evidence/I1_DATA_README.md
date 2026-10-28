# I1 capture, 2026-10-28: no registered clouds

**These two bags do not validate I1.** Both contain zero
`/qazbot_lio/cloud_registered` and zero LIO odometry messages.
The first contains 437 LiDAR IMU and 6306 native leg-odometry messages;
the second contains 16487 native leg-odometry messages only.

The vendor HAL sensor process PID 2623 produced a crash dump at 16:50:58
robot time (+08:00), during first capture startup. The text trace passes
through PointCloud2 publishing and Fast DDS 2.14.6 SharedMemTransport mutex
locking. A causal diagnosis beyond this stack and timing is not established.
The failed recordings and logs are retained at the operator's request.

Use [I1_capture_manifest_20261028.json](I1_capture_manifest_20261028.json)
for exact paths, per-file SHA256, counts, runtime provenance and the retained
core-dump location. Both SQLite databases pass quick_check; decompressing
the committed gzip reproduces the original database SHA256 exactly.

In a fresh checkout, restore each raw bag before ROS tools:

```sh
gzip -dk docs/evidence/i1_cloud_registered_20261028_084925Z/*.db3.gz
gzip -dk docs/evidence/i1_cloud_registered_20261028_084925Z_r2/*.db3.gz
ros2 bag info docs/evidence/i1_cloud_registered_20261028_084925Z
ros2 bag info docs/evidence/i1_cloud_registered_20261028_084925Z_r2
```

Raw .db3 derivatives are ignored by Git. All recorded messages are retained
losslessly in the compressed databases. metadata.yaml and capture summaries
are committed beside them. This does not authorize replay on the physical
robot graph, deployment, MC startup or motion.
