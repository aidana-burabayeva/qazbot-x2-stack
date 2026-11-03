# 20261021-x2-nav2-e2e-dryrun — liolat (FAST-LIO one-scan wait, pinning, adapter idle CPU)

Date: 2026-10-22. Role: robotics-implementer. Scratchpad (scripts, raw data):
/tmp/claude-1001/-home-agi/c9b41d51-5b1b-455f-88cf-eb93053f257e/scratchpad/liolat_*
Bag: /home/agi/control_backend/logs/bags/liolat_lidar_imu_01 (61 s, 620 clouds + 12232 IMU, 510 MB; disk 381 GB free).

## Status: DONE (item 1 root cause found and reproduced offline; item 2 live-pinned + launch option; item 3 cause measured, no code change)

## Live changes made
- `taskset -a -p -c 5,6 26189` (all 16 FAST-LIO threads -> CPUs 5-6). To undo: `taskset -a -p -c 0-7 26189`.
- Nothing else in the live stack was touched or restarted. AMCL/trust untouched.
- Each diagnostic was one long-lived process on domain 0: ros2 bag record (60 s), latprobe (3 x 60 s),
  syncprobe (C++, subscribe only, 60 s), cntprobe (30 s), one-shot graph snapshot, rates probe (10 s).
- Offline replays ran in ROS_DOMAIN_ID=88 + ROS_LOCALHOST_ONLY=1 with a separate FastDDS profile
  (SHM + UDP on 127.0.0.1 only). Each replay process was stopped after its run.
- During the scratch rebuild of the patched FAST-LIO (about 3.5 min of colcon/make) CPU load was
  temporarily higher.

## 1. One-scan wait: ROOT CAUSE = lidar/stamp buffer desync after a "lidar loop back" (bug in shipped FAST-LIO code)
### Code facts (candidates/fast_lio_portable/src/FAST_LIO)
- lidar_type 5 = AIMA (our x2 patch, preprocess.cpp:484 aima_handler). The `timestamp` field is float64 ns
  after header.stamp. timestamp_unit 3 (NS) gives time_unit_scale 1e-6, so curvature is in ms. Points are
  sorted by time, and the last point is about 95.2 ms. The bag confirms 332 us..95.18 ms. Config is CORRECT.
- Header stamp = scan START. The vendor writes the cloud at stamp + 0.100. Probe: lidar_end_time = stamp + 0.095.
- sync_packages (laserMapping.cpp:392-447) matches upstream. When the bag cloud arrives, the newest IMU
  stamp is already past lidar_end_time (median +3 ms). So, going by the data, every scan can be processed
  as soon as it arrives.
- BUG: in standard_pcl_cbk (line 296-300) and livox_pcl_cbk, a stamp regression ("lidar loop back") runs
  `lidar_buffer.clear()` and does NOT clear `time_buffer`, its 1:1 paired stamp queue. This is the same
  as upstream FAST_LIO. If a scan is pending at that moment, time_buffer keeps one extra stale entry
  forever. Every later scan k is then processed promptly but with lidar_beg_time = stamp(k-1): IMU window
  and deskew are 100 ms off, and odom/TF/cloud_registered are stamped one scan early. Seen from outside,
  this looks exactly like a "one-scan wait".
### Evidence
- Live log localization_v3_dryrun_02.log line 172 (14:10:21): "lidar loop back, clear buffer", then
  "Too few input point cloud!", then 2x "No point, skip". All latency measurements so far (debugger
  14:19 onward, and mine) were taken after this event.
- Passive replica of FAST-LIO inputs on the live topics (syncprobe: same QoS, SingleThreadedExecutor,
  100 Hz timer): lidar callback at stamp+0.102. The sync condition was met 3.5 ms later (p95 4.9 ms).
  The input path is not delayed.
- Offline replay of the bag into the SAME binary: odom 23 ms after cloud publish (p95 32), 0 % after the
  next scan. The same result came with level_bridge attached (25 ms) and after an induced 0.35 s SIGSTOP
  stall (drains, then 25 ms).
- Live content match (cntprobe: the point count of the dense cloud_registered vs the preprocess count
  of each raw scan): 255/279 registered clouds carry the points of the scan AFTER the one their stamp
  belongs to (offset +1). They are published 34 ms after that scan arrives. FAST-LIO is not waiting.
  Its stamps are one scan behind.
- Offline reproduction: one empty cloud stamped 50 ms back was injected while scan k was pending.
  ORIGINAL binary: latency 26 ms before, then 126 ms permanently (+100 ms exactly). The content offset
  becomes +1 (326 scans).
  PATCHED binary: 30 ms before and 30 ms after, content offset 0 for 608/608.
- No trajectory change from the patch when no loop-back happens: in normal replay, orig vs orig2 vs
  patched give 609/609 identical poses (dpos 0, dang 0) at identical stamps.
### Proposed fix (NOT applied live; needs a FAST-LIO restart, which would lose the AMCL pose)
- Patch: /home/agi/control_backend/candidates/fast_lio_portable/lidar_loopback_time_buffer.patch.
  In both cbks, on loop back: `lidar_buffer.clear(); time_buffer.clear(); lidar_pushed = false;`.
  The patched build exists only in the scratchpad: liolat_fix/install/.../fastlio_mapping, md5 a38eb32a....
  Apply it to the candidate and rebuild only while FAST-LIO is stopped. Rebuilding while it runs would
  overwrite the running binary.
- Effect: removes the 100 ms stamp offset. Odom age goes from ~0.13-0.14 s to ~0.03-0.04 s. It also
  fixes a correctness defect: deskew/IMU propagation 100 ms off and TF stamped 100 ms early. Both matter
  while walking. A restart alone, without the patch, clears it only until the next vendor stamp regression.
- Out of scope / open: why the vendor driver sent the empty/back-stamped cloud at 14:10:21 (not in the
  bag). FAST-LIO also publishes one odom with the old stamp at the loop back (downstream may see a
  backward stamp). The patch does not change that.

## 2. Pinning / priority
- Our user: CapEff=0, RLIMIT_NICE=0, so nice can NOT be lowered below 0. RLIMIT_RTPRIO=98, so
  unprivileged chrt SCHED_FIFO/RR up to 98 IS possible. It was not used; RT for FAST-LIO plus the polling
  ikd-tree thread needs its own test. taskset (own process) is allowed.
- Live: all FAST-LIO threads pinned to CPUs 5,6. latprobe 60 s before and 60 s after (same probe):
  | metric | before | after |
  | /qazbot_lio/odometry recv-stamp p50/p95/p99/max | 0.139/0.166/0.400/0.669 | 0.132/0.143/0.151/0.449 |
  | odom_level p50/p95/p99 | 0.144/0.173/0.414 | 0.136/0.150/0.176 |
  | scan p50/p95/p99 | 0.147/0.178/0.335 | 0.141/0.156/0.176 |
  | FAST-LIO odom write - lidar receipt p50/p95/p99/max | 0.128/0.156/0.190/0.573 | 0.123/0.128/0.134/0.146 |
  | main thread CPU / runqueue wait (% wall) | 25.3 / 15.1 | 23.8 / 6.7 |
  (All values include the +0.100 s stamp offset from item 1.) The max 0.449 after pinning was a
  system-wide transport stall: odom, odom_level and scan were all hit at the same time.
- Launch option (opt-in, default unchanged): ws/src/navigation/localization.launch.py gets launch arg
  `lio_cpus` (default ''). When set, for example `lio_cpus:=5,6`, the FAST-LIO ExecuteProcess gets the
  prefix `taskset -c <list>`. The value is validated by regex and junk raises ValueError.
  Backup: localization.launch.py.bak_20261022_pre_liolat.
  Note: an explicit prefix means the global `launch-prefix` config no longer applies to FAST-LIO (it was
  not used).
- Test: tests/launch_stagger/test_launch_stagger.py gets test_localization_lio_cpus_prefix_is_opt_in
  (backup *.bak_20261022_pre_liolat). Run with env.sh sourced and
  QAZBOT_SMAC_PREFIX=/home/agi/control_backend/candidates/nav2_smac_bounds/install/nav2_smac_planner:
  6/6 OK. Without QAZBOT_SMAC_PREFIX, test_full_nav_launch_is_paced errors; that is a pre-existing
  env requirement.

## 3. Adapter idle CPU (live 11.8 % measured now, 10.9 % in the profile)
- Live input rates (10 s): /aima/mc/common/state 11 Hz, /qazbot_d1_map/tf 18.6 Hz, trusted 6 Hz, cmd 0.
- Isolated copy (domain 88, live:=false, NO inputs): 6.1-6.3 %. So about half the cost is the 50 Hz
  `_tick` timer plus the 2 Hz report, through rclpy executor wakeups (about 100 waits/s). The other half
  is about 36 input callbacks/s, roughly 1.5 ms each: rclpy wait-set rebuild + deserialization of the
  vendor msg and TF.
- Ruled out as significant: get_publishers_info_by_topic at 1 Hz (1 call/s). Per-iteration global
  executor add/remove in rclpy.spin_once(node) was ruled out too: a persistent SingleThreadedExecutor
  gave 5.85 % vs 6.25 %. Disabling parameter services gave 5.65 % vs 6.30 %.
- No trivially-correct minimal fix: the cost is rclpy per-wakeup overhead at the AimDK 50 Hz publish
  rate. Options (not implemented): (a) when not armed/ACTIVE, tick at a lower rate and compute
  _health_fault on demand. This changes reaction timing and needs safety review plus tests. (b) Port the
  node loop to rclcpp. (c) start_parameter_services=False saves about 0.6 %. Foreign-publisher
  detection is unchanged in all options. The live adapter was not restarted. motion_adapter tests were
  not run because no adapter code was changed.

## Files changed
- ws/src/navigation/localization.launch.py (lio_cpus opt-in arg). Backup: .bak_20261022_pre_liolat
- tests/launch_stagger/test_launch_stagger.py (+1 test). Backup: .bak_20261022_pre_liolat
- NEW candidates/fast_lio_portable/lidar_loopback_time_buffer.patch (proposed, not applied)
- NEW logs/bags/liolat_lidar_imu_01 (bag)
