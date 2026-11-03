# 20261021-x2-nav2-e2e-dryrun — CPU load and FAST-LIO latency profile (debugger, passive)

Date: 2026-10-22 (host .66, uptime ~1 h after reboot). Read-only: no publish, no service calls, no
renice/restart/kill. Scratchpad: /tmp/claude-1001/-home-agi/c9b41d51-5b1b-455f-88cf-eb93053f257e/scratchpad

## 0. Status: NEEDS_DATA (stopped early on coordinator request; root driver localized, mechanism of one-scan lag unconfirmed)

## 1. System reconstruction (verified)
- Live PIDs (pid files alive): localization launch 26157 (fastlio_mapping 26189, level_bridge.py 26207,
  map_server 26233, amcl 26256, lifecycle_manager 26283); trust 26843; adapter 32079 (live:=false);
  nav_full launch 32088 (controller 32142, smoother 32167, planner 32177, bt_navigator 32207,
  lifecycle 32227, goal_coordinator 32259). ros2 daemon 26754 present.
- FAST-LIO binary: candidates/fast_lio_portable/install/fast_lio/lib/fast_lio/fastlio_mapping,
  params ws/src/lio/x2.yaml: lid_topic /aima/hal/sensor/lidar_chest_front/lidar_pointcloud (96 rings,
  10 Hz, SensorDataQoS sub), imu_topic /aima/hal/sensor/lidar_chest_front/imu (200 Hz, BE, depth 400),
  point_filter_num 3, filter_size_surf/map 0.3, max_iteration 3, dense_publish_en true, scan_publish_en true.
- FAST-LIO code structure (src/FAST_LIO/src/laserMapping.cpp):
  - single node, `rclcpp::spin` = SingleThreadedExecutor: lidar cbk (incl. full preprocess p_pre->process),
    IMU cbk (200 Hz), 100 Hz timer (whole LIO pipeline), 1 Hz map timer all serialized on ONE thread.
  - CMakeLists.txt:22-41: MP_PROC_NUM=3 only on x86; on aarch64 -> MP_PROC_NUM=1, so the OpenMP
    nearest-search loop (line 695) is single-threaded on the Jetson. env.sh also exports OMP_NUM_THREADS=1.
  - sync_packages (392-447): a scan is processed only when last IMU stamp >= lidar_end_time.
  - Output stamp = lidar_end_time (line 641) = scan header stamp + last point offset.
  - Order in timer: IMU process/undistort -> fov segment -> voxel downsample -> IEKF update ->
    publish_odometry -> map_incremental -> publish_frame_world (dense: transforms+serializes ALL
    undistorted points, not the downsampled set) -> publish.
  - Per-scan timing printf exists only when runtime_pos_log_enable (false in x2.yaml) -> no timing in log.
- Memory today: 15.6 GB total, ~960 MB free, 7.6 GB available, swap 2 MB used (differs from yesterday's
  265 MB free / 700 MB swap). PSI: /proc/pressure not present (kernel without PSI) -> use /proc/vmstat + schedstat.
- Vendor atop is running at nice -20 recording 1 s samples (atop -w /agibot/log/atop/record-*.atop 1 60).
- Hardware: Jetson Orin NX (8x Cortex-A78AE, CPU part 0xd42), nvpmodel 4, governor performance on all 8 cores,
  scaling_cur = max = 1.984 GHz; thermal 58-66 C (cpu 65.6, tj 65.4) -> no throttling.
- Scheduling: fastlio_mapping SCHED_OTHER nice 0, affinity 0-7. Vendor: nvargus-daemon SCHED_RR 75 on CPUs 1-3;
  hal_camera (3254), camera component_container (2852), hal_sensor (2710) pinned 1-3; soc1_drp 0-3;
  perception pinned CPU 4; pnc/vectorflux pinned CPU 7; irq/188-host1x SCHED_FIFO 50; vendor atop nice -20.
- FAST-LIO threads: 26189 main (executor), 26191 ikd-tree rebuild thread ("Multi thread started" in log),
  FastDDS threads dds.shm.7451 / dds.udp.7451 etc. Log localization_v3_dryrun_02.log: only startup messages,
  1x "lidar loop back", 3x "No point, skip" (startup) - no runtime timing output.

## 2. Evidence (all passive; scripts + raw data in scratchpad)
Scripts: sampler.py (/proc 4 Hz + all procs 0.5 Hz), latprobe.py (ONE rclpy process, raw=True subscriptions,
header stamp parsed from CDR, DDS source/received timestamps from message_info, BEST_EFFORT, no publishers),
thrsamp.py / mthrsamp.py (1.7-2 kHz /proc/<pid>/task/<tid>/{stat,wchan,schedstat}). Data: samp1.pkl, lat1.pkl
(80 s), thr2.pkl+lat2.pkl (40 s), mth3.pkl+lat3.pkl (35 s). Probe env: env.sh in subshell (domain 0, fastrtps,
dds_qazbot.xml). Caveat: probe adds one extra reader of the 811 KB lidar cloud.

### 2.1 System load (sampler, 100 s, 14:17-14:19)
- Total CPU: user 31.6 %, sys 15.0 %, irq 4.0 %, softirq 2.6 %, idle 46.8 %. Per core idle: cpu0 24.9 % (irq 23.8 % +
  softirq 16.5 % -> IRQ core), cpu1-3 34-40 % (vendor camera set), cpu4 56 %, cpu5 64.7 %, cpu6 64.6 %, cpu7 52 %.
- procs_running p50 8, p95 29, max 79; ctxt 77 k/s, intr 55 k/s; loadavg 8-12 (load is inflated by short
  runnable bursts, not by sustained saturation).
- Memory: MemFree ~0.85 GB, MemAvailable 7.76 GB, swap used 2 MB; pgscan_kswapd 0, pgscan_direct 0, pswpin/out 0,
  allocstall 0, kswapd0 CPU 0.00 %. -> NO memory pressure today (yesterday: 265 MB free, 700 MB swap, kswapd 43 %).
  PSI not available in this kernel.

### 2.2 Top consumers (98 s window, % of one core; sum all = 328 % of 800 %)
 vendor: soc1_hal camera 40.9 | nvargus-daemon 33.5 (RR 75) | perception 32.0 | camera component_container 30.9 |
 soc1_drp 18.9 | pnc/vectorflux 17.9 | hal_sensor_orin 15.7 | irq/188-host1x 14.6 | atop ~13 (ps snapshot) |
 hds_master 3.3 | awk (vendor log filter) 3.1.
 ours: fastlio 31.8 | x2_motion_adapter (live=false, idle) 10.9 | trust 8.8 | level_bridge 5.8 | goal_coordinator 5.3 |
 bt_navigator 4.5 | controller 4.4 | nav lifecycle_manager 4.3 | planner 3.7 | smoother 3.1 | amcl 2.5 |
 loc lifecycle_manager 2.0 | map_server 1.8 | ros2 daemon 0.7.  Ours w/o FAST-LIO ~ 57 % of a core.
 Camera chain (hal_camera + nvargus + camera_container + host1x IRQ) ~ 120 % of a core, all on CPUs 1-3/IRQ.

### 2.3 Latency (recv - header.stamp), 80 s run lat1 [and 40 s run lat2]
| topic | rate | p50 | p95 | p99 | max | recv gap max |
| lidar cloud /aima/hal/sensor/lidar_chest_front/lidar_pointcloud (811 KB) | 10.06 Hz | 0.106 | 0.113 | 0.145 | 0.445 | 0.362 |
| lidar IMU /aima/hal/sensor/lidar_chest_front/imu | 200 Hz | 0.001 | 0.005 | 0.059 | 0.352 | 0.357 |
| /qazbot_lio/odometry (FAST-LIO) | 10.01 Hz | 0.139 [0.137] | 0.161 [0.172] | 0.179 [0.204] | 0.204 [0.492] | 0.162 |
| /qazbot_lio/odom_level | 10.02 Hz | 0.143 | 0.168 | 0.195 | 0.459 | 0.419 |
| /qazbot_lio/scan | 5.02 Hz | 0.147 | 0.172 | 0.202 | 0.461 | 0.516 |
- Publisher-side (DDS source_timestamp - stamp): lidar 0.100 (p99 0.103), IMU 0.000, LIO odom 0.136 p50 / 0.203 max.
  Delivery (received - source) p50 <= 3 ms for all; rare transport spikes up to 0.31-0.34 s hit all topics at once
  (lidar, IMU, odom_level, scan) -> system-wide short stall, not FAST-LIO. Probe executor delay p50 1-2 ms.
- Lidar header stamp = scan START (vendor writes at stamp+0.100). FAST-LIO odom stamp = lidar stamp + 0.095
  (= lidar_end_time, exact for all 772 msgs). 25 of 797 scans (3 %) produced no odom.
- level_bridge adds ~4 ms (odom_level - odometry p50), consistent with yesterday.
- vs yesterday (load 27-46, kswapd 43 %): odometry p50 0.237, max 0.79-2.8 s. Today p50 0.139, max 0.20-0.49 s.

### 2.4 Inside FAST-LIO (per scan, thr2 + lat2, 386 scans)
- From lidar receipt to odom write (wall): p50 0.127, p95 0.161, max 0.182 s, made of:
  main-thread RUN 0.045 p50 (p95 0.050, max 0.054) | RUNQUEUE WAIT 0.017 p50 (p95 0.044, max 0.085) |
  SLEEPING (futex_wait_queue_me, i.e. executor idle) 0.070 p50 (p95 0.080, max 0.090).
- 100 s average: main thread CPU 25.0 %, runqueue wait 11.3 % of wall (130 us per switch, 863 switches/s);
  ikd-tree rebuild thread 1.8 % CPU but 5564 wakeups/s (polling) and 13.6 % runq wait.
- Phase analysis: odom of scan k is written 0.027 s (p5 0.022, p95 0.062) after the receipt of scan k+1, i.e.
  ~0.23 s after scan-k start, although scan k was received ~0.12 s earlier and the main thread was idle
  (S/futex) for most of that interval (CPU 4-17 % in bins 40-100 ms after receipt).
- FAST-LIO's own FastDDS receive threads (dds.shm.7451 / dds.udp.7451) burst at -5..+5 ms of the probe's lidar
  receipt -> FAST-LIO gets the cloud as early as the probe; DDS delivery is not the delay.
- Main thread busy burst after every receipt: CPU 44-75 % in 0-30 ms (preempted ~30 % of the burst).

## 3. Ranked hypotheses for FAST-LIO output latency
H1 (CONFIRMED as the dominant term today, mechanism UNCONFIRMED): structural one-scan lag inside FAST-LIO.
   Scan k is processed only in the burst after scan k+1 arrives; ~70 ms p50 of idle sleep per scan, ~0.1 s of the
   0.134 s median publish latency. Supporting: 2.4 phase data, idle futex wait while scan pending, DDS threads
   receive on time. Code path: timer_callback -> sync_packages (laserMapping.cpp:392-447) returns false while
   last_timestamp_imu < lidar_end_time. Candidate mechanisms (hypotheses, not verified):
   (a) IMU messages reach imu_cbk later than they reach the probe (rclcpp executor takes one message per ready
       subscription per wait; KeepLast(400) BEST_EFFORT IMU queue could hold a standing backlog) -> last_timestamp_imu
       lags ~0.1 s; contradicting: main thread is idle 60 % of the time, which should drain a backlog;
   (b) some other gate in sync/preprocess (e.g. stamp ordering/"lidar loop back" style buffer handling).
   Needs instrumentation (see 6) - cannot be resolved passively from /proc.
H2 (CONFIRMED, secondary today, dominant under heavy load): CPU contention. Runqueue wait 17 ms p50 / 44 p95 /
   85 ms max per scan today at 47 % idle; FAST-LIO is SCHED_OTHER on all cores incl. IRQ-heavy cpu0, competing
   with RR-75 nvargus and pinned vendor camera threads. Yesterday's tail (0.79-2.8 s) coincided with load 27-46,
   2.7 % idle, kswapd 43 % and concurrent ros2 CLI processes; not reproducible today.
H3 (REFUTED today): memory-pressure stall - zero kswapd/direct reclaim, zero swap I/O, 8 major faults in 100 s.
   (Yesterday's swap state was real but is gone after the reboot; cause of that growth is an open question.)
H4 (REFUTED): algorithmic slowness - 45 ms CPU per scan < 100 ms period; thread is idle most of the cycle.
H5 (REFUTED): thermal/frequency throttling - performance governor at 1.984 GHz, temps <= 66 C.
H6 (minor): system-wide transport stalls 0.3-0.35 s seen simultaneously on all topics (rare, p99 unaffected).

## 4. Symptom / proximate / root cause
- Symptom: T1_odom/scan stale flicker; FAST-LIO odometry age 0.24 s median yesterday, up to 2.8 s.
- Proximate: FAST-LIO publishes odom ~0.13 s (today) after scan end; under load + runqueue waits and stalls.
- Root: (1) one-scan structural lag in the FAST-LIO pipeline (~0.1 s fixed, present even with idle CPU);
  (2) CPU contention with unpinned SCHED_OTHER FAST-LIO (variable part; dominates tails under heavy load);
  memory pressure was a contributing factor yesterday only.

## 5. T1 limits vs data (no bump recommended)
- Trust odom/scan age = source latency + time since last message (<= 0.1 s odom, 0.2 s scan).
  Today: odom_level age worst-case ~ p99 0.195 + 0.1 = ~0.30 s (max 0.46 + gap) vs 0.5 s limit -> ~0.2 s margin;
  scan ~ p99 0.20 + 0.2 = 0.40 s vs 0.5 -> ~0.1 s margin (scan_every:=2 halves scan margin).
  Yesterday p99 2.3 s: no sane limit covers that. Limits are consistent with an unloaded system; fix latency first.
  Removing H1 would add ~0.1 s margin to both.

## 6. Options (ranked by expected gain; none executed)
1. Resolve H1 in our FAST-LIO candidate (no vendor change): offline bag replay in an isolated domain with timing
   instrumentation (runtime_pos_log or per-stage stamps: lidar cbk time, last_timestamp_imu at sync, sync success
   time). Expected gain ~0.07-0.1 s of the 0.134 s median (odom age -> ~0.04-0.06 s). Highest gain, needs data.
2. Launch-level pinning/priority for fastlio (our process): taskset to CPUs 5-6 (65 % idle, away from IRQ cpu0
   and vendor 1-3) and/or nice -5..-10 (or low SCHED_FIFO after testing; needs CAP_SYS_NICE). Expected gain: removes
   most of the 17 ms p50 / 44 ms p95 / 85 ms max runqueue wait today, and much of the multi-second tail under heavy
   load. Risk: the ikd-tree rebuild thread polls at 5.5 k/s - give it the same set, do not starve it.
3. Reduce our own load: no ros2 CLI during live runs (each CLI = python startup + DDS discovery burst); investigate
   idle x2_motion_adapter 10.9 % and nav lifecycle_manager 4.3 % (idle stack); stop ros2 daemon if unused.
   Gain: ~0.2-0.5 core on average, removes discovery bursts; modest for p50, helpful for tails.
4. FAST-LIO params (validate on replay first): point_filter_num 3->4 and/or filter_size_surf 0.3->0.4 cut per-scan
   CPU roughly proportionally to points (~45 ms -> ~30-35 ms, estimate); dense_publish_en true costs a full-cloud
   transform+serialize after odom publish (does not delay current odom, adds load). Lower priority: CPU is not the
   bottleneck at today's load.
5. Vendor (USER DECISION ONLY, not done): camera chain (hal_camera, nvargus RR75, camera component_container,
   host1x IRQ) ~1.2 cores on CPUs 1-3, not used by our navigation stack; perception/pnc/slam may be optional for
   this use but may be required by vendor safety/locomotion - verify with vendor before disabling.
6. Watch memory before live goals (MemAvailable, kswapd) - yesterday's swap state indicates growth over hours.

## 7. Next diagnostic commands (read-only unless noted)
- python3 latprobe.py 60 out.pkl + mthrsamp.py during a heavy-load window (with operator RViz connected) to compare.
- Offline: ros2 bag record of lidar cloud + lidar IMU (60 s), replay into the candidate in another ROS_DOMAIN_ID with
  runtime_pos_log_enable:=true -> per-stage timing; confirm H1 mechanism (a)/(b).
- grep MemAvailable /proc/meminfo; grep -E 'pgscan|pswp' /proc/vmstat periodically over hours (leak trend).
- cat /proc/26189/task/*/schedstat before/after any pinning change (runq wait metric).

## 8. Open questions
- Exact mechanism of the one-scan lag (H1a vs H1b).
- What grew memory/swap yesterday (pre-reboot); whether it recurs after hours of uptime.
- Source of rare 0.3 s system-wide transport stalls.
