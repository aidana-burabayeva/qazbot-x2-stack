# Senior-practice plan: T1 transient latch + stale costmap marks (2026-10-28)
Facts: goal#3 SUCCEEDED; goal#4 cancelled by GC (localization_untrusted) 2 s in; Trust go2 policy drops READY when T1 loss present at goal request -> reseed required. T1 scan_stale ~10-18% of statuses under load 20-30 / 8 cores. Jetson already MAXN_SUPER, cpufreq 1.984 GHz performance; HAL/FAST-LIO run SCHED_OTHER nice 0. P7: no odom jumps (native drift 0.0106 m/min); costmap lethal = real out-of-FOV marks never cleared (lidar finite only ~±45 deg, inf_is_valid=false, no decay).

Industry practice:
1. Hysteresis, not latch: stale input => immediate zero velocity (pause), auto-resume when fresh; abort goal only if stale persists > bound (e.g. 2-3 s). Safety equal (no motion on stale data), availability much higher. Nav2 collision_monitor implements same idea per source (source_timeout -> stop, resume automatically).
2. Thresholds from measured pipeline latency (p99.9 + margin), not round numbers; keep them, but do not latch on single misses.
3. Root cause load: pin/prioritize sensor path (HAL, FAST-LIO, level_bridge, native bridge) with SCHED_FIFO/nice + taskset away from vendor perception; needs sudo/CAP_SYS_NICE. Measure with ros2_tracing.
4. Limited-FOV sensors: use time decay for unobservable cells (STVL decay 10-30 s) or bounded clearing (I1 2d240dbd).

Proposed order: (A) GC/Trust pause-resume with 3 s bound [code change GC + Trust READY retention on transient T1]; (B) chrt/nice for our sensor nodes (sudo); (C) I1 redeploy or STVL for stale marks.
Sources: api.nav2.org collision_monitor Source (source_timeout); ROSCon2018 STVL; answers.ros.org 403262; arXiv 2504.12813; bit-bots ros_transport_benchmarks.
