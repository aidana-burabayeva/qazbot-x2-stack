# 20261028-scan-latency-rviz-goal — P1 latency breakdown (robotics-debugger)

Date: 2026-10-28 19:20–19:47 CST, ON X2 (agi, 192.168.1.72), ROS 2 Humble, Jetson Orin (R36.4.3, 8 cores, MAXN_SUPER, governor performance 1.984 GHz).
Evidence dir: `/home/agi/control_backend/docs/evidence/latency_20261028_192628/` (analysis.json, analysis2.json, probe_*.jsonl.gz, top/pidstat/tegrastats, snap_* (schedstat + per-socket UDP drops), launch/trust logs, owned_pids_*, owned_env_*, amcl_launch_params_files.txt, run_config.sh, chain_probe.py, analyze*.py).
Robot stationary, no initialpose, no MC/Motion Adapter/Goal Coordinator/cmd_vel/goals. No production files changed.

## 1. System / preflight
- Preflight 19:22: no QazBot FAST-LIO/level_bridge/native/AMCL/Trust/Costmap/probes/recorders. One vendor HAL `hal_sensor_orin` PID 204720 (run), camera HAL 3361 + orbbec 2514/2936, ros2-daemon 74445 (agi, pre-existing, not touched). A foreign RViz (PC, ns /qazbot_d1_map) subscribed to /qazbot_lio/scan, /qazbot_d1_map/tf, map, amcl_pose.
- Baseline load WITHOUT our stack: load avg 9–11, tegrastats CPU0-3 59–71 %, idle ~56 %. Vendor consumers: hal_camera 42 %, camera container 38 %, nvargus 33 %, perception 29 %, pnc 20 %, drp 19 %, HAL sensor 14 %, atop (1 s sampling, pinned CPU1) ~11 %.
- System UDP RcvbufErrors grow ~72/s even without our stack (vendor sockets uid run/root, ports 7420/7424/7400, rx_queue ~18 MB full). Not ours; background fact.
- Clock: ptp4l slave on develop0 + phc2sys -> CLOCK_REALTIME; ptp4l master on sensor0 (LiDAR). "System clock synchronized: yes". No future stamps observed in pipeline; raw cloud header = scan START (cloud_registered stamp = raw + 0.0952 s exactly, every frame = lidar_end_time).
- SHA (not changed by me): level_bridge.py 86c434ce… (baseline, I1 NOT installed), nav_control.launch.py ebea6637… (baseline). localization.launch.py changed on disk by someone else at 19:28:01 to feebadbb… (AMCL max_beams 60, alpha1-5 0.1, recovery_alpha 0) — see AMCL note.
- Baseline launch reused from tools/stack/stack_up.sh / previous session: `ros2 launch ws/src/navigation/localization.launch.py lio_binary:=candidates/fast_lio_portable/.../fastlio_mapping (sha 63903a5c…) map_yaml:=maps/first_map_nav2_v3/map.yaml lio_cpus:=5,6 seed_pose_file:=/nonexistent/qazbot_seed_disabled.json odom_source:=native_imu` + Trust go2 (odom_topic /qazbot_native/odom). level_bridge publish_tf=false; native bridge owns odom->base_footprint.
- Baseline level_bridge (86c434ce) publishes ONLY /qazbot_lio/scan (5 Hz, scan_every=2). /scan_obstacles and /scan_clearing exist only in I1 — not measurable now.

## 2. Runs (single probe process each phase; `full` = 120 s incl. raw cloud subscription, `noraw` = 60 s without)
| Run | DDS profile (all owned procs, verified via /proc/<pid>/environ) | AMCL params actually loaded |
|---|---|---|
| A0 19:27 (INVALID) | run_localization.sh re-sources env.sh -> dds_qazbot.xml (SHM+UDP) for launch children; Trust/probe on nav 2-iface | max_beams 1440, defaults |
| A 19:35 | dds_nav_two_iface.xml = dds_udp_only.xml sha 29d4c736 (10.0.1.41 + 192.168.1.72) | max_beams 60, alpha 0.1, recovery 0 |
| B 19:40 | dds_single_iface.xml = /tmp/i1_capture_udp_only.xml sha 449368eb (10.0.1.41 only) | max_beams 60, alpha 0.1, recovery 0 |

A0 finding (side result): with run_localization.sh the FASTRTPS override is silently replaced by dds_qazbot.xml; FAST-LIO started processing only ~42 s after start, lifecycle_manager "Waiting for service map_server/get_state" for >100 s (AMCL/map_server never activated), probe saw no /qazbot_lio topics; on SIGINT the launch left orphans and level_bridge + native_odom_bridge deadlocked in futex during rclpy/FastDDS shutdown (threads dds.shm.*, SIGINT+TERM caught but no exit) -> SIGKILL on those two owned PIDs (stuck_shutdown_A0.txt). Previous session must have called ros2 launch directly. A0 latency data not usable.

## 3. Per-stage latency table (age = probe receipt CLOCK_REALTIME − header stamp, s; zero-stamp startup frames excluded: 1 raw cloud + 1 cloud_registered/odometry/scan with stamp 0 at start)
| Stage | A full p50/p95/p99/max | B full p50/p95/p99/max | A noraw | B noraw |
|---|---|---|---|---|
| raw LiDAR cloud (815 KB, 10 Hz) age vs scan START | .107/.137/.177/.471 | .107/.129/.157/.401 | — | — |
| = age vs scan END (−.0952) | .012/.042/.082/.376 | .012/.034/.062/.306 | — | — |
| LiDAR IMU 200 Hz | .002/.037/.103/.560 | .001/.021/.056/.345 | .002/.030/.074/.221 | .001/.038/.134/.271 |
| FAST-LIO stage (raw receipt -> cloud_registered receipt, same scan) | .026/.049/.080/.188 | .023/.038/.052/.265 | — | — |
| FAST-LIO after both inputs present | .025/.048/.072/… | .023/.036/.049/… | — | — |
| cloud_registered age (vs end stamp) | .040/.071/.153/.466 | .036/.056/.092/.401 | .039/.066/.095/.150 | .037/.070/.162/.359 |
| LIO odometry − cloud_registered receipt (same stamp) | −.005/.001/.004/.013 | −.002/.001/.005/.011 | | |
| level_bridge stage (scan receipt − cloud_registered receipt, same stamp) | .005/.025/.044/.217 | .005/.017/.032/.064 | .004/.018/.037/.103 | .004/.014/.035/.062 |
| /qazbot_lio/scan age | .047/.084/.208/.481 | .042/.063/.135/.404 | .045/.079/.125/.155 | .042/.083/.165/.206 |
| scan recv gap (period .2) max | .444 | .561 | .309 | .360 |
| MC leg_odometry 168 Hz age | .007/.040/.126/.381 | .006/.024/.060/.348 | .007/.036/.157/.310 | .006/.039/.155/.309 |
| /qazbot_native/odom 30 Hz age | .010/.039/.116/.717 | .009/.029/.077/.526 | .011/.040/.084/.189 | .010/.041/.098/.192 |
| native odom STAMP gap max (source gap max) | .588 (.236) | .066 (.130) | .084 (.060) | .570 (.092) |
| TF odom->base_footprint age | .010/.040/.128/.731 | .009/.030/.091/.545 | .010/.038/.094/.209 | .010/.041/.109/.210 |
| Trust 5 Hz input age scan p50/p95/p99/max | .081/.195/.350/.684 | .180/.183/.188/.388 | .081/.089/.283/.330 | .081/.213/.282/.481 |
| Trust T1 counts (n statuses) | scan_stale 3 /600 | 0 /591 | 0 /293 | odom_message_stale 1 (.640) /297 |
(T1_tf_stale on every status in all runs = no initialpose/AMCL map->odom by design; not a latency signal.)

Same-stamp decomposition of the worst scan windows (analysis2.json):
- A full worst (also seen independently by Trust: scan age .684 at the same wall time 1793187397.97): raw cloud arrived .376 s after scan end -> FAST-LIO +.090 -> level_bridge +.015 -> scan age .481; IMU covering scan end at probe .586; leg_odometry receipt gap .163 in the same window.
- B full worst: raw .306 late -> FAST-LIO +.095 -> level_bridge +.003 -> scan .404; leg_odometry receipt gap .297 in same window.
- Other windows >0.4 s are source stamp gaps (scan stamp gap .41–.49 s = one cloud/scan missing upstream), not level_bridge.

## 4. FIRST stage where delay appears
Raw sensor delivery (HAL -> subscriber on this host): every large scan delay starts with the raw cloud/IMU arriving .3–.38 s late, before FAST-LIO. FAST-LIO adds p50 ~25 ms (p99 50–80 ms, worst +.09–.27); level_bridge adds p50 5 ms (p99 32–44 ms). Same-time receipt gaps on MC leg_odometry (different publisher) indicate a host-wide stall (CPU scheduling / DDS receive), not a LiDAR-specific fault.
Caveat: receipt is measured in the (Python) probe; probe stalls inflate receipt ages (probe IMU coverage .586 > cloud_registered .465 proves the probe itself lagged). The A event is confirmed independently by Trust, but HAL-internal publish delay vs receive-side scheduling cannot be separated without HAL-side timestamps. Confidence: medium.

## 5. CPU contention (strongest supporting evidence)
- Load avg: 9–11 before stack; 23.4 (A full) / 16.7 (B full) with stack+probe; idle mean 31–37 %, min 8–11 %.
- Mean CPU over windows (A / B): probe 48/44 % (max 85–94, observer itself is a big load), native_odom_bridge 34/30 % (Python, 168 Hz input), FAST-LIO 31/28 %, HAL sensor 24/24 %, Trust 16/13 %, level_bridge ~10 %, AMCL 7 % (max 11–12 %), map_server ~0.
- /proc schedstat run vs runqueue-wait over ~125 s (sum over threads): HAL 204720 run 30.6 s / WAIT 89.9 s (A), 29.6 / 65.0 (B); FAST-LIO 39.2/51.3 (A), 34.3/31.5 (B); native bridge 42.4/76.9 (A), 36.9/37.2 (B); Trust 19.5/46.6 (A), 15.7/19.1 (B); AMCL 8.4/37.8 (A), 5.0/15.0 (B). Threads, including the vendor HAL publisher, wait on the run queue longer than they run.
- Per-socket UDP drops on all owned sockets: 0 (A 180 sockets, B 80). Evidence AGAINST receive-buffer overflow in our processes.
- AMCL (coordinator question): 7 % mean, stationary, no initialpose (no filter updates; max_beams 60 in A/B, 1440 only in invalid A0). Not a contributor in these runs. Its per-update cost with 1440 beams while moving was NOT measured (needs motion/initialpose); AMCL is a separate C++ process, so it can only add CPU contention, not queueing in scan/odom. Hypothesis: minor vs probe/native bridge/observers.

## 6. Fast DDS warning flood (new finding, hypothesis for extra CPU)
dds_udp_only.xml has no <log> section, so Fast DDS warnings go to stdout (vendor/dds_qazbot profiles send them to a file). In A, FAST-LIO printed 15,590 `[RTPS_MSG_IN] … received DATA_FRAG from unknown writer 01.0f.88.1b.b0.1f.1c.35|0.0.11.3` (HAL participant) in ~4 min. The probe printed ~79k in 5 min (A and B both). Each process parses unwanted HAL fragments and ros2 launch relays every line. The same flood is in the previous i1_deploy localization.log (18,843 warnings). Unproven how much CPU this costs. Profile without vendor-style log config + HAL fragment traffic to every UDP participant. Not quantified.

## 7. level_bridge queue accumulation (Odometry depth 100, PointCloud2 depth 20)
NOT observed. scan − cloud_registered receipt is p50 4–5 ms, p99 32–44 ms. Longest run of consecutive >50 ms lags is 1–3 messages, with no growing trend. Exactly 0.5 scans per cloud (scan_every=2, nothing dropped). scan stamps follow cloud stamps, and scan age follows cloud age per message. level_bridge CPU is ~10 %. Under this load the DT-suspected stale accumulation is not present, so a depth=1 BEST_EFFORT change is not supported by these data. It could still appear under heavier load such as Costmap + extra observers + motion; that was not tested.

## 8. DDS profile A/B (two-interface vs single-interface)
B (single-interface) had better tails: scan p99 .135 vs .208, cloud p99 .092 vs .153, T1 scan_stale 0 vs 3, load avg 16.7 vs 23.4, and less runqueue wait. However, the runs were sequential (one sample each, not interleaved), and the vendor load varies. A also serves the foreign RViz over WiFi. This is suggestive, not proven.

## 9. Ranked hypotheses
1. (Best supported) Host CPU over-subscription on the Jetson delays the delivery and processing of the sensor stream. The first late stage is HAL->subscriber receipt, and FAST-LIO/level_bridge add only small tails. Our own Python observers (probe ~45 %, previous session ran observer + light observer + obstacle probe + standalone Costmap) and native_odom_bridge (~30 %) are the largest owned loads. Proven: runqueue waits, low idle, independent Trust confirmation. Not proven: HAL-internal vs receive-side share.
2. Two-interface profile / RViz WiFi traffic increases load (A vs B). Suggestive only.
3. Fast DDS stdout warning flood plus HAL DATA_FRAG to unmatched readers adds CPU. Plausible, not quantified.
4. level_bridge queue accumulation. Refuted for this load.
5. FAST-LIO processing. Small (p50 25 ms); it adds .09–.27 s only in the same stall windows. Secondary.
6. Clock/stamp offset. Refuted: constant .0952 s start->end offset, no future stamps, PTP synced.
7. UDP receive-buffer drops in owned processes. Refuted (0 drops).
The previous T1 failure (scan max 1.207 s, odom msg .731) happened with ~3 more Python observers + Costmap + AMCL seeded. It is consistent with H1 at higher load. In today's lighter runs: 3 or 0 scan_stale per 120 s, max scan age .684, native odom max .717/.526.

## 10. Symptom / proximate / root cause
- Symptom: T1_scan_stale / T1_odom_message_stale in Trust.
- Proximate: scan/odom messages arrive .3–.7 s late or with stamp gaps. The delay is already present in raw cloud/IMU/leg-odom receipt.
- Root cause (medium confidence): CPU contention on the sensor host (vendor baseline load ~10 + our Python nodes/observers). Threads, including the HAL publisher and FAST-LIO, spend more time runnable-waiting than running.

## 11. Proposed minimal fix (NOT applied) and verification
- No T1/threshold/stamp changes. Remove the measured CPU contributors from the T1-critical runtime:
  (a) For T1 acceptance runs, use no extra Python observers (or one lightweight 1 Hz status-only observer). Do not run a raw-cloud or IMU subscriber.
  (b) Reduce native_odom_bridge CPU (~30 % for a 30 Hz output). Candidate: avoid per-message Python work at 168 Hz (e.g. minimal callback, latest-only). This needs review.
  (c) Profile hygiene: add a Fast DDS <log> section (file consumer / Error verbosity) to the process-local UDP profile so warning floods are not printed through ros2 launch. Use the single-interface profile for processes that RViz does not need. These are config, not architecture; they need review and permission.
- Verify: rerun run_config.sh (same process set, Costmap included this time), 10 min per variant, interleaved A/B/A/B. Pass if HAL+FAST-LIO runqueue wait/run ratio drops, raw-cloud age p99 < .1, scan age max < .3, and Trust T1_scan_stale/odom_stale = 0 with thresholds unchanged.

## 12. Cleanup
All owned PIDs stopped. A: launch + children by SIGINT (second SIGINT to stragglers). B: same. A0: SIGINT/TERM, then SIGKILL for 2 deadlocked owned Python nodes (documented). 19:46:51: no FAST-LIO/level_bridge/native/AMCL/lifecycle/map_server/Trust/probe/pidstat/tegrastats left. HAL 204720 is still running, the camera was untouched, and the ros2-daemon 74445 was untouched. No files outside the evidence dir were written, except this report.

## 13. Open questions
- HAL-side publish timestamps are needed to separate HAL-internal delay from receive scheduling.
- Which HAL writer is 0.0.11.3 (likely lidar_raw_data / downsampled cloud), and why do its fragments reach unmatched readers?
- AMCL CPU per update with 1440 vs 60 beams in motion. Not measurable without motion.
- Why does run_localization.sh + dds_qazbot.xml (SHM) block lifecycle service discovery and delay FAST-LIO by ~42 s, and why does it deadlock Python shutdown?
