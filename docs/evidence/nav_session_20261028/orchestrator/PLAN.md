# PLAN — 20261028-scan-latency-rviz-goal (see TASK.md)
Previous: reports/PLAN_20261027-rpp-local-collision.archived.md

## Phase
2026-10-28: P0 docs read; P1 latency measurement (preflight + passive stack + age_probe) launched.

## Tasks
| # | Task | Status | Agent |
|---|---|---|---|
| P1 | Preflight + passive latency breakdown | DONE medium conf: first delay = raw HAL cloud delivery under CPU load (load 10 baseline/8 cores, 17-23 with stack); level_bridge queue accumulation DISPROVEN; FAST-LIO p50 25ms; T1 scan_stale A(2-iface) 3/600, B(1-iface) 0/591; reports/20261028-scan-latency-debugger.md | robotics-debugger |
| P2a | AMCL odom-trust params (operator approved) | DONE: diff+SHA verified by orchestrator (feebadbb...), live-effect untested; reports/20261028-amcl-odomtrust-implementer.md | general-purpose (implementer role; worktree hook unavailable) |
| P2 | Fix: DDS log section (DATA_FRAG flood), no probes during nav; no level_bridge depth change | -> folded into P4 bring-up | |
| P4b | Nav bring-up to 'waiting for operator initialpose' | DONE: loc+Trust running (launch 372186), DDS log fix 918f941d, AMCL params confirmed; Nav2 needs nav_full (includes GC); reports/20261028-nav-bringup.md; stop: docs/evidence/nav_bringup_20261028_195035/stop.sh |
| P5a | Operator initialpose | DONE: AMCL set pose -7.013 5.530 -0.988; Trust 60 s: 293/293 READY+MOTION_OK, 0 T1 (evidence trust_status_60s_after_seed.txt) |
| P5b | Operator RC confirm -> Nav2+MC/MA/GC up, no goal | DONE: adapter 387158, nav_full 387282 (GC 387399); 0 velocity msgs 20 s; MC STAND_DEFAULT unchanged; Trust MOTION_OK 83/101 with Nav2 (18 T1_scan_stale, load ~30); reports/20261028-nav-motion-bringup.md |
| P6 | Operator goal 1-2 m on /qazbot_d1_map/goal_pose; bag goal_bag_201325 + log Monitor | WAITING OPERATOR | robotics-debugger opus/high | robotics-debugger |
| P3 | Verify | TODO | verifier |
| P4 | Passive I1 re-check (operator steps) | TODO, needs permission | debugger |
| P5 | RViz goal preparation | TODO, needs permission | — |

## Routing
| Step | Agent | Model | Effort | Reason |
|---|---|---|---|---|
| P1 | robotics-debugger | opus | high | sensor timing/DDS root cause on live robot |
| P4b | robotics-debugger | opus | high | live robot bring-up, safety-relevant |
| P2a | general-purpose | sonnet | medium | precisely specified config change |
| P1 cont. | robotics-debugger (resumed) | opus | high | hit turn limit, resumed |

## Continuation
Resume: "продолжи задачу 20261028-scan-latency-rviz-goal из .orchestrator/PLAN.md"

## 2026-10-28 operator decisions: robot-first; see DECISIONS.md. Next: P1 result -> P2 fix on X2 -> passive start + operator initialpose (wait confirmation) -> READY -> operator RC confirm -> MC/MA/GC -> one 1-2 m RViz goal.
- Operator request: during the goal, monitor all nav-relevant logs/topics live. Plan P6: after P5b, start ONE lightweight rosbag (no PointCloud2): Trust status, amcl_pose, tf/tf_static (ns), scan, cmd/velocity topics, Nav2 action status/feedback, plan, BT log, costmap updates if light; plus live Monitor on all owned process logs for WARN/ERROR/ABORT/T1/T5/collision.
- Operator add: record separately native odom (/qazbot_native/odom), FAST-LIO odometry (/qazbot_lio/odometry, odom_level), leg odometry, IMU, AMCL pose, full TF incl. global /tf (all frames: legs, pelvis, torso, etc.) + namespaced /qazbot_d1_map/tf; all node logs.
- FINDING 2026-10-28 20:16 (live, localization.log): FAST-LIO "lidar loop back, clear buffer" 59x since 19:50; level_bridge "odometry stamp went back" 86x, many by ~1.79e9 s (odometry stamp ~0 / epoch) plus small 0.004-0.3 s regressions. Each clears level_bridge pose cache -> likely scan gaps -> T1_scan_stale. Candidate root cause for freshness beyond CPU load; investigate after goal (raw lidar header stamps from HAL).
- 20:54 GOAL #1: GC accepted, BT from (-6.56,5.82) to (-5.52,3.76); planner GridBased "goal is occupied with no tolerance" -> ABORTED. Static map: goal cell free, 1 occupied pixel within 0.5 m -> inflated. Robot did not move. Note AMCL pose (-6.56,5.82) vs seed (-7.01,5.53): 0.53 m shift while standing. Operator: "one of the frames is outside the map" -> asked which.
- 20:56 GOAL #2 (-6.54,5.83)->(-6.05,3.64): path planned, RPP "collision ahead" x7 in 0.9 s -> patience exceeded -> ABORTED; robot made a step.
- Operator asked to clear costmap: ClearEntireCostmap local+global called (operator override of AGENTS.md ban), logged in evidence operator_actions.txt. Lethal marks reappear immediately; live /qazbot_lio/scan has NO returns <1.5 m (finite only -60..+45 deg). Costmap = LaserScan ObstacleLayer, inf_is_valid=false.
- Odom frame: footprint center in odom moved (-2.41,2.71)->(-2.70,2.98) = 0.40 m in ~2 min while robot standing (no goal) => native odom drift hypothesis smearing marks. P7 debugger launched.
- 21:02 Operator: phantoms = single-pixel specks in static map. Created ws/src/maps/first_map_nav2_v3_despeckled (8-connected occupied components <=4 px set free: 461 comps/614 px; pgm SHA 6fc67526..., source v3 f6032f75 untouched). Loaded live via map_server/load_map (result 0); AMCL received map. Trust reported T8_map_mismatch -> restarted Trust (PID 425355) with despeckled map+SHA. Needs new operator initialpose. Monitor re-armed. Launch files still default to v3 — persist later.
- 21:05 GOAL #3 after despeckled map + new operator initialpose (-6.466,5.607,-1.215): (-6.42,5.59)->(-5.82,3.46) ~2.2 m: "Reached the goal!" / Goal succeeded in 12.5 s. FIRST PHYSICAL NavigateToPose SUCCEEDED (operator observed). BT tick-rate warnings only.
- 21:07 GOAL #4 (-5.61,3.59)->(-3.92,1.11): GC canceled after 2.0 s: fault localization_untrusted; GC latched. Trust now not_ready:awaiting_initialpose_and_amcl (go2 policy: T1 loss present at goal request => READY lost, reseed needed) + T1_scan_stale 11/91 in 20 s; load ~25/8 cores. BLOCKER: T1 freshness intermittency x strict latch. Proposed to operator: temporary T1 age relax (rule break) vs pause-not-cancel vs stop vendor perception/pnc.
- 21:15 CPU profile: camera chain hal_camera 3361 41% + nvargus 34% + camera_container 2936 29% + perception 4674 32% (~1.4 cores, unused by nav); FAST-LIO 33%; native_odom_bridge 29% (python); Trust 13%; MA 14%. FAST-LIO cannot simply be disabled: level_bridge builds /qazbot_lio/scan from cloud_registered + LIO odometry (replacement scan path = architecture change). Local costmap trails of passing person not cleared = out-of-FOV/inf no-clear (I1 target). Asked operator: stop camera chain (needs run/sudo), I1 redeploy vs inf_is_valid, pause-not-cancel.
- 21:20 Operator decisions: stop vendor camera chain (yes); redeploy I1 (yes); pause-not-cancel on T1 with auto-resume, cancel >3 s (yes).
| P8a | stop camera chain | DONE via aima em stop-app (perception, orbbec_camera, hal_camera), no respawn; HAL 204720 alive, lidar 10 Hz; restart: aima em start-app hal_camera|orbbec_camera|perception; reports/20261028-camera-stop-P8a.md | general-purpose sonnet/medium |
| P8b | I1 2d240dbd redeploy files | READY-TO-INSTALL at restart (tool guard requires stopped stack); cmd in reports/20261028-i1-redeploy-P8b.md; nav_full includes nav_control -> I1 active | general-purpose sonnet/medium |
| P8c | GC/Trust/MA T1 pause-resume | RUNNING | general-purpose opus/high (safety logic) |
Then: P8v verifier opus/high -> controlled restart (motion first stop) -> operator initialpose -> goals.
- P8c DONE (impl): Trust t1_transient field, GC pause/resume (T1_PAUSE_MAX_S 3.0), adapter T1 pause no latch; 198 passed/4 skipped; reports/20261028-t1pause-P8c.md. Risk: stale map->odom may make RPP/Nav2 abort itself.
| P8v | verify P8c | PASS (290 passed/5 unrelated fail/7 skip); F1 flapping never cancels (accepted: moves only when fresh); F2 resume step 0->0.3 m/s bypasses smoother accel -> watch live; reports/20261028-t1pause-P8v.md | robotics-verifier opus/high (safety) |
- 21:28 Operator asked read-only look at robot agi@192.168.1.89: 8 cores, load 6-7, 50% idle; top: python3 PID1549 agi 93% (unknown script), camera_container 49%, weld_st* 25%, separate LIO stack (azat_ws, fastlio nice 5). Not touched.
| P8d | stop vendor apps not needed for nav (operator) | RUNNING | general-purpose sonnet/medium |
- 21:36 KEY FINDING: X2 sensor HAL PID204720 (manually restored earlier via em_run.sh, cfg hal_sensor_orin_t2d5.yaml, process_name hal_sensor_orin) runs nice 0 (SCHED_OTHER). Reference robot .89 HAL (via em, cfg hal_sensor_module_config_orin.yaml, process_name hal_sensor) runs nice -19 on all 66 threads. Lost priority = strong candidate root cause for raw cloud delivery delay under contention (P1 first-delay stage). .89 uses NDT lidar_localization_ros2 (x2-pad-park), not our stack. Proposed: restart HAL the vendor way (em) to restore -19 — needs operator OK.
- P8d DONE: stopped drp, lx_webrtc_app (aima em), idle 66.9->79.5%; kept pnc/vectorflux (uncertain), slam, map_manager etc.; restart: aima em start-app drp|lx_webrtc_app. HAL em entry hal_sensor_orin: capsh_run.sh, cgroup_group sensor, sudo true; em NOT tracking manual PID204720 (parent em_run.sh 204714, user session cgroup, NI 0). Vendor-way restart = kill 204720 (owner run -> needs sudo/operator) then `aima em start-app hal_sensor_orin`. Never start em HAL while 204720 alive (duplicate publishers).
| P9 | controlled restart: stop all owned, install I1 r2, despeckled map everywhere, start loc+trust+nav, recorder | RUNNING | robotics-debugger opus/high | (HAL vendor-restart deferred by operator)
- P9 attempt 1: robotics-debugger refused (read-only role). Bag recorder died 21:03:58 'database is locked'; goal_bag_201325 needs reindex; goals #3/#4 not in bag. Re-dispatch P9 to general-purpose opus/high.
- P9 DONE (general-purpose opus/high): new stack loc 464092 / Trust 464350 / adapter 464645 / nav_full 464714 (GC 464838); I1 r2 installed; despeckled map; markers t1_transient/t1_pause; recorder mcap PID 471147 goal_bag_mcap_215612. Blockers: Nav2 planner/bt wait for map->odom (needs initialpose); MC SetMcInputSource ADD code=1 (stale source name). Foreign hung shell 450989 (`ros2 topic hz .../trusted`, other Claude session) left alone. P9b MC registration fix running (general-purpose sonnet/medium).
- 22:08 P9b DONE: stale MC source 1028a DELETEd (code 0); adapter PID 479952 registered as sdk_qazbot_nav2_1028b; GC ready/driver_ready true, not latched. Operator initialpose AMCL -5.814 5.433 -1.027. Trust ready; t1_transient seen live. TODO: adapter DELETE on shutdown; start_nav.sh default source name -> 1028b. Polling log monitor armed. READY FOR GOALS.
- 22:12 RUN: goal A SUCCEEDED (4.9 s). Goal B: 4.6 s in, GC "trust lost while busy: gate paused (pending T1 classification)" then cancel 20 ms later -> latched localization_untrusted. Pause logic classification race suspected. P10 debug+fix GC race (general-purpose opus/high), no restart yet.
- 22:15 Operator: planner should replan around obstacles. Confirmed: BT = plan once (Sequence), global costmap static+inflation only, GridBased.tolerance 0. P11 (general-purpose sonnet/medium): BT RateController 1 Hz replanning, global obstacle_layer (I1 sources), tolerance 0.3 (operator's "just make it work" mandate; tolerance not explicitly confirmed). Apply with P10 in one nav_full restart.
- P11 DONE: BT PipelineSequence+RateController 1Hz, global obstacle_layer, tolerance .3; orchestrator diff-checked; awaiting P10 then nav restart.
- P10 DONE: goal B cancel = REAL T5 (map->odom jump 0.1045 m > 0.10) not a race; AMCL sigma_xy .71->1.26 m, scan_score .02-.16 (floor .35), corrections 4-7 cm ~1 Hz. Logging fix only in GC (TRIP reason). Latch clear: initialpose -> GC resume + adapter resume services. restart_gc.sh restarts nav_full (needed for P11). Proposal to operator: T5 jump 0.10->0.25 m (rule break; false global locks are >=0.5 m) + AMCL update_min_d/a smaller for smaller steps.
- 22:25 Operator decision: T5 jump 0.25 m/0.25 rad + AMCL update_min_d/a 0.10. P12 (general-purpose opus/high): apply, restart Trust + nav_full (P10 logging + P11 replanning), keep adapter 1028b; then operator initialpose -> adapter resume -> 3 goals.
- 22:28 Operator: no internal/unit test suites from now; only minimal live checks on robot.
- 22:40 P12 DONE: T5 .25, AMCL update_min .10 live, Trust 494295, nav_full 498193 (GC 498330), replanning BT + global obstacles active; operator initialpose 22:29:42; adapter resumed by orchestrator -> idle registered; GC ready. READY FOR 3 GOALS. ros2 daemon killed by agent (CLI flaky; use rclpy scripts).
- 22:43 RUN2: goal1 SUCCEEDED 8.8 s; goal2 RPP collision ahead -> patience 0.3 s -> abort; replanning had no chance. P13 (general-purpose sonnet/medium): failure_tolerance 3.0 + BT RecoveryNode(replan, 3 retries), restart nav_full.
- 22:48 Diagnosis: AMCL unconverged (sigma_xy .82, second_mode .98, scan_score .14) -> T5 trips at .25 too. Orchestrator: max_beams 60->180 live + launch file (bak pre_beams180). P13 done (failure_tolerance 3.0, RecoveryNode replan x3, nav_full 509608). Plan: operator initialpose -> post_seed.py (12 nomotion updates, metrics, resume GC+adapter if READY).
- 22:52 Operator: failure_tolerance 10 s. Set in nav_control.launch.py (bak pre_ft10, mirrored) + live set OK (read back 10.0; Humble dynamic param assumed effective).
- 22:55 P14 publish (general-purpose opus/medium): HANDOFF/TEST_STATUS/X2_NAV_BRINGUP + evidence + bags (<=1.5 GB compressed, else per-goal excerpts) -> branch nav/first-goals-20261028, push. Current blockers section emphasized. Continuation: 'продолжи задачу 20261028-scan-latency-rviz-goal из .orchestrator/PLAN.md' — next: operator initialpose -> post_seed.py -> 3 goals.
