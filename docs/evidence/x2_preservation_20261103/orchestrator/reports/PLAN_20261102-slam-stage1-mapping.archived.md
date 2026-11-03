# PLAN — 20261102-slam-stage1-mapping (see TASK.md)
Previous: reports/PLAN_20261030-twin-data-collection.archived.md

## Tasks
| # | Task | Status | Agent |
|---|---|---|---|
| T1 | Read-only discovery (reports/20261102-slam-stage1-T1-discovery.md) | DONE-BLOCKERS: B1 GetStoredMapByName absent, B2 /integrated_command no subscriber, B3 map.db not readable as agi | robotics-debugger opus/high |
| T2 | Implement x2_slam_stage1 offline (reports/20261102-slam-stage1-T2-implementer.md) | DONE (8 unit tests pass; not live-run) | robotics-implementer sonnet/medium |
| T3 | Live mapping run (needs operator go) | PENDING | operator + orchestrator |
| T4 | Independent verification | PENDING | robotics-verifier opus/high |

## Routing
| Step | Agent | Model | Effort | Reason |
|---|---|---|---|---|
| T1 | robotics-debugger | opus | high | live robot, SLAM activation unknown, conflicts with running custom stack |
| T2 | robotics-implementer | sonnet | medium | documented interfaces, clear acceptance; no live run |

## Continuation
Resume: "продолжи задачу 20261102-slam-stage1-mapping из .orchestrator/PLAN.md"

- 14:4x T1 done: X2 Ultra PC2 10.0.1.41 Humble FastDDS dom0; lidar 10 Hz/IMU 199 Hz OK; official SLAM procs running but SDK interfaces not exposed -> Stage 1 BLOCKED pending after-sales activation / map.db access. T2 coded offline meanwhile.
- 14:5x User asked same check on robot 192.168.1.89: ssh agi@ -> Permission denied (publickey,password). Need credentials. T2 implemented (general-purpose sonnet/medium; implementer agent needs worktree=git, /home/agi not git).
- 15:0x .89 = SECOND robot (machine-id differs; same PC2 IP layout 10.0.1.41). Access: ssh agi@192.168.1.89 password-auth (user-provided; not stored in files). T1b discovery on .89 launched (general-purpose sonnet/medium, read-only checklist).
- 15:1x T1b .89 done (reports/20261102-slam-stage1-T1b-robot89.md): AimDK v1.0.0 (older); NO slam/map_manager/perception procs, NO lidar/IMU publishers, no /integrated_command, no GetStoredMapByName; map.db READABLE (group run) with 74 maps 2026-10-05..10-19 => official mapping worked there before. Custom NDT/x2pub LIO running. Both robots BLOCKED for Stage 1. Awaiting user decision.
- 15:2x .89 deeper (orchestrator, read-only): EM apps slam/map_manager/auto_integrated registered (args nav, sudo:True) but NOT started this boot (no log dirs in latest/, boot 14:02). hal_sensor_orin: lidar E1R SN 3031BEE09592 detected, then repeating 'receive timeout'/'publish timeout' (lidar_parser.cpp:320/322) => no point data. Proposed: (1) aima em restart-app hal_sensor_orin [robot must be safe: HAL restart affects IMU] (2) aima em start-app map_manager, slam, auto_integrated (≥1 s apart). Awaiting user go.
- 15:3x ROOT CAUSE robot #1: slam/map_manager/auto_integrated EM apps RUNNING (started 14:25), but /agibot/data/log/latest/slam/running_slam.log: MappingModule, LocalizationModule, RelocalizationModule 'module can't start because license denied: Expired(13): license_id not found; temporary cert expired (issued_at=2026-06-07T08:03:26Z + 7d window)'. aimrte v0.9.2.15-rc3, image soc0:v0.6.10-hotfix_v3. => official SLAM unusable until AGIBOT renews license. Do NOT bypass license. Fallback b/c (open-source mapping, viz only) pending user decision. Runbook section G to be updated with this cause.
- 18:58 User charging robot, then teleop back to office WITHOUT new pose (test: separate furniture vs initial-pose heading error). No bag recording active now; restart record_nav.sh right before drive. Pose must survive charging (no stack restart/lift).
- 18:59 Return-to-office test started: bag nav_bag_185926 (1800 s cap), trust_watch_return_office.txt (3600 s). Initial: score 0.20-0.36, sigma 0.35, 2nd 0.62, ready True. Analyze after robot parks in office (static 10-15 s).
- 19:02 Office after return (carried pose, no reinit): score 0.55-0.58, 2nd 0.28 (vs 0.23/0.83 after office initial pose). seed_fit_return_office.txt: best 0.609, AMCL offset 0.20 m / 1.1 deg; competitor modes 0.554 (0.8 m, -17 deg), 0.529. => furniture NOT primary; root cause = office initial-pose heading error (~17 deg). S-01 note: office local modes close (0.609 vs 0.554) -> uniqueness margin must be calibrated on these bags. Robot to charge; bag ends 19:29, watcher 19:59.

## 2026-11-02 live nav session (separate from slam-stage1) — report docs/hardware/PHYSICAL_NAV_SESSION_20261102.md
- Stack LIVE: localization+trust (start_loc.sh), Nav2 + GC, adapter LIVE P23 registered sdk_qazbot_nav2_1102a; evidence dir docs/evidence/nav_20261102_live.
- Fixed: D-01 speech crash-loop (disabled), D-03 CPU (vendor camera trim). Confirmed: D-05 MC overspeed ×1.64 (tape), D-06 leg odom −8 %, D-04 marking FOV ≠ clearing FOV.
- Open: D-04 fix candidate (level_bridge marking FOV) needs operator OK + localization restart; D-05 inverse map needs ≥3 tape points; goal_pose topic drops (action works).
- Stop all: docs/evidence/nav_20261102_live/stop.sh (nav first). Resume: "продолжи live nav сессию 20261102 из .orchestrator/PLAN.md".
