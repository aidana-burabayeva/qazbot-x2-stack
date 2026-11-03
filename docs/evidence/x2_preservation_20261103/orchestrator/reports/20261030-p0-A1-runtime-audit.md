# 20261030-p0-repo-deploy-alignment / A1 — Live runtime audit (read-only)

Time of audit: 2026-10-30 16:15–16:20 CST (+08). Host: X2 (agi@, Jetson Orin, L4T 5.15.148-tegra, ROS 2 Humble).
Boot: 2026-10-30 13:40:49 (uptime ~2h35m at audit). Load avg ~12–13 (all from vendor processes).
Method: /proc, ps, systemctl, journalctl, crontab, docker/podman CLI, find/grep. **No ROS graph query, no DDS participant,
no ros2 daemon, no sudo, no git writes** (git used: `rev-parse`, `log -1`, `--no-optional-locks status`).

## 0. Headline
- **None of our stack is running now.** No FAST-LIO, level_bridge, native_odom_bridge, AMCL, map_server, lifecycle managers,
  localization_trust, Nav2 servers, goal_coordinator, x2_motion_adapter, rosbag, ros2 daemon. The robot rebooted at 13:40
  and the stack was not started afterwards. Last stack PIDs (17061, 213707..., 333643..., 397013/398491, 405674) do not exist.
- Nothing executes from the Git checkout `/home/agi/control_backend/qazbot-x2-stack`. **No runtime dependency on that path found.**
- Our stack is never autostarted (no systemd unit, timer, cron, docker, bashrc hook). It is started manually by scripts that
  hard-code **deployment** paths (`/home/agi/control_backend/ws/src/...`, `.../runtime/...`, `.../candidates/...`).
- There is no `ws/install`/`ws/build`; code runs **directly from `ws/src`** (python scripts + launch files by absolute path);
  binaries come from `runtime/` deb-extract prefixes and `candidates/*/install`.
- Side finding (outside A1 scope, possibly relevant to the earlier "new DDS participant delayed FAST-LIO" observation):
  user unit `qazbot-speech.service` crash-loops every ~6.5 s and creates a ROS 2 node each iteration (see §6). Hypothesis only.

## 1. Process table (live, 16:15)
Vendor processes run as user `run`/`root`; `/proc/PID/{environ,exe,cwd}` are EPERM for agi (sudo needs password; not used).
| PID | user | start | cmdline (short) | class | source root |
|---|---|---|---|---|---|
| 2696 / 3041 | run | 13:41:11 | `ros2 launch orbbec_camera gemini_330_series.launch.py` / `component_container camera_container` (/opt/ros/humble) | vendor camera | vendor (/opt/ros/humble, orbbec pkg) |
| 2797, 3495 (em_run 2743, 2872) | run | 13:41:12 | `aima-hal_sensor-app-main` hal_sensor_orin / hal_camera | vendor HAL | /agibot/software/hal_sensor_orin |
| 2640 (1865→1870) | root | 13:41:10 | `em-server -c cfg/.run_agibot.yaml` (agibot_software.service → run_agibot.sh) | vendor process manager | /agibot/software |
| 3056, 3744, 3855, 3953, 4088, 4209 | run | 13:41 | aima-drp-app, hds-slave/master, robot-chores, aima-ec-app, aima-dm-app | vendor | /agibot/software/* |
| 4408, 4691, 4801, 4930, 5185 | run | 13:41 | vendor nav: `slam`, `map_manager`, `perception`, `vectorflux`, `auto_integrated` | vendor nav | /agibot/software/nav |
| 4926/4960 | run | 13:41:30 | `static_transform_publisher 0 0 -0.6 0 0 0 map foot_contact` (from start_vectorflux.sh) | vendor TF | /opt/ros/humble |
| 3745/3780 | run | 13:41:19 | `ros2 run housekeeper housekeeper` | vendor | /agibot/software/housekeeper |
| 5424/5428 | run | 13:41:40 | `ros2 run lx_webrtc_app` | vendor | /agibot/software/lx_webrtc_app |
| 3279, 3395 | root | 13:41 | OTA slaves | vendor | /agibot/software/ota |
| 1594/1698/1743/1858 | root | 13:41 | ptp4l / phc2sys (time sync) | vendor/system | /etc/systemd/system/agibot |
| 1212 | root | 13:41:06 | containerd (dockerd **inactive**, no docker.sock) | system | — |
| 1595 | agi | 13:41:06 | `bash ~/qazbot-speech/robot/lingxi_off.sh` (user unit lingxi-off) | ours-other (qazbot-speech project, not control_backend) | /home/agi/qazbot-speech, cwd /home/agi |
| (transient, ~6.5 s period) | agi | — | `python3 -u -m qazbot_speech --env ~/.config/qazbot_speech/env --robot` (user unit qazbot-speech) | ours-other | /home/agi/qazbot-speech; cwd /home/agi/qazbot-speech |
| 1607 | agi | 13:41:06 | tailscaled (user unit) | ours-infra | /home/agi/tailscale_1.102.3_arm64 |
Not present: any process with cmdline/exe in `/home/agi/control_backend` (checked `pgrep -af` for fast_lio|laserMapping|amcl|nav2|bt_navigator|controller_server|planner_server|level_bridge|trust|motion_adapter|map_server|robot_state_publisher|ros2-daemon|rviz|docker).

## 2. What our stack loads when it runs (from launch scripts + last recorded PIDs/env)
Authoritative launchers (all manual, all in deployment, none in Git checkout operational path):
`/home/agi/control_backend/docs/evidence/nav_20261029_1425/{start.sh,start_loc.sh,start_nav.sh,restart_gc.sh,restart_trust.sh,p20_restart_adapter.sh}`
(copies in Git under `docs/evidence/nav_session_20261029/scripts/`; Git `tools/stack/stack_up.sh` also hard-codes deployment paths).
Last recorded owned processes (`owned_pids.txt`, `owned_nav_pids.txt`, `owned_env.txt`, 2026-10-29):
| component | exe / script | params / config | root |
|---|---|---|---|
| localization launch | `/opt/ros/humble/bin/ros2 launch /home/agi/control_backend/ws/src/navigation/localization.launch.py` | lio_binary, map_yaml args | deployment ws/src |
| FAST-LIO | `/home/agi/control_backend/candidates/fast_lio_portable/install/fast_lio/lib/fast_lio/fastlio_mapping` (real ELF, not symlink) | `--params-file /home/agi/control_backend/ws/src/lio/x2.yaml` | deployment candidates/ + ws/src |
| level_bridge | `python3 /home/agi/control_backend/ws/src/lio/level_bridge.py` | — | deployment ws/src |
| native_odom_bridge | `python3 /home/agi/control_backend/ws/src/lio/native_odom_bridge.py` | — | deployment ws/src |
| map_server | `/home/agi/control_backend/runtime/user_pkgs/opt/ros/humble/lib/nav2_map_server/map_server` | `/tmp/launch_params_*` (generated) | deployment runtime/ |
| AMCL, lifecycle_manager, controller_server, planner_server, velocity_smoother | `/home/agi/control_backend/runtime/qazbot_nav_deps/root/opt/ros/humble/lib/...` | `~/.ros/log/qazbot_ctl_*.yaml`, `qazbot_plan_*.yaml`, `/tmp/launch_params_*` (generated by launch) | deployment runtime/ |
| bt_navigator | `/home/agi/control_backend/runtime/nav2_bt_deps/root/opt/ros/humble/lib/nav2_bt_navigator/bt_navigator` | BT xml from ws/src/navigation/bt | deployment runtime/ + ws/src |
| smac planner plugin | `QAZBOT_SMAC_PREFIX=$NAV/../../../candidates/nav2_smac_bounds/install/nav2_smac_planner` | — | deployment candidates/ |
| goal_coordinator | `python3 /home/agi/control_backend/ws/src/navigation/goal_coordinator.py` | — | deployment ws/src |
| localization_trust | `python3 localization_trust_node.py` with cwd `/home/agi/control_backend/ws/src/navigation` (relative path, `cd $NAV`) | map `.../ws/src/maps/d1_qazbot_20261029/map.yaml` | deployment ws/src |
| x2_motion_adapter | `python3 /home/agi/control_backend/ws/src/navigation/x2_motion_adapter.py` | aimdk_env.sh → /agibot/software/common | deployment ws/src + vendor msgs |
Recorded env of every owned PID: `FASTRTPS_DEFAULT_PROFILES_FILE=/home/agi/control_backend/docs/evidence/nav_20261029_1425/dds_nav_two_iface_log.xml`
(sha256 661979e2...), `ROS_DOMAIN_ID=0`, `ROS_LOCALHOST_ONLY=0`, RMW unset (=> Humble default rmw_fastrtps_cpp).
Note: the DDS profile actually used lives in **docs/evidence/** of the deployment (not in ws/src, not in Git ws/src); env.sh's
default `ws/src/navigation/dds_qazbot.xml` is overridden by the start scripts.
Lock file present: `/home/agi/control_backend/run/x2_motion_owner.lock` (dated 2026-10-29 21:56; stale since adapter not running).

## 3. Overlays and order (deployment `ws/src/navigation/env.sh` + `aimdk_env.sh`, resolved in a clean `env -i` subshell; nothing launched)
`env.sh` computes `QAZBOT_ROOT=<dir of env.sh>/..` and `QAZBOT_BACKEND = QAZBOT_ROOT/../..` when `runtime/` is not next to it → `/home/agi/control_backend`.
AMENT_PREFIX_PATH (first wins):
1. `/agibot/software/common` (aimdk_env.sh, vendor msgs; only for nav/adapter scripts)
2. `/home/agi/control_backend/candidates/nav2_smac_bounds/install/nav2_smac_planner` (only when QAZBOT_SMAC_PREFIX set, start_nav.sh)
3. `/home/agi/control_backend/runtime/nav2_bt_deps/root/opt/ros/humble`
4. `/home/agi/control_backend/runtime/qazbot_nav_deps/root/opt/ros/humble`
5. `/home/agi/control_backend/runtime/user_pkgs/opt/ros/humble`
6. `/home/agi/control_backend/runtime/fast_lio` (**symlink-install into `/home/agi/ws_lio/...` which no longer exists — ~70 dangling links; not used for the binary, which comes from candidates/fast_lio_portable**)
7. `/opt/ros/humble`
PYTHONPATH: `/agibot/software/common/local/lib/python3.10/dist-packages`, `runtime/user_pkgs/.../site-packages`, `/opt/ros/humble/...`.
LD_LIBRARY_PATH mirrors the above. COLCON_PREFIX_PATH empty. CMAKE_PREFIX_PATH=/opt/ros/humble. RMW_IMPLEMENTATION unset. ROS_DOMAIN_ID unset in env.sh (set =0 by start scripts).
Nav2 packages are **not** in /opt/ros/humble (nav2_amcl/controller/planner/bt_navigator/map_server/lifecycle/smac/RPP all absent there); they exist only in runtime/ prefixes.
Other overlays on disk: `/home/agi/aimdk` (+ `aimdk.bak`) — not referenced by env.sh or any live process seen; `/agibot/software/common` (vendor). No colcon workspace sourced from shell init.
Login shell: `~/.bashrc` only sources `~/.aima/env/bashrc` (vendor-generated: `/opt/ros/humble/setup.bash`, `FASTRTPS_DEFAULT_PROFILES_FILE=/agibot/data/home/agi/.aima/env/ros_dds_configuration.xml` (whitelist 10.0.1.41 + SHM), AGIBOT_* vars). `.profile` sources `.bashrc`. No `.bash_aliases`. No control_backend/qazbot reference in shell init.

## 4. Latent hazard: running from the Git checkout
`qazbot-x2-stack/ws/src/navigation/env.sh` is byte-identical to deployed (cmp). Sourced from the checkout it resolves
`QAZBOT_BACKEND=/home/agi/control_backend/qazbot-x2-stack`, whose `runtime/` and `candidates/` do **not exist** → all runtime/
overlays point to missing dirs, only /opt/ros/humble remains, which has no Nav2 → nav launch would fail (not silently fall back);
also `FASTRTPS_DEFAULT_PROFILES_FILE` would become the checkout's `dds_qazbot.xml`. Conclusion: checkout is not runnable in place; the deployment layout (ws/src next to ../../runtime, ../../candidates) is an implicit contract.

## 5. Dependencies on `/home/agi/control_backend/qazbot-x2-stack` — NONE found at runtime
Search method: `grep -rIlF` fixed strings `'/home/agi/control_backend/qazbot-x2-stack'`, `'qazbot-x2-stack'`, `'/home/adn'` over `/home/agi/control_backend`
(include *.sh *.bash *.py *.yaml *.yml *.xml *.launch *.json *.cfg *.toml *.service *.env *.ini *.rviz *.md; exclude build, log, logs, install, .git, __pycache__, .pytest_cache, and the checkout itself), plus ~/.bashrc, ~/.profile, /etc/profile.d, /etc/bash.bashrc, system+user systemd units, crontab.
Results outside the checkout:
- `/home/agi/control_backend/qazbot-x2-stack`: 0 files.
- `qazbot-x2-stack`: 1 — `ws/src/navigation/nav_control.launch.py:27`, a **comment** (`# (qazbot-x2-stack/docs/X2_FOOTPRINT.md) ...`). Not a dependency.
- `/home/adn`: 5 docs/test-result files only (docs/RVIZ_LOCAL.md, GO2_ADMISSION_EXPERIMENT_20261023.md, RPP_MC_ALIGNMENT_20261022.md, digital_twin/go2_braking_audit_20261027.md, tests/integration_results/twin_baseline_01/results.json). No executable dependency.
- systemd (system + user), cron, shell init, docker: 0 references to control_backend/qazbot-x2-stack. (System unit hits for `/home/agi` are vendor home-setup scripts in /etc/systemd/system/agibot/{data_reset_boot,mount,user_reset_boot}.)
Inside the checkout (reverse direction, informational): operational files `tools/stack/{stack_up,stack_down,check}.sh`, `tools/stack/{trust_trace_copy,ingress_trace}.py`, `tools/lio_calib/lc_common.py`, `tools/replay_t5_bag.py`, AGENTS.md hard-code **deployment** paths `/home/agi/control_backend/{ws/src,logs,candidates,tools}` (note `check.sh` uses `/home/agi/control_backend/tools/stack` = deployment copy). Self-references to `qazbot-x2-stack` exist only in docs (HANDOFF.md, ROBOT_SYNC_VALIDATE.md, AGENTS.md, docs/evidence/*).
Symlinks: `find /home/agi/control_backend -type l` (excl. logs/, docs/ has none): 121 in runtime/ (fast_lio → /home/agi/ws_lio/* dangling; ompl/GraphicsMagick internal relative), and `qazbot-x2-stack/tests/twin -> ../../digital_twin/research/tests/twin` (dangling on X2; PC-layout relic). 79 dangling links total. **No symlink points into the checkout; no symlink in the checkout points into deployment.**
Deployment has no `.git` (verified); checkout HEAD `nav/first-goals-20261028` @ 982f61802035ab5b033e26ce9df881dab2e27468.

## 6. systemd / cron / docker
- System: our stack has no unit. Enabled units are vendor (agibot_*) + system. `agibot_software.service` → `/etc/systemd/system/agibot/software.sh` → `/agibot/software/entry/run_agibot.sh` → em-server (vendor). Timers: vendor/system only.
- User (agi): enabled `lingxi-off.service` (~/qazbot-speech/robot/lingxi_off.sh), `qazbot-speech.service` (~/qazbot-speech/robot/run.sh --robot), `tailscaled.service`; `hello_world.service` disabled; no user timers.
- **qazbot-speech crash loop**: `Restart=always RestartSec=3`; NRestarts=1473 at 16:16 since boot (13:41); journal: ~2159 starts on 10-28, 390 on 10-29, 1478 on 10-30. Every run fails with `KeyError: 'SONIOX_API_KEY'` because `QAZBOT_ENV=/home/agi/.config/qazbot_speech/env` does **not exist**. In `__main__.py` `io = ros_io()` (line 130: `rclpy.init(); rclpy.create_node("qazbot_speech")` + publisher + 3 service clients, robot.py:134-139) runs **before** the crash at line 145 → one DDS participant joins/leaves domain 0 (vendor FastDDS profile, SHM+UDP 10.0.1.41) roughly every 6.5 s. Hypothesis (unverified): this participant churn contributes to discovery load/latency spikes seen by FAST-LIO and other stack nodes. Verify with journal timestamps vs FAST-LIO latency logs, or `ros2 node list` sampling (DDS observer — only with approval). Not touched (outside scope, no service changes allowed).
- cron: `crontab -l` → none for agi; /etc/cron.d: atop, e2scrub_all, sysstat only; /var/spool/cron/crontabs not readable (EPERM).
- docker: dockerd inactive, docker.socket inactive, no /var/run/docker.sock; containerd active (vendor/system, no containers inspectable). `podman ps -a` (user): empty.
- Other: root `/tmp/auto_atop_record.sh` (atop recording to /agibot/log/atop) — vendor/system.

## 7. Graph/TF snapshot — SKIPPED
Reason: no stack processes are running, so there is nothing of ours to observe; the only ROS publishers are vendor (camera, HAL,
vendor nav incl. static TF map→foot_contact z=-0.6 visible in cmdline). Creating a DDS participant was avoided per TASK constraint. No ros2 daemon is running (pgrep), none was started.

## 8. Before-snapshot
- Script (read-only, no DDS, no sudo, uses `git --no-optional-locks`): `/home/agi/.orchestrator/reports/20261030-p0-before-snapshot/snapshot.sh [OUT_DIR]`
- Baseline: `/home/agi/.orchestrator/reports/20261030-p0-before-snapshot/before_20261030T161912/` (processes, per-process exe/cwd/env-hash for robot-related PIDs (vendor = EPERM), system+user service states, timers, user unit files, cron, shell-init sha256, containers, deployment layout, checkout HEAD + tracked status, deployment-no-.git, ros2 daemon presence).
- Diff after later changes: run script to a new dir, `diff -r before_20261030T161912 <new>`; `32_volatile_speech_restarts.txt` is expected to change (crash loop).
- Impact note: an earlier draft run used plain `git status` (may refresh the stat cache in `.git/index`; no content change) before the script was fixed; its output dir was deleted and regenerated. `.git` mtime 16:15 predates this (likely the parallel A2 agent).

## 9. Process → source-root classification (acceptance table)
| process group | source root | evidence |
|---|---|---|
| Our nav/loc stack (FAST-LIO, bridges, AMCL, map_server, Nav2, GC, trust, adapter) | **deployment** (`ws/src`, `runtime/`, `candidates/`, DDS profile in `docs/evidence/`) — **not running now** | start*.sh absolute paths; owned_pids.txt/owned_env.txt 10-29; pgrep empty now |
| Git checkout `qazbot-x2-stack` | nothing executes from it | no process cmdline/cwd/exe in it; 0 path refs outside checkout (comment only) |
| Camera, HAL, MC/EM, vendor nav (slam, perception, vectorflux, map_manager, auto_integrated), housekeeper, webrtc, OTA, PTP | **vendor** (/agibot/software, /opt/ros/humble) — do not touch | ps user=run/root, paths |
| qazbot-speech, lingxi_off, tailscaled | other (ours, /home/agi/qazbot-speech, /home/agi/tailscale_*) — not control_backend | user systemd units |

## 10. Open questions
1. Is the qazbot-speech crash loop known/intended to be disabled during nav work? (Operator decision; would remove a recurring DDS participant.)
2. Should the DDS profile used in production (`docs/evidence/nav_20261029_1425/dds_nav_two_iface_log.xml`) be promoted into ws/src (it is outside the tree A2 compares)?
3. `runtime/fast_lio` dangling symlink prefix stays in AMENT_PREFIX_PATH — harmless today (binary comes from candidates/), but should be documented in DEPLOYMENT_CONTRACT.
4. Vendor process env (ROS_DOMAIN_ID/RMW/profile) not verifiable without sudo; vendor default profile is `~/.aima/env/ros_dds_configuration.xml` (assumption for vendor processes: domain 0, FastDDS).
