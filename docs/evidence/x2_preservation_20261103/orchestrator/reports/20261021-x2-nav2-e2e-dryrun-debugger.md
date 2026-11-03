# 20261021-x2-nav2-e2e-dryrun — gap analysis (debugger, read-only)

Date: 2026-10-21. Host X2 .66 (Jetson, 5.15-tegra), ROS 2 Humble, Nav2 1.1.18 (user-space overlay runtime/qazbot_nav_deps).
Source: /home/agi/control_backend (not git). No source/config edited. No motion, no MC publisher, no registration.

## 0. Snapshot
- /home/agi/control_backend_backup_20261021_pre_e2e.tar.gz — 50,678,864 B (49 MB), 2315 entries,
  sha256 206180b5d3c93c1e0a81475a8e55d846a60d685fb613e3faf8013bee414d39e1.
- Excluded: runtime_arm64.tar.gz, runtime/, */build, */install, */log, *.db3, *.mcap, __pycache__.
  NOTE: candidates/nav2_smac_bounds/install (qualified Smac .so, hash-checked by nav_full) is NOT in the tarball;
  source+patch are. Re-build or separate copy needed for full restore.

## 1. Running processes (ps, 20:27)
- Vendor: aima-* apps, hal_sensor, housekeeper, orbbec camera, lx_webrtc, static_transform_publisher map->foot_contact (vendor, global /tf).
- OURS, live on hardware DDS since ~19:25 (etime 01:02): `ros2 launch .../localization.launch.py lio_binary:=candidates/fast_lio_portable/...`
  -> fastlio_mapping (qazbot_lio_mapping), level_bridge.py, map_server, amcl, lifecycle_manager_localization (ns /qazbot_d1_map).
  This is the "localization_manual_01"-style stack left for RViz. No Nav2 planner/controller/bt, no goal_coordinator,
  no motion adapter, no localization_trust node running. Nothing stopped/started on hardware DDS by me.

## 2. Baseline tests (before any change)
| Suite | Command | Result |
|---|---|---|
| Adapter core + early takeover + new faults | `cd tests/motion_adapter && python3 -m unittest -v test_adapter_core test_early_takeover test_new_faults` | 37 run, 37 OK |
| Loc trust core + source failures | `cd tests/loc_trust && python3 -m unittest -v test_loc_trust test_source_failures` | 29 run, 29 OK (SciPy/NumPy version warning) |
| Launch stagger | `QAZBOT_SMAC_PREFIX=$PWD/candidates/nav2_smac_bounds/install/nav2_smac_planner; source ws/src/navigation/env.sh; set +e; python3 -m unittest -v tests/launch_stagger/test_launch_stagger.py` | 5/5 OK (WITHOUT QAZBOT_SMAC_PREFIX: 4 OK + 1 ERROR test_full_nav_launch_is_paced — env-dependent, not a regression) |
| Footprint polygon | `cd tests/footprint && python3 -m unittest -v test_vendor_polygon` | 4/4 OK |
| Footprint rest envelope | `cd tests/footprint && X2_URDF_DIR=/agibot/software/aimdk/extra/mc-rl/x2_urdf/X2_URDF-v1.3.0 python3 -m unittest -v test_envelope_rest` | 1/1 OK |
| control_geometry (body_frame candidate helper) | `cd ws/src/navigation && python3 -m unittest -v test_control_geometry` | 5 run, 4 OK, 1 FAIL (test_world_need_not_have_z_up: yaw 0 vs pi/2). PRE-EXISTING; module not used by nav chain (only audit_control_frame.py / candidates/body_frame_v1, which has a newer control_geometry.py). |
| nav_e2e model (domain 87, localhost, unicast profile) | `bash tests/nav_e2e/run_model.sh nav_e2e_baseline_20261021_pre_e2e` | see §2b |

Not run (by constraint): ros_driver_gate.py, ros_registration_fake.py (create MC publishers / fake SDK in domain 89),
ros_faults_dry.py (advertises a foreign McLocomotionVelocity publisher in domain 89), ros_smoke_dry_run.py
(designed for real MC graph, domain 0), real_register_observe.py (hardware). Last recorded: adapter_ros_explicit_01 7/7,
adapter_driver_gate_01 14, adapter_registration_fake_03 28, nav_e2e_driver_unicast_01 73/0 fail exits 0,
nav_e2e_14 65/0 fail, nav_restart_03 11/0 fail but abnormal_children=2.

## 3. Actual command/action chain (from code)
Launch: `ws/src/navigation/nav_full.launch.py ns:=qazbot_d1_map scan_topic:=... odom_topic:=... robot_radius:=<no default>`
(requires env QAZBOT_SMAC_PREFIX + manifest hash check, nav_full.launch.py:25-36). Nav2 nodes live in `/<ns>/nav2`.
- Public API: `/qazbot_d1_map/navigate_to_pose` (NavigateToPose action SERVER in goal_coordinator.py:47) and
  `/qazbot_d1_map/goal_pose` (PoseStamped -> coordinator re-sends to its own public action, goal_coordinator.py:46,62-67).
- Coordinator forwards accepted goal to `/qazbot_d1_map/nav2/navigate_to_pose` = standard nav2 bt_navigator (goal_coordinator.py:44,142).
- bt_navigator BT: bt/navigate_to_pose.xml = Sequence{ComputePathToPose(GridBased) -> FollowPath(FollowPath, goal_checker)}; no recovery nodes;
  plugin_lib_names only compute_path_to_pose + follow_path (nav_full.launch.py:73-81). NavigateThroughPoses -> bt/unsupported_through_poses.xml (AlwaysFailure).
- planner_server SmacPlanner2D (bounds-fixed overlay), allow_unknown false, tolerance 0.
- controller_server RPP, 20 Hz, cmd_vel remapped -> `/qazbot_d1_map/nav2/cmd_vel_nav` (nav_control.launch.py:91-92).
  Progress checker SimpleProgressChecker 0.1 m / 15 s; goal checker SimpleGoalChecker xy 0.10 / yaw 0.10 stateful; failure_tolerance 0.3;
  allow_reversing false; use_collision_detection true (nav_control.launch.py:50-72).
- velocity_smoother: cmd_vel_nav -> cmd_vel_smoothed remapped to `/qazbot_d1_map/cmd_vel` (private) (nav_full.launch.py:107, nav_control.launch.py:93-95).
  min_velocity x=0 (no reverse), lateral 0.
- goal_coordinator subscribes `/qazbot_d1_map/cmd_vel`, forwards only while gate&busy&trusted&!latched to
  `/qazbot_d1_map/cmd_vel_authorized` (goal_coordinator.py:35,49,90-103).
- x2_motion_adapter (`/qazbot_d1_map/motion_adapter`, run_x2_driver.sh) subscribes cmd_vel_authorized (x2_motion_adapter.py:268);
  dry_run -> `~/dry_run_cmd` Twist (:459-462); live+registered -> /aima/mc/locomotion/velocity (:429-430, only after explicit ~/register ACK).
- One lifecycle_manager_navigation for controller, smoother, planner, bt_navigator; attempt_respawn_reconnection false; any process exit -> whole launch Shutdown.
- No publisher on global /cmd_vel or /aima/mc/locomotion/velocity anywhere in the nav chain (grep + launch remaps). Adapter MC publisher exists only live+registered.

Is the coordinator a "second navigator"? It is an action proxy + velocity gate: it does not plan, control, or run a BT; exactly one
bt_navigator/planner/controller exists. BUT it does expose a second NavigateToPose action server (public) in front of the Nav2 one.
If the user's rule "no second navigator" is interpreted strictly as "no second NavigateToPose server", this violates it; DECISIONS.md
("Публичный NavigateToPose — координатор владения заданием; внутри ровно один стандартный Nav2 navigator") records it as accepted design.
-> OPEN QUESTION for user. Side note: bt_navigator still exposes its own `/qazbot_d1_map/nav2/goal_pose` subscription and internal
action; a goal sent there is planned/followed but commands are dropped by the coordinator gate (gate false when not busy) — HYPOTHESIS, not tested.

## 4. Requirement-by-requirement status

### R1 Motion adapter — PARTIAL
DONE:
- Unit tests for the 3 refusals: tests/motion_adapter/test_new_faults.py (latch while driving, refuse from idle, dry_run reports)
  and test_adapter_core.py (test_untrusted_localization_latches_with_its_own_reason, test_untrusted_refuses_to_start,
  test_foreign_publisher_latches_while_driving, test_foreign_publisher_blocks_start_and_clearing_it_alone_does_not_resume,
  test_vendor_autonomy_blocks_nav2). Core: x2_motion_adapter.py:178-195.
- States idle/active/latched (live) and dry_run: x2_motion_adapter.py:33,85,101-111; tests DryRun class.
- PS5 takeover at core level latches (test_operator_takeover_latches_and_never_auto_resumes, test_early_takeover.py).
- Coordinator-level: takeover aborts Goal A and needs resume + NEW goal: nav_e2e run_suite.py:144-151 (adapter status INJECTED, not real adapter);
  trust-loss path: old goal rejected before resume, resume alone does not move, Goal B accepted+succeeds run_suite.py:99-108.
MISSING/GAPS:
- G1 (blocker): goal_coordinator requires `driver_ready = live && registration=='registered' && ready` (goal_coordinator.py:75,105) and
  adapter `~/arm_motion` succeeds only if live&registered (x2_motion_adapter.py:372). With the REAL adapter in dry_run every public goal is REJECTED.
  The objective chain "...-> X2 Motion Adapter dry_run -> kinematic model" is therefore not runnable today. nav_e2e bypasses the adapter:
  model subscribes cmd_vel_authorized directly and model_fixture.py:33-49 FAKES adapter status (live=True, registered).
- G2: In dry_run the adapter never latches and never tracks takeover (by design, test_dry_run_does_not_latch_on_faults; takeover detection
  only in live ACTIVE path :163-170). 'rc' is a builtin source, so a real PS5 takeover during live dry-run is invisible. A dry-run
  "shadow" state machine (idle/active/latched evaluated in dry_run without MC output) is needed to test R1/R6 PS5 takeover through the real adapter.
- G3: "PS5 takeover cancels Goal A at action level" is tested only via injected JSON; no test with the real adapter node + fake McCommonState
  source switch (ours->rc) driving coordinator cancel; no check that Goal A's action status is ABORTED/CANCELED and the internal nav2 goal is terminal.
- adapter ROS test with fake state (ros_faults_dry.py) covers the 3 refusals at node level in dry_run (not re-run here).

### R2 Single owner of motion — PARTIAL (documented limits exist)
Present: flock run/x2_motion_owner.lock (live only, x2_motion_adapter.py:214-236,299-300); DDS graph scan of MC velocity publishers every 1 s
with vendor allow-list regex (:320-332, VENDOR_PUBLISHERS :37); MC state source check pnc/foreign sdk (:190-194); coordinator single-publisher
GID check on cmd_vel (goal_coordinator.py:90-99); explicit state machine + latch.
Missing: GetCurrentInputSource is only used in diagnostic read_motion_state.py, not in the adapter (adapter relies on McCommonState.input_source 10 Hz).
ContinuousWalker does not take the flock (code on .89). Doc X2_LOCALIZATION_TRUST.md:39-43 still describes removed 10 s/15 s timed exemptions
(code removed them: x2_motion_adapter.py:329 "never a timed exemption") -> doc stale. No consolidated "limits" section in one doc.

### R3 Localization trust — PARTIAL
Implemented in loc_trust.py / localization_trust_node.py:
- anchor + FAST-LIO drift T6 (loc_trust.py:308-321), but anchor_xy_per_m=0 (no drift growth; test_no_unmeasured_tolerance_growth enforces 0).
- ambiguity arm-check + T7 (:156-177, :322-329); degenerate_budget_m default 0.0 => any ambiguity blocks. The requested EXPERIMENTAL 2 m limit
  exists only as a test config (test_loc_trust.py:195) — not a marked experimental parameter in runtime. Docs table T7 "<= 2 m" contradicts code default 0.
- loss of scan (T1_scan_stale), odom (T1_odom_stale + T1_odom_message_stale), map->odom (T1_tf_stale) — tested.
- AMCL loss: amcl_pose stale only when moving (pose_max_age_moving_s 3 s, :262); particle_cloud has NO staleness check (last cloud reused forever,
  localization_trust_node.py:98-101). Standing AMCL death is caught only via map->odom TF staleness.
- frozen TF: only stamp-age; a TF republished with fresh stamps but frozen content (e.g. LIO odom->base_footprint stuck while robot moves) is not detected
  (no cross-check vs leg odometry / MC). HYPOTHESIS gap.
- stale/replayed/future stamps: SourceStamp (source_stamp.py; test_source_replay_cannot_refresh_age etc.).
- pose jumps T5 (:298-306); wrong initial pose -> T6 anchor/T4 score/ambiguity; map mismatch -> T4 scan score only. No check that the
  map served by map_server equals the grid trust loaded (trust reads map_yaml independently, localization_trust_node.py:44) — gap.
- No-resume-after-recovery: hysteresis stable_s 5 s + coordinator latch + explicit resume + new goal (run_suite.py:99-115).
- Thresholds labelled CANDIDATES in docstring; not all marked experimental in docs. T4 baseline required (T4_no_baseline) => trust can never be
  true on hardware until F0 baseline is collected.
- localization_trust node has never run against the live AMCL stack (doc X2_LOCALIZATION_TRUST.md "Что не доказано").

### R4 Full NavigateToPose via standard Nav2 — MOSTLY DONE (one open interpretation)
Standard bt_navigator/planner/controller(RPP)/velocity_smoother/progress+goal checker; separate BT navigate_to_pose.xml; plan_to_pose.xml kept
for nav_plan.launch.py. No nav2_controller rewrite. Open: public coordinator action server (see §3) — needs user confirmation.

### R5 Recovery — DONE (code), PARTIAL (docs)
No BackUp/DriveOnHeading/Spin anywhere; no behavior_server launched; BT has no RecoveryNode; failure -> ABORTED; allow_reversing false;
smoother min vx 0. Safe termination exists de facto (abort -> coordinator gate closed, zero published, latch for faults) but is not written
as a defined spec in X2_NAV2_CONTROLLER.md (only a comment in the XML + one line in INTEGRATION_STATUS).

### R6 E2E dry-run tests — PARTIAL
nav_e2e/run_suite.py (65 checks in nav_e2e_14; 73 in driver_unicast_01) covers: straight 1.5 m (.225 from -1.275), rotate (final_yaw pi/2 in place),
already-at-goal (no motion), cancel, trust loss, scan loss, odom loss, operator stop, unreachable/out-of-map/occupied, obstacle (wall), goal_pose topic,
adapter takeover (injected), driver loss/recovery, Goal B after recovery, model pose checks (xy/yaw <=0.11).
Missing: real adapter in chain (G1); PS5 takeover through the real adapter (G2/G3); AMCL loss as separate case (fixture uses ideal localization;
trust loss is a single Bool); false localization (wrong pose/jump) through real trust node; "Nav2 restart" in the main suite
(separate run_restart.py; nav_restart_03 has abnormal_children=2 -> blocked by lifecycle defect); "repeat goal" (same goal twice / re-send
while active) not explicitly; recovery scenario label. Wall test kinematics only (ideal localization).

### R7 Stop test — PARTIAL / location
There is no literal "last 3 zero messages" check. The tail-zero stop criterion is in tests/controller_dry/harness.py:104-105
(`tail = sm[-4:]`, `tail_zero = all(... for ... in sm[-2:])`, last 2 smoother outputs) used by checks 'S1 ends at zero' (:133),
'S4 ends at zero' (:153), 'S3 ends at zero' (:169). With Nav2 1.1.18 velocity_smoother possibly going silent after stop, this criterion is fragile/wrong.
nav_e2e already uses better criteria for cancel (status==5, no nonzero cmd after model stop, run_suite.py:93-97) but not "last cmd + time"
nor "no re-motion after recovery" uniformly. Replacement: cancel/terminal status, last nonzero cmd timestamp <= terminal+bounded, no nonzero after,
model velocity < eps and pose stable over window, after recovery/resume no nonzero until new goal.

### R8 Live dry-run bring-up on X2 — MISSING (procedure + blocker)
Pieces exist: localization.launch.py (running now), nav_full.launch.py, run_x2_driver.sh (default live=false; no MC publisher, no
registration client in dry_run: x2_motion_adapter.py:299-303), localization_trust_node.py. Not existing: a combined live dry-run launcher /
runbook; graph+TF before/after diff tool for this run; trust node + coordinator never run on hardware DDS. Blocker G1: with live=false
the coordinator rejects all goals. Also nav_full requires robot_radius (no accepted value; 0.454 used in tests) and env QAZBOT_SMAC_PREFIX.
Clean shutdown risk: lifecycle manager shutdown defect (below).
Flags guaranteeing no MC publisher: do NOT pass `-p live:=true`; never call `~/register`. In dry_run, adapter only SUBSCRIBES /aima/mc/common/state.

### R9 Docs — PARTIAL
Stale vs code: X2_LOCALIZATION_TRUST.md (test counts 16/33, T6 +0.03 m/m, T7 <=2 m, 10/15 s exemptions); X2_MOTION_ADAPTER.md body table
(27 tests, automatic ADD/ENABLE, lock on start) superseded only by header note; X2_NAV2_CONTROLLER.md lacks full-chain/coordinator/BT/safe-termination
section; README lacks dry-run e2e + live dry-run instructions; X2_FIRST_PHYSICAL_TEST_PLAN has no F0 stage; DECISIONS lacks the sdk=30 "config only"
wording? (present: "принят только как конфигурация" — OK). X2_FOOTPRINT has F1 protocol (record_walk.sh) — OK.

### R10 F0/F1 preparation — PARTIAL
F1: tests/footprint/joint_record.py, envelope.py, envelope_phases.py, record_walk.sh exist; protocol in X2_FOOTPRINT.md + test plan row F1. Not run (correct).
F0: referenced only in loc_trust.py docstring and trust services (~/confirm_anchor, ~/baseline_start/finish); no F0 procedure in test plan, no script.

## 5. Lifecycle manager shutdown defect
Standard lifecycle_manager in normal runs; candidate candidates/nav2_lifecycle_shutdown NOT QUALIFIED (STATUS.md: SIGSEGV in RMW wait / Bond::~Bond abort).
nav_restart_03: 11/11 functional checks OK but abnormal_children=2 -> a "Nav2 restart" e2e test with a clean-shutdown assertion will FAIL.
Recommendation: in R6 keep the restart test functional checks (goal lost on restart, no resume, new goal works) and record the shutdown
exit codes as a KNOWN-DEFECT expected-failure, not suppressed; do not add timeouts/disable bond (INTEGRATION_STATUS forbids).
nav_e2e main suite (no restart) shuts down cleanly (nav2 0, abnormal 0 in nav_e2e_14/driver_unicast_01).

## 2b. nav_e2e baseline (run once, isolated)
`cd /home/agi/control_backend && bash tests/nav_e2e/run_model.sh nav_e2e_baseline_20261021_pre_e2e`
(ROS_DOMAIN_ID=87, ROS_LOCALHOST_ONLY=1, dds_domain87_unicast.xml; model fixture + real Nav2 + coordinator; NO adapter, NO MC types).
Result: 73 checks, 0 failed; shutdown nav2=0 model=0 abnormal_children=0; rc=0.
Artifacts: tests/integration_results/nav_e2e_baseline_20261021_pre_e2e/{results.json,nav2.log,model.log}. No leftover processes.

## 6. Ranked gaps (most important first)
1. G1 dry-run chain blocked: coordinator driver_ready + arm_motion require live&registered -> real adapter in dry_run rejects all goals.
2. G2/G3 PS5 takeover not observable in dry_run; takeover->Goal A cancel tested only by injected status JSON.
3. nav_e2e does not include the real adapter nor the real trust node; model consumes cmd_vel_authorized instead of adapter dry_run_cmd.
4. Stop criterion tail-zero (controller_dry/harness.py:105) fragile with 1.1.18 smoother.
5. Trust: no particle_cloud staleness; AMCL-loss while standing only via TF; no frozen-content TF check; no map identity check;
   2 m experimental ambiguity budget not a runtime-marked experimental option (default 0).
6. No live dry-run launcher/runbook + graph/TF diff; trust node never run on hardware DDS.
7. Lifecycle shutdown defect makes Nav2-restart e2e unclean (known, unfixed).
8. Docs stale (LOC_TRUST, MOTION_ADAPTER body, NAV2_CONTROLLER chain/termination, README, test plan F0).
9. Pre-existing unrelated failure: ws/src/navigation/test_control_geometry.py::test_world_need_not_have_z_up.

## 7. Minimal implementation plan (ordered, file-level)
1. ws/src/navigation/x2_motion_adapter.py: add dry-run "shadow" state machine: in dry_run run the same idle/active/latched logic
   (fault latch, rc/ takeover detection via McCommonState.input_source change while shadow-active, resume) but NEVER create MC publisher /
   registration client; report `{live:false, mode:'dry_run', shadow_state, ready, motion_armed}`; make `~/arm_motion` succeed in dry_run when
   shadow idle & healthy. Keep test_dry_run_* semantics explicit (update tests deliberately, not silently).
   Takeover in dry_run: define "operator input observed" = MC source == 'rc' with joystick activity while shadow ACTIVE (needs evidence of how
   McCommonState reports rc when operator touches stick; HYPOTHESIS — verify passively).
2. ws/src/navigation/goal_coordinator.py: accept driver_ready for dry_run only when an explicit parameter `allow_dry_run_driver:=true` is set and
   adapter reports mode dry_run+ready; never treat dry_run as permission for MC. Keep all trips.
3. tests/motion_adapter/: new unit tests for shadow states + arm in dry_run + takeover latch in dry_run; extend ros_faults_dry.py (domain 89,
   no MC velocity publisher other than the existing foreign-endpoint diagnosis) for rc takeover -> latched.
4. tests/nav_e2e/: new suite (e.g. run_suite_adapter.py) with REAL adapter (live=false) in chain: model subscribes
   `/<ns>/motion_adapter/dry_run_cmd`; fake McCommonState publisher (domain 87 only) to drive source ours/rc; cases from R6 incl. PS5 takeover
   (Goal A status ABORTED, internal nav2 goal terminal, resume alone no motion, Goal B only), AMCL loss, false localization (pose jump via real
   trust node in a sub-suite), repeat goal, restart (expected-known-defect for shutdown). Use the new stop criterion (R7).
5. tests/controller_dry/harness.py:104-105,133,153,169: replace tail_zero with terminal-status + last nonzero cmd time + no nonzero after +
   model stopped + no re-motion window.
6. ws/src/navigation/loc_trust.py + localization_trust_node.py: particle cloud staleness; AMCL liveness while standing (amcl_pose/cloud age
   with explicit candidate threshold); optional map identity check (hash of /map vs loaded yaml); `degenerate_budget_m` documented as
   EXPERIMENTAL 2.0 opt-in parameter (default stays 0 unless user decides); unit tests for each.
7. New ws/src/navigation/run_live_dry.sh (or launch) + tools graph/TF snapshot diff: start trust node, nav_full (robot_radius explicit), adapter
   live=false, staggered; assert no new endpoint on /aima/mc/locomotion/velocity, no SetMcInputSource client, no /cmd_vel publisher; clean SIGINT.
   Needs user consent before running on hardware DDS (localization already running).
8. Docs: X2_LOCALIZATION_TRUST.md, X2_NAV2_CONTROLLER.md (chain, coordinator role, BT, safe termination), X2_MOTION_ADAPTER.md (shadow dry-run),
   README, DECISIONS (dry-run driver acceptance, coordinator), X2_FIRST_PHYSICAL_TEST_PLAN (add F0 procedure), X2_FOOTPRINT (F1 link).

## 8. Open questions
- Is the coordinator's public NavigateToPose action server acceptable under "no second navigator"? (DECISIONS says yes; user rule ambiguous.)
- How does McCommonState.input_source behave when the operator touches the PS5 while no SDK source is active (needed for dry-run takeover)?
- Accepted robot_radius for live dry-run (0.454 test value vs measured)?
- Should the experimental 2 m ambiguity budget be enabled by default in dry-run, or opt-in only?
- Consent to start trust/Nav2/adapter(dry_run) on hardware DDS alongside the running localization stack.
