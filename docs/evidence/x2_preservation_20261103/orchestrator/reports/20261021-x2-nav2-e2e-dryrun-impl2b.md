# 20261021-x2-nav2-e2e-dryrun — phase 2b: re-derive Nav2 map from NEW D1 first_map

Status: DONE — real-chain e2e nav_e2e_real_dryrun_08 160/160 on first_map_nav2_v2
Repo: /home/agi/control_backend (not git). Pre-task snapshot: /home/agi/control_backend_backup_20261021_pre_e2e.tar.gz
(contains old first_map + first_map_nav2_v1). ws/src/maps/first_map/ (D1-owned) NOT modified.

## 1. Inputs observed
- NEW ws/src/maps/first_map (files written 21:11:48-50 +08): map.pgm 319x426 P5, res 0.05, origin [-13.3,-10.4,0],
  free_thresh 0.25, occupied 0.65, trinary; meta.yaml saved 2026-10-21T18:07:30+05:00 (slam_toolbox via d1_mapping.launch.py).
  Pixel histogram {0: 4708, 205: 76223, 254: 54963} -> same 3-value D1 encoding as the old map.
  sha256: map.pgm 545630c1…2e33, map.yaml a5938d70…de54, meta.yaml 3a8e9fd9…a29. home.yaml absent.
- OLD (snapshot): map.pgm 438x415, origin [-8.15,-11.5], pgm 7bf9bcbf…5d8a; first_map_nav2_v1 manifest.

## 2. Derivation procedure (reproduced, not invented)
Documented procedure (docs/NAVIGATION_STATUS.md §1.2 + §5, docs/DECISIONS.md §1, v1 manifest "change"): PGM copied
byte-identically; YAML = source YAML with only free_thresh 0.25 -> 0.196 (so stock nav2_map_server trinary reads 205 as
unknown, like D1 QORGAN map_io; 254 free, 0 occupied); origin/resolution unchanged; manifest with hashes.
No script for it existed in the repo (grep for first_map_nav2/free_thresh across tools/tests/ws). I encoded exactly this
procedure in tools/derive_nav2_map.py and VALIDATED it: run on the snapshot's old first_map it reproduces
first_map_nav2_v1 byte-identically — map.pgm 7bf9bcbf…, map.yaml 9dae2c03…, manifest.json 119afa47… (all 3 sha256 equal).
The tool asserts free_thresh==0.25/occupied 0.65/trinary/negate 0 and pixel values ⊆ {0,205,254} before deriving.

Derivative: ws/src/maps/first_map_nav2_v2/ (new name; v1 name not reused)
  `python3 -I tools/derive_nav2_map.py ws/src/maps/first_map ws/src/maps/first_map_nav2_v2 "D1 first_map (saved 2026-10-21T18:07:30+05:00)" --extra`
  map.pgm    545630c14f0e37cf2385af8780a09ad7a3fddf7765460c1624e645c42aee2e33 (== source pgm)
  map.yaml   f49dd066e7ed40699fb26558745d743383b49b16043b1d079af645d17d821e6d
  manifest   e8c77a18e6cf79c024b2d213bc5a8eab23f6c1032f1d7cd0a8cb7a2ecf06d646
  Manifest keeps the 5 v1 fields (source, source_yaml_sha256, pgm_sha256, change, status) + provenance (--extra):
  source_meta_sha256/saved, map_yaml_sha256, size_px, resolution, origin, pixel_histogram, derived_by.
  status stays "localization candidate, not accepted for movement".

## 3. References updated (per-file backups *.bak_20261021_pre_map_v2)
| File | Change |
|---|---|
| tools/derive_nav2_map.py (new) | encodes the documented v1 procedure; validated byte-identical on old source (section 2) |
| ws/src/maps/first_map_nav2_v2/ (new) | map.pgm, map.yaml, manifest.json (section 2) |
| ws/src/navigation/nav_full.launch.py, nav_plan.launch.py, nav_debug_bounds.launch.py, localization.launch.py | default map_yaml -> first_map_nav2_v2 (running PID 284182 unaffected: launch defaults are read at start) |
| ws/src/navigation/localization_trust_node.py | default map_yaml -> v2 (the trust grid = T8 expected identity; served grid compared by res/origin/size/occupied cells). expected_map_sha256 default stays '' (no hard-coded hash in the node) |
| tests/nav_e2e/e2e_map.py (new) | single source for MAP_YAML, EXPECTED_PGM_SHA256 (read from v2 manifest), START and goals (section 4) |
| tests/nav_e2e/run_suite_real.py | poses from e2e_map; trust node started with expected_map_sha256 = manifest pgm_sha256; new check trust_map_is_nav2_v2 (status map_sha256 == manifest) |
| tests/nav_e2e/run_suite.py (legacy), loc_model_fixture.py, model_fixture.py | map + START/goals/wall from e2e_map (sim yaw also set from START) |
| tests/nav_e2e/run_restart.py, repro_manager_shutdown.py, repro_bounds.py | same pose constants via e2e_map (aux repro scripts; they use model_fixture, which now starts at the v2 START) |
| tests/loc_trust/test_loc_trust.py, test_trust_gaps.py, ros_e2e_synthetic.py | map path -> v2; synthetic TRUE (old "near point B" 1.07,-6.26) -> fixture pose (-1.275,-4.825,0) (104° FOV scan there: not ambiguous, competitor 0.50 < 0.95) |
| tests/launch_stagger/test_launch_stagger.py | map path -> v2 |
| tests/planner_clearance/run_isolated.py | map v2; result dir from argv (old fixed dir already exists); identical-cases baseline per map (results/x2_cell_margin_true_<map>.json; old baseline is old-map only) |
| tests/planner_clearance/run_variant.sh, harness.py (PC-only path, NOT run here: broad pkill of map_server/lifecycle_manager would hit the running hardware localization) | map v2; start TF (2.025,-5.025 is unknown on v2) -> (-4.375,-4.875) clearance 1.56 m; NavigateToPose goal (0.175,-5.925: clearance 0.38 < R) -> (-2.875,-4.875) clearance 1.87 m |
Not changed (historical, tied to old-map bags/evidence; need v1 from the snapshot if re-run): tests/localization/{multiview_diagnostic,height_diagnostic,navigate_plan_clearance}.py; candidates/body_frame_v1/navigation/localization.launch.py (candidate, not in the chain).
controller_dry uses a synthetic wall world, no map: unaffected.

## home.yaml
Nothing in code/tests/launch read home.yaml (grep over ws/, tests/, tools/, run/: 0 hits). It was documentation of the old
mapping start (map (0,0), heading +x). The e2e START is an explicit fixture pose (e2e_map.py), not a physical home pose.
No home pose was fabricated for the new map.

## 4. Fixture pose selection on v2 (offline, scratchpad scripts; distance = OpenCV L2 distance transform to nearest non-free (0 or 205) pixel)
Constraint kept from the suites: drive direction +x, box obstacle = line x = wall_x with wall_x < 0 meaning "disabled"
(sim_robot.py / loc_model_fixture.py), so wall at START_X+1.3 must be >= 0 -> START_X >= -1.3. Same x offsets as before.
Search over all free cell centres: min clearance over [s-0.1, s+1.8] at row y. Best with the wall constraint:
0.65 m at s=-1.275, y=-4.825 (old corridor -1.275,-6.375 on the old map: 0.61 m, start 0.61 m). Unconstrained best
1.55 m at (-4.3,-4.875) — not used because it would need the wall sentinel changed in the shared sim (controller_dry).
| Pose | value | clearance | purpose |
|---|---|---|---|
| START (-1.275,-4.825,0) | 254 | 0.92 m | start; rotate in place (pi/2) — > 0.454+0.01 inflation |
| (0.225,-4.825) | 254 | 0.71 m | straight 1.5 m |
| (-0.375,-4.825) | 254 | 0.73 m | repeat goal |
| (0.425,-4.825) | 254 | 0.67 m | cancel / obstacle / fault / takeover goal A (1.7 m) |
| (-0.775,-4.825) | 254 | 0.74 m | goal B / RViz goal_pose (0.5 m) |
| wall x=0.025, |y+4.825|<=0.3 | — | — | unmapped box 1.3 m ahead (local costmap only) |
| OCCUPIED (-1.225,-3.575) | 0 | centre of 3x3 occupied block | occupied goal, 1.25 m from START |
| out-of-map | — | — | left x=-13.300001, right x=2.650001 (y=START), bottom y=-10.400001, top y=10.900001 (x=0); computed from yaml origin+size±1e-6 |
| unreachable (99,99) | — | — | unchanged |
Trust-node ambiguity at START (fixture 360-beam ray-cast, L.ambiguity defaults): nominal 1.0, competitor 0.34 (old map START: 0.25) -> not ambiguous.

## 5. Test runs (all domain 87, ROS_LOCALHOST_ONLY=1, unicast profile; PID 284182 hardware localization untouched)
Unit (log tests/integration_results/impl2b_unit_20261021.log; commands as in the implA report):
adapter 43/43 OK; trust 46/46 OK; stop_criterion 7/7 OK; footprint polygon 4/4 + envelope_rest 1/1 OK;
launch_stagger 5/5 OK (QAZBOT_SMAC_PREFIX set). controller_dry `bash tests/controller_dry/run_dry.sh`: 19/19 ok, rc 0
(tests/integration_results/impl2b_console_20261021/controller_dry_01.log; controller_dry/last_run.json is stale 16:34 — the harness prints to stdout).
Legacy `run_model.sh --legacy nav_e2e_legacy_impl2b_01`: 73/73, rc 0, shutdown nav2 0/model 0, abnormal_children 0
(wall_body_clearance 0.749 m).
Real chain:
- nav_e2e_real_dryrun_06 (as requested): bring-up OK (no bt_navigator race), 84 checks passed, then FAILED at
  scan_loss_goal_accepted (goal rejected: "Driver arm rejected: localization_untrusted"). Shutdown clean (all 0).
- nav_e2e_real_dryrun_07: harness extended with a diagnostic trust_log (each change of trust reasons + scan_score) and
  the rejection detail. Failed at takeover_goal_A_accepted. trust_log shows the cause: right after reset_pose one
  evaluation with scan_score 0.58 -> ['T4_scan_score','T4_scan_drop'], then 'settling' (stable_s = 5 s hysteresis).
  Root cause: reset_pose TELEPORTS the model (~0.4–1.7 m jump). The scan around the discontinuity is scored at a pose
  inconsistent with it (odom interpolated across the jump), so the trust node correctly reports one bad match and then
  needs 5 s of clean evaluations. Product behavior correct; precondition missing in the fixture. On the old corridor
  the same mismatch evidently stayed above the T4 floor (more x-parallel walls), on the new one it does not.
  Fix (test only): reset_pose additionally waits until trust is back (`wait(trusted, 40)`, same 40 s observation
  deadline already used by the suite after restores). No runtime threshold/timeout changed.
- **nav_e2e_real_dryrun_08: 160/160 passed, rc 0**, shutdown nav2_restarted/adapter/trust/model 0, abnormal_children 0,
  first group abnormal 0. Includes trust_map_is_nav2_v2 (trust status map_sha256 == manifest pgm), straight 1.5 m
  (xy err 0.064), rotate in place (yaw err 0.055), obstacle clearance 0.745 m > 0.464, out-of-map x4 + occupied
  aborted without motion, PS5 takeover, losses, false localization, driver loss, Nav2 restart (goal B xy err 0.057).
  10 teleport-induced T4 transients recorded in trust_log, all recovered before the next case.
planner_clearance: NOT run (user priority change: minimize run time). run_isolated.py prepared for v2 (per-map baseline;
no v2 baseline exists yet -> first run would enforce only violations_lt_R == 0). run_variant.sh is PC-only.

## 6. bt_navigator startup race
Did not recur in runs 06/07/08 (0 "not available after waiting"/"Exception when loading BT" lines). Root cause from
run 04 nav2.log: planner_server action server exists since configure (35.89 s) and is active at 36.95 s; bt_navigator
activation creates the BT's compute_path_to_pose action client at 36.95 s and BtActionNode waits
`wait_for_service_timeout` (bt_navigator param, Nav2 default 1000 ms, not set in nav_full.launch.py) -> fails at
38.04 s. So the failure is DDS endpoint discovery of a NEW client (created during activation) taking > 1 s on the
loaded Jetson. Contributing (to be confirmed, not proven): tests/nav_e2e/dds_domain87_unicast.xml lists initial peers
only for participant ids 0–7 (ports 29160–29174) while the real-chain run has >= 10 participants; participants with
id >= 8 (bt_navigator/lifecycle/coordinator start last) are found only via their own announcements, and a lost
reliable SEDP message is repaired only at the next builtin heartbeat. Candidate fixes for a later task (not applied):
extend the peers list to the participant count, and/or set wait_for_service_timeout from a measured discovery latency.

## 7. Doc spots mentioning the old map (phase B; not edited)
- docs/README.md:34 — first_map_nav2_v1 as the Nav2 derivative.
- docs/DECISIONS.md:12 — example `first_map_nav2_v1`.
- docs/NAVIGATION_STATUS.md:72-74 — "six source files, SHA-256 match", maps/first_map_nav2_v1 (now 5 files, no home.yaml; v2).
- docs/X2_NAV_STACK_RUNBOOK.md:77 — point B, occupied pixel (1.43,-5.88) etc. on the old map.
- docs/INTEGRATION_STATUS.md:9,27,219-220 — first_map transfer / nav_e2e_14 / relocalization peaks analysis on the old map.
- docs/X2_LOCALIZATION_TRUST.md:4 — ros_e2e_synthetic "real map" (TRUE pose changed).
- tests/planner_clearance/README.md — results table is for the old map.
- Also mention tools/derive_nav2_map.py and tests/nav_e2e/e2e_map.py (new) in README/test docs.

## 8. Open issues
1. Running hardware localization (PID 284182) still serves the OLD map from memory; v1 files are gone -> phase C restart with v2.
2. Fixture poses are fixture-only; no physical home/start pose for the new map (home.yaml absent). Physical anchor pose still needed for live.
3. planner_clearance on v2 not run; no v2 baseline. ros_e2e_synthetic (domain 88) not re-run.
4. tests/localization/* diagnostics and candidates/body_frame_v1 still reference v1 (historical; need snapshot).
5. bt_navigator discovery race: diagnosed, not fixed (section 6).
6. Real-chain fixture teleport needs trust re-settle (~5 s per case, test time only).
