# PLAN — 20261030-p0-repo-deploy-alignment (see TASK.md)
Previous: reports/PLAN_20261028-scan-latency-rviz-goal.archived.md (P6 was WAITING OPERATOR; not resumed)
Skill install 2026-10-30: nav/first-goals-20261028 @ 982f618; PR branch tooling/robot-sync-validate-skill-main @ a77161d (manual PR pending).

## Tasks
| # | Task | Status | Agent |
|---|---|---|---|
| A1 | Live runtime audit | DONE: our stack not running (reboot 13:40), manual start*.sh from deployment docs/evidence; no deps on checkout path; env.sh runtime/ relative-path risk; qazbot-speech.service crash loop (1473 restarts, DDS churn) untouched; before-snapshot reports/20261030-p0-before-snapshot/; reports/20261030-p0-A1-runtime-audit.md | robotics-debugger opus/high |
| A2 | Git ws/src vs deployed ws/src SHA256 diff + other trees | DONE: ws/src 86 identical/0 different; 63 only-live = .bak/candidates; no ws/install; reports/20261030-p0-A2-git-vs-live.md | robotics-debugger sonnet/medium |
| D1 | Docs + skill symlink | DONE 4f492d1 pushed; symlink ~/.claude/skills/robot-sync-validate; discovery from deployment cwd OK | robotics-implementer sonnet/medium |
| V1 | Independent verification | PASS after cycle 1 (d3db63a). Earlier FAIL: 3 doc fixes (fast_lio 68 dangling; stack now running since 16:24 by other session PID 201530 incl. motion_adapter live:=true; live map changed 16:23 sha 72ee4094 != HEAD). reports/20261030-p0-V1-verifier.md |
| D1b | Apply V1 REQUIRED_FIXES | DONE d3db63a pushed | general-purpose sonnet/medium | robotics-verifier opus/high |

## Routing
| Step | Agent | Model | Effort | Reason |
|---|---|---|---|---|
| A1 | robotics-debugger | opus | high | live robot, safety-relevant, DDS impact |
| A2 | robotics-debugger | sonnet | medium | mechanical read-only comparison with interpretation |
| D1 | robotics-implementer | sonnet | medium | docs + one reversible symlink, precisely specified |

## Open for operator
- Other session (PID 201530) runs our stack since 16:24 incl. motion_adapter live:=true; live map changed 16:23 (72ee4094, not in Git).
- qazbot-speech.service crash loop (DDS participant churn).
- Migration to /home/agi/qazbot-x2-stack: NOT executed, needs approval + stack-down check.
- PRs: tooling/robot-sync-validate-skill-main, docs/repo-deploy-alignment-20261030 (gh unavailable).

## Continuation
Resume: "продолжи задачу 20261030-p0-repo-deploy-alignment из .orchestrator/PLAN.md"

## 2026-10-30 session
- 16:23 Robot rebooted; operator edited D1 map (pgm sha 72ee4094...) = main map. EV=/home/agi/control_backend/docs/evidence/nav_20261030_1623. Vendor trim; loc+Trust 207979.. (T5 1.0); adapter 208744 (P20 9c05bb34, source 1030a); nav_full waits seed; recorder/status_logger/seed_loop started.
- 16:3x Adapter registered 1030a (MC STAND rc); native odom not latched; monitor armed. Waiting operator initialpose on edited D1 map.
- 16:3x initialpose #2: READY, sigma .33, 2nd .20, score .36. Ready for heading tests.
- 16:44 goal_watch.py (final xy/yaw error per goal) + monitor b80zdn7fa armed; operator will test final alignment then doorway pass.
- 16:5x GOAL (-4.65,-7.06, yaw 176): SUCCEEDED 25.7 s (~7.5 m, 3 short T1 pauses auto-resumed) but final yaw 30 deg SHORT (CW finish), 7 deg roll-back. adapter.log: integral I always 0; small target +0.07 inflated to u=+0.60 opposite. P23 fix candidate (general-purpose opus/high), no restart yet.
- 17:0x Goal (-7.54,6.41) REJECTED at accept: momentary T1 (trust false at that instant); 1 s later trust ready, scan_score .85 sigma .21. Minor blocker: GC rejects new goals during transient T1 instead of accept+pause.
- 17:1x GOAL (-7.48,6.38,-8deg) SUCCEEDED 50.7 s ~14 m (collision-ahead wait + T1 pauses + TF future-extrapolation errors). P23 candidate ready: yaw_w_min .15 (no small-target inflation), integral held across short exits, CW not weak after dead time. Goal-checker finding: Nav2 'Reached' while still turning when err crossed 20 deg, then MC roll-back ~11 deg -> final 30 deg. Recommend P23 + yaw_tol .17.
- 17:2x GOAL (-8.48,1.19,180): reached xy ~0.14 m but final alignment oscillated (err -72 -> +30 -> +99 deg) ~80 s with frequent T1 pauses + TF future-extrapolation errors; operator kept sending goals (rejected busy); orchestrator stopped it (GC stop) and resumed. Next: deploy P23 + yaw_tol .17 (awaiting operator).
- 17:2x GOAL (-7.52,1.11,-35): SUCCEEDED 12.5 s, FINAL dist 0.08 m, yaw err -14.4 deg (within 20 tol).
- 17:3x GOAL (-5.39,-6.92): stuck 1.07 m short (collision ahead, patience, global clear, retries) -> Goal failed at 57 s. Local lethal ~0.5 m at -135 and 90 deg, ~1 m at 45 deg: narrow spot/doorway; RPP projection blocks.
- 17:4x P23 file deployed on disk (c76ac1d1, bak pre_p23) but adapter restart ABORTED (GC busy: operator sent goal (-3.35,-3.56,80deg)); running adapter still P20. yaw_tol .17 live + scripts. Doorway analysis: frames ~0.5 m each side of centre, robot half-width ~.36 -> ~12-14 cm clearance; RPP straight-to-carrot sweep + replans 'no valid path' (doorway narrow in global map w/ inflation).
- 17:4x Goal (-3.35,-3.56) cancelled: TRIP localization_untrusted T1 scan_stale >3 s. Since 17:00: 50/498 statuses not motion_ok, scan age p50 .22 p95 .82 max 1.63; load 22; recorder 75% CPU -> recorder stopped.
- 17:59 GC resumed; adapter restarted with P23 (c76ac1d1), registered 1030a; yaw_tol .17 live. Ready for goals. (goal_watch currently tracks rejected goal -3.11,-3.18 — ignore.)
- 18:0x Goal (-2.48,-3.56,176): stuck at start 30+ s: collision ahead (nearest lethal ~0.75 m at -45 deg), patience, global clear, T1 pause, repeated TF future-extrapolation + 'Unable to transform robot pose'. Timing (odom TF lag) degrading.
- 18:0x Goal (-2.48,-3.56) FAILED at 51 s without moving (collision ahead/TF/T1). Awaiting operator decision: TF/odom timing diagnosis then MPPI.
- 18:1x BT recovery now clears LOCAL+GLOBAL after 10 s blocked; nav_full restarted 264319 (yaw_tol .17 via script). Operator observation: clearing works in front; trails stay when person goes very close (<~0.45 m lidar min/below slice) or to sides/back (out of +-45 deg FOV). Permanent fix: time decay for out-of-FOV marks (STVL) — TODO.
- 18:3x P23+tol .17 test1: goal (-2.75,-2.13,-174): SUCCEEDED 12.1 s, FINAL dist .23 m, yaw err -7.9 deg (was 14-30 deg before).
- 18:3x Operator observation: final heading was exact, then MC mode switch (LOCOMOTION->STAND) rotated robot back slightly (~8 deg). => residual error = MC stand-transition yaw shift (matches earlier 7-11 deg CCW / 4-5 deg CW roll-back). Plan: collect per-direction stats over next goals, then compensate (GC goal-yaw pre-bias by expected roll-back, or post-settle single re-alignment).
- 18:3x test2: goal (-2.57,-0.46,87): SUCCEEDED 11.6 s, settled dist .35 m, yaw err +8 deg (CCW finish?).
- 18:4x test3: goal (-4.52,-6.13,114): reached xy .16 m but final ~150 deg CW rotation failed: Nav2 target sign flips (-.15,+.15,+.47) CCW long way?, collision ahead, MC meas +.75/0; settled err -149 deg at 52 s. P24 diagnosis launched (general-purpose opus/high).
- 18:5x P24: RPP rotates to goal yaw only while d<xy_tol(.20); in-place turn drifts nav point ~.19 m -> d>.2 -> RPP faces goal point (behind) -> CW/CCW flips. Interim: xy_tol .35 live + scripts. Proper fix: latch 'xy reached -> rotate to goal yaw' with hysteresis (candidates/nav2_pose_refine). Also collision-ahead during final rotation (local marks), adapter eps (>yaw_w_min+1e-6).
- 19:0x test4 (after xy_tol .35): goal (-2.62,-1.20,-100): start blocked 10 s -> LOCAL clear recovery worked -> walked 5.4 m; final ~164 deg turn completed; SUCCEEDED 46 s; settled dist .16 m, yaw err -5 deg.
- 19:1x Operator: robot turned toward chair side (shortest angle, RPP ignores obstacles in direction choice). Recorded senior proposal in DECISIONS (swept-footprint direction choice; MPPI).
- 19:2x Operator: one more test now, then implement all proposed fixes. Queue after test: (a) standstill AMCL refine node (periodic nomotion updates when robot still >1 s), (b) AMCL alpha rotation noise .1->~.3, update_min_a .05 (map now good), (c) adapter eps fix, (d) docs incl. clearing decision + rotation direction. P25 MPPI/STVL prep launched in background (no live changes, nice build).
- 19:3x Localization lost mid-goal (operator). Stopped goal, GC resumed. Fixes live: AMCL alpha1 .3/alpha2 .2/alpha3-5 .15, update_min_a .05 (persisted, bak pre_alpha) + standstill_refine.py (nomotion 2 Hz when still >1 s). Result standing, NO new initialpose: sigma .12, 2nd .01, scan_score .54 (self-recovered, best so far).
- 19:4x Operator: localization drifted near tables (unmapped chairs) -> robot went wrong side; re-seed snapped to wrong place. Cause: scan dominated by unmapped furniture; D1 map 0.2-0.3 m vs X2 slice .26-1.71; T5 1 m allowed slide; continuous refine may sharpen wrong mode -> limited to 10/still period. Proposals: AMCL beam skipping (do_beamskip), high wall-only slice for localization, 3D NDT localization (as robot .89), corridor tests.
- 19:5x P25 DONE: MPPI 1.1.18 (deb same snapshot) + STVL 2.3.4 (source build) in runtime/nav_ext_ws; offline load OK (domain 77); candidates nav_control/nav_full .p25_candidate with switches controller:=rpp|mppi obstacle_layer:=i1|stvl (defaults = current). MPPI CPU 0.10 core @ batch 1000/20 Hz. Risk: STVL decay 8 s forgets chair out of FOV during rotation. Not enabled; live A/B pending operator.
- 19:5x Operator: send everything to independent senior reviewer (3D excluded). robotics-verifier opus/high launched (read-only) -> reports/20261030-independent-review.md.
- 20:0x Independent review (reports/20261030-independent-review.md): root cause = localization integrity, MPPI/STVL premature. REMOVE standstill_refine (stopped now), seed_loop max 2 nomotion; restore quality gate (T5 .30/.25, PAUSE on sigma>.35/2nd>.3/score<.2 >1 s, READY thresholds); AMCL likelihood_field_prob+beamskip, update_min .20/.15, resample 2, separate AMCL scan FOV-cut max_beams 90 range 15; vertical-extent filter + edited localization map; RPP keep; recovery: wait 2 s, clear global, local ClearCostmapExceptRegion 1.0; decouple position & heading (turn-to-angle primitive); adapter bug line 352; move nav point to rotation centre (0.087 m); robot vs operator clock offset ~1 h.
- 20:1x Operator approved review items 1-2. P26 (general-purpose opus/high): Trust quality pause (sigma/2nd/score, transient like T1) + T5 .30/.25 + READY thresholds; seed_loop 2 nomotion; AMCL likelihood_field_prob + beamskip + update_min .2/.15 + resample 2; adapter eps fix; rollback_p26.sh. Operator worried about regression -> rollback script mandatory.
- 20:1x Operator: decide scan source by NAVIGATION in motion, not standing. raw_scan_bridge restarted (pid 373764) + ab_eval_motion 3600 s running (moving windows only) -> ab_motion_20261030.json.
- 20:3x P26 DONE (reports/20261030-p26-loc-integrity.md): Trust Q-pause + T5 .30/.25 + READY gating, seed 2 nomotion, AMCL prob+beamskip etc, adapter eps; rollback CONFIRM=1 rollback_p26.sh. Nav2 waits initialpose.
