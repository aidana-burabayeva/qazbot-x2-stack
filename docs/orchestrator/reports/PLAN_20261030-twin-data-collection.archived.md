# PLAN — 20261030-twin-data-collection (see TASK.md)
Previous: reports/PLAN_20261030-p0-repo-deploy-alignment.archived.md (last: P26 approved; ab_eval_motion 3600 s running, raw_scan_bridge pid 373764).

## Tasks
| # | Task | Status | Agent |
|---|---|---|---|
| T1 | Discovery + config snapshot + start load-guarded selective recorder | DONE (19:13 robot; agent died on 5h limit, work saved: snapshot/, core recorder pid 415855 running, hf DROPPED by guard — load; hf only on demand) | general-purpose opus/high |
| T2 | Dynamics + scan A/B metrics from bag (after A/B run ends) | PENDING | robotics-debugger opus/high |
| T3 | Docs, manifest, calibration contract, commit/push, transfer | PENDING | robotics-implementer sonnet/medium |
| T4 | Independent verification | PENDING | robotics-verifier opus/high |

## Routing
| Step | Agent | Model | Effort | Reason |
|---|---|---|---|---|
| T1 | general-purpose | opus | high | live robot under load, recorder can degrade nav (precedent 17:4x) |

## Open for operator
- External ground truth: operator declined (18:5x) -> all GT-dependent metrics GROUND_TRUTH_UNAVAILABLE; odom-only metrics labeled as such.

## Continuation
Resume: "продолжи задачу 20261030-twin-data-collection из .orchestrator/PLAN.md"
- 20:4x Seed in corridor: after 2 nomotion sigma .26/2nd .29 -> not READY (gate .25/.15); standing gives no further AMCL updates (update_min .2/.15) -> could stall. Orchestrator +3 nomotion -> READY sigma .22, 2nd .00-.18, score .81 (excellent). Proposal: seed_loop adaptive: up to 5 nomotion, stop as soon as READY (avoid over-sharpening).
- 20:5x Corridor goal (-8.17,1.09): at 6.5 s T5 jump 0.40 m (along corridor; 2nd mode .46 just before; after jump score .85 -> likely a valid correction of along-corridor drift) -> cancel + READY lost (needs reseed). T5 .30 too strict for corridor aliasing; propose T5 -> pause + auto re-anchor when post-jump quality good (score>=.35, sigma<=.35), else cancel; interim T5 .5.
- 21:0x Operator: do variant 1 (T5 -> pause + re-anchor) - P27 running; keep testing. alpha3 .05 live (trust odom distance in corridors).
- 21:1x Corridor goal (-8.04,0.85) cancelled at 6 s by T5 again (P27 not yet live).
- 21:2x P27 DONE (Trust only, pid 413218; reports/20261030-p27-t5-reanchor.md; rollback_p27.sh). Caveat: today's incident would still fail (post-jump sigma .36>.35, 2nd .32-.34>.30) and during pause robot stands -> AMCL doesn't update -> metrics frozen. Proposal: reanchor thresholds .40/.35 + request 2 nomotion updates inside the reanchor window. Needs operator OK (Trust restart -> reseed).
- 21:3x P27 BUG: Trust crashed at first live re-anchor (rclpy 'Logger severity cannot be changed between calls' — ternary warn/info same call site) -> GC trip. Fixed by orchestrator, Trust restarted; needs initialpose.
- 21:4x Trust restarted with fix (pid 417274); operator seed -> READY sigma .06 2nd 0 score .90; GC was stuck latched ('requires terminal old action') -> stop+resume cleared. Note: GC resume after Trust crash needs stop first (bug to document).
- 21:5x Goal (-12.42,-4.38,-13): SUCCEEDED 25 s with two live T5 re-anchors ACCEPTED (0.37 m: sigma .07 score .47; 0.50 m: sigma .02 score .71) — P27 works live. FINAL dist .19 m, yaw err -12.3 deg (needed ~159 deg turn). Check: two jumps in 5 s at goal — investigate (corridor along-axis?).

- 19:13 Resumed by laptop session (limit hit in both robot sessions, resets 22:20 CST). Uncommitted in checkout: bt/navigate_to_pose.xml, goal_coordinator.py, loc_trust.py, localization.launch.py, localization_trust_node.py (review fixes + P27 + Trust crash fix) — commit/push awaits operator consent. T2 waits for ab_eval_motion end (~19:34 robot); analysis offloaded off-robot to keep scan age clean.
- 19:2x T2 preliminary done on closed core bag (reports/20261030-t2-prelim-t5-jumps.md). Full T2 still waits for ab_eval_motion end (~19:34). Open item from session f5eb7204 (two jumps at goal) = partially explained, not confirmed.
