# TASK

## Task ID
20261027-rpp-local-collision

## Objective
Root-cause and fix the RPP local collision gate that aborts goals on physical X2 although space ahead is free
("detected collision ahead" -> "Controller patience exceeded" -> NavigateToPose ABORTED). Owner hypothesis:
local collision check is too strict and strangles navigation.

## Robot context
- Physical X2 PC2 (this host), ROS 2 Humble, Nav2 1.1.18, repo /home/agi/control_backend (non-git).
- Runtime since R0 (27.10 11:07Z): odom = native_imu (single odom->base_footprint owner), AMCL on D1 map v3,
  Trust go2, live adapter, coordinator. Controller RPP, global Smac2D (allow_unknown=false), local costmap
  obstacle+inflation in odom frame, scan /qazbot_lio/scan (front ~92-103 deg FOV only, no rear/side).
- Robot radius 0.454 + padding 0.01. Goal tol 0.20 m / 10 deg, stateful=true.
- Evidence: control_backend/docs/roadmap_v13/{STATUS.md,RVIZ-goal01-result.md,RViz-goal02-result.md},
  evidence/Static_map_trial_1230 (bag, static_trial_summary.json, local_costmap_before_clear.npy),
  evidence/First_RViz_goal_C3_1205 (local costmap before/after clear), evidence/RViz_goal2_1216.
- Known: clear_entirely_local_costmap removed most stale lethal cells near base (254: 246->10).
  Hypotheses: stale marks not cleared by limited-FOV raytrace; RPP collision lookahead/footprint vs costmap;
  odom/plan frame misalignment after reseed; inflation/inscribed handling; costmap transform tolerance.

## Scope
Diagnosis offline (bags, params, nav2 source, saved costmaps) + scoped param/config fix with evidence.

## Non-goals / forbidden
No robot motion, no goals, no MC registration/arm, no restart of live stack without explicit user approval.
Do not shrink body footprint/radius or disable collision checking without evidence that it is wrong.
Avoid creating new DDS participants (ros2 CLI) on domain 0 while stack runs: known to cause T1 stale bursts.
Digital twin out of scope.

## Acceptance criteria
1. Root cause of each collision-ahead abort (Goal01, trial03) identified with evidence (bag timestamps, costmap cells, params, source line).
2. Proposed fix addresses root cause, keeps true-obstacle stopping; quantified effect (e.g. replay/offline check of collision checker on recorded costmaps/paths).
3. Change staged with before/after bytes and tests; live apply only after user approval.
