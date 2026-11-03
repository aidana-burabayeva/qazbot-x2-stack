# TASK

## Task ID
20261021-x2-nav2-e2e-dryrun

## Objective
Full Nav2 NavigateToPose on D1 first_map for AGIBOT X2 works end-to-end in dry-run
(RViz goal → BT Navigator → Planner → Controller(RPP) → Velocity Smoother → private
/qazbot_d1_map/cmd_vel → X2 Motion Adapter dry_run → kinematic model), with safe
refusal/termination; no physical motion, no real MC.

## Robot context
- Hardware: AGIBOT X2 humanoid, Jetson (Linux 5.15 tegra), host .66; reference code on .89.
- Sensors: X2 3D lidar → FAST-LIO odom; virtual LaserScan → AMCL on D1 first_map.
- ROS 2: humble, nav2 1.1.18 (velocity_smoother may stop publishing after stop).
- Repo: /home/agi/control_backend (NOT git; snapshot before edits).
  ws/src/{lio,maps,navigation}, tests/{motion_adapter,loc_trust,nav_e2e,...}, candidates/, docs/.
- Existing state: docs/INTEGRATION_STATUS.md, docs/CURRENT_CONSTRAINTS.md.
  Known open defect: lifecycle manager shutdown hang/SIGABRT (candidates/nav2_lifecycle_shutdown not accepted).

## Scope
1. Motion Adapter: tests for localization_untrusted, foreign_velocity_publisher, vendor_autonomy_active;
   idle/active/latched/dry_run; PS5 takeover cancels active Goal A at action level; resume never revives A; only new Goal B moves.
2. Single-owner-of-motion analysis (DDS discovery, flock, MC arbitration, GetCurrentInputSource, state machine); document limits; no arbitrary watchdogs/timeouts.
3. Localization Trust: anchor + FAST-LIO drift, experimental 2 m ambiguity limit, loss of scan/odom/AMCL, frozen TF, stale stamps, pose jumps, wrong initial pose, map mismatch; no resume of old goal after recovery; unconfirmed thresholds marked experimental.
4. Full NavigateToPose via standard Nav2 actions + separate BT; keep plan_to_pose.xml as planning test; RPP, Velocity Smoother, Progress/Goal Checker. No second navigator/planner/controller; no nav2_controller rewrite.
5. Recovery: no BackUp/DriveOnHeading backwards; define safe termination.
6. E2E dry-run tests: straight 1.5 m, rotate in place, already-at-goal, obstacle, unreachable, cancel, PS5 takeover, loss scan/odom/AMCL, false localization, recovery, Nav2 restart, repeat goal; check kinematic model pose.
7. Fix stop test: no "last 3 zero msgs"; check cancel, last cmd + time, no later nonzero cmds, model stopped, no re-motion after recovery.
8. Live dry-run bring-up on X2 (FAST-LIO, AMCL, map, planner, controller, BT, smoother, trust, adapter live=false, no MC publisher, no source registration); compare ROS graph/TF before/after; clean shutdown.
9. Docs: X2_LOCALIZATION_TRUST.md, X2_NAV2_CONTROLLER.md (exist—complete), update README, DECISIONS, X2_MOTION_ADAPTER, X2_FOOTPRINT, X2_FIRST_PHYSICAL_TEST_PLAN. sdk_qazbot_nav2=30 is config only.
10. Prepare (not run) F0 localization and F1 dynamic footprint trials. Do not substitute static 0.387–0.454 m.

## Non-goals / forbidden
Autonomous walking, MC source registration, publishing real velocities to MC, global /cmd_vel or
/aima/mc/locomotion/velocity, vendor service changes, FAST-LIO rewrite, disabling safety checks,
Docker migration, big refactors, verbatim copy of Teleworker/Nova-Valley code. 3D NDT localization: design note only unless trivial.
If real motion would be needed → STOP and ask user.

## Acceptance criteria
- [ ] Full NavigateToPose works in dry-run incl. model pose checks
- [ ] New adapter refusals covered by tests
- [ ] Localization failures block navigation
- [ ] PS5 takeover & recovery do not resume old goal
- [ ] Previous tests do not regress (before/after counts)
- [ ] Docs match code
- [ ] No physical motion; no MC publisher/registration in live dry-run

## Current status
IN_PROGRESS
