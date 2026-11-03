# TASK 20261103-reuse-first — targeted reuse-first engineering (research only)
Source: user message 2026-11-03 (Russian). Priorities: 1 AIMDK C++ & D-05 → 2 Collision Monitor → 3 D-07 → 4 C++ LiDAR → 5 MPPI. D-07 mandatory before next autonomous runs.

## Inputs
- Nova-Valley/agibot-x2-navigation == AgiBot-Community/agibot-x2-navigation (tree 9ad1ad8af1b4) — one implementation. Root MIT; src/examples/package.xml license empty → check provenance separately.
- byggmesterPRO/agibot-x2-slam — browser JS sim, NO license: ideas only (camera hazard layers), no code copy, no sim detection probabilities.
- Local AIMDK: /home/agi/aimdk/src/{examples/src/mc, aimdk_msgs, ruckig}.
- Prior evidence: .orchestrator/reports/20261102-d05-D1-debugger.md, 20261103-d05-*.md, 20261103-controller-compare.md (36 runs baseline), 20261103-pd04-analysis.md, 20261103-pd04-cpp.md, commit 5980d1f (branch robot/feedback-d04-d05-20261102, worktree /tmp/claude-1001/-home-agi/875283d9-732e-4c1e-ba4d-0d8188cfcd4d/scratchpad/wt_feedback).

## Workstreams
W1 AIMDK mc examples (mc_locomotion_velocity, set/get_mc_input_source): is 0.2 m/s min / MC start thresholds demo-only or real MC X2 behaviour; compare D-05; AIMDK types for future C++ Motion Adapter; keep P20/P23, Trust, GC, MC arbitration, operator takeover.
W2 nav2_velocity_smoother (OPEN_LOOP) vs Ruckig: sufficiency for X2 command profile; commanded vs actual velocity limits; no coefficient change before MC calibration.
W3 nav2_collision_monitor Humble / installed Nav2 1.1.18: presence + API; isolated offline test with real X2 PointCloud2; Stop/Slowdown/Approach; independence from stale costmap marks; freshness, TF, QoS, latency, missing msgs, FOV, D-05 effects; must not break cmd authorization chain.
W4 D-07: low objects from raw E1R cloud without cameras; FOV, height filter, floor geometry; continue C++ qazbot_lidar_projector check. Cameras stay off.
W5 MPPI: official Nav2 MPPI; 36-run baseline kept; no reruns w/o new question; no X2 install.
W6 D-05 safety: verifiable criteria for allowed speed, coast and stop distance before calibration; BLOCKED if capability missing.
W7 Licenses/portability per solution: file+SHA, license (repo+component), Humble compat, QazBot analogue, functions to keep, regression checks.

## Non-goals / constraints
No production X2 changes, no installs on X2, no goals, no motion, no Nightly, no general audit, no long reruns. Cameras off. No new components where Nav2 solves it. Nav2 only from official Nav2. Don't mix branches. Isolated ROS_DOMAIN_ID for any offline test. Henki standard mandatory.

## Deliverable
Table | Ready solution | Source & license | Replaces | Benefit | Risks | Decision | + immediate use / needs adaptation / reject / minimal next changes & regression checks. Show decisions+evidence first.
