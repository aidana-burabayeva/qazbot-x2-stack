# Autonomous Nav2 optimization: qualification gates

## IMPLEMENTED

P0 image parity uses `digital_twin/docker/nav_control_patch.py` in both Docker build and preflight. It requires exactly one specific experimental-turn block and one I1 literal. The other ROS_LOCALHOST_ONLY guard is preserved. Preflight verifies the SHA256 of the complete resulting image file, including whitespace; any extra change fails. Production nav_control.launch.py is untouched.

`qazbot_tf_probe` is a small ament/rclcpp package built into the stack image. Its Humble subscription callback uses `rclcpp::MessageInfo::get_rmw_message_info().publisher_gid`. Per-message GIDs are joined to repeatedly discovered endpoint names. Two GIDs are competing publishers even when their node names are equal. Missing data, unresolved/zero GIDs, fewer than three messages, unexpected names or multiple endpoints fail. Duplicate occurrences of an edge in a single message count as one sample. Native code observes for 28 seconds. It has no robot transport or deployment path, and refuses before ROS init unless QAZBOT_SIM=1, TWIN_SANDBOX=1 and ROS_DOMAIN_ID=87.

Before inspecting the actual graph, doctor runs four native DDS fixtures on a private topic: one publisher per edge (must pass), duplicate map→odom publishers (must fail), two endpoints with the same ROS node name (must fail), and no data (must fail). The normal check requires AMCL for map→odom and /kinematic_plant for stack-lite odometry. For a separate official-MC qualification run the probe with TWIN_ODOM_OWNER=/hal_plant; only these two explicit model owners are accepted. Diagnostic fixtures never publish to the navigation TF topics.

The append-only `tools/nightly/optimization_archive.py` library uses the existing nq SQLite DB and adds one experiment table. `record <json-file>` ingests immutable content-addressed observations, including rejected/incomplete failures; `view --out <path>` exports the full archive. It launches no workers or optimizers. Each cohort retains baseline(s), safest, most_accurate, fastest_among_safe, full nondominated set and all rejected observations. Safety gates precede objective comparison. Ties use remaining objectives before deterministic record ID.

Comparison cohorts include fidelity, source SHA, map/world SHA256, stack/simulation image digests, ROS/Nav2/MC versions, scenario set, seeds, fixed parameters, safety contract and baseline identity. An eligible observation requires all explicit failure flags false, complete finite measured metrics, matching STORED/PASS in the existing result store, independent validation PASS and matching local evidence hashes. L3 additionally requires measured CPU, RAM, disk/evidence volume, RTF, missed MC cycles, wall duration, one worker and timing-isolation PASS. Unknown versions, missing gates, damaged evidence and resource degradation are retained but rejected. These are input/provenance checks, not proof of physical fidelity. A supplied qualification label alone is insufficient. Automatic optimization remains disabled.

## VERIFIED

The native executable compiles against the installed Humble headers/libraries with -Wall -Wextra -Werror. Offline native-core tests reject competing GIDs, including the same node name, missing endpoints, no samples and insufficient samples. Python regressions cover the two-ROS_LOCALHOST_ONLY transformation, exact owner policy, partitioned Pareto selection, unsafe fast candidates, all-result retention and tampered evidence. Detailed results are in [evidence](../evidence/navopt_p0_20261031/README.md).

## UNVERIFIED

The new executable's DDS fixtures and the full live owner observation have not run in this restricted environment. Docker socket and systemd user bus are denied. Doctor therefore FAILS before starting the stack; neither P0 runtime closure nor a new MC/MuJoCo qualification is claimed. Stack-lite is not official MC evidence. Nav2 1.1.20 versus robot 1.1.18 compatibility remains UNVERIFIED. No worker resource profile or new navigation result is available.

## OPEN: required next sequence

1. From the normal lab Linux account, while the lab is stopped, run `tools/nightly/labctl.sh doctor`. Require the exact image hash check, all four native DDS fixtures and actual TF owners to pass. Preserve every failed preflight report. Do not bypass a fixture failure by using node listings or grep.
2. Qualify one official MC + MuJoCo worker in the internal Docker network on domain 87, using this checkout's current integration source, D1 map and October 30 installed parameters/P23. Rebuild both sim and stack as needed; save image digests, MC binary/version, map/world hashes and frame-anchor registration. The old `run_l2.sh` / `run_nav2_trace.sh` truth-localization/P20 presets are historical experiments, not a qualified current L3 runner.
3. Measure that worker's CPU, peak RAM, disk delta, wall duration/evidence bytes, simulation RTF and missed MC cycles from actual simulator/MC instrumentation. Unknown or missing timing counters prevent qualification. Preserve partial evidence on interruption or low disk. With about 7 GB free, use exactly one L3 task with the existing `resource=sim` lease; do not increase heavy concurrency automatically. L1 thread limits/CPU scheduling are separate. There is no measured basis yet for choosing multiple L3 workers.
4. Run the baseline matrix below with fixed seeds/configuration and record failures separately. Compare MC yaw-rate, stop/coast and timing against saved physical evidence; the historical P20 torso surrogate and hardware calibration records have limitations and must not be relabelled as new measurements. Door passage requires verified geometric clearance in the simulated world plus raw contact evidence; map clearance alone is insufficient.
5. Only after an independently validated L3 baseline: implement a deterministic controller-only search above existing runners, baseline → parallel L1 screening → full Nav2 shortlist → serial L3 → independent validation → cohort Pareto update → engineering feedback. Hold localization, P23, costmap and all T1/T5/footprint/goal/collision thresholds fixed. The new archive is available, but this search/promotion scheduler is not implemented or enabled by this change.
6. Cluster recurring real failure signatures across affected configurations and create idempotent engineering tasks through nq/Agent Adapters. Require reproduction, root cause, minimal fix, controlled A/B, regressions and existing Merge Queue. Reevaluate changed-source cohorts; do not tune around a systemic defect. Existing nq signature/handoff infrastructure is reused; cross-configuration optimization feedback remains OPEN.
7. Bayesian optimization requires proven comparability and enough measured observations. RL remains a separate future experiment requiring an incremental task/benefit over deterministic/Bayesian/coverage-guided search.

| Baseline/regression case | Required fidelity/evidence | Current state |
|---|---|---|
| Free motion and varied speeds | real Nav2 + MC + MuJoCo, raw dynamics/contacts | OPEN |
| CW / CCW | same config/seeds, raw HAL/world yaw and physical-data comparison | OPEN |
| Office corridor and door approach | frame alignment, geometry and contacts | OPEN |
| Door passage with sufficient clearance | actual world clearance + footprint/contact measurements | OPEN |
| Stop and coast | raw commands, measured motion/timing | OPEN |
| Recovery / odometry reset | explicit evidence, continuity, no safety relaxation | OPEN |
| AMCL initialpose / localization errors | separate ROS replay and full AMCL E2E, no truth substitute | OPEN |
| Sensor delays/dropouts, collision | fail-closed T1/T5/collision regressions | OPEN |
| Agent interruption / stop during sim / repeated start | owned processes, partial evidence and no duplicate tasks | infrastructure tests exist; new live L3 acceptance OPEN |
| Pareto preservation | immutable observations and cohort/safety gates | offline tests VERIFIED; real-run population OPEN |

Physical X2 access, SSH, deployment and movement are outside this task. No new orchestrator is introduced. The production queue remains frozen; no optimization or L3 campaign has been started by this continuation.
