---
name: robot-sync-validate
description: Synchronize new QazBot laboratory results with the physical X2 and validate eligible fixes on real hardware under explicit safety gates; publish robot feedback for the digital twin.
disable-model-invocation: true
---

# QazBot physical X2: sync, validate, and report

You are running on, or in the software workspace controlling, the **physical X2**. This is an operator-invoked workflow. Do not interpret the existence of a lab commit as authorization to deploy, command motion, restart safety-critical services, or alter the production robot.

## 0a. Two permanent branches (V3) — read this first

- `robot/stable` = what is actually INSTALLED on the X2 (source of truth for the robot). `lab/stable` = what the autonomous lab verified in simulation (SIM_VERIFIED). Never use `origin/main` as the installed state.
- Discover new lab fixes automatically: `git fetch --prune origin`, then `python3 ws/tools/operator_skills/robot_stable.py --repo . list` (reads `docs/hardware/CANDIDATES.json` from `origin/lab/stable`; effective status also reflects hardware results already on `origin/robot/stable`).
- Prepare a candidate (Git only, no hardware): `python3 ws/tools/operator_skills/robot_stable.py --repo . make <candidate-id>` -> branch `robot/candidate/<id>` cut from the CURRENT `robot/stable`, carrying ONLY the listed `ws/src/**` production files plus the plan. Simulator, test stubs and lab tools are refused. A candidate whose dependency is not HW_VERIFIED is refused.
- Hardware validation (deployment, motion, restarts) only with explicit operator approval in this session, following the candidate's `hardware_validation_plan` and `rollback`.
- Record the outcome: `... result <id> HW_VERIFIED --evidence <ref> --installed-manifest <path->sha256 json measured on the robot>` integrates the files into `robot/stable` ONLY if the installed SHA256 equals the candidate; `HW_FAILED` records the failure and evidence without changing robot files. Failed candidates are not installed and stay in lab history.
- New installed-state snapshot (config/map/params/hardware evidence, bag references with sha256 — bags themselves need not be copied): publish via an integration branch, then `... snapshot <branch> --manifest <file>` (SHA256 compared with the previous baseline, forbidden paths refused, CAS update of `robot/stable`).
- Pointers (`docs/hardware/SYNC_POINTERS.json`): last_robot_processed_sha, last_robot_snapshot_sha, last_lab_published_sha, last_lab_reviewed_sha, last_hardware_verified_candidate_sha — move them only after the corresponding step fully succeeded.

## 0. Scope and baseline (read-only)

1. Positively verify that this is the authorized physical X2 workspace and that physical hardware identity, robot mode, and operator procedures match the documentation. If the host is a simulation/lab machine or identity is uncertain, do not issue hardware actions. Read `CLAUDE.md`, `AGENTS.md` and the existing robot operating/safety instructions. Locate the robot's actual Git repository and the documented laboratory integration branch; do not assume a path or branch name. Read `docs/hardware/LAB_TO_ROBOT_HANDOFF.md`, `docs/hardware/TWIN_FIDELITY_REQUESTS.md`, and applicable nightly handoff files **if present**. If absent, record the missing contract instead of inventing one.
2. Record robot mode, whether motors are enabled, who is operating it, active processes, ROS domain, current Git branch/HEAD, `git status --porcelain=v1`, dirty/untracked files, and known rollback procedure. Do not alter active robot processes during this step.
3. Respect existing T1/T5 and all motion authorization, collision prevention, E-stop, and sensor checks. Never disable or bypass them.

## 1. Git synchronization and change selection

1. From the relevant repository, run `git fetch --prune origin` (fetch is non-deploying). Verify remote provenance and signed/known commit policy if configured.
2. Resolve `last_lab_integrated_sha` from a persistent robot-side checkpoint; compare it with the remote laboratory integration SHA recorded in the latest lab handoff. If the checkpoint does not exist, establish the baseline from evidence and report ambiguity. Do **not** silently pick `HEAD`.
3. Inspect **only incoming commits/changes and required dependencies** with `git log --oneline <BASE>..<LAB_SHA>`, `git diff --name-status <BASE> <LAB_SHA>`, and targeted `git diff <BASE> <LAB_SHA> -- <paths>`. First check whether `<BASE>` is an ancestor of `<LAB_SHA>`; if not, handle divergence explicitly using merge-base and separate left/right changes.
4. Do not `git pull`, checkout, merge, reset, clean, or overwrite dirty production files as a side effect of fetching. Use a clean, isolated integration worktree for candidate changes and read-only review of code, parameters, launch files, build dependencies, and migrations.
5. Classify each incoming change: `SIM_ONLY`, `READY_FOR_ROBOT_READONLY`, `NEEDS_MOTION_APPROVAL`, `NEEDS_HARDWARE`, `REJECTED`, or `CONFLICT`. The label `READY_FOR_ROBOT_READONLY` does not grant installation rights.

## 2. Root-cause-oriented test plan

1. Prioritize new safety issues, blocking Nav2/TF/localization/MC bugs, reproducible regressions, then performance tuning. Preserve the distinction between simulated qualification and real X2 qualification.
2. For each candidate, record linked bug ID, hypothesis/root cause, changed components, original failure reproduction, expected behavior, baseline tests, regression tests, and pass/fail criteria.
3. Verify map/odom/base_link TF chain, frame conventions, time (`/clock`, timestamps), odometry resets, lidar freshness, QoS, Nav2 goal checker, motion adapter, SDK/MC and safety constraints where affected.
4. Run allowed **read-only/non-motion** diagnostics first. Ensure reproducibility, isolated dependencies, and a recorded before-state.
5. For any test that moves the X2, enables motors, sends `cmd_vel` / Nav2 goals, changes MCU behavior, restarts live safety-critical components, or deploys new configurations: **present the precise test plan and obtain explicit current-session operator approval before doing it**. State location, clear area, E-stop readiness, proposed bounds based on existing validated robot limits, abort conditions, baseline/rollback, and expected observations. A slash-command invocation alone is not approval for arbitrary physical motion.
6. If approval or prerequisites are missing, mark `NEEDS_APPROVAL` and continue permitted read-only tests and documentation. No automatic live deployment or action on the robot.

## 3. Physical validation

1. Before modifications, record a regression baseline on the current robot system; do not call a test `PASS` without measured evidence.
2. Validate only explicitly approved candidate changes in a controlled rollout and with a rollback path. Change one subsystem at a time unless a coupled fix is technically unavoidable and justified.
3. Compare command vs measured motion, TF pose, XY/yaw errors, stopping/coast, collision clearance, sensor freshness, Trust state, planner/controller/goal statuses, and recoveries. `NavigateToPose SUCCEEDED` alone is not success.
4. Run nominal, edge, recovery, and relevant sensor-fault scenarios within approved bounds. Abort on safety violations or stale/missing critical data, preserving evidence.
5. Compare physical measurements with twin predictions. Any unmatched behavior is `TWIN_MISMATCH` with reproducible data and an explicit fidelity request.
6. Mark each finding `PASS`, `FAIL`, `UNVERIFIED`, `NEEDS_APPROVAL`, or `NEEDS_HARDWARE`. Physical tests not executed must not be described as passing.

## 4. Publish ROBOT -> LAB feedback automatically

1. Update `docs/hardware/ROBOT_TO_LAB_FEEDBACK.md` (create it if needed) and a dated, versioned validation report with: robot Git SHA; lab candidate SHA; tested parameters; relevant ROS 2/SDK versions; conditions; original bug; root cause; test results before/after; regression results; failure traces; safety status; exact reproduction commands; and remaining blockers.
2. Update `docs/hardware/TWIN_FIDELITY_REQUESTS.md` with measured deviations and requests for missing physical measurements. Include ROS topics, bag/trace metadata, source timestamps, map/config SHA, and sensor/MC geometry as relevant.
3. Preserve artifacts with SHA256 and a verifiable accessible location. Avoid committing raw rosbags, credentials, runtime queues, logs, or personally sensitive data. If large artifacts are external to Git, publish a manifest and access instructions; don't claim that `git fetch` downloads external files.
4. Commit only verified, owned files on an appropriate robot feedback branch and push; verify the remote SHA. Do not merge unqualified changes into robot production. If push fails, keep a local checkpoint and report `PUBLISH_PENDING`.
5. Persist `last_lab_integrated_sha` **only for changes actually integrated and validated according to policy**, not for every fetched remote commit. Keep a separate `last_lab_reviewed_sha` to prevent re-reading already triaged changes.
6. End with a concise report: received commits, reviewed changes, read-only checks, approved physical tests, PASS/FAIL/UNVERIFIED, feedback branch + SHA, unresolved safety issues, and next lab priorities.

## 5. Regression checks for the skill itself

Test with clean and dirty worktrees, no new lab commits, divergent branches, missing handoff, unavailable network, inaccessible external evidence, read-only robot, denied motion approval, test abort, and push failure. Make repeated invocation idempotent and never silently overwrite previous results.
