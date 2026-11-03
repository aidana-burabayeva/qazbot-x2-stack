# Automatic per-fix L3 continuation of x2-blockers-20261103

This extends the existing nq tick, SQLite queue, qualification module, execution capability and merge queue. It does not start a second scheduler. ENGINEERING retains Codex-only routing; historic tasks remain excluded. Operator authorization is offline LAB only, never physical X2.

Enable on the existing campaign (this is consent, not an L3 capability):

```
bash tools/nightly/labctl.sh enable-auto-l3
bash tools/nightly/labctl.sh l3-status
```

Per-fix stages: ENGINEERING_VERIFIED → original FAIL/fixed proof → FIX_VERIFIED → INTEGRATED → L3_PREFLIGHT → L3_READY → L3_RUNNING → L3_PASS/L3_FAIL/L3_BLOCKED. Preserve operator STOP, all evidence and result history. The root-cause task being COMPLETED is insufficient. Restoration itself is not a perception/safety fix or an L3 PASS.

Readiness requires COMPLETED+VERIFIED producer, stored PASS result, verified result SHA, a matching INTEGRATED merge-queue record and ancestry of the current lab/stable. The specific fix must be inside that verified result. VERIFIED root cause and the source-fix diagnosis are mandatory. The merge queue safety checks are rerun against the baseline and integrated source. No default generic AMCL/doorway/fidelity work is added.

Each accepted engineering_result.json may contain `l3_fixes`. Separate entries/commits are mandatory for D04 and D07; a BLOCKED D04 question does not prevent an independent D07 fix. Example schema (replace placeholders with actual measured/committed values):

```json
{
  "l3_fixes": [{
    "blocker": "d05",
    "fix_sha": "<committed fix SHA inside the verified producer result>",
    "baseline_sha": "<pre-fix ancestor SHA>",
    "root_cause": "<key of the producer's VERIFIED root cause>",
    "tests": ["tests/d05/test_actual_fix.py"],
    "test_files": ["tests/d05/test_actual_fix.py"],
    "runtime_tests": ["tests/d05/test_actual_runtime.py"],
    "inputs": [{"path":"<available recording or fixture>","sha256":"<actual SHA256>"}],
    "safety_invariants": true,
    "dependencies_available": true,
    "nav2_version": "<measured image Nav2 version>",
    "requires_mc": false
  }]
}
```

A missing recipe, unavailable dependency/input or unproven root cause is L3_BLOCKED, not readiness. Use `blocked` for a component whose necessary evidence is absent. Recipes are not executable shell commands: only explicitly listed repository pytest files are accepted. Test files are overlaid on a new baseline worktree; production files are never overlaid. The orchestrator independently runs the identical cases before/after: before must exit 1 with real assertion failures (not import/collection errors), after must pass with zero skipped/error/failing tests. JUnit case identities must match. Inputs are SHA256 checked; their size/mtime/inode and expected hashes participate in the conditions seal.

Only when ENGINEERING/VERIFYING/delegation and merge leases are absent may the finite proof/preflight child run. Existing twin containers or host MC/simulator processes block admission and are retained. The 5 GiB disk floor remains unchanged. A SQL unique index allows one active qualification transition. No engineer is signalled to make room for L3.

Preflight is the existing `preflight.py all --mode L3_QUALIFICATION`, including all `first_l3_errors()` prerequisites. Its own real report, current integration SHA, actual Nav2 version, Docker image digests, map and parameter fingerprints must match. No gate is converted from FAIL/UNVERIFIED to PASS. In particular, existing strict captured-baseline/source checks may block a source-changing fix; this mechanism reports that exact prerequisite failure rather than silently allowing divergence. Official MC dynamics require the existing independently validated runtime qualification receipt as a separate prerequisite.

Every successful preflight calls ordinary `execution.authorize(..., L3_QUALIFICATION, ...)` afresh. It never increments l3_remaining. nq consumes the one-use capability on dispatch and execution.require() verifies the task, attempt, token, SHA and claim. After the result, ENGINEERING is available again; the next L3 run requires another complete positive preflight/authorization.

The runtime pytest is executed in the existing hardened stack-lite service, on an internal Docker network/domain 87, with pinned source read-only and evidence writable. No host network/Docker socket is bound. The passive witness independently records observed ROS nodes, publisher GIDs and actual PointCloud2/Twist/LaserScan traffic. Only helper-owned projects/worktrees are cleaned; foreign runtime is never stopped.

Runtime tests write `$QAZBOT_L3_EVIDENCE/runtime_contract.json`, with:

- `fidelity: REAL_ROS2_NAV2`;
- `source_sha` from QAZBOT_L3_SOURCE_SHA and the measured `nav2_version`;
- integer `safety_violations: 0` only if actually observed;
- `cases`: booleans derived from real runtime assertions, never unconditional canned PASS values.

Required D05 cases: normal_navigation, obstacle, missing_cloud, stale_cloud, tf_failure, stop, held_stop, sensor_recovery, cancel, untrusted_no_motion. Actual controller_server, collision_monitor, goal_coordinator, motion_adapter and cloud/command traffic must be witnessed. Use the whole available chain; simulated braking is not measured X2 braking.

Required D04 cases: real_cloud, tf_odom_pairing, qos, publication_loss, python_parity, marking, clearing, data_recovery. A real recorded compact interval is sufficient; absent original publication-loss evidence is not invented. Projector, Nav2 costmap, clouds and LaserScan must be witnessed.

Required D07 cases: low_obstacle, floor_separation, marking, clearing, false_obstacles, missing_cloud, sensor_recovery. Use a real Nav2 costmap; synthetic obstacles qualify algorithm behavior only. Hardware boxes 21/45 cm remain NEEDS_HARDWARE.

Result acceptance requires real runtime traffic/graph evidence, complete required case coverage and passing, non-skipped JUnit cases. Missing runtime dependencies/reports become L3_BLOCKED. Confirmed runtime assertions failing become L3_FAIL. Hardware acceptance is never inferred from L3.

SQLite rows are keyed by blocker/fix/source/test conditions. Repeated ticks/restarts do not create duplicate tasks; each runtime task has max_attempts=1. A helper with a live matching PID is observed, not relaunched. Missing durable outcomes become BLOCKED. An orphan runtime lease is released only after its process is gone and its own container project is proven absent; otherwise it is retained. Source moves invalidate pending/finished qualification. Changed inputs/images/parameters/Nav2/disk admission/MC receipt permit a distinct assessment, never an unconditional retry loop.

Live deployment: stage and gate changes in a separate worktree. Admit no new engineering task during the update; let existing Codex, verification and merge finish. Only after all task and merge leases are absent, use the operator-authorized one-time daemon restart. Keep the same campaign and SQLite file, then resume dispatch. No process is stopped while it holds an ENGINEERING/VERIFYING/L3 lease.
