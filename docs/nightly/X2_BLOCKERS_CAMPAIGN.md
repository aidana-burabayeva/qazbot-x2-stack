# Targeted X2 engineering campaign

Use the existing control checkout, nq daemon, model router, isolated process attempts and regression/CAS merge queue. No second scheduler or new service.

```
cd /home/adn/QazBot/control_backend_p20_twin02
bash tools/nightly/labctl.sh plan --mode ENGINEERING --campaign x2-blockers-20261103
bash tools/nightly/labctl.sh start --mode ENGINEERING --campaign x2-blockers-20261103
```

`plan` is read-only: it copies the SQLite database to RAM and performs enqueue planning there. It neither authorizes execution nor probes an agent. `start` requires a fresh genuine Engineering preflight, retains all task/result history, transactionally selects the scope, and calls the existing resume/systemd lifecycle. Its temporary admission lock never truncates or breaks an existing orchestrator lock. Any RUNNING/DELEGATED/VERIFYING row, unconfirmed delegation or pending merge prevents campaign activation. A scope change is refused if a live daemon owns its lease. No lease is force-released.

Only these task IDs can be enqueued/dispatched in the initial phase:

1. `x2_p0_preservation_restore` — verify selected restoration from `/home/adn/QazBot/x2_preservation_20261103/`, bundle, tar permissions/symlinks, dependency/backend/map/build checks, independent raw-bag copies; return RESTORE_VERIFIED or evidence-backed RESTORE_BLOCKED.
2. `x2_p0_d05_collision_monitor` — stock Nav2/QazBot freshness/held STOP/recovery/cancel contracts, stopping distance with MEASURED/ESTIMATED/UNKNOWN terms, actual Nav2 version, original FAIL and before/after regressions. No safety-threshold relaxation or unmeasured physical PASS.
3. `x2_p1_d04_cpp_d07` — establish missed-publication cause only from adequate records, test pairing/timestamps/QoS/DDS/TF, retain Python-C parity; separately evaluate low-obstacle/floor/dropout/marking/clearing using stock layers. D04 and D07 require separate commits. The 183015 recording has no claimed boxes.

Full requirements are stored in workflow.TARGETS and each task's repro, beginning with `--agents=codex`. NIGHTLY_CODEX_MODE remains exec. Router model/effort selection and genuine capability receipts are retained. A missing capability receipt is qualified synchronously and finitely (120 seconds) in the existing probe mechanism, before the one engineering process; it does not create another research task. No Claude version/auth/probe/task is called in this phase.

Eligibility is enforced in nq.ready AND immediately at run_mechanical/dispatch_codex entry. Navigation-band ordering is bypassed for this scope. Exactly one task may run or verify at once; merge completion is a barrier before the next one. A failed/blocked terminal task does not indefinitely block the next independent task. PAUSED_LIMIT/PAUSED_AGENT retain their position until provider availability is resolved. No historic task becomes eligible when the three finish.

General Engineering enqueue, daily timer expansion, failure-generated generic research/reviews and resume robot/AMCL/P23/doorway/fidelity triage are disabled for this scope. Old PAUSED_LIMIT/PAUSED_AGENT/parked rows are not revived or restored. Scoped restart refuses active leases rather than killing/recovering them. Scoped STOP parks only scoped tasks; it preserves legacy rows/leases. Unknown campaign scope fails closed.

A Codex limit pauses until a provider reset (or the existing unknown-reset recheck policy), keeps the attempt worktree/checkpoint/evidence, and never falls back to Claude. There is no automatic 06:00 assumption. After independently confirmed Claude availability, a maintainer may explicitly authorize a review phase tied to exact Codex result SHAs and original FAIL/regressions; this initial strict allowlist intentionally grants no Claude review work automatically.

Engineering remains OFFLINE_VERIFIED, never SIM_VERIFIED or hardware-certified. No robot access, deployment, ROS goals or MC control. Missing SDK/bags/sandbox-visible archives block the exact dependent checks only. Do not weaken sandbox to access private backups.

Existing operator STOP remains:

```
bash tools/nightly/labctl.sh stop
```

Evidence: `docs/evidence/x2_target_campaign_20261103/`. Dry-run on the real 79-task/11-result snapshot excludes all 79 historic tasks and plans exactly 3 new rows in RAM. Actual task and result rows remained byte-for-byte identical. Focused tests use only fake providers/temp SQLite and never start real agents. Final merge-gate/start evidence is recorded in the external session directory `/home/adn/QazBot/x2_target_campaign_20261103/`.
