# Navigation-first execution policy — 2026-11-01

The existing nq/workflow/router/adapters/SQLite/Merge Queue remain the only orchestrator. `lead.py` contains classification, durable capability evidence and ordering policy; it has no worker loop.

## Causal findings

1. Historical Codex attempts all report `workspace routing discovery failed`, including a terminal `turn.failed`. This proves an execution transport failure, not AMCL/P23/TF bugs. It does not prove which proxy/DNS/server component caused routing discovery to fail. The app-server availability/quota checks did not qualify `codex exec`.
2. Historical Claude command placed a prompt beginning `--agents=claude,codex` in a CLI argument position. The real log says `--agents file not found`. Options now precede an explicit `--` prompt delimiter. This adapter defect is independent of usage limit and Codex transport failure.
3. `harvest()` used to replace failure causes with missing result. Infrastructure errors now retain their category/evidence in BLOCKED_EXECUTION. A missing result with no infrastructure evidence remains MISSING_RESULT; a unit/regression failure remains a separate REGRESSION_FAILURE, not provider unavailability.
4. A two-connection WAL reproduction demonstrates that `ready()` yielded while retaining its live SELECT cursor. After another connection commits, the first connection's write fails with `database is locked`. `fetchall()` now ends that read before yielding/dispatch. Busy timeout is unchanged. No historical daemon traceback was available to prove that this is the *only* cause of the reported historical SQLite error. Long/nested transaction sites were inspected; result-store BEGIN/COMMIT remains atomic.
5. Timer and daemon previously shared `checkpoint.json.tmp`; they now write unique temporary files before fsync/atomic replace. This removes a separate identified concurrent-file race. It is not asserted as the cause of SQLite locking.

## Availability and recovery

Availability has separate axes: installed/authenticated, quota/reset, model catalog, and **actual execution**. Managed LAB tasks require a matching provider/model execution capability. Quota/app-server facts cannot overwrite a failed execution capability. Fingerprints use adapter/isolation/binary/config metadata and hashed environment facts; secrets are not written into planning records. No provider calls run on every tick.

For an unknown or changed execution configuration, a finite existing-queue diagnostic can be enqueued once per fingerprint. The trusted mechanical helper creates its own isolated worktree, then invokes the same adapter through `execution.engineering_sandbox`. It is not nested inside another read-only sandbox. Known usage limits are checked first. No new retry loop, daemon, watchdog or busy-timeout increase is added. Existing nq wall/heartbeat limits apply to queued diagnostics. Standalone probes run to the CLI's own terminal outcome.

After a causal environment/configuration fix, explicitly requalify the affected capability:

```bash
python3 tools/nightly/workflow.py capability-probe --provider codex --model gpt-6.1-sol
# For Claude, use the model actually routed/pinned by the task:
python3 tools/nightly/workflow.py capability-probe --provider claude --model claude-opus-5-5
```

A challenge is accepted only after exit 0 and actual assistant output (Codex also needs a completed turn), never because the prompt was echoed. Failed probes preserve logs/receipts and do not qualify tasks. A successful probe returns eligible BLOCKED_EXECUTION/PAUSED_AGENT tasks to PENDING, preserves their prior attempts/evidence and clears the exhausted provider-selection list. Subsequent engineering results still undergo identity fencing, file hashes, source ancestry, independent test rerun and review; a challenge response is not an engineering result.

Infrastructure incidents deduplicate by provider/category/decisive signal, and link every affected task/attempt. Original tasks are retained. One confirmed provider outage cannot generate separate navigation bugfixes. Deterministic jobs remain eligible under Engineering; neither provider outage nor absent L3 receipt enables Docker/MC. A domain assertion failure does not revoke a provider capability.

## Transparent navigation plan

Hard dependency, frozen-state, tier/source, resource, availability and independent-review rules remain mandatory. Eligibility is rechecked after provider selection so a concurrent stop cannot be ignored.

Lexicographic ordering, without a sum of weights:

1. P0 infrastructure/sensors/localization/AMCL/Trust/TF/reset/safety; P1 motion/controller/costmap/planning/L3/D1; P2 robustness; P3 GPU/RTF/optimization.
2. Resource/evidence prerequisites inherit the most critical dependent's band.
3. AMCL initialpose is the first substantive diagnostic task after execution recovery.
4. More blocked direct dependants, then confirmed evidence, then original numerical priority and enqueue id break ties.

`planning_decisions` records base/effective band and explanation. A facts hash includes task/dependency state, execution health and root-cause evidence; unchanged facts produce no new plan rows. Risk is enforced through existing router/model choice, protected safety contracts and review rather than being assigned an opaque numerical reward. Missing hardware facts remain NEEDS_HARDWARE; the planner does not invent evidence or runnable prerequisites.

For source fixes the result must include hypotheses, reproduction, causal evidence, original-FAIL retest, regression and VERIFIED root cause. These declarations are necessary, not self-proving: existing independent verification/review and Merge Queue still apply. `COMPLETED`, `OFFLINE_VERIFIED`, `SIM_VERIFIED` and `HW_VERIFIED` remain distinct. No AMCL/P23/costmap bundle or production safety relaxation is introduced.

## Current runtime limits

See [current evidence](../evidence/engineering_lead_20261101/README.md). Both real probes failed in this session: Codex local-client initialization cannot write its state; Claude reports API/DNS EAI_AGAIN. This uses the same sandbox helper and adapters but inherits additional outer-session restrictions, so it is not a host-systemd environment equivalence claim. Historical transport failure is separately retained.

The systemd user bus is denied here. The saved RUNNING_ENGINEERING value is not proof of a currently live daemon: the observed orchestrator lease was free. No real verified engineering result or running queue restart is claimed. Do not bypass preflight or run a second daemon to manufacture acceptance.

When services and provider execution are repaired in the operator terminal:

```bash
tools/nightly/labctl.sh start --mode ENGINEERING
tools/nightly/labctl.sh status
```

Full/L3 gates remain unchanged. Physical X2 is forbidden. No deployment, robot SSH or ROS command was performed.

Official OpenAI documentation consulted for the noninteractive CLI boundary: https://learn.chatgpt.com/docs/non-interactive-mode . Execution health is established by observed commands, not documentation assertions.
