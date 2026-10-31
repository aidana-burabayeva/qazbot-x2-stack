# Nightly capabilities — 2026-11-01

This supersedes the previous requirement for a full runtime receipt before **any** autonomous engineering. The existing `labctl → lab.py → nq.py → Agent Adapters → Merge Queue` remains the only orchestrator. SQLite owns authorization and task lifecycle; an environment variable does not grant a capability.

```bash
cd /home/adn/QazBot/control_backend_p20_twin02
tools/nightly/labctl.sh doctor --mode ENGINEERING
tools/nightly/labctl.sh start --mode ENGINEERING
tools/nightly/labctl.sh status
tools/nightly/labctl.sh stop
```

`doctor` and `start` default to ENGINEERING. A successful start must verify systemd, a live daemon PID and its held lease. Saved state alone is insufficient. `status --json` through `lab.py` also verifies running state. Preflight failure launches nothing and records `last_start_attempt.json=START_FAILED`; a previous stopped checkpoint is retained.

| Mode | Required proof | Allowed work |
| --- | --- | --- |
| ENGINEERING | own positive Git/role/isolation/auth/resource/basic-test/systemd preflight | saved-data replay, source changes in isolated attempts, tests, audit, independent review |
| L3_QUALIFICATION | source/image/map/parameters/isolation/sensor/TF/frame/resource preflight | one bounded official MC+MuJoCo run on the authorized SHA |
| FULL_NIGHTLY | current complete baseline runtime receipt, hashed evidence and two verified provider tasks | explicitly qualified scopes only |

Engineering does not require Docker or an L3 receipt. Public GitHub refresh failure is diagnostic when the retained baseline is available; it does not discard local integration commits. systemd and actual bubblewrap namespace creation remain mandatory. The sandbox makes the host read-only, hides runtime sockets, isolates process/IPC namespaces and permits writes only to the attempt/results and provider state. Provider HTTP access is retained. This is not a claim of a host-wide network firewall; physical robot access remains prohibited by the task and agent contract. ROS defaults are local-only, domain 87. One shared `engineering` resource serializes these jobs. Existing resource/time/quota/handoff rules remain.

An Engineering PASS is `OFFLINE_VERIFIED`, never `SIM_VERIFIED`. Safety AST invariants are checked again before integration. Producers requiring independent review are held until a different provider verifies their review branch. A production candidate additionally requires matching independently validated simulation archive evidence; offline tests cannot register a hardware candidate. No deployment is performed. Published `HW_FAILED`/`TWIN_MISMATCH` feedback returns through existing robot-sync into Engineering.

## Current campaign and bootstrap scope

`make nightly`, the existing systemd timer and `enqueue_nightly.sh` now enqueue capability-aware jobs. The timer is nonpersistent and is enabled last, after successful preflight/start verification. Historical v2/FOV130 experiments are retained under `campaigns/historical/` and the legacy script; historical runtime entrypoints refuse automatic execution.

The current campaign is `digital_twin/campaigns/d1_p23_amcl_v1.json`. Its source SHA and file SHA256 are fixed in each queued command. D1 map hash, actual robot parameter snapshot, P23, AMCL, sensor topics, experimental **uncalibrated** FOV103 and seed are explicit. Current stack and sim image IDs plus vendor binary hashes come from the successful preflight authorization; a changed digest refuses execution. Requalification is required after an integration SHA change. Engineering may continue on the latest integration SHA without declaring simulation compatibility.

The first L3 scope is deliberately only `stationary_official_mc`, bounded to 30 seconds observation / 600 seconds wall budget with one worker. It verifies the official sensor/TF graph before measurement, records project-specific Docker statistics and logs, and tears down only its own compose project. A single-use task/attempt claim, dispatch token, host flock, container flock and expiring capsule guard normal LAB entrypoints. A crash or exhausted capability does not authorize an automatic retry. No future dynamic-obstacle receipt is required to attempt this first scope.

**This stationary probe is not a D1 route or L3 qualification PASS.** RTF, missed MC cycles and raw contacts remain unknown until instrumented and measured. Missing required metrics produce UNVERIFIED plus an evidence-linked Engineering issue, and the queue returns to Engineering. Full route E2E, dynamic obstacle, doorway, stop/coast and reset scopes remain unqualified. No optimizer, Bayesian search or RL is automatically enabled.

## Continuous audit and defect workflow

The existing timer schedules a dated finite vendor/hardware-snapshot audit plus stable idempotent Engineering tasks. AMCL initialpose is first substantive source task, followed by sensor contract/P23/frame/reset/GPU evidence. Vendor claims retain URL, section, documentation revision, retrieval metadata, installed-interface compatibility and evidence. Fetching a page is not semantic validation. Per-unit measurements stay separate; absent firmware/extrinsics facts become NEEDS_HARDWARE / TWIN_FIDELITY_REQUEST, without blocking other work. Only public vendor HTTPS and the configured GitHub origin are refreshed; no physical query is made.

Campaign structured outcomes feed `workflow.analyze/ingest`. Failure signatures cluster reproducible component/tier/error patterns, preserve every distinct source experiment/config/seed/evidence payload, and create one producer/review pair per cluster. Missing data is not a PASS. Completed issues can recur on a new source SHA; identical evidence is not blindly retried. Existing finite attempt limits and provider limit recovery apply.

## Current acceptance limits

See [evidence](../evidence/nightly_modes_20261101/README.md). The environment denies the user systemd bus and Docker socket, and public GitHub DNS is unavailable to shell Git. Therefore no live daemon, real provider Engineering result, official L3 route, measured stop/resume or full nightly qualification is claimed. The saved queue stays frozen; do not mistake mocked lifecycle tests for runtime acceptance. Run the command above from an operator Linux terminal with working services after reviewing the recorded blockers. No physical X2 was contacted.
