# Engineering stage 1 — 2026-11-01 (simulation/offline only; physical X2 untouched)

## Execution root causes (each: reproduction -> fix -> FAIL->PASS test)
| # | Defect | Evidence | Fix SHA |
|---|---|---|---|
| 1 | `bwrap --tmpfs /run` hid `/run/systemd/resolve`; `/etc/resolv.conf` dangled. `getent` rc=2 in sandbox vs 0 on host; old sandbox + `codex exec` produced no output in 150 s (`old_sandbox_codex_hang.log`), fixed sandbox: real Codex and Claude challenges PASS (`probe_*_receipt.json`). docker.sock and /run/user stay hidden. | test `test_engineering_bwrap_keeps_dns_but_hides_docker_and_user_bus` failed on old code | 1dae63c |
| 2 | Capability fingerprint included mtime of `~/.claude.json` (changes in seconds) -> unstable within 20 s, endless re-probing, tasks parked in PAUSED_AGENT | measured UNSTABLE; test red->green | e0bfe49 |
| 3 | Login shell changed PATH -> fingerprint mismatch (Codex M1 agent, separately evidenced in `nightly_m1_20261101`) | see M1 README | 17d3951 |
| 4 | `claude -p --output-format text` is silent until exit; heartbeat watchdog SIGKILLed a working AMCL attempt at 902 s (empty log, rc -9). Now `stream-json --verbose`; probe parses final result event; LIMIT_RE verified not to match benign events | tests red->green | f5be4bc |
| 5 | Probe-pending candidate was parked until another provider's multi-hour reset | test red->green | 09818e0 |

Operator repairs (attempts lost to infra defects/provider limit, prior attempts preserved, events `OPERATOR_REPAIR`): tasks 39, 41. Codex and Claude usage limits were respected (waited for resets, no bypass).

Regression: full `tests/nightly tests/motion_adapter` at each `labctl stop`: 299 -> 302 -> 303 PASS; `labctl doctor` 21/21 PASS; Merge Queue gates passed for `a6e92ad` and `4ed68de`.

## Real Engineering E2E (Task -> Result -> Verify -> Review -> Merge -> next task)
`engineering_amcl_initialpose_v3` attempt 4 (Codex gpt-6.1-sol, sandboxed): VERIFIED tests rc=0 -> `_review` COMPLETED (Codex) -> additional cross-provider review by Claude (no blocking defect) -> Merge Queue `a6e92ad` -> next task `engineering_sensor_contract_v3` started automatically.
Independent check (this session): new test file on base `0c652f8`: **23 FAIL / 4 pass**; on candidate: **27 PASS**.

## AMCL initialpose — what is and is NOT established
Established (OFFLINE_VERIFIED): both replay entrypoints discarded the recorded operator `initialpose`, synthesised a seed from recorded AMCL map->odom output, and removed source-stamp age; fixed (`tools/twin/replay_amcl.py`, `replay_inputs.py`). The historical strict criterion stays FAIL on 5 saved episodes (future-TF lookup, 8.5-34.3 ms).
**UNVERIFIED:** runtime cause of the 0.1-1.72 m / up to 35.7 deg post-seed jump (13 episodes). Observations only: jump appears at +0.4-0.7 s (first forced no-motion update), scan_score 0.16-0.36, seed header stamp leads receipt by 0.06-0.44 s and drifts. No scan/odom/initialpose full bag with CDR is available locally; needs approved recording + L3 replay. Not SIM_VERIFIED, not HW_VERIFIED.

## Open
Review of AMCL was Codex-authored/Codex-reviewed + this Claude review; sensor_contract/p23/frame/reset/GPU tasks continue under the daemon (ENGINEERING). L3/FULL closed. Known legacy: replay summaries label transform translation as T5; follow-up.
