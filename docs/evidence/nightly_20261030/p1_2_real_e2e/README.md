# P1.2 real-agent E2E: Claude -> Codex -> Claude (2026-10-30)

Run by `tools/nightly/e2e_real.py --i-understand-this-spends-quota` against orchestrator code at commit `1f6a88c` (final P1.2 code: router, usage-aware, stdin isolation). The first run (commit before router, `run1_before_router/`) also passed in 94 s.
Real binaries: `claude -p` (claude-haiku-5-5, effort low) and `codex exec` (gpt-6.1-sol, effort low; codex-cli 0.165). Throw-away repo in /tmp, no robot, no production, no Codex TUI.

| step | agent | result |
|---|---|---|
| s1_fix_add | claude | COMPLETED / VERIFIED, orchestrator re-ran pytest rc=0, branch attempt/s1_fix_add-1-82ce98 |
| s2_fix_mul | codex (exec) | COMPLETED / VERIFIED, based on s1's verified SHA, branch attempt/s2_fix_mul-1-d70ca4 |
| s3_integrate | claude | COMPLETED / VERIFIED, based on s2's verified SHA, branch attempt/s3_integrate-1-525a8c |

- Total 94 s. Each attempt in its own worktree; commits made by the orchestrator (author qazbot-nightly), not by the agents; hashes computed by the orchestrator.
- Independent check (clean clone of the final branch, own pytest, outside the orchestrator): 4 passed, tip 1ab446e.
- Nothing merged anywhere; branches are INTEGRATION_READY only.
- Files: e2e_summary.json (tasks, results, events, independent check), agent_logs/ (raw agent output), MANIFEST.sha256.json.
- Limits of this evidence: the handoffs here are *planned sequential* handoffs through Git + SQLite, not triggered by a real usage limit (never exhausted on purpose). Quota consumption was small and not measured for Claude; Codex ~ a few thousand tokens per step.
- Observation: `codex exec` printed "Reading additional input from stdin..." because the child inherited stdin; fixed afterwards (stdin=DEVNULL).
