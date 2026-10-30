# Audit of the old Codex session queue (read-only, 2026-10-30)

Source: `~/.codex/queue_1.sqlite` → `queued_items` (opened `mode=ro`), thread `01a18872-7bd1-7701-b367-b63e344d449d`. Nothing was deleted, edited, consumed or executed.
The Codex session is idle/interrupted (last turn `interrupted`), so none of the 12 items has been consumed.

| order | id (prefix) | created | class | meaning / risk if the session were activated blindly |
|---|---|---|---|---|
| 0 | 01a18b53 | 10:41 | HISTORICAL_STOP | operator stop relayed at the first stop. Harmless, still correct |
| 1–5 | 01a18b92, 93-27cc, 93-f880, 95-5cb4, 9e-7b8a | 11:49–12:03 | HISTORICAL_STOP (duplicates) | five repeated `OPERATOR STOP` from `lab.py stop` runs (tests/real runs). Harmless duplicates |
| 6 | 01a18ba5 | 12:10 | HISTORICAL_REVIEW_REQUEST | review of `5b64245` (Operator Skills). **Stale**: the code moved on (`c3cce90`+); a review now would target an old SHA. Still REVIEW_PENDING |
| 7 | 01a18bb6 | 12:29 | TEST_ARTEFACT | `x --agents=codex,claude ...` sent by a unit test before the real-agent guard existed. **Must not be executed** |
| 8 | 01a18be4 | 13:19 | RETRACTION | retracts item 7 |
| 9–11 | 01a18c1b, 1d, 1f | 14:19–14:23 | TEST_ARTEFACT (STOP) | three `OPERATOR STOP` emitted by a unit test that called `lab.py stop` with a hard-coded `codex` binary (fixed: adapter binary + conftest guard). Harmless but pollutes |

## Consequences
- Nothing in the queue is a legitimate new task. Items 0–5, 9–11 are STOP noise; 6 is stale; 7 is dangerous-by-mistake and neutralised only by 8 (an LLM would have to read 8 before 7 — queue order puts 7 *before* 8).
- **Do not activate the old session blindly**: the first thing it would read are 12 queued messages in order; item 7 would be seen before its retraction.
- The orchestrator no longer depends on this session: Codex engineering work now runs through `codex exec` (own session, own process group, own worktree). The old queue is irrelevant to autonomous operation.
- Operator options (not done by the agent, the queue belongs to Codex): in the TUI, clear the queued messages manually, or start a fresh review thread via `codex exec` on a current SHA.

## Separation from new tasks
New tasks are never sent to this queue in default mode (`NIGHTLY_CODEX_MODE=exec`). `queue` mode remains available but is legacy and should be used only for human-visible notes.
