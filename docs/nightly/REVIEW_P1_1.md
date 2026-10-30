# Independent review of P1.1 (2026-10-30)

Reviewer: separate read-only reviewer agent (fresh context). **Codex review: REVIEW_PENDING** (its queue is not consumed; see CODEX_COMPLETION_CONTRACT.md). This is not a Codex review.
Initial verdict: *not safe to ship* — 7 high, 8 medium findings. Status after the fix commit:

| # | Finding | Resolution |
|---|---|---|
| 1 | Valid-JSON-but-wrong-shape result crashed the daemon (crash-loop on restart) | `verify_engineering_result` never raises; per-row try/except in `poll_delegated`; test with `[]`, garbage, symlink |
| 2 | Engineering task COMPLETED on agent exit 0 without result | engineering always needs a verified result file (Claude path too); `NEEDS_ENGINEERING` otherwise |
| 3 | `busy` computed once per ready(): two DELEGATED writers in one tick | lease checked per candidate; test |
| 4 | Dispatch timeout freed the lease / crash between delivery and record lost the intent | write-ahead `DISPATCHING` intent before `codex queue`; timeout = `DISPATCH_UNKNOWN`, lease kept, never re-sent; test |
| 5 | Codex LIMITED without `resetsAt` fell through as available | bounded `recheck_after`; test |
| 6 | COMPLETED committed before `phase=VERIFIED`; self-declared root causes; non-atomic copy | phase + root causes inside the same transaction, VERIFIED decided by orchestrator (verified result + evidence), atomic replace |
| 7 | STOP_ACKED overrode open delegated attempts; `delegated_open` never cleared | DB-derived: any DELEGATED row => STOP_UNVERIFIED; cleared on resume |
| 8 | Supersede released lease while old delivery could still run | auto-supersede removed; manual `nq.py supersede <task> --force-lease-release` only (residual risk documented) |
| 9 | `poll`/`mark`/`verify-evidence` race with daemon | guarded UPDATEs (`state`, `attempt_id`); `poll` takes the orchestrator lease |
| 10 | Path traversal / absolute / symlink in `files[]`; empty `files` skipped binding; self-attested tests | realpath containment, no symlinks, `files` non-empty, orchestrator re-runs allow-listed `tests.cmd` (no shell) |
| 11 | Symlink in result dir copied into evidence | `os.walk(followlinks=False)`, regular files only |
| 12 | DELEGATED with unreadable queue stays silent | alert by age regardless of queue visibility |
| 13 | Unanchored token substring match | `match_turn`: first user message of the turn only |
| 14 | `assert` denylist for dangerous app-server methods | explicit allowlist (`initialize`, `account/rateLimits/read`, `thread/turns/list`), `PermissionError` |
| 15 | Attempt cap never fires on handoffs | `NIGHTLY_MAX_HANDOFFS` (default 4) |

Residual risks (UNVERIFIED): manual supersede cannot stop a late Codex delivery from editing files; "queue row gone = consumed" is inferred; orchestrator re-run executes allow-listed commands with the orchestrator's privileges and blocks the loop during the run; protocol version skew CLI 0.165 / daemon 0.166.
