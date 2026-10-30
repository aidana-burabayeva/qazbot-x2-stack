---
name: qazbot-nightly
description: Operate the QazBot simulation nightly lab (queue, L1/L2 runs, handoff to Codex). Use when asked about nightly status, enqueueing sims, or resuming after an agent limit. Simulation only.
---

State lives in SQLite, not in the chat: `python3 tools/nightly/nq.py status`, `run/nightly/checkpoint.json`, `docs/nightly/{STATUS,HANDOFF,DECISIONS}.md`.
- Orchestrator: `systemctl --user status qazbot-nightly` (restart: `systemctl --user restart qazbot-nightly`). One owner (flock).
- Enqueue sim work only via `nq.py enqueue NAME CMD --resource sim` (one sim at a time). Task names are unique; never overwrite history.
- L1 model: `tools/nightly/run_l1.sh SEED`. L2 real chain: `tools/nightly/run_l2.sh ID TSV`. Report: `python3 tools/nightly/report.py`.
- Engineering failures: read `docs/evidence/...`, one hypothesis, one experiment. Hand off to Codex with `codex queue --thread 01a18872-7bd1-7701-b367-b63e344d449d --message ...` in a separate worktree. Never bypass a provider limit.
- Forbidden: physical X2 (ssh/deploy/ros2 pub), weakening safety/tolerances for PASS, reset/clean/stash of foreign changes.
