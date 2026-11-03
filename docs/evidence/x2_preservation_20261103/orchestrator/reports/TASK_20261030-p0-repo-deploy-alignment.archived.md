# TASK 20261030-p0-repo-deploy-alignment
Repo (Git checkout): /home/agi/control_backend/qazbot-x2-stack, branch nav/first-goals-20261028 @ 982f618. Deployed runtime: /home/agi/control_backend (no .git). Running ON physical X2.
Previous task archived: reports/TASK_20261028-scan-latency-rviz-goal.archived.md

## Objective
Read-only audit of source vs live deployment; docs REPOSITORY_LAYOUT / DEPLOYMENT_CONTRACT / MIGRATION_PLAN under docs/architecture/ on a separate branch; make /robot-sync-validate discoverable from sessions opened in /home/agi/control_backend via single Git source (no divergent copies).

## Constraints
- No restarts, deployments, motion, reset, deletion, git init in deployment, checkout move. No changes to active services.
- Minimize DDS observers (new participant previously delayed FAST-LIO); prefer /proc inspection.
- Compare by SHA256, not timestamps. Do not run the skill workflow.

## Acceptance
- Exact Git-vs-live diff (only-live / only-git / differing / identical) with SHA256 evidence.
- What code runs now (process -> cwd/cmdline/overlay -> files).
- Three docs committed + pushed to separate branch, remote SHA verified.
- /robot-sync-validate in slash_commands of fresh session started in /home/agi/control_backend; workflow not run.
- Before/after snapshot of processes/graph shows no impact.
