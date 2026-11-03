# REPORT 20261030-p0-repo-deploy-alignment — DONE (verifier PASS)
- Docs branch docs/repo-deploy-alignment-20261030 @ d3db63a (remote verified): docs/architecture/{REPOSITORY_LAYOUT,DEPLOYMENT_CONTRACT,MIGRATION_PLAN}.md + README line.
- Live = deployment /home/agi/control_backend (manual start*.sh, ws/src direct, runtime/ overlays); nothing depends on checkout path. Tracked ws/src identical to 982f618 at 16:15; map drift from 16:23.
- Skill: ~/.claude/skills/robot-sync-validate symlink -> checkout; discovered from deployment and checkout cwd; hidden from model.
- Not executed: checkout migration. No restarts/deploy/motion by this session.
- Evidence: reports/20261030-p0-{A1,A2,D1,D1b,V1}-*.md, before/after snapshots.
