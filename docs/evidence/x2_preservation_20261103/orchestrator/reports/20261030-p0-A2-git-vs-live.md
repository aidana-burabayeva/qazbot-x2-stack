# A2: Git checkout vs live deployment (read-only SHA256 comparison)
Task: 20261030-p0-repo-deploy-alignment / A2. Date 2026-10-30. Nothing was modified in either tree.
Script: 20261030-p0-A2/cmp.py (python -I). Full table: 20261030-p0-A2/all_files.tsv (columns: class, live sha, git worktree sha, tracked?, HEAD blob sha256, live-vs-HEAD, worktree-vs-HEAD).
Other TSVs (all in 20261030-p0-A2/): ws_src_nonidentical.tsv, differences_all_dirs.tsv, manifest_vs_live.tsv.

## 1. Git state (checkout /home/agi/control_backend/qazbot-x2-stack)
- HEAD 982f61802035 on nav/first-goals-20261028; worktree list: single worktree. `git status --porcelain=v1` is EMPTY (clean; ignored files exist, see .gitignore).
- Local branches: handoff/first-rviz-goal-20261028 2b0f9a7 (upstream origin/handoff/..., same SHA); nav/first-goals-20261028 982f618 (no upstream configured in -vv, but origin has refs/heads/nav/first-goals-20261028 = 982f618, identical); tooling/robot-sync-validate-skill-20261030 ce4537e; tooling/robot-sync-validate-skill-main a77161d.
- ls-remote origin: HEAD/main = 6e70c0f; others: fix/i1-rpp-collision 2d240dbd, integration/operator-skills-robot-20261030 e2f44be, nightly/codex-stall-20261030 e54067f, test/p20-odom-regression-20261029 ab83725, test/p20-twin02-sim-20261029 ab9c723, tooling/operator-skills-20261030 ab9c723, tooling/robot-sync-validate-skill-20261030 ce4537e, -main a77161d, PR 1 head 2d240dbd / merge d3da3eb, tags lab-stop/20261030T0652..T0704. Local checkout is not at origin/main (6e70c0f); the deployed content corresponds to the nav/first-goals line (see below). Only the local ref set is known here; whether 6e70c0f is ahead/behind was not evaluated (objects may not be fetched; fetch refspec limited).
- Recent commits: 1c65bd5 "P20 yaw-rate loop adapter now deployed (9c05bb34), D1 free_thresh 0.25->0.196", 7eed037 evidence 10-29 evening, e114dac HANDOFF, ce4537e/982f618 tooling skill.

## 2. Layout / mapping
Deployment root /home/agi/control_backend (no .git) top level: backups, candidates (269M, git-ignored), candidates_v13, docs (17G, mostly docs/evidence 16G), logs (6.9G, skipped), passive_probe.py, README.md(+.bak), run, runtime (122M, binaries, ignored), runtime_arm64.tar.gz (43M, ignored), tests (12M), tools, vendor_snapshots (356M, ignored), ws (only ws/src, 13M), .pytest_cache, nested qazbot-x2-stack (excluded).
IMPORTANT: live has NO ws/build, ws/install, ws/log. It is not a colcon workspace; the scripts in ws/src/{lio,navigation,maps} are run directly (python/launch/sh). Items mapped: ws/src, tools, tests, run, docs, candidates_v13, top-level passive_probe.py + README.md. Git-only: .claude, AGENTS.md, CLAUDE.md, .gitignore. Not compared (not in Git, ignored): backups, candidates, logs, runtime, vendor_snapshots, tarball, .pytest_cache.
Skipped hashing: 21 files >20 MB in docs/tests/tools/ws (total ~17.3 GB: bags .db3/.mcap, fastdds.log, status_log jsonl, asbuilt.tar.gz) all live-only, untracked, under docs/evidence/{nav_20261029_1425 7.1G, nav_bringup_20261028_195035 7.9G} and docs/roadmap_v13/evidence (1.3G). logs/ (6.9G) not walked. Excluded: .git, __pycache__, *.pyc, ws/build|install|log.

## 3. Counts per dir (class: IDENTICAL / DIFFERENT / ONLY_LIVE / ONLY_GIT)
| dir | IDENT | DIFF | ONLY_LIVE | ONLY_GIT |
|---|---|---|---|---|
| ws/src | 86 | 0 | 63 | 1 |
| tools | 109 | 0 | 6 | 6 |
| tests | 82 | 0 | 1508 | 11 |
| run | 0 | 1 | 0 | 0 |
| docs (top-level files) | 47 | 0 | 18 | 6 |
| docs/evidence | 74 | 2 | 352 | 296 |
| docs/roadmap_v13 | 16 | 0 | 161 | 0 |
| candidates_v13 | 5 | 0 | 0 | 0 |
| top-level files | 1 | 1 (README.md) | 0 | 3 (+1 .claude) |

## 4. ws/src (primary) - result
- DIFFERENT: 0. Every one of 73 git-tracked files under ws/src is byte-identical in live, checkout worktree and HEAD blob. All code, launch, yaml, maps, BT xml in the Git checkout (982f618) equal what is deployed.
- ONLY_LIVE 63: all are *.bak*/*.p19_kick_rejected/*.p20_yawloop_candidate backup files, untracked and ignored by .gitignore (*.bak, *.bak_*). No live-only functional file. (The two exceptions with non-.bak names, x2_motion_adapter.py.p19_kick_rejected and .p20_yawloop_candidate, are also untracked: p20_yawloop_candidate is byte-identical to the live x2_motion_adapter.py.) Full list in ws_src_nonidentical.tsv.
- ONLY_GIT 1: ws/src/navigation/candidates/x2_motion_adapter_yawloop_p20.py (git-ignored via `candidates/`, untracked; sha256 9c05bb34... = same as the deployed adapter). Harmless duplicate.
- 13 further IDENTICAL untracked files are .bak files present on both sides.
Known markers:
- P20 yaw-rate adapter 9c05bb34: live ws/src/navigation/x2_motion_adapter.py sha256 = 9c05bb340c38aac2... (contains _yaw_loop, P20 comments); Git checkout and HEAD the same. Pre-P20 71db8ebd = .bak_20261029_pre_p19/pre_p20 backup. NOTE docs/HANDOFF.md line 109 still says "P20 not deployed, deployed=71db8ebd" - stale; TEST_STATUS.md line 50 and HANDOFF line 17 say deployed.
- D1 map free_thresh 0.196: live and git ws/src/maps/d1_qazbot_20261029/map.yaml has free_thresh: 0.196 (sha f747664c...); map.yaml.orig_from_d1 (c8dc4169) keeps original.
- AMCL odom-trust (feebadbb): feebadbb875b240d was the sha of localization.launch.py at the 10-28 session (sha_at_B.txt, start_run.txt launch_sha). Current localization.launch.py is 0807263e1c2e5a30, same on both sides, so it was modified after 10-28 (backups .bak_20261028_pre_beams180 / _pre_p12 exist). The odom-trust comment ("odometry trusted more than laser per operator 2026-10-28") is present. feebadbb itself is not a currently present file.
- DDS log fix (918f941d): 918f941dd565... is dds_nav_two_iface_log.xml in docs/evidence/nav_bringup_20261028_195035/ (live) - it is the session profile loaded by start scripts (see start_run.txt), NOT in ws/src. Live file hash matches 918f941d. The ws/src/navigation/dds_qazbot.xml (70634db1) is a different profile, identical live vs git.
- T5 (Trust jump_xy_max 1.0 / jump_yaw_max 0.5): set via start.sh/restart_trust.sh arguments in the evidence dir (HANDOFF line 106), not hard-coded in ws/src; loc_trust.py/localization_trust_node.py are identical live vs git.

## 5. Other DIFFERENT / ONLY_* (non-evidence-bulk), see differences_all_dirs.tsv
DIFFERENT:
- README.md: checkout (HEAD) has one extra line "Синхронизация лаборатории ... /robot-sync-validate ... docs/ROBOT_SYNC_VALIDATE.md". Live is older doc state (Oct 21).
- run/x2_motion_owner.lock: live pid=405674 since=2026-10-29 21:56:19 (current motion owner = P20 adapter); git copy pid=133021 since=2026-10-22 (stale tracked runtime artifact). Live is truth.
- docs/evidence/i1_deploy_2d240dbd/archive_manifest.json and process_snapshot.txt: live differs from HEAD (live files changed after commit; not examined in detail).
ONLY_GIT: AGENTS.md, CLAUDE.md, .gitignore, .claude/skills/robot-sync-validate/SKILL.md, docs/{DIGITAL_TWIN_FINDINGS,HANDOFF,MPPI_MIGRATION,ROBOT_SYNC_VALIDATE,TEST_STATUS,X2_NAV_BRINGUP}.md, tools/roadmap_v13/{affinity_ab,burst_probe_v13,readiness_monitor,recover_test_elf,restart_native_r0,snapshot_asbuilt}.py (tracked but absent from live tools/; live has them presumably under docs/roadmap_v13 or never deployed - not verified), tests/goal_coordinator/test_gc_t1_pause.py, tests/loc_trust/test_trust_t1_pause.py, tests/motion_adapter/test_adapter_t1_pause.py (+ .bak), tests/.pytest_cache/*, tests/twin (symlink -> ../../digital_twin/research/tests/twin; points outside the checkout to a path that may not exist).
ONLY_LIVE: docs/*.bak_* (18), tools/rviz and tools/stack *.bak (6), tests/integration_results (1474 files; git-ignored), a few tests/* (loc_trust 10, nav_e2e 10, planner_clearance 4, controller_dry 3 ...): live-only test files - these are untracked in Git and may be real tests not committed (see all_files.tsv, filter tests/ ONLY_LIVE excluding integration_results). docs/evidence: 352 live-only (bags, raw logs, nav_bringup/nav_20261029 dirs; Git holds curated copies under docs/evidence/nav_session_2026102x, so ONLY_GIT 296 are mostly renamed/gzipped copies). docs/roadmap_v13/evidence 161 live-only (git-ignored).

## 6. Manifests
- Git docs/evidence/nav_session_20261028/manifest.json (155 entries) and nav_session_20261029/manifest.json (113): record origin sha256 of live files at publication. Re-checked against live origin paths: 241 MATCH, 25 MISMATCH, 2 SKIP(>20MB). All mismatches are growing logs/jsonl (nav_full.log, localization.log, p19/p20 *_raw.jsonl, etc.) and /home/agi/.orchestrator/{PLAN,TASK,DECISIONS}.md that were modified after publication. No code/config in ws/src mismatches (see manifest_vs_live.tsv).
- Other manifests: ws/src/maps/first_map_nav2_v2|v3/manifest.json, docs/roadmap_v13/evidence/C3_install_20261027_120444Z/manifest.json (live-only, historical), tests/integration_results/random_batch01/coordinator_before_guard.sha256 (not checked).

## 7. ws/install question
There is no ws/install (nor ws/build) in the deployment or checkout, so the "installed vs src" comparison is not applicable: nothing is symlink-install or copied. The only install/ dirs are under candidates/{nav2_lifecycle_shutdown,nav2_smac_bounds,fast_lio_portable} (third-party candidate builds, not our python packages) and /home/agi/aimdk/install. Running processes execute ws/src files directly (not verified from /proc per constraints; hypothesis consistent with HANDOFF PIDs and scripts run_nav_full.sh etc.).
Symlinks: none inside deployment ws/src, tools, tests, docs, run. In the checkout only tests/twin -> ../../digital_twin/research/tests/twin (relative, leaves the checkout; target resolves to /home/agi/control_backend/digital_twin/research/tests/twin which does not exist in deployment dirs listed). No symlinks cross between deployment and checkout.

## 8. Conclusion
- ws/src live == checkout worktree == HEAD 982f618 (and == 1c65bd5 content) for every tracked file: zero code drift. The running code (P20 adapter 9c05bb34, D1 map free_thresh 0.196, current launch/localization files) corresponds exactly to Git nav/first-goals-20261028 @ 982f618 (also pushed as origin/nav/first-goals-20261028). Not equal to origin/main 6e70c0f necessarily (not evaluated).
- Drift is limited to: untracked-but-ignored backups, README.md (checkout newer), run lock (live newer), 2 evidence files, 3 t1_pause test files and tools/roadmap_v13 scripts only in Git, and many live-only tests/evidence not committed (ignored by design: integration_results, bags, roadmap_v13/evidence).
- Caveat: files are on-disk content; what a running process actually loaded (started before edits) was not verified. P20 adapter process start 21:56 on 10-29 after the file was deployed; later file mtimes were not checked.
- Open: stale HANDOFF.md line 109; DDS profile used at runtime lives only in docs/evidence (live-only, git has copy under nav_session_20261028); tests live-only subset worth listing for commit.
