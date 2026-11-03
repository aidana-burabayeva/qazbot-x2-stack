# X2 → LAB selective baseline synchronization, 2026-11-03

Classification: BASELINE_SYNCHRONIZATION. No SIM_VERIFIED fix, hardware candidate, deployment, ROS goal, replay, campaign or agent execution is claimed.

## Source and preserved state

Pinned hardware: `1212d7eda003022d74c6ecc4a2c745fd48f57c2f`; engineering evidence: `dd1d0af5bfb30733db33811a8cb0b3d8d632f13e`; feedback: `617999a77b3af19a5dc80175e16b52066ea3c7db`.
Import starts from origin/lab/stable `55ad923eaeaaf2062cea86f613d82a278b639c37` in separate worktree `/home/adn/QazBot/control_backend_import_x2_20261103`, branch `lab/import-x2-20261103`.
Actual initial local integration `047b8b375d145a527c9e0e99e772aad018427158` and control `69945ad745572acf512b93e53348f3718f32c510` include subsequent operator SIM work; retain it through merge queue. Expected historical control was `1e7e3a2faac4c512fc7cf2f3768ffe02a1c80009`.
All initial refs, statuses, worktrees, branch listings, local delta logs and consistent SQLite backup are preserved externally in `/home/adn/QazBot/x2_import_evidence_20261103/`. Control was clean. No existing worktree or queue was removed.
SQLite backup used SQLite backup API, integrity_check=ok. Historical queue says frozen=false/RUNNING_ENGINEERING, with no RUNNING tasks; this is saved database state, not a current daemon health claim. No lifecycle command was executed.

Both required Git network commands failed: `Could not resolve host: github.com`. Connected GitHub fetch_commit queries independently confirmed all three remote heads match the user-provided SHAs. Objects were fetched from the local verified `robot_git_preserved.bundle`; only the confirmed preservation ref was copied to its remote-tracking ref. This is not a successful network fetch/prune. No physical X2 access.

## Import and exclusions

[Exact imported files](imported_files.txt), [per-file source/LAB SHA256](import_manifest.json), [excluded robot-only files](excluded_robot_files.txt).
125 files: C++ projector (including its 4.9 MiB compact deterministic test fixture, not vendor binary), Collision Monitor offline harness/unit tests, D05 cancel-latency tools, D07 scripts, missing D04 tools/tests, requested engineering reports, R2/R3b, compact recovery manifests.
D04 candidate Python reference and its scope documentation are explicitly imported under docs for test/parity dependency only; production level_bridge.py is unchanged. No odom_hold or refactor branch code is imported.
Eight source files required trailing-whitespace normalization for diff --check. C++ parity test now resolves its Python reference inside this repo. Other engineering scripts retain historical input assumptions; compile success does not imply runnable replay.
No preserved_home/private SDK, backups/.bak, rejected candidates, runtime/results directories, .orchestrator, vendor runtime, raw bags or robot simulator files are imported. Historical measurements within explicitly requested engineering reports remain historical.

## Semantic conflict decisions

| Path | Robot delta vs LAB | Decision |
|---|---|---|
| tools/twin/amcl_sim.launch.py | 1440 beams; removes explicit alpha/recovery/update settings | Keep LAB 180 beams and parity settings |
| tools/twin/replay_amcl.py | loses recorded operator initialposes, covariance/epochs and source-age alignment; substitutes map-odom transform delta for robot-displacement T5 metric | Keep LAB replay fixes |
| tools/operator_skills/robot_stable.py | removes TWIN_MISMATCH result; partial manifests treat uncovered files as removed | Keep LAB safety/accounting fixes |
| ws/src/maps/d1_qazbot_20261029/manifest.json | provenance description differs; map image/YAML identical | Keep LAB descriptor; source-state diagnostics explicitly distinguish metadata when both data blobs match |
| docs/hardware/ROBOT_TO_LAB_FEEDBACK.md | adds historical D04 UNVERIFIED / D05 NEEDS_HARDWARE entry | Keep LAB contract and content; preserve statement via imported engineering README and this decision |

Full diffs are alongside this report. main_office, AMCL/TF/frame-contract/Nightly and all existing ws/src files are retained.

## Checks and limits

- Pinned tree facts reconfirmed: 1417 LAB files, 3813 robot files; 673 common identical; 10 common different. No repeated independent audit.
- 73/73 captured baseline files match SHA256. Every pre-existing production file matches the import base, including T1/T5, P20/P23, Motion Adapter and Goal Coordinator. D1 and main_office files match; see invariants.json.
- D1 map PGM SHA256 `72ee4094b394241e7b2761878774efbdd99e93731d7cb52709b2231c3975b354`; YAML `c86aa2aaefbefcaa2ddee994572cf6d13dd777b37b8e070de947f7685be43c1b`.
- CMake configure/build with real installed ROS Humble dependencies PASS, external build/install bases. 23 pure core tests PASS; two compact-fixture C++/Python parity tests PASS. No launch_testing ROS execution.
- Initial short regression: 77 passed, one new pointer freshness test failed; fixed by refreshing pointers after analysis. Focused sync regression subsequently 14 passed. Final checks/gate are recorded separately.
- 52 imported Python files compile; lab/labctl/preflight/source_state, D04 libraries, CM evaluator import with real dependencies. aimdk_msgs: BLOCKED (absent), no SDK stubs added. Existing harness stubs are imported historical test code, not SDK qualification.
- D07/PD04 scripts with top-level I/O require original scratch/derived inputs and path adaptation: BLOCKED for execution/import. No fabricated import or replay success.
- Three raw D04 bags (204153, 211835, 183015) were not found in LAB searches. Expected SHA256 and sizes are in tests/d04/BAGS.md; actual availability and SHA256 are BLOCKED, not verified. No X2 query or raw bag publication.
- All four local recovery archive SHA256 values PASS (recovery_sha256.log). Original archive_verification.json remains historical: 18431 files, zero errors; not a fresh extraction audit. Historical preservation contains an incomplete archive, as stated in original REPORT.md.

## State contract

robot/stable is the hardware baseline; preservation is engineering evidence; lab/stable is the integration ref; lab/control is the controller checkout. NIGHTLY_ROBOT_REF / preflight ROBOT_REF cannot select preservation/candidates as hardware. Preservation production additions/changes are inspected even when hardware SHA is already processed; matching integration blobs establish source presence. The D1 metadata exception requires identical PGM/YAML blobs. Analysis no longer advances the misleading legacy last_robot_integrated_sha alias. No task result or hardware status is accepted from preservation. Regression covers preservation-only new code, unchanged processed hardware and later explicit import.

## Readiness

P0 recovery: source/manifests and local archives preserved/hashed; complete historical/raw-data disaster recovery remains incomplete.
P0 D05/Collision Monitor: offline code/report baseline available; real-node stale-cloud/TF fail-open blocker remains unresolved. No stopping guarantee or SIM_VERIFIED fix.
P1 D04 C++/D07: C++ builds and compact parity passes; D07 raw-bag/scratch inputs and low-box/walking qualification remain BLOCKED. No production wiring changed.
