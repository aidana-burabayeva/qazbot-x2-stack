# X2 preservation and continuation, 2026-11-03

Robot: agi@192.168.1.72. Read-only deployment inspection; no production edits, node launches, goals, velocity commands, or restarts. Git work was performed in a separate checkout at /tmp/x2_preservation_checkout_20261103.

## Answer

All inspected current project source files in deployed control_backend/{ws/src,tools,tests}, excluding historical backups/candidates/results and Python caches, matched the already published feedback branch 617999a77b3af19a5dc80175e16b52066ea3c7db. This includes P23, the live BT/AMCL edits, C++ projector and D1 map. The nav/first-goals branch is NOT a faithful deployment snapshot: its adapter and two map files differ. The preservation branch consolidates the feedback and WIP histories and saves the previously ignored historical files and additional custom projects.

The stronger statement "absolutely everything on the physical X2 is in git" is NOT established and must not be claimed. Binary runtime, SDK/vendor dependencies, raw recordings and the incomplete historical archive are not all published to git. A private backup on the operator PC is distinct from a GitHub push. Vendor firmware and the entire /agibot and /opt trees were not audited or imaged.

## Previously published work independently confirmed

- robot/wip-20261103: 3640a5a8d86a7f3a2e2f90243455faa7e17de7f2, remote confirmed. BT + live AMCL.
- robot/feedback-d04-d05-20261102: 617999a77b3af19a5dc80175e16b52066ea3c7db, remote confirmed. P23, map/manifest, C++ projector, tests and report backup.
- refactor/x2-lio-planar-bridge-20261103: c1ccbde055cd0203dde2b8f5092df979995c38ba, fetched for inspection only. Author/committer Aidana; rename + call-site changes. Authorship metadata does not prove exclusive ownership; this work was not merged or deployed.

## New preservation

- Full SHA256 inventory of 19,428 entries in the selected code/runtime trees; files.json includes sizes, modes and symlink targets.
- Published custom source: preserved_home/{x2_slam_stage1,qazbot-speech,linkcraft_player.py}. Speech vendor libraries and generated aimdk overlay are in the private backup instead.
- Published own candidate scripts/patches in preserved_candidates; candidate build/install/runtime and vendored src were not copied into git.
- Historical deployed backups and P19/P20/P23/P25 candidates, integration results, all orchestrator records and selected deployed docs text/config/code records are preserved. Python caches and runtime ownership locks are excluded.
- Active process command lines and generated launch/controller/planner parameters are captured without creating a ROS participant. The observed adapter was live:=false; this is a process-argument observation, not a robot-wide motion safety certification.

Private backup location on operator PC: /home/adn/QazBot/x2_preservation_20261103/.

- robot_code_runtime_private.tar.gz: selected deployed code, tests, candidates (including source/build/install), runtime, vendor_snapshots, AIMDK tree, speech with dependencies, SLAM scripts, linkcraft and orchestrator. All 18,431 eligible regular files matched the pre-archive SHA256 inventory, zero errors. .git, Python caches and pytest caches excluded. Symlink targets are recorded; absolute targets outside the archive are not implicitly backed up.
- robot_git.bundle: all refs reachable in the robot repository at bundle creation, before the new preservation branch. Extra/mc-rl and extra/x2_rl_deploy .git histories are not included in the tar; their working files are.
- historical_preservation.tar: complete copy of robot_preservation_20261030, including the already incomplete history.tar.zst.partial. Copying the partial file does not make its original cancelled backup complete.
- SHA256SUMS and archive_verification.json provide local integrity evidence.

Excluded bulk data: control_backend/docs recorded bags and control_backend/logs are tens of GB; selected text/config/code records were copied, not every recorded payload. No claim that all historical sensor data is in GitHub. Prior operator instructions require any future publication of those bags to use recoverable compression and manifests. No credentials or secrets are intentionally published.

## Task V continuation: independent review of prior Claude reports

Reviewer is a separate Codex session, not the author of R1-R5. This is a code/report review and arithmetic correction, not a fresh hardware or full bag replay validation.

Fresh validation: PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 python3 -m pytest -q -p no:cacheprovider tests/collision_monitor/test/test_cm_eval_lib.py: 11 passed in 0.20 s. These test the offline decision emulator, not the real Collision Monitor node or physical stopping.

### C1 correction

R1/R5 used D_new = 0.695 - 0.17 = 0.525 m. R3 identifies raw returns as close as 0.105 m; 0.695 was a scene observation, not a sensor limit. A literal substitution gives 0.105 - 0.17 = -0.065 m. Under the old expression:

| Quantity | Arithmetic | Value |
| --- | --- | --- |
| stopping budget, estimated typical latency | 0.482 * 0.29 + 0.30 + 0.025 | 0.46478 m |
| stopping budget, estimated worst latency | 0.482 * 0.56 + 0.30 + 0.025 | 0.59492 m |
| worst budget minus literal new D_new | 0.59492 - (-0.065) | 0.65992 m deficit |

This literal substitution does NOT create a valid detection-distance model. Raw minimum 3D range, horizontal forward detection distance, object height/bearing/occlusion, the LiDAR mount, FAST-LIO blind range and projector filtering are different quantities. Neither a minimum observed return nor a minimum sensor range guarantees that a newly appearing obstacle is detected that far ahead. In particular arbitrary obstacles appearing arbitrarily close cannot be covered by a fixed stopping guarantee. C1's old positive 0.525 m allowance is invalid; a usable C1 needs a specified obstacle/appearance protocol and measured first-detection distance relative to the swept footprint. C1 is NOT VERIFIED, not PASS. No speed or threshold changes are justified.

### C2 correction

C2 does not depend on R_min: horizon distance = 1.5 * 0.30 = 0.45 m. With the old combined estimated worst latency, budget = 0.59492 m, deficit = 0.14492 m. If the obstacle is already in the costmap and downstream latency is estimated as 0.12 s, budget = 0.482 * 0.12 + 0.30 + 0.025 = 0.38284 m, apparent margin 0.06716 m. This is conditional, not a safety PASS: obstacle-ramp stopping coast is unmeasured; 0.30 m came from a few abrupt step-stop samples, and downstream end-to-end latency is estimated rather than measured. With hypothetical 0.40 m coast the budget becomes 0.48284 m, exceeding 0.45 m by 0.03284 m. The extra distance-dependent 3% uncertainty in R1 is not included in these numbers; including it consumes more margin.

### R1-R5 disposition

- R1: keep separation between commanded speed and measured physical speed. Cancel/GC stop and smoother obstacle-stop have different command profiles. Neither the vendor demo minimum nor the current tests calibrates MC stopping dynamics.
- R2: 11 emulator unit tests rechecked. Historical real-node offline results remain historical. Missing cloud/stale TF fail-open is still a deployment blocker until an independently verified freshness interlock/backport exists. No install performed.
- R3: 0.105 m correction invalidates the old range-based C1 derivation. Real low-box detection was NOT tested: bag 183015 contains the operator, not boxes. D-07 stays OPEN.
- R3b: mostly stationary bags do not establish sustained-walking false-positive performance. No new walking evidence was acquired.
- R4: license conclusions are carried as prior local report findings, not a fresh legal/provenance assessment. AIMDK remains a private backup/dependency, not redistributed source in this commit.
- R5: downstream latency harness is preserved but not run in this session. No physical obstacle-ramp stop distance or end-to-end detection-to-stop measurement is available. D-05 stays OPEN.

V has progressed through the arithmetic correction, artifact review and fresh emulator tests. Full replay/hardware verification remains incomplete. Next useful work: audit the dry harness isolation, run it on an otherwise idle isolated environment, then plan separately authorized real recorded low-object and stopping experiments. Record raw cloud arrival and saved data before requesting object placement; log operator confirmations and action times.
