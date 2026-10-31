# Terminal lab bootstrap

Run from an ordinary Linux terminal as the lab user:

```bash
/home/adn/QazBot/control_backend_p20_twin02/tools/nightly/labctl.sh doctor
/home/adn/QazBot/control_backend_p20_twin02/tools/nightly/labctl.sh start
/home/adn/QazBot/control_backend_p20_twin02/tools/nightly/labctl.sh status
/home/adn/QazBot/control_backend_p20_twin02/tools/nightly/labctl.sh stop
```

`labctl.sh` delegates execution to the existing `lab.py`, `nq.py`, SQLite, Agent Adapters and CAS Merge Queue. It does not create another orchestrator. Claude uses noninteractive `claude -p`; Codex uses `codex exec`. No TUI, `/goal`, `/loop` or old Codex session is needed. The provider CLI must already be authenticated. Provider limits pause or hand off engineering attempts through the existing adapter logic; completed results stay in SQLite and the atomic evidence store.

Use the `lab/control` checkout at the path above. `lab/stable` is the permanent integration ref and must not be checked out in any worktree: CAS updates would otherwise move HEAD without updating its files. Attempts use the current integration ref, not the control checkout HEAD; review attempts explicitly use the producer's verified SHA. Resume merges new integration commits into the control checkout. Do not switch this checkout back to `lab/stable` for normal operation.

`doctor` runs static checks, rebuilds the stack, runs a disposable AMCL/Trust stack on the internal Docker network, checks live parameters and map hashes, then removes its containers. It never launches agents or the queue daemon. With a failed static check it saves the diagnostic report and skips simulation and regression; regression can still be run independently. Reports: `run/nightly/preflight/latest.{json,md}`. Source parity is checked both on disk and on the integration Git blobs. Only the two exact, documented Dockerfile substitutions in nav_control.launch.py are allowed in the image.

`start` synchronizes the control checkout, calls the same complete preflight, and refuses on FAIL. It checks that Git refs stayed unchanged, classifies four historical failures without requeueing them, and adds a campaign with stable names. The first main task is AMCL initialpose instability (saved-data ROS replay and full AMCL E2E kept separate); next are P23 L2 stall and frame-anchor runtime qualification. Codex-first independent review tasks follow the first three producers. Later tasks cover odometry reset, T1 and collision/doorway behavior. The existing orchestrator independently reruns result tests and the Merge Queue gates merged trees. A reviewer CLI is preferred different from the producer, but provider fallback can use the same provider; this is not proof of two independent models.

The legacy robot pointer migration is limited to the explicitly reviewed published snapshot `1212d7eda003022d74c6ecc4a2c745fd48f57c2f`, and only with 73/73 integration blobs matching its manifest. Historical robot blocker/parity tasks are replaced by this explicit campaign. New robot tips still use normal robot-sync. Existing `NEEDS_ENGINEERING` and `NEEDS_APPROVAL` rows, evidence and completed results are retained. Repeat starts use the same task names, preserving history. No old failed command is blindly retried.

Only after preflight does resume start systemd. RUNNING requires an active service, live nq.py daemon PID and held orchestrator lease; the timer is enabled last, with `Persistent=false`. Failed startup disables the services and freezes/parks owned tasks. `stop` calls existing `lab.py stop` without messaging an old Codex TUI, parks tasks, preserves checkpoints/evidence, stops Compose projects whose working directory belongs to this checkout/attempts, writes Git handoff and publishes. Unknown process/service shutdown is `STOP_UNVERIFIED`; failed publication is `STOPPED_PUBLISH_PENDING`.

The lab uses map `d1_qazbot_20261029`, PGM SHA256 `72ee4094b394241e7b2761878774efbdd99e93731d7cb52709b2231c3975b354`, goal tolerances XY 0.35 m / yaw 0.17 rad from actual_parameters.json dated October 30, and P23. Trust T5 stays at 0.10 m/rad in simulation. Known odometry and map path substitutions are checked against exact values. TF checks associate observed edge messages with DDS publisher GIDs over an eight-second observation; this proves observed ownership only for that interval. Nav2 twin 1.1.20 versus robot 1.1.18 remains UNVERIFIED pending compatible regressions.

Automatic LAB checks compare against published `origin/robot/stable` and the saved manifest. Direct robot comparison is a separate manual operator procedure; the automatic chain does not call it. Hardware deployment, physical X2 access and motion are outside this workflow.

Qualification for this implementation is **incomplete** in the restricted Codex environment: Docker socket, user systemd bus and GitHub DNS are unavailable. See [evidence](../evidence/lab_bootstrap_20261031/README.md). A blocked `start` is not a successful lifecycle qualification. Do not infer new SIM PASS from the historical Claude preflight.
