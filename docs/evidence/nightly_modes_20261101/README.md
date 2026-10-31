# Capability-scoped bootstrap evidence — 2026-11-01

## IMPLEMENTED

[Execution policy and commands](../../nightly/EXECUTION_MODES.md). Existing orchestration retained. Engineering own preflight; one-run L3 capability; current SHA-pinned D1/P23 campaign; nonpersistent mode-aware timer; finite vendor audit/failure clustering; independent-review promotion; protected safety invariants; validated simulation proof required for hardware candidates; TWIN_MISMATCH feedback.

## VERIFIED

- [273 regression PASS](regression_273.txt): `python3 -m pytest -q tests/nightly tests/motion_adapter`. This run started before the last incremental changes; final affected areas were rerun below, with overlap, not additive totals.
- [52 final focused PASS](focused_52.txt): `python3 -m pytest -q tests/nightly/test_execution_modes.py tests/nightly/test_labctl.py tests/nightly/test_lab.py tests/nightly/test_runtime_qualification.py`.
- [24 candidate/feedback PASS](feedback_24.txt): `python3 -m pytest -q tests/nightly/test_stable_branches.py tests/nightly/test_robot_sync.py`. Synthetic Git repositories; no physical access.
- [39 focused PASS](additional_39.txt), including timed-out evidence collectors still attempting teardown, missing queue/capsule, single-use claims and AST safety contracts.
- [8 sensor PASS](sensor_8.txt), [8 frame/world PASS](frame_world_8.txt): source ROS Humble then set `PYTHONPATH=$PWD/digital_twin/ws/src/x2_twin_sim:$PYTHONPATH` and run `tests/sensor_parity` / `digital_twin/ws/src/x2_twin_sim/test/test_frame_anchor.py digital_twin/ws/src/x2_twin_sim/test/test_world.py` respectively. Offline evidence only.
- [73/73 source hashes](source_parity.txt), [inventory](initial_inventory.json), [vendor binaries](vendor_identity.txt).
- [Actual entrypoint refusals](entrypoint_refusals.json): no MC capsule/queue task => refusal. `make nightly` on frozen queue => empty enqueue.
- Doctor's actual namespace smoke and provider authentication passed; [implementation-time doctor](doctor_during_implementation.txt) correctly refused dirty code/systemd failure. Docker optional WARN in Engineering.

## UNVERIFIED

Real daemon/systemd/held lease, genuine Claude/Codex Engineering results, official MC+MuJoCo D1 route, live sensor/TF/AMCL/Trust runtime, resource/RTF/missed-cycle/contact measurements and actual running stop/resume. None is inferred from mocks, CLI installation or offline PASS. Nav2 1.1.20 versus robot 1.1.18 compatibility and FOV103 calibration remain UNVERIFIED. No SIM PASS is hardware evidence.

## OPEN

This execution environment denies `systemctl --user` (bus: Operation not permitted) and Docker socket access; shell Git cannot resolve github.com. [Fresh official-doc attempts](official_docs_attempt.json) timed out. No full L3 receipt exists. Autonomous Engineering must pass its own real systemd preflight from an ordinary operator terminal; Full remains forbidden until runtime evidence exists. The saved queue checkpoint remains frozen/STOPPED_OK; this is not a newly verified live stop. Do not clear results, bags, volumes or old failures to work around disk pressure.

Clean-source doctor/start and merge-gate receipts are appended after the source commit. Source and integration SHAs are recorded in `completion.json`; documentation commits can follow without changing tested source. Physical X2 was not queried or deployed.
