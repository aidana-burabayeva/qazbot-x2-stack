# AIMDK-aligned X2 twin qualification — 2026-11-01

Status: NOT READY for autonomous runtime E2E. Source-contract fixes and offline replay are implemented; no new official MC/MuJoCo or hardware PASS is claimed. Use existing labctl/nq/SQLite/Agent Adapters/Merge Queue/archive only. Physical X2 was not contacted.

## Official sources read before changes

These are the vendor sources, not recommendations copied from another robot project:

- [Sensor Overview, AimDK 1.1.0](https://x2-aimdk.agibot.com/en/latest/about_agibot_X2/sensor_fov.html): X2 Ultra E1R hardware aperture is 120° horizontal (−60…+60) and 90° vertical (−45…+45). This is not the angular support of QazBot's height-sliced scan.
- [Coordinate System / Factory Calibration](https://x2-aimdk.agibot.com/en/latest/about_agibot_X2/coordinate_system.html): nominal URDF geometry and per-robot factory extrinsics serve different purposes. Sensor fusion must use that unit's calibration; no factory file or transforms were changed here. The current unit's complete calibration is not established by this continuation.
- [Sensor Interfaces](https://x2-aimdk.agibot.com/en/latest/Interface/hal/sensor.html): the raw LiDAR interface is PointCloud2 at 10 Hz; the documentation distinguishes it from downstream processed scans. Cached English pages carry differing 1.0/1.1 labels; installed message definitions remain authoritative.
- [Locomotion Control](https://x2-aimdk.agibot.com/en/latest/Interface/control_mod/locomotion.html): commands are target velocities with firmware-dependent tracking/start behavior. The documented command timeouts and approximate thresholds are not new calibration constants for QazBot.
- [MC Status](https://x2-aimdk.agibot.com/en/latest/Interface/control_mod/mc_status.html): state is reported at 10 Hz; speed bounds must be read from the current mode/status. No current bounds were queried from hardware here.
- [Official MuJoCo Motion-Control Simulation](https://x2-aimdk.agibot.com/en/latest/sim_contents.html): the SDK→MC→HAL simulation chain is distinct from a kinematic plant. The vendor notes that its current simulated gait does not guarantee angular-only startup. That observation is not a license to change P23 or add linear motion.

Published robot version evidence: `docs/evidence/robot_20261030/versions.txt` reports platform v1.1.5, message interface v1.1.0.4 and ROS 2 Humble/Nav2 1.1.18. Platform firmware version is not MC binary identity. `docs/X2_AIMDK_VERSIONS.md` explicitly limits knowledge of PC1 MC firmware. Current physical MC identity remains UNVERIFIED; no new PC1 access was made.

## LiDAR: measured differences and fixtures

`tools/nightly/extract_lidar_fixture.py` verifies all existing bag-part, gzip and raw SQLite SHA256 values, then invokes the actual published production bridge callbacks without constructing a ROS node or initializing DDS. It uses the October 30 published `range_max=8.0` for replay of the October 28 recording. It does not claim that recording is a new October 30 acquisition.

Reproduce from the repo root:

```bash
source /opt/ros/humble/setup.bash
python3 tools/nightly/extract_lidar_fixture.py --out /tmp/x2_recorded_lidar.json
```

Source: saved `i1_cloud_registered_udp_20261028_092700Z` bag (100 cloud/odom pairs). Result: 71 usable frames after level initialization, matching prior evidence. The tracked fixture contains one real output frame plus all frame summaries, timing samples and source/code hashes. Raw bag parts are preserved unchanged.

| Property | Recorded processed QazBot data | Current twin / qualification limit |
|---|---|---|
| Hardware aperture | E1R 120°×90° from vendor | planar map raycaster has no vertical sensor model |
| Filtered angular support | finite scan-bin extremes −46…+44.25° in these scenes; clearing policy −46…+44.5° | 103° symmetric caster default retained, not declared calibrated |
| Angular bins | 1440 over 2π, 0.25° increment | same bin count; half-bin offset in the plant's bearing mask corrected to the actual caster ray angle |
| Density | 202–215 finite bins; 362 actual bounded clearing points per usable cloud | ideal ray density remains different; only finite-return clearing is generated, no inferred floor rays |
| Range | observed in-slice min ≈2.13 m, max ≈7.98 m; runtime limit 8 m | .45–8 m caster unchanged; these scene values are not raw E1R maximum range specs |
| Timing | header intervals p05/p50/p95 ≈.0987/.1000/.2551 s; receive-minus-header ≈.0340/.0362/.0556 s | configured 5 Hz, publication-time stamps; no calibrated cloud/LIO latency model |
| Missing angles / dropout | finite gaps depend on scene, slicing and acquisition | missing bins cannot independently identify dropout; no guessed random dropout distribution added |
| Noise | cannot infer range noise from this scene's range dispersion | no calibrated noise or 3D E1R density model |
| Objects | saved cloud contains actual scene returns | static map rays and perception-only discs; physical dynamic world matching remains OPEN |

A hardware FOV, a post-slice finite-return envelope and an explicit clearing policy are different quantities. No arbitrary replacement of 103° was made. Runtime sensor fidelity remains UNVERIFIED.

## Costmap observation contract

Production `nav_control.launch.py` consumes `/qazbot_lio/scan_obstacles` and `/qazbot_lio/scan_clearing`, both PointCloud2. Inspection found only a LaserScan publisher in PlantNode. Before modification, all five callback contract cases failed on the missing PointCloud2 streams. This reproduces the source-contract defect, not a complete live ROS graph diagnosis; Docker access blocks that diagnosis here.

The plant now publishes both clouds using the same scan header and base_footprint frame, reliable depth-20 QoS, z=0 projection, and its resolved scan topic prefix. Marking uses finite observed returns only. Clearing uses endpoints strictly before finite returns, restricted to production's explicit clearing policy. Infinity, NaN, missing/blocked/empty scans and angular coverage do not authorize max-range rays. Compared with true 3D production observations this is deliberately incomplete: no floor/out-of-height-slice returns are invented.

The map raycaster previously returned walls/discs behind unknown cells. Its regression failed before correction. Unknown/out-of-map cells now bound visibility evidence; they are not inserted as physical walls. A finite hit behind that boundary cannot authorize marking/clearing. This is a conservative map-only fallback, not a model of what a real LiDAR might see in genuinely uncharted physical space.

`costmap_sensor_probe.py` is integrated into doctor and runs only in the isolated SIM environment. It checks the full three-topic graph, endpoint GIDs/types, synchronized headers and actual finite PointCloud2 messages against same-stamp LaserScan returns. It uses one-argument Humble callbacks. Missing or competing publishers and insufficient valid observations fail. Its live execution is UNVERIFIED. Actual Nav2 ObstacleLayer behavior remains a separate required regression; callback tests are not relabelled as costmap PASS.

Offline source/message regressions (no DDS):

| Case | Expected | Measured scope/result |
|---|---|---|
| Fixed wall | finite marking and bounded clearing clouds with scan header | PASS callback/message harness |
| Perception-only disc disappears, finite wall behind | ray can extend to measured background | PASS harness; not a physical MuJoCo cylinder |
| Object disappears without a finite background | no invented free ray | PASS empty/no-return unit case |
| Limited/blind FOV | blind angles produce no clearing endpoints | PASS harness |
| No scan | no LaserScan or cloud publication | PASS harness |
| All infinity/NaN / out of range | no clearing endpoint | PASS numeric regression |
| Unknown strip | no ray evidence for wall/disc behind it | FAIL before, PASS after correction |

## MC and odometry identities

| Layer | Evidence / remaining mismatch |
|---|---|
| Physical platform / interface | published v1.1.5 / v1.1.0.4; this does not identify current PC1 MC firmware |
| Official x86 MC | supplied `mc-x86-v1.0.0-20260522.zip`, pinned archive manifest; local binary SHA saved in evidence |
| Experimental r9 | existing supervisor selects an aarch64 `v1.1.0-r9-rc9` path under qemu-user; this label is not fresh confirmation of current physical MC or timing equivalence |
| MuJoCo module | inspected ELF requires libmujoco.so.3.3.7; both 3.3.7/3.3.8 files exist in vendor tree, so filename alone must not establish the loaded library |
| HAL odometry | official twin uses simulated pelvis/world pose plus nominal frame anchor; no demonstrated native leg odometry or FAST-LIO dynamics |
| Native MC odometry | physical bridge owns odom→base_footprint; cannot substitute HAL truth and claim identical reset/drift/timing behavior |
| FAST-LIO / chest origin | height-sliced cloud tracks the LiDAR IMU; pelvis/chest offsets, gravity leveling and timestamp matching are not eliminated by correct frame anchoring |
| Stop/coast/startup/saturation | historical physical traces retained; no new matched sim A/B or current MC bounds/cycle measurements |

The saved physical P20 run `p20_yaw_test_20261029_215906_summary.json` reports target ±.5 rad/s, sustained measured +.5342/−.3541 rad/s, coast −3.18°/+2.68°, drift .235/.187 m for the two rate steps. These are P20 physical references, not P23 or newly matched simulations. Earlier +.501/−.451 values in twin handoffs belong to separate simulator tests and must not be mislabeled as the physical reference. Different conditions forbid declaring equivalence from those numbers.

## Runtime gate and safe terminal control

`labctl start` now requires current stored L3/runtime-E2E evidence even after PASS lite doctor. It validates `run/nightly/nightly_qualification.json` referencing an existing immutable archive record by `archive_run_id`. The record must pass archive safety/resource/evidence checks; match integration SHA, robot SHA, D1 map SHA and both current image IDs; include all required runtime gates; and refer to STORED/PASS, COMPLETED/VERIFIED tasks owned by each real CLI provider on that source SHA. No receipt is generated from these offline tests. Do not manually assert gates to make start pass. Producing the real validated receipt remains OPEN.

```bash
/home/adn/QazBot/control_backend_p20_twin02/tools/nightly/labctl.sh doctor
/home/adn/QazBot/control_backend_p20_twin02/tools/nightly/labctl.sh start
```

The second command is guarded, not a claim of readiness. Current doctor FAILS before starting containers because this environment denies Docker socket and user systemd bus access; Git CLI DNS also fails. No preflight was bypassed. Real TF/DDS fixtures, one official MC worker, short D1 route, contacts, CPU/RAM/disk/RTF/missed MC cycles, stop/resume and real Claude/Codex tasks remain UNVERIFIED. The existing frozen STOPPED_OK checkpoint is preserved, not a new observation of host process shutdown.

Use exactly one L3 worker with the existing `resource=sim` lease and preserve partial/failed evidence. No measured basis for multiple heavy workers exists. T1/T5, P23, production costmap/footprint/tolerances are unchanged. The archive is preserved, and controller search/Bayesian/RL remain disabled.

## GPU assessment

PCI/proc reports NVIDIA RTX 2060 (Mobile PCI identity). NVML/nvidia-smi cannot communicate with the driver in this environment; VRAM is UNKNOWN. The existing headless supervisor starts Xvfb; actual GL renderer is not measured. ELF dependency inspection is not an EGL capability test. Hardware EGL support of the supplied runtime, GPU enablement and software/GPU A/B on RTF, CPU, cycle misses, latency, RAM and VRAM are UNVERIFIED. No MJX conversion or rendering switch was made.

## OPEN after sensor/runtime qualification

- True 3D collision-world ray sensing, verified per-unit extrinsics, and calibrated sensor timing/density/noise/dropout as measurements support them.
- Real Nav2 marking/clearing regressions for fixed/disappearing obstacles, limited FOV and missing scan; localization/reset/P23/recovery/CPU-contention and stop/resume E2E cases.
- One physical MuJoCo cylinder absent from D1, reproducible appearance/disappearance and trajectory, crossing/blocking cases, raw contacts, clearing and replanning counts. Existing `ghosts` are sensor-only and do not satisfy this gate.
- Furniture primitive overlays shared by physical collision and sensing geometry, with duplicate-map-object checks. D1 stays unchanged. No unqualified furniture overlay was enabled.
- After verified L3 only: baseline → deterministic L1 screening → Nav2 shortlist → L3 → independent validation → existing Pareto archive. Future interfaces must preserve fixed localization/P23/costmap/safety contracts; Bayesian/RL stay separate.

See [evidence](../evidence/aimdk_fidelity_20261101/README.md) for commands, failures and hashes. Task completion requires confirmed runtime E2E and is not achieved by this source-only continuation.
