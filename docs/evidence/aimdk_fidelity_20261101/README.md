# AIMDK fidelity continuation — 2026-11-01

Base local integration: ed387a694f0547ded278a274e092d4b4db5a7eb0. Control checkout lab/control. Physical X2 untouched; no new orchestrator or autonomous optimization.

IMPLEMENTED: real-bag fixture extractor (no DDS), two production-compatible PointCloud2 plant streams with finite-only bounded observations, unknown visibility barrier, live SIM costmap-sensor graph probe wired into doctor, and fail-closed runtime-E2E qualification gate before unattended labctl start.

VERIFIED: archived bag hash chain and offline production callback replay (100 clouds, 71 usable frames); source/message regressions; recorded before/after failures; receipt validator unit cases. See log files for exact counts. The synthetic receipt tests do not establish real MC or CLI qualification. Production source baseline remains 73/73.

UNVERIFIED: full live ROS graph, actual Nav2 costmap updates, TF fixtures on rebuilt Docker, official MC/MuJoCo route, real resource/cycle metrics, GPU/EGL and real stop/resume/CLI tasks. Doctor fails safely before containers because Docker and systemd access are denied. The environment evidence includes nvidia-smi failure; PCI presence is not GPU rendering enablement.

OPEN: runtime E2E acceptance and the subsequent dynamic physical cylinder, furniture, calibrated sensor/MC fidelity, and autonomous optimization pipeline. No SIM result is promoted to HARDWARE PASS. The saved frozen STOPPED_OK checkpoint is retained without claiming a new successful runtime shutdown.

Reproduction and official vendor source links: docs/nightly/AIMDK_FIDELITY.md. Original saved bags, failures and Pareto archive are not overwritten. New fixtures preserve source/code hashes and are explicitly tagged as recorded callback replay.

Validation: full nightly/motion-adapter/sensor suite **270 PASS**; final focused receipt/archive/sensor suite **39 PASS**; early twin-package plus sensor run **39 PASS / 1 SKIP** (old research I1 candidate not mounted). The sensor-only final suite has **8 PASS**. The initial missing-publisher contract produced five expected failures, unknown visibility one failure, and an intermediate blind-sector boundary test exposed floating-point boundary handling before correction; all logs are retained. These are offline tests, not Nav2 costmap or physical runtime acceptance.

Existing Merge Queue replayed the verified hardware bag on the merged tree and compared the regenerated fixture byte-for-byte, then ran the latest sensor/twin/receipt/lifecycle/adapter regressions: **155 PASS / 1 SKIP** (unmounted historical research I1 candidate). Source integrated locally as 434241faac0f. Native sensor graph, Nav2 ObstacleLayer and MC runtime are still UNVERIFIED. Connected GitHub read confirmed published robot/stable at 1212d7eda003022d74c6ecc4a2c745fd48f57c2f, matching the local robot ref.
