# TASK 20261030-twin-data-collection
Physical-to-twin dataset from live X2 (P0, budget 120 min from 18:51 robot clock; robot clock ~1 h behind operator).
Repo: /home/agi/control_backend/qazbot-x2-stack (branch nav/first-goals-20261028). Runtime: /home/agi/control_backend. Evidence dir: /home/agi/control_backend/docs/evidence/twin_20261030/.
Previous task archived: reports/TASK_20261030-p0-repo-deploy-alignment.archived.md

## Objective
Collect synchronized command-chain, native legs+IMU odom, raw IMU, TF, both scan variants (A/B), MC mode data during ALREADY-AUTHORIZED motion; snapshot config; compute dynamics + scan A/B metrics; produce TWIN_CALIBRATION_20261030.md, update ROBOT_TO_LAB_FEEDBACK.md, TWIN_FIDELITY_REQUESTS.md, manifest with SHA256; calibration contract for lab.

## Constraints / non-goals
- Do NOT interrupt running ab_eval_motion (3600 s) or raw_scan_bridge; do NOT switch Nav2 scan source; no production config changes; no restarts.
- No new physical maneuvers without operator approval; record only operator-driven motion.
- Odometry = native legs+IMU; FAST-LIO not in loop.
- No `ros2 bag record -a`. Selected topics only; load guard (load avg already ~18; recorder at 75% CPU previously caused scan_stale trips 17:4x). Stop extra recording if scan age p95 > 0.5 s or Trust T1 pauses rise.
- Minimize new DDS participants. Large bags not in Git.
- Ground truth only from operator measurements, else GROUND_TRUTH_UNAVAILABLE. Never claim SIM-TO-REAL VERIFIED.

## Acceptance
- Config snapshot (git SHA, versions, URDF/footprint, Nav2 params, TF tree+latency, scan A/B chains, map SHA256).
- Bag(s) with integrity check + SHA256; recording impact quantified.
- Dynamics + scan A/B metrics, or explicit MISSING markers.
- Docs committed + pushed, remote SHA verified; artifact transfer done or ARTIFACT_TRANSFER_PENDING.
- Independent verifier PASS.
