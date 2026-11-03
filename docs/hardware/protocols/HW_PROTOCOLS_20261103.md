# X2 hardware protocols — 2026-11-03 (D-04 static, D-05 tape)
Roles: OPERATOR = physical actions only, RC in hand. CLAUDE = start/stop stack, recording, live checks, analysis.
Each protocol needs a separate operator OK. One protocol per session; no other experiments in parallel.

## P-D04 — static obstacle test (robot does not move)
Purpose: real-data PASS/FAIL for D-04 candidate C vs baseline A (replayed offline from the same recording).
Production stays baseline A during recording; candidate is NOT installed.

Preconditions (CLAUDE):
1. Vendor trim active (perception, orbbec_camera, hal_camera, drp, lx_webrtc_app stopped).
2. Start localization + Nav2 with adapter in DRY-RUN (start_loc.sh, start_nav_dry.sh). No goals.
3. Before any action: confirm `/qazbot_lio/scan` shows nothing < 1.5 m in front ±60° (empty zone).

Setup (OPERATOR): robot standing, free zone ≥ 2 m in front and to the sides. Mark on the floor with tape:
- L = 0.7 m from chest, +50° (left front), R = 0.7 m, −50° (right front), F = 1.0 m straight ahead, S = 0.5 m, +75° (left side, blind).

Sequence (CLAUDE calls each step after confirming in live scan that the object is seen; step only starts after "seen"):
| # | Action (operator) | Hold | CLAUDE live check |
|---|---|---|---|
| 0 | Nobody near | 30 s | baseline, no returns < 1.5 m |
| 1 | Person stands on L | 15 s, then walks straight back ≥ 2 m | returns at +45..+55°, then gone |
| 2 | Person stands on R | 15 s, then walks back | returns at −45..−55°, then gone |
| 3 | Chair (seat ≥ 0.45 m) on L | 30 s, then remove | seen while present |
| 4 | Person stands on F, then walks to S (into blind side) and stays 20 s, then leaves | — | seen at F, lost at S |
| 5 | Box ≥ 0.40 m tall on R | 30 s, remove | seen while present |
| 6 | Box ~0.25 m tall on F (D-07 probe) | 30 s, remove | expected NOT seen (D-07) |
| 7 | Nobody near | 30 s | end |
Total ≈ 6 min.

Recorded (CLAUDE, tools/d04/record_d04_static.sh): raw + registered cloud, lidar IMU, TF, odometry (LIO + native), scan/scan_obstacles/scan_clearing, local costmap_raw + updates. Plus a timestamped step log.

PASS/FAIL (offline replay of A and C on this recording, levelling fitted to live):
| Check | PASS |
|---|---|
| Steps 1, 2, 5 (object seen) | lethal cells present in A and C for the whole hold |
| Steps 1, 2 (person leaves edge) | C: edge cells cleared within 1 update after leaving; A: expected stale (reproduces D-04) |
| Step 3/5 still present | C never clears a cell of an object still present at object level; 0 clears on RPP footprint outline |
| Step 4 blind exit | cells at S NOT cleared by C (absence of observation ≠ free) |
| Step 6 | documented result for D-07 (no PASS claim) |
| Step 0/7 | no new lethal cells < 1.5 m without an object (no floor/reflection marks) |
FAIL = any lost present object, any clear in a blind bearing, or new false marks in empty zone.

After the test: stop stack via stop.sh. Do NOT relocate the robot while the stack is running (D-04b).

## P-D05 — MC speed calibration with tape (robot walks straight)
Purpose: actual speed vs command in LOW speed_mode at 0.10–0.60 m/s; stopping distance; D-06 leg odometry.
Prerequisite (CLAUDE, before asking for OK): fixed-command source script through the existing adapter path (no adapter changes), duration-limited, RC takeover wins; verified in dry-run.

Setup (OPERATOR):
- Straight free corridor ≥ 8 m, flat floor, nobody in front.
- Tape on floor: START line, then marks every 1 m up to 7 m.
- Reference point on the body: chest LiDAR front edge, projected to floor with a plumb line or straight edge. Same point for every run.
- RC in hand; stopping = RC takeover.

Sequence per run (≈ 1 min): place robot with reference on START, facing the corridor → CLAUDE starts recording + command (2 s settle + steady segment) → robot stops by itself at end of command → operator marks reference-point stop position → CLAUDE logs tape value.
| Run | cmd vx (m/s) | Command duration | Tape | Repeats |
|---|---|---|---|---|
| 1–2 | 0.30 (anchor) | 10 s | yes | 2 |
| 3–4 | 0.10 | 12 s | no | 2 |
| 5–6 | 0.15 | 12 s | yes | 2 |
| 7–8 | 0.20 | 10 s | no | 2 |
| 9–10 | 0.25 | 10 s | no | 2 |
| 11–12 | 0.40 | 8 s | no | 2 |
| 13–14 | 0.50 | 8 s | yes | 2 |
| 15–16 | 0.60 | 7 s | no | 2 |
Order: anchor first, then low → high. Command durations are capped so expected travel ≤ 6 m at the measured 1.6× gain.
Total ≈ 25 min.

Recorded (CLAUDE): /aima/mc/locomotion/velocity, /aima/mc/common/state, /aima/mc/leg_odometry, /qazbot_lio/odometry, /qazbot_native/odom, adapter status/cmd (best_effort QoS override, no clouds).

Stop criteria (abort immediately, RC takeover):
- Reference point within 1 m of corridor end or any person/object in the path.
- Lateral drift > 0.3 m from the corridor line, stumbling, or unusual gait.
- Adapter/MC status not as expected (ownership lost, latch, foreign publisher).
- Measured speed > 1.0 m/s at any point.

Result criteria:
| Item | Accept |
|---|---|
| FAST-LIO vs tape (0.15/0.30/0.50) | error ≤ 3 % (else LIO not valid as reference) |
| Repeat spread per point | ≤ 5 % |
| Curve | monotonic; minimum walking command identified (or shown absent) |
| Stopping distance | measured per point (tape stop − position at command end) |
Output: speed table, minimum stable speed, stopping distance, leg/LIO ratio per point → decision A (inverse map) / C (limits set to real speeds).
