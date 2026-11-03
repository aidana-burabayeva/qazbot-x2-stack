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
| 6 | D-07 height ladder on F (1.0 m ahead), one at a time: box 0.15 m → 0.25 m → 0.35 m → 0.45 m tall (≥ 0.3 m wide, matte, not glass) | 20 s each, remove before next | per box: seen / not seen in scan_obstacles and in raw cloud |
| 6b | Repeat the 0.35 m box at 0.6 m and at 1.5 m ahead | 20 s each | detection threshold vs range |
| 7 | Nobody near | 30 s | end |
Total ≈ 8 min. Box heights measured with tape by the operator and told to CLAUDE before placing.

Recorded (CLAUDE, tools/d04/record_d04_static.sh): raw + registered cloud, lidar IMU, TF, odometry (LIO + native), scan/scan_obstacles/scan_clearing, local costmap_raw + updates. Plus a timestamped step log.

PASS/FAIL (offline replay of A and C on this recording, levelling fitted to live):
| Check | PASS |
|---|---|
| Steps 1, 2, 5 (object seen) | lethal cells present in A and C for the whole hold |
| Steps 1, 2 (person leaves edge) | C: edge cells cleared within 1 update after leaving; A: expected stale (reproduces D-04) |
| Step 3/5 still present | C never clears a cell of an object still present at object level; 0 clears on RPP footprint outline |
| Step 4 blind exit | cells at S NOT cleared by C (absence of observation ≠ free) |
| Step 6/6b (D-07) | measured minimum detected height per range, raw cloud vs scan_obstacles; any box present in raw cloud but absent from scan_obstacles = D-07 confirmed (input for a separate D-07 fix, no PASS claim) |
| Step 0/7 | no new lethal cells < 1.5 m without an object (no floor/reflection marks) |
FAIL = any lost present object, any clear in a blind bearing, or new false marks in empty zone.

After the test: stop stack via stop.sh. Do NOT relocate the robot while the stack is running (D-04b).

## P-D05 — MC speed calibration via Nav2 goals (robot walks straight)
Decision 2026-11-03: option B (operator RViz goals). Direct `cmd_vel_authorized` publishing (tools/d05/const_cmd.py) is REJECTED.
Purpose: actual speed vs command in LOW speed_mode; stopping distance; D-06 leg odometry — with GC admission, gate,
Trust/T1, adapter arming per goal and MC checks unchanged in the path.
Tool: `tools/d05/nav2_calib/nav2_calib.py` (README there). Per run it only publishes `nav2_msgs/SpeedLimit` (percentage ≤ 100 %,
can only lower RPP's 0.30), records, waits for the operator goal, restores the limit, writes metadata. No goals, no velocity, no services.
Ramp = live velocity_smoother: +0.4 m/s² / −0.6 m/s² (x). Envelope today 0.30 (RPP desired = smoother max_velocity):
points > 0.30 are NOT possible without raising both limits = separate decision.

Setup (OPERATOR): straight free corridor ≥ 6 m, flat, nobody in front. Tape START + 1 m marks to 5 m.
Reference point: chest LiDAR front edge projected to the floor, same for every run. RC in hand; stop = RC takeover.
Robot on START, on the corridor centreline, heading along it (≤ 0.3 rad error, else RPP rotates in place first).
Goal (RViz, map frame): on the centreline 4.4 m ahead, heading = corridor direction (xy tolerance live 0.35 → ~4 m walked).

Per run (≈ 1.5 min): CLAUDE runs `nav2_calib.py --run-id rNN_vvv --target-vx V --path-len-m 4.4` → "READY" →
OPERATOR sends the goal → robot ramps, walks, approach-slows (last 0.6 m, only for V > 0.12), stops →
operator marks the stop position → CLAUDE notes tape into the metadata JSON → reposition (stack stays up).

### Run 1 — first check (only this run until GO)
0.30 m/s (100 %, anchor, comparable with LIVE-2/10-30), one pass, 4.4 m goal. Expected: ~6.5 s steady at ~0.48 m/s real.
| # | GO criterion (all required) | Source |
|---|---|---|
| 1 | Goal admitted by GC (busy), adapter armed by GC (`motion_armed` true), no manual service call | GC/adapter status |
| 2 | Controller output ≤ limit (meta `limit_honored`); `cmd_vel` = `cmd_vel_authorized` = MC sdk vx while gate open | bag |
| 3 | Steady command 0.30 ± 0.005 for ≥ 4 s on `cmd_vel` (meta `steady_cmd_s`) | bag/meta |
| 4 | Ramp up and down present, within +0.44 / −0.66 m/s² (meta `ramp_within_limits`) | bag/meta |
| 5 | Steady v_LIO / cmd in 1.48–1.67 (D1 pooled 1.61) | LIO |
| 6 | Stop distance (LIO, first zero on `cmd_vel_authorized` → standstill) ≤ 0.20 m | LIO |
| 7 | GC fault "" / not latched at end, adapter latch "" throughout, no foreign publisher, no non-zero `rc`, no T1 pause, speed_mode 1 | bag |
| 8 | LIO displacement vs tape (START → stop mark) ≤ 3 % | tape |
| 9 | Lateral drift ≤ 0.3 m, normal gait; speed limit restored (meta event `limit_restored`) | operator/meta |
NO-GO on any miss: stop, analyse, no further runs. Criterion 5 outside range = MC behaviour changed (mode/gait), not a tool issue.

### Runs 2+ (after GO, separate OK)
0.25, 0.20, 0.15, 0.10 (2 × each, then 0.30 repeat), same 4.4 m goal. Tape on 0.15 and 0.30 repeat. Extra GO for the
first reduced run: controller output = target ± 0.002 (lowering proven on the real binary in dry test only).
A run with GC pause/trip, rotate-in-place at start or path curvature (radius < 0.9 m → RPP slows) is invalid → repeat.

Recorded (`record_calib.sh`): `nav2/cmd_vel_nav` → `cmd_vel` → `cmd_vel_authorized` → `/aima/mc/locomotion/velocity`
(commanded chain), `/qazbot_lio/odometry`, `/aima/mc/leg_odometry`, `/qazbot_native/odom`, MC state, speed_limit,
GC/adapter/trust status, `nav2/plan`, goal/action status (best_effort overrides on vendor/odom topics, no clouds/TF).

Stop criteria (RC takeover): reference within 1 m of corridor end or anything in the path; drift > 0.3 m, stumble,
unusual gait; adapter/GC latch or foreign publisher; robot visibly faster than a brisk walk (operator judgement, RC takeover). Speed > 1.0 m/s is checked after the run from LIO and is NO-GO for the next run.
Result: speed table, minimum stable speed (or its absence down to 0.10), stopping distance, leg/LIO per point →
decision A (inverse map, only inside measured range) / C (limits set to real speeds).
