# 20261021-x2-nav2-e2e-dryrun — PS5 pad (rc) vs our SDK commands, in-place rotation (offline, read-only)

Bag: /home/agi/control_backend/logs/bags/g_motion_03 (t 1792657209.2 … 1792659741.6). All times below = t − 1792659000 (receive time;
LIO times = header stamp, ~40 ms earlier than receive). Nothing was published, no params/lifecycle touched.
Extraction (scratch, not repo): /tmp/claude-1001/-home-agi/c9b41d51-5b1b-455f-88cf-eb93053f257e/scratchpad/pad/extract.py
(sqlite + rclpy deserialize, with /opt/ros/humble + ws/src/navigation/aimdk_env.sh) → CSVs in the same dir
(aima_mc_locomotion_velocity.csv, aima_mc_common_state.csv, qazbot_lio_odom_level.csv, aima_mc_leg_odometry.csv (1/10 decimated), adapter status, cmd_vel_authorized).
Yaw reference = /qazbot_lio/odom_level (base_footprint, derived from the chest lidar IMU). leg_odometry yaw is unusable (±hundreds of deg jumps,
child frame lidar_imu_chest_front; same finding as g3debug) — leg odom used only for translation.

## 0. Timing correction vs the task brief
- rc non-zero messages exist ONLY at 448.20–454.05 (backward walk −0.8 + brief yaw −1.0, not mentioned in brief) and 652.84–695.32.
  The "pad session" in the bag is therefore 652.8–695.3 (MC state LOCOMOTION 653.0–697.3), not 569–741. 569–652 = rc all zeros, STAND.
- Actual pad sequence: left turn 652.8–671.4 (+96°), stop, right turn 677.0–680.9 (−95…−103°), forward 685.25–687.6 (1.21 m), BACKWARD 693.1–695.3 (0.81 m), stand.
- S3 (ours) steps: 398.27–401.31 (0.1), 403.87–406.91 (0.2), 409.47–412.51 (0.3), 414.87–417.91 (0.4); matches s3_rot_01 summary.

## 1. Field-by-field: rc vs ours (/aima/mc/locomotion/velocity, aimdk_msgs/McLocomotionVelocity v1.1.0.4)
Definition (/agibot/software/common/share/aimdk_msgs/msg/McLocomotionVelocity.msg): MessageHeader header; string source;
float64 forward_velocity (+fwd); lateral_velocity (+left); angular_velocity (+left/CCW; comment says "(m/s)", a vendor typo).

| field | rc (pad) | ours (sdk_qazbot_nav2) |
|---|---|---|
| header.stamp | ≈ receive (p50 1 ms, p99 9 ms) | ≈ receive (p50 1 ms, p99 7 ms) |
| header.frame_id | 'locomotion_velocity' | '' |
| header.sequence | 0 always | 0 always |
| header.meas_stamp | 0 | 0 |
| source | 'rc' | 'sdk_qazbot_nav2' |
| rate | 49–50 Hz (dt p50 20.6 ms; publishes zeros continuously when idle; one 1195 s silence in the bag) | 50.3 Hz (dt p50 20.1 ms, p99 29 ms), only while commanding (+ zero burst) |
| forward | stick-scaled, plateau ±0.8; ~150 ms onset spike to ±1.8 (685.42–685.56, 693.18–693.34) — exceeds MC-reported bounds fwd −1.0…+0.8 | 0 in S3 |
| lateral | small stick crosstalk 0.02–0.08 during fwd/back | 0 |
| angular | continuous stick value, ±1.0 max; slow-left 0.02→0.72, right = −1.0 pulses | 0.1/0.2/0.3/0.4 constant |
| sign | +CCW (left) confirmed: rc +0.55 → yaw increases; −1.0 → yaw decreases | +0.4 → yaw increased (+19.8°) — same convention |
There is NO priority/mode/gait/trigger field in the message. MC arbitration is via SetMcInputSource (priority rc 80, ours 30; timeout 1000 ms).
rc zero stream (50 Hz) was present throughout S3 in parallel with ours; MC state input_source stayed 'sdk_qazbot_nav2' 398–448 and switched
to 'rc' at 448.22, 20 ms after the first non-zero rc message (448.20) → MC arbitrates per source and rc zeros do not claim control.
Our values reached MC unchanged: cmd_vel_authorized == MC msg values; adapter state active/driving, trusted=True, no saturation.

## 2. MC common state: pad vs ours — identical
Both: action_desc STAND_DEFAULT(status 100) → on first non-zero cmd: status 200 (~0.15 s) → LOCOMOTION_DEFAULT, fsm 4→5, body STAND(1),
speed_mode LOW(1), bounds fwd −1.0…0.8, ang −1.0…1.0. After last non-zero: LOCOMOTION persists ~1.5 s (status 200) then STAND_DEFAULT.
- ours: 398.27 cmd → 398.42 st200 → 398.52 LOCOMOTION → 401.82 st200 → 403.43 STAND (same for each step).
- pad: 652.84 cmd → 653.02 LOCOMOTION … 673.42 st200 → 674.92 STAND.
→ No explicit mode switch, no forward/step trigger needed; MC auto-enters locomotion for an angular-only command (ours did too).
Difference in the test protocol (not in MC handling): S3 pauses (≈2.5 s zeros) let MC drop back to STAND before EVERY step, so every
3-s step began with a stand→walk transition that costs ~0.3–0.6 s and produces a gait-start yaw kick of −5…−11° (RIGHT), seen in all S3 steps and at
pad start 653.0–653.5 (−10.7°). The pad left turn kept LOCOMOTION for 18.5 s.

## 3. Command → actual yaw (LIO odom_level)
| segment | angular cmd | net yaw | avg rate | pattern |
|---|---|---|---|---|
| S3 step0 | 0.1 ×3.04 s | −5.6° | −0.03 | start kick only, no rotation |
| S3 step1 | 0.2 ×3.04 s | −4.0° | −0.02 | same |
| S3 step2 | 0.3 ×3.04 s | −4.7° | −0.03 | same (wz ≈0 after kick) |
| S3 step3 | 0.4 ×3.04 s | **+19.8°** | **+0.114** | 12° step burst at +0.7 s (peak 0.63 rad/s) then steady creep 0.06–0.10 rad/s |
| pad L 652.8–656.6 | 0.02–0.16 | −11.4° (start kick) | — | no CCW rotation |
| pad L 656.6–664.5 | ramp 0.16→0.72 | +22.6° | 0.050 | creep starts at rc ≈0.29–0.32 (657.3); 0.32–0.72 → 0.04–0.07 rad/s |
| pad L 664.5–671.4 | 0.49–0.72 | +84.7° | 0.21 | two discrete pivots: 664.6–665.5 +31° (peak 0.96 rad/s), 670.6–671.3 +30° (peak 1.22) + creep |
| pad L whole | median 0.546 | **+95.9° in 18.5 s** | **0.090** | |
| pad R #1 | −1.0 ×1.0 s | −81.6° | −1.3 (peak −2.68) | single fast pivot |
| pad R whole | −1.0 pulses + small + | −95…−103° | −0.6 | |
Minimum rc yaw that produced CCW rotation: ≈0.29–0.32 (creep onset); 0.02–0.16 produced none (0.14–0.21 only ~0.01 rad/s, within noise).
Our data agree: 0.3 = no rotation in 3 s from a fresh gait start, 0.4 = rotation. docs/X2_MOTION_ADAPTER.md row 5 (.89: 0.4) is consistent; the AIMDK doc "0.03–0.05" is NOT valid for this robot/LOW mode.
Mapping is strongly nonlinear and quantized: 0.3–0.72 → creep ~0.05–0.1 rad/s with occasional ~30° pivot steps (avg 0.09–0.21 rad/s);
−1.0 → ~80° pivot in <1 s. No samples at |w| 0.72–0.99 → unknown. Our 0.4 for 3 s performed like the pad at ~0.4–0.55 (pad avg 0.09, ours 0.114).

## 4. Forward / backward (rc) vs actual
- rc forward +0.8 plateau (with 1.8 onset spike) 685.25–687.61 → LIO 0.61/0.71/0.76/0.71 m/s (0.5 s windows), leg 0.57/0.74/0.74/0.68; 1.21 m. Ratio ≈0.9.
- rc backward −0.8 (−1.8 spike) 693.1–695.3 → 0.36–0.48 m/s; ratio ≈0.55. Early 448–454 backward −0.8 → 0.31–0.43 m/s.
- Ours (g3debug, G3): forward 0.12–0.30 → 0.39–0.49 m/s (ratio 1.6–1.9).
→ Hypothesis (2 operating points, needs S3 linear steps): MC LOW-mode forward is not proportional — once stepping it walks at ≥~0.35–0.45 m/s
(floor), saturating ~0.75 at 0.8. The "1.6–1.9×" is a floor effect at small commands, not a scale factor. Safety: max_linear 0.3 behaves as ~0.45–0.5 m/s.
Also: rc itself emits a 1.8 onset kick beyond MC bounds (our adapter saturates to 0.8; we cannot and should not imitate it without the owner's approval).

## 5. Symptom / proximate / root cause
- Symptom: S3 0.1–0.4 rad/s "almost did not rotate"; summary steady_actual 0.002/0.009/−0.001/0.078.
- Proximate: (a) 0.1–0.3 are below X2's in-place yaw threshold (~0.3) → only gait-start kick (right); (b) 0.4 did rotate +19.8° (0.114 rad/s average),
  but the S3 metric (steady window) under-reports the quantized pivot, and 3 s × ~0.1 rad/s is small; each step also restarted the gait from STAND.
- Root cause: X2 MC in-place yaw is threshold + nonlinear: command values <~0.3 produce no turn, 0.3–0.7 produce ~0.05–0.2 rad/s.
  Message format/field/units/sign/rate/source/mode are NOT the cause — field-for-field identical in semantics to rc, MC state transitions identical, sign verified.
  For Nav2: RPP accel clamp (measured + max_angular_accel·dt = +0.05) can never reach 0.3–0.4 from standstill (g3debug), and even the configured
  rotate_to_heading_angular_vel 0.5 yields only ~0.1–0.2 rad/s real, in ~30° quanta.

Ranked hypotheses:
1. HIGH — threshold/nonlinearity (above). Support: pad 0.02–0.16 no turn, creep from ~0.3; ours 0.3 none, 0.4 turn; .89 measured 0.4. Contra: pad crept at 0.32 while our 0.3 did not (borderline; pad was already in LOCOMOTION for 4 s).
2. LOW-MED — gait restart per step (STAND between steps) reduces effectiveness in short steps. Support: −5…−11° kick each start, 0.3–0.6 s lost. Contra: pad also started with the same kick.
3. LOW — rc zero stream interferes with our source. Contra: MC input_source = ours throughout S3, switched to rc only on first non-zero rc; our 0.4 rotated in the commanded sign.
4. REFUTED — wrong field/units/sign/rate/missing mode/header (frame_id '' vs 'locomotion_velocity' — G3 showed MC executes ours with ''; hypothesis it is unused).

## 6. Minimal fix direction (no edits made; for implementer/owner)
- RPP: rotate_to_heading_angular_vel ≥0.5 (provisional 0.5–0.6; pad slow turn used 0.5–0.7, measured avg 0.09 rad/s; do NOT use ≥0.8 — −1.0 gives ~80° in <1 s, envelope unknown).
- RPP max_angular_accel so that one 20 Hz cycle reaches ≥0.4 from rest: ≥0.4/0.05 = 8 rad/s² (provisional ~10); the cmd still caps at rotate_to_heading_angular_vel.
  Smoother angular accel limit must also allow it (currently 1.0 → would again ramp 0.05/cycle; with OPEN_LOOP it ramps over 0.4 s, acceptable only if feedback is not used).
  Alternative: adapter min-|wz| lift for vx=0 (adapter doc currently forbids lifting — owner decision).
- Expect ~0.1–0.2 rad/s real and ~30° quanta: yaw_goal_tolerance 0.10 rad will hunt; provisional ≥0.25–0.3 rad pending repeats. Progress checker 15 s vs 90° at 0.09 rad/s = 17 s → borderline (in-place turn is not progress in xy).
- Min angular for X2 (vx=0): 0.35–0.4 provisional (0.3 failed once, 0.4 worked once; pad creep onset 0.29–0.32).
- Next S3: hold LOCOMOTION (no gaps >1 s between steps), steps 0.35/0.4/0.5/0.6/0.7 × 8 s each, CCW and CW; forward 0.1/0.2/0.3/0.5 × 3 s to confirm the speed floor. Needs explicit user go.

## 7. F1 envelope on segment B — NOT possible
g_motion_03.joints.npz covers 1792657213.63–1792659012.29 (1798.7 s, the recorder's 1800 s cap, 89512 snapshots, 2 gaps >0.1 s), i.e. it ends
~384 s BEFORE S3 and ~640 s before the pad session; it contains only standing. envelope.py was not run on B (no data). Bag ran 2532 s → recorder duration must
be ≥ bag duration next time. Joint names include waist_yaw_joint → with coverage, it would also tell whether the creep is torso (waist) yaw vs foot rotation.

## 8. Open questions
- Is the 0.05–0.1 rad/s creep pelvis rotation or partly waist yaw (LIO is on the chest)? Net yaw persists after STAND (S3 +19°, pad +96°) → real body rotation, split unknown.
- Behaviour for |w| 0.72–0.99 and CW at 0.3–0.7 (no samples).
- Does MC clamp rc's 1.8 forward spike to 0.8? (rc exceeds reported bounds.)
- Forward speed floor (2 points only).

## Commands used
python3 extract.py <db3> <outdir> (sourced /opt/ros/humble/setup.bash + aimdk_env.sh); inline python per-segment analysis (see scratch dir);
numpy inspect of g_motion_03.joints.npz (t range, names).
