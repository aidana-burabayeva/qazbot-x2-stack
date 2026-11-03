# 20261103-reuse-first / R1 — AIMDK MC examples vs X2 / D-05, velocity_smoother vs Ruckig, D-05 safety criteria
Agent: robotics-debugger (read-only, offline). No node started, no domain-0 traffic, no motion, no installs, no param/coefficient change.
Standard: ~/.claude/standards/henki_ros2_best_practices.md (checked; deviations in §7).
STATUS: Q1 ROOT_CAUSE_FOUND (0.2 m/s = demo choice, vendor-confirmed); Q2 DONE (gaps marked); Q3 DONE (smoother sufficient as command shaper, Ruckig REJECT); Q4 PARTIAL — criteria defined, current 0.30 m/s config FAILS one criterion on existing data (hypothesis-grade latency), measurement of two inputs BLOCKED.
Coordinator asked to stop early: items marked NOT CHECKED were not inspected.

## 0. Decisions first
| Item | Ready solution | Source & license | Replaces | Benefit | Risks | Decision |
|---|---|---|---|---|---|---|
| MC min-speed 0.2/0.2/0.1 in example | none (demo constant) | AIMDK local examples, package.xml license "TODO" (no license) | — | — | Using it as limit would forbid valid 0.10-0.19 cmds / wrongly imply lateral 0.2 starts | REJECT as MC model |
| MC start threshold (vendor doc) | doc values fwd ~0.1, lat ~0.5-0.7, ang ~0.03-0.05; "learned, not constants, firmware-dependent" | AIMDK local docs v1.1.0 (no license stated) | D-05 deadband hypothesis H5 | authoritative order of magnitude | must be measured on our firmware | USE as prior only; measure in P-D05 |
| Input-source registration / retry pattern | example ADD->ENABLE fallback + 8x250 ms retry | AIMDK examples (no license; upstream MIT claim by re-publisher, provenance unclear) | future C++ adapter registration | vendor-recommended retransmission for cross-host srv | ADD retry after lost response -> duplicate-ADD code; blind ENABLE fallback hides wrong state | ADAPT the idea (bounded async retry, explicit ADD/ENABLE policy as in current adapter); do NOT copy code |
| Example shutdown / signal handling | — | same | — | — | sets variable to 0 then exit(): no zero published, relies on MC 200 ms timeout; non-async-signal-safe calls in handler; never DISABLE/DELETE | REJECT |
| Example priority 40 / source names | — | same | — | — | priority 40 == built-in `pnc` (40); names `node.*`/`node` violate vendor `sdk_` prefix rule | REJECT (keep adapter 30, `sdk_qazbot_nav2*`) |
| aimdk_msgs types | McLocomotionVelocity, SetMcInputSource, GetCurrentInputSource, McInputAction, McInputSource, McCommonState, McLocomotionSpeedStatus/Bounds/Mode, GetMcLocomotionSpeedStatus, CommonRequest/CommonTaskResponse/MessageHeader | aimdk_msgs (package.xml license "TODO") | — | the only MC interface | license undeclared; use as build dependency only, no redistribution | USE (dependency) |
| nav2_velocity_smoother OPEN_LOOP | installed Humble 1.1.18 | official Nav2, Apache-2.0 (package.xml) | — (already live) | command vel/accel/decel caps, deadband-to-zero, timeout | limits are on COMMAND only; cannot lift to start threshold; no jerk | USE as is; no coefficient change before P-D05 |
| Ruckig | 0.9.2 | MIT, Copyright 2021 Lars Berscheid (src/ruckig/LICENSE, package.xml) | — | jerk-limited trajectories | adds dependency; does not fix the gain/offset/dead-time problem; MC is open-loop learned policy | REJECT for Nav2/MC path |
| D-05 safety criteria C1-C5 (§5) | derived here | — | GO criterion 6 (0.20 m, goal-end stop only) | verifiable before/after calibration | C2/C4 inputs partly hypothesis | USE as acceptance; C1/C2 currently NOT MET at 0.30 cmd (hypothesis-grade) |

BLOCKED: (B1) stop-from-cruise (non-approach) coast at the operating speed is not produced by P-D05 (it only measures goal-end stops after approach scaling); (B2) end-to-end obstacle->zero-command latency T_nav has no measurement tool (scan stamp -> costmap mark -> RPP collision -> cmd zero). Exact missing capabilities in §5.4.

## 1. System reconstruction (relevant slice, from files; no runtime queried)
- Chain (20261103-d05-cmd-source.md, d05-nav2-calib.md): RPP controller_server 20 Hz -> velocity_smoother 20 Hz OPEN_LOOP -> /qazbot_d1_map/cmd_vel -> goal_coordinator (gate, trips on 2nd publisher) -> cmd_vel_authorized -> x2_motion_adapter.py (50 Hz, source `sdk_qazbot_nav2_1102a`, priority 30) -> /aima/mc/locomotion/velocity -> MC.
- Smoother live params (qazbot-x2-stack/ws/src/navigation/nav_control.launch.py:117-123): smoothing_frequency 20, scale_velocities False, feedback OPEN_LOOP, max_velocity [vmax,0,wmax], min_velocity [0,0,-wmax], max_accel [0.4,0,1.0], max_decel [-0.6,0,-1.5], deadband_velocity [0,0,0], velocity_timeout 0.5, odom_duration 0.1. vmax 0.30 / wmax 0.6 per d05-nav2-calib.md finding 4 and controller-compare.md:12 (vmax/wmax defaults themselves NOT CHECKED in launch).
- RPP (same file :101-114): desired_linear_vel vmax, lookahead 0.5 (0.3-0.9), use_collision_detection True, max_allowed_time_to_collision_up_to_carrot 1.5, approach_velocity_scaling_dist 0.6, min_approach 0.12, regulated min speed 0.15, rotate_to_heading 0.6 rad.
- Local costmap (:68-90): update 5 Hz, 4x4 m, 0.05 m, footprint X2_FOOTPRINT (:30-31) front edge x=+0.17 m from base_footprint (= chest LiDAR projection, :24-28), padding 0.01; obstacle_layer obstacle_max_range 3.0, scan clearing False, bounded_rays clearing True; inflation 0.50 / 3.0.
- Sensor: raw chest LiDAR min 3D range 0.695 m, horizontal FOV about +-61 deg (20261103-d04b-debugger.md:18,20).
- MC (vendor doc locomotion.html, v1.1.0 local docs): open-loop velocity-level control, "actual velocity deviates ... cannot be tracked precisely"; LOW (default) fwd bounds -1.0..+0.8; MC zeroes command if no msg for 200 ms; returns to stable stand after 2000 ms; QoS BEST_EFFORT+VOLATILE, recommended 50 Hz.
- Installed smoother version: package.xml of nav2_velocity_smoother 1.1.18 (copy at /home/agi/control_backend/candidates/nav2_smac_bounds/src/navigation2-1.1.18/nav2_velocity_smoother/package.xml:5,8, Apache-2.0). Runtime binary location per prior reports: runtime/qazbot_nav_deps (1.1.18, d05-nav2-calib-V.md). `ros2 pkg prefix nav2_velocity_smoother` NOT RUN (no ROS env sourced, to avoid domain-0 discovery). Note: src/velocity_smoother.cpp in that copy is 0 bytes -> semantics taken from its README.md:56,87 + prior disassembly-based verification (d05-nav2-calib-V.md Check 2).

## 2. Q1 — "min nonzero 0.2 m/s" in the example: demo choice or MC behaviour?
Evidence:
- /home/agi/aimdk/src/examples/src/mc/mc_locomotion_velocity.cpp:29-35 max 1.0/1.0/1.0, min 0.2/0.2/0.1; comments :134,149,164,226 "mc has thresholds to start movement"; prompts :228-238 "0 or +-(0.2 ~ 1.0)". sha256 711cd0c8...4cfe, git blob 7e6cf2617aa6b24869819b090d1bc6b56fc77c2b.
- Vendor doc (aimdk/docs/cn/_build/html/en/dev/Interface/control_mod/locomotion.html, "Locomotion start threshold constraints"): threshold applies only to the FIRST step from standstill; "After the robot is already moving, smaller target velocities can be used"; forward ~0.1 m/s order, lateral ~0.5-0.7 m/s, angular ~0.03-0.05 rad/s; "learned internally by the base model and are not constants", "may vary across firmware versions; always refer to real device testing". Changelog v0.8.1 "Added explanation of directional trigger-speed thresholds", v0.8.2 "notes on validating trigger-speed thresholds after switching firmware".
- Internal inconsistency: example lateral min 0.2 < documented lateral threshold 0.5-0.7 -> the 0.2 is not derived from the threshold table. keyboard example uses step 0.2 m/s / 0.1 rad/s (Cpp.html keyboard section) = same demo granularity.
- Interface: McLocomotionSpeedBounds has only min_value (= max reverse) / max_value; no start-threshold field exists in any MC msg (aimdk_msgs/interface/robot/mc/motion/McLocomotionSpeedBounds.msg:2-6) -> no API to read the threshold.
Verdict:
- CONFIRMED (vendor doc): 0.2 m/s is a demo-level conservative input range, not an MC constant; real start threshold is ~0.1 m/s forward, start-only, firmware-learned.
- HYPOTHESIS: on our firmware the forward start threshold is ~0.06-0.10 (D1 §4 weak hint: 0.06-0.10 bursts produced ~0-0.05 m/s, <1 s, inseparable from 0.4 s dead time).
- The D-05 overspeed (R = 1.61 +- 0.044 at cmd 0.30, 16 segs, 3 days; R 0.93-0.99 at 0.80; -0.78 -> 0.54) is NOT explained by a start threshold (threshold limits starting, not steady gain). Vendor doc acknowledges non-precise tracking (qualitatively CONFIRMS D-05 mechanism "MC characteristic, not command path"); the curve shape below 0.28 remains NEEDS_DATA (P-D05 runs 0.25..0.10).
- Implication: an inverse map that commands ~0.19 for 0.30 real is inside the documented "can start" region (>0.1) — so the threshold alone does not forbid option A; the unknown is the steady characteristic there (offset/floor hypothesis v ≈ 0.34+0.53*cmd, D1 §5, NOT a model).

## 3. Q2 — examples: registration, retries, priority, shutdown; reuse safety
### 3.1 Local vs upstream (AgiBot-Community/agibot-x2-navigation @ commit 9ad1ad8af1b47b1d8fda42c3ca9c828bc5625629, tree 154eea81fd6b91d9e8914f92f82a9c4071644301, 2026-07-24 "Update README.md"; fetched via GitHub API/raw, gh not installed)
| file | local git blob | upstream blob | diff |
|---|---|---|---|
| src/examples/src/mc/mc_locomotion_velocity.cpp | 7e6cf261... | d5ca7f3b... | upstream older: name "node", ADD only (1001 literal), returns true WITHOUT checking response header.code (logs "succeeded" on any response); no ENABLE fallback; no threshold comments; min speeds identical 0.2/0.2/0.1 |
| set_mc_input_source.cpp | b00120b6... | 8c76faad... | upstream: name "node", literals, %lu/%ld formats; logic same (checks code) |
| get_current_input_source.cpp | 02c1ab6d... | 88d3d690... | format strings only |
| src/examples/package.xml | (sha256 9e1190df...) | 6434b4a3... | both `<license>TODO: License declaration</license>` |
| src/ruckig/package.xml | 0.9.2 MIT | aefb8144..., 0.9.2 MIT | same version |
| root LICENSE | none in local /home/agi/aimdk (only extra/mc-rl/LICENSE, src/ruckig/LICENSE) | 42be95b9..., "MIT, Copyright (c) 2026 Simon525" | upstream MIT is a re-publisher's root license over AgiBot SDK files whose package declares no license -> provenance unclear; treat examples as unlicensed: ideas only, no copy |
Upstream msg tree layout differs (mc/motion/McInputSource.msg vs local mc/motion/msg/McInputSource.msg) -> different AIMDK release; local (docs v1.1.0, file mtime 2026-08-28) is newer and authoritative for this robot. Local AIMDK version file: "aimdk v0.0.0-g14fe2e8".

### 3.2 Example behaviour vs vendor arbitration rules (MC_control.html §5.1.3)
- Registration: mc_locomotion_velocity.cpp:50-78 wait_for_service 2 s up to 8 s, ADD, on failure ENABLE; :80-109 8 tries x 250 ms spin_until_future_complete, success = header.code==0; name "node.locomotion_velocity", priority 40, timeout 1000 ms (:84-86).
- Vendor rules: built-ins linkswarm 90 (exclusive), rc 80, vr 70, app_proxy 60, interaction 50, pnc 40; SDK default range 20-39; names must use `sdk_` prefix; zero commands accepted but do NOT reset the timeout; timeout counts from last non-zero command, after timeout data cleared but source unchanged; takeover requires registered+enabled+non-zero cmd and (no current / current timed out / higher priority); priorities reset after system restart; cross-host services unreliable -> retransmit; do not start >2 nodes/s while standing (DDS discovery congestion can make robot fall).
- Unsafe in example: priority 40 ties built-in pnc; name without sdk_ prefix; signal handler (:197-207) only clears a member and exit()s — no zero burst, RCLCPP/rclcpp::shutdown in a signal handler (not async-signal-safe); after the 5 s run it keeps publishing zeros at 50 Hz forever (:254-257) — harmless for MC (zeros don't renew lease) but keeps a publisher alive; INFO log every 20 ms (:121-124) = log spam; std::cin/cout; never DISABLE/DELETE on exit; ADD retry after a lost response can yield "duplicated ADD" and the blind ENABLE fallback then masks a state the node did not create.
- Reusable ideas (adapt, not copy): bounded retransmission of SetMcInputSource with per-try timeout (vendor-recommended); success only on header.code==0; querying GetCurrentInputSource to confirm ownership; 50 Hz BEST_EFFORT publishing.

### 3.3 Where QazBot functions live now (must be preserved by a future C++ Motion Adapter)
| Function | Current location (file:line) | Note |
|---|---|---|
| MC arbitration / SDK source | x2_motion_adapter.py:40 BUILTIN_SOURCES {linkswarm, rc, app_proxy, vr, interaction, pnc}; :54 source_name 'sdk_qazbot_nav2'; :415-417 priority param default 30, enforced 20..39; :431-433 registration_action ADD or explicit ENABLE; :577-585 async SetMcInputSource, timeout 1000; :603 "no retry or ENABLE fallback" | already stricter than example; keep. Retry policy question for C++ port: adapter has NO retransmission while vendor recommends it -> OPEN (see §8) |
| Bounds saturation (no lift) | :18-19, :94 fwd (-1.0,0.8), :265, :516-520 live speed_status from McCommonState | keep: pass-through, saturate, never lift to a minimum |
| Zero burst / release | :58 zero_burst 5, :217, :248, :262, :376 release(), :669-675 "last word: stop, then release" | whether release() sends DISABLE/DELETE: NOT CHECKED |
| Command timeout | :82 cmd_timeout_s (0.15 s per D1 §1), :235 | MC itself zeroes at 200 ms (doc) -> adapter timeout < MC timeout: correct order |
| Operator takeover | :8, :13-14 (idle source 'rc'; PS5 takeover), :281 latch operator_takeover, :286-292 control_not_acquired vs takeover | RC priority 80 > 30 -> RC always wins at MC level (vendor rule) — consistent |
| P20 yaw loop | :20-21, :68-70 (pure turn |vx|<0.02, vy=0, |wz|>0.05/0.15 closed loop on /qazbot_native/odom yaw) | keep; MC sustained-turn deadband/overspeed |
| P23 | NOT CHECKED (location not inspected; controller-compare.md uses "P23 pure-rot/low-vx/switches" metrics) | must be located before any port |
| Trust | localization_trust_node.py, loc_trust.py; arm_motion requires trust true <0.6 s, map->odom <1 s (d05-cmd-source.md "Admission") | unchanged |
| Goal Coordinator | goal_coordinator.py (gate cmd_vel -> cmd_vel_authorized, trips on 2nd publisher; arm per goal; status keys :245-249 per d05-nav2-calib-V.md) | unchanged |
A C++ port is a refactor of a safety node -> must keep all rows above with their existing tests (tests/motion_adapter/*) as regression gate; no port is justified by D-05 itself (D-05 is MC characteristic, not adapter).

## 4. Q3 — velocity_smoother sufficiency vs Ruckig
- What the smoother does (README.md:4,15,56,87 of 1.1.18; d05-nav2-calib-V.md Check 2): clamps commanded v to [min,max], limits commanded accel/decel, zeroes |v| < deadband, zero on input timeout 0.5 s. OPEN_LOOP = integrates its own last COMMAND, not odom -> all limits are COMMAND limits. No jerk limit in Humble 1.1.18 (not in README parameter list).
- Commanded vs actual: D1 §4: MC reaches 0.39 m/s within 0.9 s and 0.47 at 1.5 s while the command ramps 0->0.30 at 0.4 m/s^2 -> actual accel is MC's own; dead time 0.4 s (x-corr r=0.994); actual steady 1.61x command. So smoother max_velocity 0.30 is NOT an actual-speed limit (actual ~0.48-0.50); max_decel -0.6 is not an actual decel (coast 0.09-0.30 m after last nonzero cmd).
- Deadband vs MC start threshold: smoother deadband can only ZERO sub-threshold commands; it cannot lift a command to the start threshold (and lifting is forbidden by adapter design :18). Zeroing is the safe direction; but since MC accepts smaller targets once moving (doc), a forward deadband near the start threshold would also truncate the decel tail (earlier zero -> MC's own stop). Decision: keep deadband [0,0,0] until P-D05 measures minimum stable speed; candidate later: deadband x = measured value below which MC produces no motion (adapt, after data).
- Can it produce the needed X2 profile? The needed profile is "commanded value whose ACTUAL result is bounded"; with an open-loop learned MC the shaping problem is gain/offset/dead time, not smoothness. Smoother is sufficient as command shaper (caps, ramps, timeout) — YES. It cannot enforce actual-speed limits — nothing on the command side can without a calibrated map (option A) or limits set from measured real speeds (option C) (D1 §7).
- Ruckig: /home/agi/aimdk/src/ruckig v0.9.2, MIT (LICENSE: Copyright (c) 2021 Lars Berscheid). Users: only examples/src/hal/motocontrol.cpp:9,126,152 (joint-level, 0.002 s) and py_examples motocontrol.py; linked by CMake into hal/utils/autonomous executables (examples/CMakeLists.txt:105,138,171); MC executables (:198-233) do NOT link it. Not used anywhere in /home/agi/control_backend (grep: no hit). Jerk-limiting the command would not reach the body through the 0.4 s dead time / learned gait. Decision: NO (REJECT) — not proven necessary.
- MPPI note (out of scope, W5): MPPI 1.1.18 has no accel limits and relies on the smoother (controller-compare.md:99) -> smoother stays required regardless of controller.

## 5. Q4 — D-05 safety acceptance criteria (derived, verifiable)
### 5.1 Inputs (with source and grade)
| Symbol | Value | Source | Grade |
|---|---|---|---|
| R_min | 0.695 m 3D min range chest LiDAR | d04b-debugger.md:18 | measured |
| x_front | 0.17 m footprint front from base_footprint (= LiDAR projection) | nav_control.launch.py:24-31 | config (includes +0.05 unmeasured arm swing) |
| D_new | R_min - x_front = 0.525 m: first possible detection distance of a NEWLY APPEARING obstacle at sensor height, ahead of footprint front edge (lower objects: horizontal >= sqrt(0.695^2-h^2) -> smaller; FOV +-61 deg) | derived | geometry; low-object case is R3/D-07 |
| D_static | up to obstacle_max_range 3.0 m (marking), costmap window 2 m | launch :82 | config |
| T_horizon | RPP collision horizon 1.5 s x COMMANDED v | launch :112 | config; uses commanded speed (d05-nav2-calib-V Check 4) |
| T_nav | scan period (~0.1, NOT CHECKED) + costmap update <=0.2 (5 Hz) + controller <=0.05 + smoother <=0.05 + GC/adapter <=0.02 ≈ 0.42 s worst-case | config periods | HYPOTHESIS (no end-to-end measurement; scan rate and projector latency not checked) |
| d_coast | after last nonzero cmd (includes MC 0.4 s dead time): ramped goal-end 0.09-0.12 m; abrupt 0.30->0 at ~0.49 real 0.17-0.20 m, worst 0.26-0.30 m; from rc 0.8 0.3-0.45 m | D1 §4 | measured (few samples, LIO) |
| v_real(0.30) | 0.482 m/s (sd 0.013) | D1 §2 | measured |
| margin M | 0.025 m (half costmap cell) + 3 % of walked distance (LIO vs tape bound, P-D05 crit. 8) | derived | — |

### 5.2 Criteria
- C1 (new obstacle, reaction budget): v_real * T_nav + d_coast_max(v_real) + M <= D_new (0.525 m). At cmd 0.30: 0.482*0.42 + 0.30 + 0.025 = 0.53 m > 0.525 -> NOT MET (zero/negative margin) — HYPOTHESIS-grade because T_nav unmeasured and d_coast_max from 2-3 abrupt samples.
- C2 (known obstacle, controller horizon): T_horizon * v_cmd >= v_real * T_nav + d_coast_max + M (the RPP check must fire early enough in REAL distance). At 0.30: 0.45 m < 0.53 m -> NOT MET (consistent with controller-compare.md:88-92 sim contact under MC hypothesis). Do NOT fix by coefficients before calibration.
- C3 (coast / выбег, measured): per calibration speed point, >=3 repetitions, d_coast = LIO path length from first zero on cmd_vel_authorized to |v_LIO| < 0.02 m/s held 0.5 s; report max (not mean); separate classes: goal-end ramped (P-D05 GO 6 <= 0.20 m) and stop-from-cruise (zero at steady speed through smoother -0.6 m/s^2). Accept a speed point only if d_coast_max(stop-from-cruise) satisfies C1 and C2 with that point's v_real.
- C4 (max allowed speed): v_allow = largest measured v_real point for which C1 and C2 hold; Nav2 desired/smoother limits may be set only to commands whose measured v_real <= v_allow, inside the measured range (option A/C per D1 §7). Additionally v_real <= 1.0 m/s hard NO-GO (protocol) and command <= MC LOW bound 0.8.
- C5 (loss-of-command stop): with stream stopped mid-walk (adapter timeout 0.15 s < MC 200 ms), d_coast <= d_coast_max(stop-from-cruise) + v_real*0.2 m; MC must return to stable stand within 2 s (doc). Verifiable offline in bag only if such an event occurs; otherwise not tested (would need a deliberate fault = separate decision).
- Note: existing GO criterion 6 (<= 0.20 m) covers only the goal-end ramped stop; abrupt worst 0.30 m already exceeds it, so it must not be read as an emergency-stop criterion.

### 5.3 What existing tools can verify
- P-D05 / nav2_calib.py (d05-nav2-calib.md, -V PASS): v_real per point 0.10-0.30, goal-end ramped coast, leg/LIO. Gives C3 (ramped class) and inputs to C4.
- Cannot produce: stop-from-cruise coast (B1), T_nav (B2).
### 5.4 BLOCKED — exact missing capability
- B1: an authorized, GC-admitted way to end a goal at steady speed (before approach scaling). Candidate existing capability: operator RViz "Cancel goal" during steady phase -> controller_server zero -> smoother -0.6 ramp. GC/adapter behaviour on Nav2 cancel (zero burst, no latch/trip) is NOT CHECKED; needs an offline dry check on an isolated domain (e.g. extend tests/d05/nav2_calib dry harness) + a protocol line in P-D05 before use. Until then C3 stop-from-cruise = BLOCKED.
- B2: an end-to-end latency probe obstacle->zero cmd. Possible offline: replay a recorded approach bag on an isolated ROS_DOMAIN_ID and timestamp scan header -> costmap update -> RPP "collision ahead" -> cmd_vel zero. No such tool verified (p23_replay/replay_lio.py exist, not inspected — NOT CHECKED). Until then T_nav = hypothesis and C1/C2 are evaluated with 0.42 s worst-case periods.
- Collision Monitor (R2) would change T_nav (raw cloud -> stop, bypassing costmap 5 Hz); C1/C2 must be re-evaluated if adopted.

## 6. Symptom / proximate / root cause (D-05 safety view)
- Symptom: controller sim contact with box under MC hypothesis; real speed 1.6x command; coasts 0.1-0.3 m.
- Proximate: RPP/smoother limits and collision horizon act on COMMANDED speed (OPEN_LOOP), while actual speed and stop distance come from MC's open-loop learned locomotion.
- Root cause: MC velocity interface is open-loop with non-unity, firmware-learned characteristic (vendor-confirmed qualitatively; D-05 measured at 0.30/0.80) and no API exposes thresholds/gain -> safety budgets must be expressed in measured real speed and real coast, which are not yet calibrated below 0.30 nor for stop-from-cruise.

## 7. Standard (Henki) compliance / deviations observed
- AIMDK examples: std::cout/cin, emoji logs, 50 Hz INFO log spam, hardcoded params, no config YAML, signal-handler shutdown, priority/name violating vendor rules -> not reusable as-is (REJECT copy; also license undeclared).
- QazBot nav_control.launch.py: Nav2/smoother params hardcoded in a Python launch file (standard: YAML in config/, XML launch preferred). Legacy -> record only; apply standard when refactoring.
- A future C++ Motion Adapter (refactor of safety node): must separate core logic (as adapter_core tests already imply) from ROS node, YAML params (source_name, priority 20..39, registration_action, timeouts), ROS logger with throttling at 50 Hz, SingleThreadedExecutor with async service client (no spin_until_future_complete inside callbacks), README API/params, tests without sleeps.

## 8. Open questions
- Does adapter release() send DISABLE/DELETE? (NOT CHECKED; x2_motion_adapter.py:376)
- Vendor recommends SetMcInputSource retransmission; adapter deliberately has none (:603). Is adapter on the same host as MC service (cross-host)? NOT CHECKED. If cross-host, single-shot registration can fail spuriously (fails closed = safe, availability issue only).
- P23 location NOT CHECKED.
- vmax/wmax launch defaults NOT CHECKED directly (taken from prior reports: 0.30 / 0.6).
- LiDAR rate and projector latency for T_nav NOT CHECKED.
- Whether speed_mode can be changed by SDK (doc: SetMcLocomotionSpeedMode is used by mobile app; srv not in local interface list) — not pursued; out of scope (would be a protection change).

## 9. Evidence commands (all read-only)
cat/sed of the listed files; sha256sum + git hash-object of local mc/*.cpp; GitHub API trees/commits + raw.githubusercontent.com fetch of upstream @9ad1ad8 into scratchpad /tmp/claude-1001/-home-agi/b5c1d480-a1af-4b3a-828c-b93cffe75cd0/scratchpad/up, diff -u; HTML->text of local docs locomotion/MC_control/mc_status/changelog/Cpp into the same scratchpad; grep in qazbot-x2-stack. No ros2 CLI used.
