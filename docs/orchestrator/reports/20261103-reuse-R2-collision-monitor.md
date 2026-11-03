# 20261103-reuse-first / R2: official nav2_collision_monitor (Humble 1.1.18) as an independent X2 safety layer

## Decision first
**ADAPT. Do not use it as-is, do not reject it.** Measured on the real X2 cloud: it detects the operator reliably and it
does not depend on the costmap. Without adaptation it is **NOT sufficient protection**, for these reasons:
- (1) **It is fail-open.** If the cloud is missing, stale (> source_timeout) or its TF is broken, it passes the full
  commanded speed. This was measured with a person standing 0.6 m ahead.
- (2) **It has no data beyond about ±55° bearing, nor behind the robot.** A person beside the robot is invisible
  to it, as are in-place rotation and reverse motion.
- (3) **Slowdown and Approach act on the commanded velocity.** Under D-05 (actual speed 1.61× commanded, MC below
  0.28 m/s unknown) their effect on actual speed cannot be verified.
- (4) **Low objects are NOT EVALUABLE.** Per the user correction, the bag contains no boxes.

Minimal adaptation needed before use:
- A stale/missing-data interlock that fails safe.
- A STOP-only polygon.
- `base_shift_correction: false`.
- A calibrated static TF base_footprint→lidar_chest_front.
- Insertion between velocity_smoother and the Goal Coordinator (GC), so that GC, Trust, the adapter, MC arbitration and
  operator takeover all stay downstream and unchanged.

Key numbers (real node, offline, bsc=false, 4648 outputs):

| metric | p50 | p99 | max |
|---|---|---|---|
| Cloud stamp → STOP onset (38 events) | 0.132 s | 0.138 s | – |
| Data age at each output | 0.141 s | 0.190 s | – |
| Cloud receipt → STOP (wall) | 29 ms | 37 ms | 38 ms |
| CM processing (cmd_in→cmd_out) | 0.75 ms | 2.6 ms | – |

- Emulation and real node agree on 5201/5201 outputs (scenarios s1–s5, s8).
- 0 false STOP/SLOWDOWN in 4980 empty-scene clouds (min_height 0.05).
- The person is released within one cloud (≤0.1 s) after leaving. D-04 stale marks have no effect.

## 1. Installed / API facts (evidence)
- **Not installed.** `/opt/ros/humble/share` has only nav2_common, costmap_2d, msgs, util and voxel_grid (dpkg: all
  `1.1.18-1jammy.2025072x`). No deb in runtime/qazbot_nav_deps/debs.
  - `candidates/nav2_smac_bounds/src/navigation2-1.1.18/nav2_collision_monitor` is a stub: every file is 0 bytes and
    include/ is empty. It is unusable.
  - packages.ros.org now ships `ros-humble-nav2-collision-monitor 1.1.20` (arm64), not 1.1.18.
- **Source:** official https://github.com/ros-navigation/navigation2, tag `1.1.18` =
  `1c68c212db01f9f75fcb8263a0fbb5dfa711bdea`, package.xml license **Apache-2.0**.
  - The humble branch HEAD `3c3db59d6969d8ecee8e68468693d006397f4a0c` (1.1.20, 2026-06-03) differs only in whitespace in
    polygon.{hpp,cpp} and in the test, so the API is identical.
- **Build:** scratch overlay `/tmp/claude-1001/-home-agi/b5c1d480-a1af-4b3a-828c-b93cffe75cd0/scratchpad/cm_ws`.
  - Sparse clone of the tag; `colcon build --parallel-workers 1`, MAKEFLAGS=-j2, Release, BUILD_TESTING=OFF.
  - Built against the installed Nav2 1.1.18 (nav2_util, nav2_costmap_2d, nav2_common) in **1 min 24 s, rc=0, no
    warnings**. Nothing was installed system-wide.
- **Humble 1.1.18 API**, verified in source (collision_monitor_node.cpp, source.cpp, pointcloud.cpp, polygon.cpp,
  circle.cpp, kinematics.cpp, types.hpp, nav2_util/robot_utils.cpp):
  - **Node and topics**
    - Lifecycle node `collision_monitor`, bonded to a lifecycle_manager.
    - `cmd_vel_in_topic` (default `cmd_vel_raw`) and `cmd_vel_out_topic` (`cmd_vel`). Both are **geometry_msgs/Twist**
      (no TwistStamped), QoS depth 1, reliable.
    - It has **no state topic**.
  - **Processing**
    - It runs **only inside the cmd_vel_in callback**; there is no timer. No input means no output.
    - Each source keeps **only its latest message**; there is no memory.
    - Points from all enabled sources are evaluated against the polygons in list order. STOP short-circuits.
  - **Polygons** (`polygon` or `circle`):
    - Actions are `stop`, `slowdown` (`slowdown_ratio`) and `approach` (`time_before_collision`,
      `simulation_time_step`).
    - **Trigger rule:** an action fires when the number of points inside is **> `max_points`** (default 3). The approach
      static check uses `>=`.
    - An approach polygon of type `polygon` takes its shape from `footprint_topic` (default
      `local_costmap/published_footprint`), i.e. it depends on the costmap. Use `circle` (`radius`) to avoid that.
    - Approach runs a constant-twist forward simulation of the **commanded** cmd_vel_in.
    - The output is the minimum of the candidates, using `Velocity::operator<` on x²+y²+tw². Output can only be ≤ the
      input; the monitor never increases or creates motion.
  - **Sources**
    - `scan`, `pointcloud` (`min_height`/`max_height` on base-frame z, inclusive; default 0.05/0.5) and `range`
      (`obstacles_angle`).
    - All of them use SensorDataQoS (best effort).
  - **Global timing and TF parameters**
    - `source_timeout` (default 2.0 s): a source is ignored if `now - stamp > timeout`. **Ignoring a source means no
      points, i.e. pass-through.**
    - `transform_tolerance` (default 0.1) is a **blocking** tf2 lookup timeout inside the cmd callback.
    - `base_shift_correction` (default true):
      - true: lookup at (stamp, now) via `odom_frame_id`, so it needs fresh odom TF.
      - false: latest static transform only.
    - `stop_pub_timeout` (default 1.0): after a STOP has been held this long, publishing stops (silence).
  - **Logging:** "Ignoring the source" WARN and getTransform ERROR are logged on every cmd and are **not throttled**.
    Measured: 114 WARN in 6 s, 223 ERROR in ~24 s, at 20 Hz.
- **Newer upstream, for adaptation only:** jazzy `d5d768b7b38a05e7cfd2b6c693b786b402161bed` (1.3.14).
  - It adds per-source `source_timeout`, a state topic and TwistStamped.
  - It also adds **"invalid source → STOP"**: when getData fails (no data, stale, TF) and source_timeout ≠ 0
    (collision_monitor_node.cpp ~L436-447; pointcloud.cpp returns false on each failure).
  - This is not in Humble 1.1.18 or 1.1.20.

## 2. Offline test (ROS_DOMAIN_ID=82, ROS_LOCALHOST_ONLY=1, sim time, no motion, no robot topics)
- **Bag:** `d04_static_bag_183015` (tests/d04/BAGS.md; size 9039573139 B, matches; sha not recomputed).
- **Topics replayed:** the raw `/aima/hal/sensor/lidar_chest_front/lidar_pointcloud` (10 Hz, 25.8 k pts, frame
  `lidar_chest_front`) and `/qazbot_d1_map/tf`, both remapped under `/cm_test/*`.
  - The bag has **no tf_static**.
  - The cmd_vel_in is a synthetic constant: vx 0.30 at 20 Hz.
- **Fixture TF base_footprint→lidar_chest_front** (tools/fit_fixture_tf.py), fitted from the raw floor:
  - Gravity comes from the LiDAR IMU (0.27° from the floor normal).
  - LiDAR origin is **0.9705 m** above the floor (IMU 0.967 m), pitch **−4.28°**.
  - Floor residual h p99.9: 0.041 m (0.45–1.0 m), 0.022 m (1–2 m).
  - Static only: body sway is not modelled.
- **Harness:** `/home/agi/control_backend/tests/collision_monitor/` (new directory; no existing code changed).
  - XML launch, and all parameters in `config/*.yaml` plus `config/overrides/*.yaml`.
  - Nodes (communication only): cmd source, fixture static TF, fault gate, probe.
  - Logic is in `lib/cm_eval_lib.py`: an emulation of the 1.1.18 decision logic, with 11 unit tests passing.
  - Full-bag emulation over all 7180 clouds is in `tools/offline_emulate.py`.
  - Runs, emu.pkl and analysis.json files are under `/tmp/claude-1001/-home-agi/b5c1d480-a1af-4b3a-828c-b93cffe75cd0/scratchpad/r2/`.
- **Test polygons, in base_footprint** (footprint x −0.305..0.17, y −0.372..0.34):
  - STOP: x ≤ 0.65, |y| ≤ 0.45.
  - SLOW: x ≤ 1.5, |y| ≤ 0.7, ratio 0.3.
  - Approach: circle r 0.45, t 1.5 s.
  - Source: min_height 0.05, max_height 2.0; source_timeout 0.5; transform_tolerance 0.1.
- **Run conditions:** 9 runs of 15–95 s sim each. Load average was 6–16 on 8 cores; another agent was reading the same
  bag and the live stack was running on domain 0. The latency figures include that load.

### Per-scenario table
Ground truth is the pd04-analysis §1 timeline (operator only). Results show the real node where a run exists and the
full-bag emulation otherwise. The two agree 100 %.

| # | Scenario | Ground truth | CM result | Verdict |
|---|---|---|---|---|
| 1 | Empty (4 windows, 4980 clouds; s1 real) | nothing < 3 m | 0 non-PASS at min_height 0.05/0.10/0.15. Max 2 floor points inside STOP (trigger needs 4). At min_height 0.03: STOP 100 % (floor) | **PASS**, thin margin |
| 2 | Person front, 18:34:06–51 (s2) | person 1.14 m @−7.5°, front 0.67 m | Onset 18:34:06.14. Every cloud non-PASS (52 % STOP, 48 % SLOW: front edge 0.6–0.7 m is at the STOP boundary x=0.65) | **PASS** |
| 3 | Person at L-edge +35..+56°, 18:34:52–35:19 | front 0.64–0.71 m | 100 % non-PASS (75 % STOP) | **PASS** |
| 4 | Person leaves (s2) | leaves 18:35:19–24 | Last non-PASS 18:35:22.69, PASS from 18:35:22.74 (one cloud after the last in-zone points). No residue, unlike A_live (6+10 stale edge cells to bag end) | **PASS** (D-04 independence verified) |
| 5 | Operator crossing at 0.6–0.7 m, 18:36:27–29 (s3) | person | STOP 18:36:27.54 → PASS 18:36:30.44 | **PASS** |
| 6 | Operator at −47..−50°, 0.77 m (s3) | person beside-front | SLOWDOWN only, because |y| 0.58 > STOP 0.45. Correct for these polygons | **PASS** |
| 7 | Walker approaches and exits into the R blind side, 18:38:35–41 (s4) | to 0.26 m at −57° | SLOW at 1.6 m, STOP from 0.59 m. **PASS output at 18:38:41.95 while the person is still about 0.75 m away at ≤−57°** (no returns) | **FAIL**: FOV gap. NOT sufficient protection beside the robot |
| 8 | Persons L/R at ±50° (protocol steps 1/2), blind S | absent in bag | – | NOT EVALUABLE |
| 9 | Low boxes 21/45 cm | **no boxes in this bag** (user correction) | – | **NOT EVALUABLE** (no recorded data) |
| 10 | Cloud dropout 6 s with the person present (s5) | person ~0.6 m | 0.5 s after the last cloud, the output is **0.30 m/s PASS for the whole dropout** (18:34:14.3–20.0), with WARN at 20 Hz | **FAIL** (fail-open) |
| 11 | No cloud at all (s7, TF+clock only) | person present | 445/445 outputs = full 0.30 | **FAIL** (fail-open) |
| 12 | Stale odom TF, bsc=true (s6) | person present | getTransform ERROR → no points → **PASS 0.30** from 18:34:14.05 to end | **FAIL** (fail-open) |
| 12b | Stale odom TF, bsc=false | – | Uses the static TF only, so it is immune to odom staleness. A missing static TF fails open (by code) | PASS by design / FAIL if static TF is missing (source) |
| 13 | cmd_vel_in stops (s8) | – | No output at all (no zero is sent). Downstream adapter cmd_timeout 0.15 s → zero | PASS only with the downstream timeout |
| 14 | STOP held > stop_pub_timeout (s2/s5/s9) | – | Silence after 2 s of STOP (gap 11.9 s in s2) | PASS only with the downstream timeout. GC check needed (see §4) |
| 15 | bsc=true, normal (s9) | – | Same decisions (95 % agreement; the shift changes edge points). **Processing p50 21 ms, p99 43 ms, max 51 ms**: blocking TF wait | Works, but rejected for X2 (couples it to LIO TF, adds 20–50 ms) |

### Latency
- **Stamp → cmd_out:** 0.13 s at onset (p50 0.132, p99 0.138, n=38). About 0.10 s of this is the recorded driver
  latency (cloud stamp → bag receive p50 99 ms). The rest is waiting for the next cmd_in (rx→STOP wall p50 29 ms,
  max 38 ms, bounded by the 50 ms cmd period).
- **Processing:** 0.75 ms p50 / 2.6 ms p99 per cmd. It iterates all 25.8 k points on every cmd_vel_in, i.e. 20×/s.

### FOV and geometry, measured on the raw cloud
- **Bearings:** points exist within p0.1–p99.9 of −54.8..+53.5°. The walker was lost beyond −57°.
- **Rear:** zero coverage.
- **Minimum range:** **no 0.695 m minimum range was observed.** Person returns reached a slant of 0.29 m (0.26 m
  horizontal). The empty-scene minimum slant (1.02 m) is just the floor.
- **Near-field low blind zone:** the floor is first visible at 0.498 m at 0°, so the lowest ray is about −62.8° levelled.
  An object of height h is visible only beyond (0.97 − h)/1.94 m:
  - h = 0.05 m → 0.47 m.
  - h = 0.21 m → 0.39 m.
  - In the STOP zone, the part ahead of the footprint front (0.17–0.47 m) is therefore blind for floor-level objects.
- **min_height against the floor:**
  - 0.03 false-triggers on every cloud.
  - 0.05 is clean on the static bag but only 1–2 points from triggering.
  - Walking sway (1–2°) raises the floor at 0.65 m by 1.1–2.3 cm and at 1.5 m by 2.6–5.2 cm. That is **not in this bag
    (NOT EVALUABLE)**. **Provisional min_height ≥ 0.10.**
  - This band is still far below level_bridge's band of 0.33 + 0.035·r m. Whether that helps D-07 cannot be shown
    without low-object data.

## 3. D-05: is the Approach model (and Slowdown) valid when actual ≠ commanded?
**No, not until MC is calibrated.** Evidence: 20261102-d05-D1-debugger.md §2/§4 and 20261103-controller-compare.md.
- Approach simulates cmd_vel_in (kinematics.cpp projectState).
  - At cmd 0.30 the robot walks 0.482 m/s (R 1.61), so the predicted time-to-collision is 1.61× too optimistic.
  - Unit test `test_approach_uses_commanded_velocity_only`: a 0.55 m gap gives *no action* at commanded 0.30 (contact
    1.83 s > 1.5 s horizon), but the actual contact is about 1.1 s away.
  - The ~0.4 s MC lag (HYPOTHESIS from x-corr) is not modelled at all.
- Slowdown/Approach output reduced commands such as 0.09 m/s. D-05 has **no steady data below 0.28 m/s (UNKNOWN)**.
  - The 2-point line v ≈ 0.34 + 0.53·cmd ("NOT a model") hints at a minimum gait speed.
  - The real speed reduction is therefore unverifiable. It may be none, or the robot may stop and go.
- **STOP (zero) is the only action whose effect is measured:**
  - Abrupt 0.30→0 while walking 0.49 m/s coasts 0.17–0.20 m, worst 0.26–0.30 m.
  - Budget: 0.48 m/s × (0.14 s CM + ≤0.02 s adapter) ≈ 0.08 m, plus coast 0.30 m, gives ≈ 0.38 m.
  - The test STOP zone gives 0.48 m beyond the footprint front. This is untested live.
- **Conclusion:** STOP-only polygons with a size derived from the measured coast. Slowdown and Approach stay disabled
  until W6/MC calibration.

## 4. Integration analysis (proposal only, nothing changed)
Real chain, as in production ws/src (= branch robot/feedback-d04-d05-20261102); all hops are Twist:
- controller_server (RPP) → `/qazbot_d1_map/nav2/cmd_vel_nav` (nav_control.launch.py:166-167).
- velocity_smoother (OPEN_LOOP, velocity_timeout 0.5) → `/qazbot_d1_map/cmd_vel` (:168-169).
- goal_coordinator.py:63 subscribes `cmd_vel`.
  - It trips if publisher count ≠ 1 or the GID changes (:132-138) and gates on trust/busy/latch (:142).
  - It publishes `cmd_vel_authorized` (:49).
- x2_motion_adapter (cmd_timeout 0.15 s, MC state, TF freshness, SetMcInputSource lease 1000 ms, operator_takeover
  latch) → `/aima/mc/locomotion/velocity` (McLocomotionVelocity) → MC.

Proposed insertion, between the smoother and GC:
- **Smoother:** remap its output to `/qazbot_d1_map/cmd_vel_smoothed`.
- **collision_monitor:**
  - Placement: namespace `/qazbot_d1_map/nav2`, added to lifecycle_manager_navigation `node_names` so a bond loss
    deactivates navigation.
  - Topics: `cmd_vel_in_topic: /qazbot_d1_map/cmd_vel_smoothed`, `cmd_vel_out_topic: /qazbot_d1_map/cmd_vel`.
  - TF remaps: `/tf` → `/qazbot_d1_map/tf`, `/tf_static` → `/qazbot_d1_map/tf_static`.
- **Effect on GC:** GC is unchanged. Its single-publisher check then proves that the CM is the only feeder, so the
  smoother cannot bypass it.
- **Downstream:** Trust, GC goal admission, the adapter, P20/P23, MC arbitration and operator takeover are all
  downstream and untouched. The CM can only scale down or zero a command, and it publishes nothing without input.

Required adaptations (minimal, in order):
1. **Fail-safe on missing, stale or TF-failed data** (blocking). Without it the CM is NOT sufficient protection.
   - (a) Preferred, with no fork: a freshness interlock in GC that pauses/zeros when the raw cloud header age exceeds
     about 0.3 s or collision_monitor is not `active`. The cloud is subscribed raw and only the stamp is decoded, as in
     nodes/cm_probe.py. GC already has trust-age gating.
   - (b) Alternative: backport the Jazzy "invalid source → STOP" (Apache-2.0, jazzy d5d768b) as a local patch of the
     official 1.1.18 package. That means patch maintenance.
2. **Static TF base_footprint→lidar_chest_front on `/qazbot_d1_map/tf_static`** (XML launch + YAML).
   - It does not exist in production: base_footprint is a level_bridge child of odom, and the LiDAR frames are only in
     the vendor `/tf_static`.
   - The fitted value is in config/harness.yaml and must be re-verified live.
3. **`base_shift_correction: false`**: independent of LIO/odom TF, 0.75 ms instead of 21–43 ms.
4. **STOP-only polygon** (Slowdown/Approach disabled, §3), min_height ≥ 0.10 provisional, max_height ≥ robot height,
   source_timeout 0.3–0.5.
5. **Rules for motion outside coverage**, which the CM does not protect (sides beyond ±55°, rear, in-place rotation):
   keep RPP collision checking, and do not treat the CM as covering these.
6. **Silence after a STOP** (stop_pub_timeout): GC must accept it. GC then stops forwarding and the adapter zeros after
   0.15 s. Check that GC's `zero_seen` admission is not affected.
7. **Log spam:** the upstream WARN/ERROR at 20 Hz is unthrottled. Accept it (it only occurs in fault states) or patch it;
   record this as a deviation.

Regression checks:
- **R1:** `tests/collision_monitor/tools/run_all.sh` expectations:
  - empty: 0 non-PASS.
  - person F / L-edge: 100 % non-PASS.
  - release ≤ 0.2 s after the last in-zone point.
  - onset ≤ 0.2 s.
  - After adaptation, s5/s7/s6 must give STOP within source_timeout (they FAIL today).
- **R2:** `ros2 topic info -v /qazbot_d1_map/cmd_vel` shows exactly one publisher, `collision_monitor`. GC records no
  `command_publisher_count` or `command_publisher_changed` trip.
- **R3:** killing the CM → bond → lifecycle manager deactivates navigation. cmd_vel goes silent and the adapter is zero
  within 0.15 s.
- **R4:** with no obstacle, output equals input bitwise, so the 36-run controller-compare baseline is unchanged.
- **R5:** dry-run adapter (no motion) with a static person: `cmd_vel_authorized` is zero while the person is in the
  STOP zone.
- **R6:** the TF tree has a single parent for lidar_chest_front in /qazbot_d1_map.
- **R7:** CM processing p99 < 5 ms under FAST-LIO load on X2.
- **R8:** operator takeover and the foreign-publisher latches are unchanged.
- **R9:** live static plus walking-floor min_height check, when permitted.

## 5. Failure-mode list
1. **Fail-open** on stale/missing cloud (measured, s5/s7) and on TF failure (s6). This is the decisive blocker.
2. **FOV:** blind beyond about ±55–57°, no rear. The walker beside the robot was lost (s4).
3. **Memoryless.** An object that leaves the FOV is forgotten immediately. That is the opposite trade-off to the
   costmap's D-04 stale marks: it gives independence, but no persistence.
4. **Near-field low blind zone** of 0.17–0.47 m ahead for objects below 5 cm.
5. **Floor false positives** at min_height ≤ 0.03. At 0.05 the margin is 1–2 points; walking sway is not evaluated.
6. **Slowdown/Approach use commanded velocity.** Under D-05 they are invalid or unverifiable.
7. **Event-driven only:** silent without cmd input or after stop_pub_timeout, so a downstream timeout is required.
8. **Blocking TF lookup in the callback** with bsc=true (+20–50 ms).
9. **Unthrottled WARN/ERROR at the cmd rate.**
10. **Fixture TF ignores body sway** (±2° → ±2–5 cm height error at 0.65–1.5 m).
11. **Approach `polygon` type depends on the costmap** (published_footprint). Use `circle`.

Deviations from the Henki standard:
- The harness follows it: XML launch, YAML under config/, ROS logger in nodes, logic separated and unit-tested, README.
- Offline tools/scripts use print (analysis tooling, like pd04).
- `run_scenario.sh` drives the lifecycle with `ros2 lifecycle set`, because there is no lifecycle_manager in the
  isolated test.
- The upstream CM's unthrottled logging is not ours.
- No production files were changed, nothing was installed system-wide, and no goals or motion were sent.
