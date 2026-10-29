# TWIN-02 recovery candidate (simulation only)

This is an isolated C++ decision/math core with an experimental ROS odometry publisher. It is not installed on X2. Production `SourceGuard`, `native_odom_bridge`, localisation parameters and I1 are unchanged.

The reset sequence is `tracking -> suspended -> verifying -> tracking`. An expected MC STAND-to-DAMPING transition with motion already inhibited suspends output. A jump without that event latches; neither a long data gap nor subsequent stable samples clears the latch.

To reanchor, the caller must provide all of:

- the active reset epoch (older/replayed epochs are rejected);
- explicit evidence that the source origin reset, beyond the mode change itself;
- fresh source pose and independently verified pose in the previous published odom frame, matched within 1 ms;
- current MC STAND and an active motion inhibit.

The mapping becomes `T_published_source = T_verified * inverse(T_new_source)`. Subsequent source increments retain translation and rotation in the correct frame. The raw stamp is preserved. A two-sample warmup plus explicit fresh Trust requalification is required before releasing the recovery interlock. The motion adapter's own latch remains in force and needs its existing explicit resume and a new command.

Never reuse the last pre-gap pose as a verified post-gap pose just because MC changed mode: real motion during the gap is unobserved. The independent provider in the present tests is simulator-known pose. **No equivalent source-reset proof or independent-pose provider is implemented/qualified for physical X2.** The bool proof inputs are a test boundary, not authentication or a deployable evidence mechanism. The experimental ROS transport is tested below. Production admission integration and a real evidence provider remain required before deployment.

The core retains production limits: displacement `0.05 + 1.5 * min(dt, 0.2)` m, yaw step 0.3 rad, source age/future bound 0.2 s. It does not weaken T1/T5. Frame/quaternion projection remains a responsibility of the existing source bridge and is not implemented twice here.

Checks from repository root:

```bash
g++ -std=c++17 -Wall -Wextra -Werror -O2 tests/lio/test_reanchor_core.cpp -o /tmp/test_reanchor_core
/tmp/test_reanchor_core
source /opt/ros/humble/setup.bash
python3 -m pytest -q tests/lio/test_twin02_trust.py
```

The C++ replay covers 0.467/0.5/1.563/1.6 m jumps at 0.1/40 s gaps, confirmed versus unexplained reset, stale proof, wrong epoch, MC state, inhibited motion, SE(2) increments and replay prevention. Python tests load that same compiled core through a test-only C ABI and connect it to the actual `TrustEvaluator` and `AdapterCore`; simulated time is deterministic. These are component-chain checks, not DDS/TF/AMCL acceptance or physical tests.


## Experimental ROS replay

Packages `qazbot_odom_recovery_msgs` and `qazbot_odom_recovery` provide the explicit `RecoverOdometry` service, a C++ odometry node, and a C++ 6 ms source injector. No physical MC topic is used: inputs are `/twin02/leg_source` and `/twin02/mc_state`; outputs are `/twin02/odom` and `/twin02/status` (`diagnostic_msgs/DiagnosticArray`). Service `/twin02/recover` accepts BEGIN, CONFIRM and RELEASE with an epoch, explicit inhibit, reset evidence, independently verified pose and new Trust-qualification acknowledgement. State numeric values: tracking=0, suspended=1, verifying=2, latched=3. Source timestamps and production IMU-to-footprint yaw projection are retained. Pose output is decimated to 30 Hz; twist is intentionally unknown (variance 1e6), not reconstructed across a gap. No TF is published by this experimental transport. External Trust must be joined with `recovery_gate`; the diagnostic alone never authorizes motion.

Both nodes require `TWIN_SANDBOX=1`, `ROS_DOMAIN_ID=87`. Source fixture parameters: `gap_s` (default 1, test range .1..40), `jump_m` (default .467, range .467..1.6), `announced_reset` (default true), `invalid_after_reset` (default false). Scenarios persist their YAML fixture with results. The recovery node currently uses fixed production source-guard limits; there are no tunable weakened thresholds.

Build in existing `x2-twin-stack:dev` with source mounted `/backend` and writable build directory `/work`:

```bash
colcon --log-base /work/log build --base-paths /backend/candidates/native_odom_reanchor/qazbot_odom_recovery_msgs /backend/candidates/native_odom_reanchor/qazbot_odom_recovery --build-base /work/build --install-base /work/install --cmake-args -DCMAKE_BUILD_TYPE=Release
source /work/install/setup.bash
python3 /backend/tests/lio/twin02_ros_scenario.py --jump 1.563 --gap 40 --announced true --out /results/new-confirmed-long
```

Use a disposable container with `--network none` and no hardware mounts; mount a writable `/results`. The scenario starts/stops both C++ nodes and drives the actual TrustEvaluator and AdapterCore against received DDS messages. It confirms fail-closed behaviour before reset confirmation, after pose recovery but before new Trust qualification, and until adapter resume plus a fresh command. It is not a full AMCL/TF/Goal Coordinator run.
