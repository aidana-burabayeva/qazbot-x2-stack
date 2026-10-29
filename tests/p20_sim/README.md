# P20 official-MC calibration probe

SIM ONLY. Runs the actual navigation-branch ROS motion adapter against official x86 MC v1.0.0 and MuJoCo in the internal twin network, domain 87. It does not launch AMCL, Trust, Nav2, or a physical robot. `require_localization=false` / `require_trust=false` are explicitly scoped to this isolated calibration; this cannot qualify navigation admission.

The first run attempted the actual native bridge but received no native leg odometry. A separate 5 s observation received 665 HAL odometry and 164 TF messages, but zero native samples. Therefore `torso_odom.cpp` computes a torso-yaw surrogate from MuJoCo pelvis pose and the measured waist-to-torso joint transforms. High-rate callbacks are C++; output is 30 Hz and retains the source stamp. Twist is deliberately invalid (angular.z=1.78, linear.y=0.30) to exercise P20's pose-only measurement. This is not a claim of physical chest-IMU or leg-odometry fidelity.

Build without creating a new Docker image:

```bash
source /opt/ros/humble/setup.bash
cmake -S tests/p20_sim -B /tmp/qazbot-p20-build
cmake --build /tmp/qazbot-p20-build -j2
```

Run with the existing `x2-twin-stack:dev` image, mounted repository at `/backend`, build directory at `/tools`, writable evidence directory at `/results`, and network `x2-twin_twin`:

```bash
python3 /backend/tests/p20_sim/p20_official_mc.py --loop true --direction 1 --out /results/new-run
```

The probe registers one SDK source, commands 1 s of forward walking at 0.3 m/s, pauses 0.8 s, requests yaw 0.5 rad/s for 6 s, then tests a 90-degree turn after a second walk. Alignment uses proportional yaw commands capped at 0.5 rad/s and a 10 s limit. It records raw pose/command traces, adapter logs, stable rate, alignment time and settled error. The source is deleted during cleanup when the MC service responds; every owned child is stopped. Use a fresh simulator/MC for comparison runs so source arbitration and gait history do not leak between cases.

The probe stops on stale pose, abnormal MC mode or adapter latch. Baseline or candidate failures remain failures; it does not modify gains/guards or retry a failed step. S2/S3 full Nav2 criteria and `goal_alignment_not_converging` are not evaluated by this calibration probe.
