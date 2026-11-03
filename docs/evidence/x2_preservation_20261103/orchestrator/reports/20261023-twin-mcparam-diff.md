# 20261023-twin-mcparam-diff — Sim MC (x86 v1.0.0-20260522) vs real X2 MC param (mc_param v1.1.0-r9-rc3 / mc v1.1.0-r9-rc9)

Mode: read-only. Nothing was modified on the laptop (192.168.1.64), on .66, or on PC1 (PC1 was not accessed). No docker or processes were touched.

Paths:
- SIM = ~/qazbot-twin/third_party/official/mc/mc-x86-v1.0.0-20260522/bin/mc_param/src/robot/lx2501_3_t2d5 (laptop)
- ROBOT = ~/qazbot-twin/data/pc1_mc/pc1_mc_param_r9-rc3/robot/lx2501_3_t2d5 (laptop). The .66 copy at /home/agi/control_backend/vendor_snapshots/pc1_mc_param_r9-rc3 is the same: 314 files, and the md5 of mc.yaml and foundationwalk_config.yaml match.
- .version: `mc_param 2873 v1.1.0-r9-rc3 be6171a9`; pc1_metadata.yaml: mc v1.1.0-r9-rc9, platform v1.1.5.

## 1. Which config the sim MC actually loads
- twin/docker/inside_mc.sh runs `unset AGIBOT_SOFTWARE_HOME` and then `echo 0 | ./em_run.sh`.
- em_run.sh then picks SIM mode: `mc_config_file=mc_sim.yaml` and `base_path=bin/mc_param/src/robot`. The robot comes from sim_robot_name_cache.txt (`lx2501_3_t2d5`, the only robot dir), and `MC_ENV_PATH=base_path`.
- **This means the sim loads mc_sim.yaml, not mc.yaml.** The "mc.yaml differs (30 lines)" finding compares the wrong file. The right comparison is SIM/mc_sim.yaml against ROBOT/mc.yaml (section 3).
- mc_sim.yaml has a `SimConfigAdapter` block. At startup it generates SIM/sim/action_{setting,ruler,state}.yaml (timestamps 17:03 today). Changes it makes:
  - removes priority and cpu_affinity
  - removes common_rules.skip_actions
  - sets `estimator.check_fall: false`
  Every other file in SIM/sim/ is a symlink to the parent directory.

## 2. Active locomotion path (task 1)
- `set_mc_action LD` maps to LOCOMOTION_DEFAULT (=300) (run/aimdk/src/examples/src/mc/set_mc_action.cpp:104).
- In both versions, action_setting.yaml has:
  `LOCOMOTION_DEFAULT: runners [rl_foundationwalk, cfg ./rl/foundationwalk_config.yaml], period_ms 20`. This is identical in SIM and ROBOT; rl_cpgwalk is commented out in both.
- Policy: `./rl_models/foundationwalk_v1.onnx`. md5 2f1503a5a75e8fc2a38d65b0eff3b2c7 in both.
- Speed mode LOW: speed_state.yaml `low_speed` sets forward [-1.0, 0.8], lateral [-1, 1], angular [-1, 1]. Identical in both.
- motion_list.yaml and motion_handler.yaml differences: only mimic/dance motions and animations that are commented or uncommented. They are not on the locomotion path.
- The action_setting/ruler/state differences only add HEAD_ONLY, UPPERBODY_REMOTE_SPLIT and BMIMIC_STEP (rl_bmimic_step instead of rl_cpgwalk), plus bmimic_stop_estimator. None of these are on the LD path.
- Other diffs not on the LD path:
  - foundation_config auto_transition_action: STAND_DEFAULT -> BMIMIC_STEP
  - foundationplus `enable` true -> false
  - cpgtelecon `min_waist_ptch` -> `min_waist_pitch`, plus extra stand_actions

## 3. Parameter diff on the active path (task 2)
foundationwalk_config.yaml (`diff`): **the only difference is that ROBOT adds `urdf_filename: "x2_whole_body_fix.urdf"`.**
- The x86 v1.0.0 lib libmc_runner_rl_foundationwalk.so does not have the string `urdf_filename`. It does have the hardcoded string `/x2_whole_body_fix.urdf`.
- That file is identical in both trees (md5 b9ced63e2e32b41090911a455f7579d2).
- So the URDF that feeds the 15 ee_pos observations is the same file in both versions.

Every value below is identical in SIM and ROBOT:

| group | values |
|---|---|
| timing | dt 0.02, period 20 ms, frame_stack 6, obs_nums 111, action_nums 29 |
| action | action_scale 0.25 x29, obs_clip 100, action_clip 100 |
| IMU | imu_roll_offset 0.0, imu_pitch_offset -0.015 |
| PD | kps [120,120,40.18,100,100,200,100,100,200,150,150,50,50,40,40,...], kds [5,5,2.56,4,4,2,4,4,2,5,5,3,3,2,2,...], default_dof_pos same, default_kp 20, default_kd 1 |
| obs_scales | commands 1.0, ang_vel 0.25, projected_gravity 1.0, dof_pos 1.0, dof_vel 0.25, actions 1.0 |
| command filter | max_acceleration 2.0, max_deceleration 2.0 |
| foundationwalk | run_scale 0.9, run_scale_y 0.65, run_scale_yaw 1.5, cmd_threshold 0.1, vel_yaw_threshold 1.5, vel_yaw_threshold_walk 1.0, vel_yaw_cooldown_speed 0.25, vel_yaw_cooldown_sec 0.6, vel_yaw_run_speed 1.1, min_velx_command -0.5, min_run_velx 0.5, max_vel_y 0.65, max_vel_yaw 1.5, exit_grace_sec 1.5 |
| speed clip (LOW) | identical, see above |

The ONNX model, the speed_state file, and all URDFs common to both trees are identical. The top-level urdf/x2_31dof_hand.urdf (used by JointTF) is also identical: md5 b624d378…

There is no gait-frequency parameter for foundationwalk in either config. Gait timing is inside the policy.

### Differences between SIM mc_sim.yaml and ROBOT mc.yaml (MotionControlModule section and the runtime .dump files)

| key | SIM | ROBOT | could it affect LD velocity response? |
|---|---|---|---|
| InputManager.expiration_time | absent (the v1.0.0 libmc_manager_input.so knows this key; its default is UNKNOWN) | 200 ms | Possibly. It is a command-staleness timeout. Both sim and real publish at 50 Hz, so it only matters for gaps. Unlikely to cause the gain difference. |
| input_sources | rc/vr/app_proxy/interaction/pnc, no `enable` key | adds linkswarm (82, disabled) and remote_teleop_pc (81); every source has `enable: true` | Priority arbitration only. pnc=40 in both. |
| SecureManager.check_fall | false (and SimConfigAdapter forces estimator.check_fall=false) | true, includes LOCOMOTION_DEFAULT | Safety only, unless a fall is detected. |
| leg_odometry publish freq | 200 | 170 | No (odometry output only). Note that /aima/mc/leg_odometry is not published in the sim at all (official_sim_results.md). |
| wait_ready for arm/waist/chest_imu | false | true | Startup gating only. |
| upper_body_command subscriber | absent | present | No |
| JointTF model_path_update | absent | /agibot/data/param/protected/x2_31dof_hand_update.urdf (per-robot calibrated URDF) | JointTF/TF only. No evidence that foundationwalk uses it. |
| executor_bind_cpus, RT priorities | removed by SimConfigAdapter | [4,5,6], SCHED priorities | Timing only (see H3). |

## 4. Can the robot params be dropped into the sim MC? (task 3, assessed but not tried)
Not as a whole tree.
- ROBOT action_setting.yaml references runners that the x86 v1.0.0 lib directory does not contain. `ls lib | grep runner` has no `upper_body_external` and no `rl_bmimic_step`, and no lib contains the strings `upper_body_external` or `bmimic_step`.
- HEAD_ONLY, UPPERBODY_REMOTE_SPLIT and BMIMIC_STEP (rl_bmimic_step) would therefore probably fail when the runners are created. This is a hypothesis: whether the MC fails at Init or lazily was not verified.
- Schema drift is confirmed in cpgtelecon. The v1.0.0 lib reads the key `min_waist_ptch` (typo), while ROBOT has `min_waist_pitch`. With rfl-based parsing, a missing required field is likely a parse error. This is a hypothesis, and it affects STAND_DEFAULT, which the twin needs before LD.
- ROBOT has no mc_sim.yaml for SIM mode (ROBOT/mc_sim.yaml exists but differs from SIM's). ROBOT mc.yaml uses `${MC_ENV_PATH}/../urdf/...` and `/agibot/data/param/protected/...` paths, plus REAL-mode env (AGIBOT_PROJECT_ID / AGIBOT_HARDWARE_VERSION).
- A cherry-pick of the locomotion files is feasible but pointless: foundationwalk_config (the extra `urdf_filename` key is either ignored or rejected by the v1.0.0 runner; this is UNKNOWN, so the safe choice is to leave the sim file as is), speed_state and the onnx are already identical. The only meaningful candidate is `InputManager.expiration_time: 200` in mc_sim.yaml, and only if gap behaviour matters.

## 5. Ranked hypotheses for the sim vs real mismatch (task 4)
Symptoms:
- Forward speed: sim about 2.2x command at 0.2 m/s; real 1.9–2.8x.
- Yaw 0.5 rad/s is weak and asymmetric in both. Sim: +51.7° / −19.0°. Real: +16.6° and +85.7° / −6.6°.
- Yaw 0.6 rad/s: sim is symmetric (+148.9° / −152.4°). Real is asymmetric: left 0.29 rad/s, right 0.92 rad/s.

H1 (most likely): the forward over-speed and the poor in-place turn at 0.5 come from the policy/controller itself (foundationwalk_v1 plus the runner's command handling). They do not come from a parameter mismatch.
- For: on the LD path the policy, PD gains, scales, thresholds and speed clips are identical. The sim reproduces both effects qualitatively. The vendor docs say "The current MC gait does not guarantee starting an in-place turn using angular velocity alone".
- Against: none found. Root cause: policy behaviour. Proximate cause: identical commands produce large forward velocity and stepping that is marginal at |wz| 0.5.

H2: the remaining quantitative and asymmetry mismatch, especially the real left/right asymmetry at 0.6, comes from physical differences that are outside mc_param:
- per-robot joint offsets and calibrated URDF (model_path_update, not used in sim)
- IMU mounting bias on the real unit
- MuJoCo model vs real mass, friction and actuators
- floor contact
For: params are identical, yet the sim is symmetric at 0.6 and the real robot is not. Unverified.

H3: MC binary version. Sim is v1.0.0-20260522; robot is v1.1.0-r9-rc9. The runner code differs (the robot reads `urdf_filename`, renames `min_waist_pitch`, has new runners). The command-shaping code in foundationwalk (run_scale use, yaw thresholds, filter) may have changed even though the YAML is the same. This cannot be verified without the robot binary; no aarch64 lib is in the snapshot.

H4: sim timing. The container MC logs `leg_joint_command worker missed wakeup ... miss 12.6%` with no RT scheduling. This could affect gait and yaw, but the size of the effect is unknown.

H5 (low): InputManager expiration_time and check_fall differences. These only matter for command gaps or detected falls.

## 6. Recommendation
- Do not port robot params into the sim MC. On the LD path the locomotion parameters are already identical (same policy, gains, scales, thresholds, speed clip). A whole-tree drop-in would probably break the x86 v1.0.0 binary (missing runners, renamed key).
- Minimal optional twin change: add `expiration_time: 200` under MotionControlModule.InputManager in mc_sim.yaml, for command-timeout parity. It does not change the forward-gain or yaw findings.
- Treat the sim's forward gain (about 2.2x) and the 0.5 vs 0.6 yaw behaviour as policy properties. The twin is valid for comparing alternatives qualitatively.
- To close the quantitative gap, use repeated runs (n≥5 per condition) in both sim and real, and calibrate the residual with per-axis gain/deadband/asymmetry terms from the real bags (twin v0 plan). Do not expect a param change to do this.
- If a vendor x86 build of MC v1.1.0-r9-rc9 can be obtained, that is the only way to remove H3.

## 7. Open questions / next diagnostics (read-only)
- Robot MC binary (aarch64 libmc_runner_rl_foundationwalk.so v1.1.0) for a strings/symbol diff against x86 v1.0.0. This needs PC1 access approved by the user.
- The v1.0.0 default for InputManager.expiration_time. Next step: `strings -n4 lib/libmc_manager_input.so | grep -i -C3 expiration`, or check the MC startup log for printed params.
- Whether the sim MC log (bin/log/mc.log) prints the parsed foundationwalk Option. Next step: `grep -i -A40 foundationwalk ~/qazbot-twin/third_party/official/mc/mc-x86-v1.0.0-20260522/bin/log/mc.log*`.
- Repeat the sim measurements (n≥5): fwd 0.1/0.2/0.3 and yaw ±0.4/0.5/0.6, with RT priority or idle CPU, to bound H4.
