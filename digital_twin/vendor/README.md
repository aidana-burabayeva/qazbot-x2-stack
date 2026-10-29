# vendor/ — external AgiBot artifacts (not QazBot code, never edited, not committed)
Populate with `make vendor` (from the four zip archives, sha256-verified against `manifest.sha256`).
| Dir | What | Source archive |
|---|---|---|
| `aimdk_sdk/` | AimDK SDK 1.1.0 (`src/` interface definitions + examples, `docs/`) | aimdk-sdk-v1_1_0-…-artifacts.zip |
| `mc/` | official motion controller, x86, **v1.0.0-20260522** | mc-x86-v1.0.0-20260522.zip |
| `sim/` | official MuJoCo simulator, x86, v1.0.0-20260522, robot `lx2501_3_t2d5` | sim_mujoco-x86-v1.0.0-20260522.zip |
| `rl_deploy/` | RL deployment example (reference) | aimdk-extra-x2_rl_deploy-v1.1.zip |
| `pc1_mc/` | snapshot of the robot's mc_param **v1.1.0-r9-rc3** and `pc1_metadata.yaml` (mc **v1.1.0-r9-rc9**), read-only | robot PC1 |
| `robot66/` | factory-calibration extract (LiDAR/IMU extrinsics), read-only | robot .66 |
The robot runs a different MC version (v1.1.0-r9-rc9) than the sim (v1.0.0): see docs/model_fidelity.md.
Licence: AgiBot artifacts — do not push images or archives containing them to public registries. The images built here do not contain the mc/sim binaries (they are mounted).
| `pc1_mc/mc_r9-rc9_aarch64/`, `common_aarch64/`, `syslibs_aarch64/`, `ros_humble_aarch64/` | the robot's MC **v1.1.0-r9-rc9** (aarch64) with its runtime (`common` aimrte v0.9.2.14, system and ROS humble libs), read-only copies from PC1 | robot PC1 (soc0) |
| `pc1_mc/sysroot_ubuntu2204_arm64/` | arm64 Ubuntu 22.04 userland (libc), extracted from the official image; used as `QEMU_LD_PREFIX` | ubuntu:22.04 (arm64) |
Run only inside a container with `qemu-user-static` (`QEMU_LD_PREFIX`, `QEMU_SET_ENV=LD_LIBRARY_PATH=...`): no binfmt on the host. Smoke test 2026-10-27: `aima-mc-app-main --help` runs.

r9 MC cannot start here: its aimrte needs a device-bound licence (DecryptFailed). Decision 2026-10-27: no licence copies kept and no host-identity spoofing; the twin stays on the official x86 v1.0.0 mc. The r9 wiring (TWIN_MC=r9) stays for the day AgiBot provides a sim licence.
