# A/B of the nav-frame fix on the real MuJoCo simulator (2026-10-31)

Same stack, same scenarios, same order; only the `x2_twin_sim` package differs: **base** = image `x2-twin-stack:p20` as built, **fixed** = package from `lab/integration` (`FrameAligner(anchor='nominal')`, default in `HalPlantNode`, produced by the Codex engineering attempt `nav_frame_fix`).
Ground truth = raw MuJoCo pelvis pose mapped by the world contract (`map = start (+) world`, the transform that placed the walls). `pose_probe.py` records raw pose and the published nav pose (TF map->odom->base_footprint) at 10 Hz.

| run | base pos/yaw (static anchor error) | fixed |
|---|---|---|
| A static x2 | 0.055 m / -0.033 rad ; 0.059 m / -0.108 rad | 0.000 / 0.000 (both) |
| A CCW walk+turn | 0.040 m / -0.119 rad | 0.003 m / 0.000 |
| A CW walk+turn | 0.155 m / -0.129 rad | 0.000 / 0.000 |
| A, start yaw 1.57: static / CCW | 0.117 m / +0.032 ; 0.076 m / -0.021 | 0.000 / 0.000 ; 0.000 / 0.000 |
| B (other map region) static / CCW | 0.034 m / -0.113 ; 0.105 m / +0.034 | 0.000 / 0.000 ; 0.000 / 0.000 |

Effect at 7 m of path from the yaw anchor error alone: base 0.15–0.90 m (varies per run), fixed 0.00 m (matches Codex's 0.69–0.73 m audits).
Dynamic residual while turning (p95 <= 0.06 rad, max <= 0.16 rad) exists in both variants: sampling skew between the TF pose and the raw odometry, not a frame error.

Not changed by the fix (by design, scope = frame only): NavigateToPose of `A_ccw` is FAIL in base and fixed (known stall, now to be re-examined on the robot's P23 adapter, see `docs/nightly/SIM_ROBOT_DIFFERENCES.md`); `A_cw`, `AY_ccw`, `B_ccw` PASS in both.
Not covered (OPEN): odometry-reset recovery and sensor delay/jitter under the fix; reproduction on the real robot is not needed (AMCL/T5 own map->odom there).
