# Digital twin vs installed X2: parity status and intentional differences

Source of truth for the robot: `origin/robot/stable` (initially `baseline/x2-robot-20261030` @ fac640f; 73/73 tracked `ws/src` files match the installed SHA256 — `tools/twin/verify_robot_baseline.py`).

## Parity established (static, 2026-10-31)
| item | status |
|---|---|
| Installed backend `ws/src` (73 files) | lab tree now identical for the three drifted files + D1 map: `x2_motion_adapter.py` (**P23**), `navigate_to_pose.xml`, `localization.launch.py`; `verify_robot_baseline.py` -> 73/73 |
| Map | sim `main_office` map.pgm sha256 `72ee4094…` == robot runtime `d1_qazbot_20261029/map.pgm`; map.yaml identical (origin -30.0103,-22.4092, 0.05 m) |
| AMCL parameters in the twin launch | aligned to the robot (max_beams 180, alpha 0.3/0.2/0.15/0.15/0.15, update_min_a 0.05) in `x2_twin_bringup/launch/amcl.launch.py`; **live values inside the running sim NOT yet compared** (needs a sim run) -> UNVERIFIED |
| Motion adapter | **was P20 in all earlier L2 work, the robot runs P23** (yaw_hold_s, pause instead of reset, yaw_w_min 0.15). All L2 stall conclusions (H2 windup etc.) were made on the older adapter: they must be re-examined on P23. `tests/motion_adapter` updated to the P23 contract |
| Codex sim-only `QAZBOT_SIM_YAW_LIN_EPS` fixture | removed from the adapter (A/B closed); adapter is byte-identical to the robot's |
| Nav2 versions | robot: nav2 1.1.18 (runtime packages); twin image Nav2 1.1.20 — **difference, UNVERIFIED impact** |
| Docker image | twin `x2-twin-stack:p20` bakes the older twin packages; the frame fix and AMCL launch change need an image rebuild or the overlay mount used in the A/B (`frame_ab.sh`) |

## Intentional differences (contract)
1. Truth localization in L2 runs (map->odom = identity) instead of AMCL+T5; AMCL runs only in explicit AMCL scenarios.
2. MuJoCo/official MC plant instead of the real MC: velocity saturation (~0.45 m/s on the robot), coast and yaw response must be compared (FID-02, real data in `docs/evidence/robot_20261030`).
3. Ideal sensor timing in the twin; robot shows scan age p95 0.6–1.9 s under load and map->odom stamped +0.95 s ahead (FID-03).
4. Plant frame: `HalPlantNode` anchors on the nominal spawn (contract of `world.py`), fixed in lab (A/B: 0.03–0.155 m / ≤0.13 rad -> 0.000).
5. The robot's open P0 blockers (not reproduced yet): AMCL estimate jumping up to 1.72 m / 35.7° after `initialpose` (13 episodes in `docs/evidence/x2_twin_20261030/snapshot/initialpose_episodes.json`).
