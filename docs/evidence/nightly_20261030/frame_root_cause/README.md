# Root cause of the Nav2-map vs sim-world discrepancy (up to 0.7 m)

**Contract A (world.py `map_to_sim`)**: walls are placed assuming the pelvis spawns at sim (0,0) with yaw 0 (`map = start ⊕ sim`).
**Contract B (`geometry.FrameAligner`, used only by `HalPlantNode`)**: the map pose is `start ⊕ (first_sample⁻¹ ⊕ sample)`, i.e. anchored at the **first HAL sample the plant happens to receive**.
A = B only if the first sample is the spawn pose. Measured on the real MuJoCo sim (`probe_output.txt`): the robot spawns at (0,0,0) and *settles at yaw ≈ +0.108 rad, xy ≈ (+0.04,-0.015)* within ~1 s of the stand-up transient.
If the plant's first sample arrives after that, B anchors on the settled pose, A does not -> the nav frame is rotated by ≈ -0.1 rad about the start relative to the walls (run-dependent: -0.104 / -0.089 / -0.003 rad in Codex's three audits; 0.69 / 0.73 / 0.08 m at the end point).
Earlier Codex data (`../codex_stall_ab/*_frame_audit.json`) already showed the symptom; this probe shows the mechanism.

Fix direction (minimal, no offsets, no tuning): anchor on the **nominal spawn pose** (identity in the sim world) so that `map = start ⊕ world_pose` is exactly contract A; keep first-sample anchoring only as an explicit option.
Not proven by this probe: whether the real robot shows the same transient (irrelevant for the robot: there AMCL/T5 owns map->odom).
Observation: the first container start in this probe shows the robot collapsing (z 0.077) before a re-spawn at t=16.8 s — separate sim start-up issue, not investigated here.
