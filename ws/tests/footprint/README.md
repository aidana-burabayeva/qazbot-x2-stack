# X2 footprint evidence (2026-10-21)

Question: why a 0.65 m circle, and what is justified? Short answer: 0.65 m has no verifiable physical source.
It was derived as hypot(0.35+0.0915, 0.45+0.0157) from constants ("body extent 0.35", "arm swing 0.45") quoted from code
on .89 that could not be inspected. It lies between the arms-at-sides envelope (0.34-0.39 m) and one arm stretched
anywhere in its joint range (0.77 m). It also equals the vendor planner's *soft* `inflation_radius` (0.65), which is a
cost-gradient radius, not a body radius.

| Source | Value about the control point (LIO IMU projection, +0.0867/+0.0173 m from base_link) |
|---|---|
| Vendor footprint polygon, `a2/app2.yaml` (used by this robot's own planner), half-extents 0.20 x 0.336 | circumradius 0.389 m |
| X2 Ultra URDF v1.3.0 zero pose (arms at sides), FK, collision primitives/meshes | 0.370 m (|y| 0.333) |
| Same, body without arms | 0.269 m (|y| 0.202) |
| Measured at rest standing (joint states + FK, `joint_record.py`, `envelope.py`) | 0.343 m (x -0.189..0.223, |y| 0.309) |
| One arm anywhere in joint range (1500 random configs) | 0.769 m (upper bound, not a walking pose) |
| Vendor paddings | global 0.20, ego 0.10, hybrid 0.10, local 0.05; inflation_radius 0.65 soft, cost_scaling 8 |

NOT yet measured: the swept envelope while walking (arm swing, foot placement, sway). Plan: record joint states
during a short PS5 walk (`joint_record.py` on the robot, then `envelope.py`) - no tape measure needed.
Tape-measure fallback (standing, neutral walking pose, from the lidar IMU axis): farthest forward point, farthest backward
point, left and right extreme (hands included), all projected on the floor.

Scripts: `fk.py` (URDF forward kinematics), `envelope.py` (swept envelope from a joint recording),
`joint_record.py` (passive recorder; needs AMENT_PREFIX_PATH and PYTHONPATH for /agibot/software/common),
`cell_cloud_probe.py` (X2 cloud around a map cell, needs the localization stack).
