# ISSUE (open): FAST-LIO position drifts ~0.2 m after the robot stops

Registered 2026-11-05, not being worked on now (operator decision: no SLAM optimisation, no FAST-LIO/Trust/AMCL changes).
Observation (P0 3 m Goal, bag `docs/evidence/p0_goal3m_profile_20261105`, report `docs/results/20261105_p0_goal3m_profile.md`): after the command went to zero (+7.0 s) and MC reached
STABLE (+10.1 s), `/qazbot_lio/odom_level` position moved 0.21 m between +10 s and +13 s (y -3.462 -> -3.255, odom yaw -0.09 rad over the stop) while `/aima/mc/leg_odometry` moved 0.00 m.
Earlier measurement: stationary speed sd 0.01-0.02 m/s (before motion). Effect: final-pose checks and goal checker use map->odom->base built on this odometry; a 0.2 m error is inside
the 0.30 m production tolerance but not negligible. Not yet separated: corridor/degeneracy, gait vibration after stop, lever arm. Data to collect when worked: LIO vs leg odom at standstill after several stops.
