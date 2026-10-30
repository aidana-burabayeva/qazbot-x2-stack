"""Offline reproduction of the measured HAL/world anchor mismatch.

Measured inputs: docs/evidence/nightly_20261030/frame_root_cause/probe_output.txt.
The world spawns at identity; receiving the settled sample first must not change
the map frame used to place walls. No ROS or simulator processes are needed.
"""
import math

import pytest

from x2_twin_sim.geometry import FrameAligner, compose, inverse, planar_pose
from x2_twin_sim.world import map_to_sim


START = (-7.885, 12.966, 0.0)
SPAWN = (0.0, 0.0, 0.0)
SETTLED = planar_pose(
    0.0455, -0.0174, 0.0, 0.0, math.sin(0.1082 / 2), math.cos(0.1082 / 2)
)


def test_first_sample_anchor_rotates_nav_frame_vs_world_contract():
    """Document current behavior: a late HAL sample rotates navigation vs walls."""
    aligner = FrameAligner(START)
    assert aligner.align(SETTLED) == pytest.approx(START)

    world_pose = compose(SPAWN, (7.0, 0.0, 0.0))
    world_contract_map_pose = compose(START, world_pose)
    assert map_to_sim(world_contract_map_pose[:2], START) == pytest.approx(world_pose[:2])

    nav_pose = aligner.align(world_pose)
    assert nav_pose == pytest.approx(compose(START, compose(inverse(SETTLED), world_pose)))
    error_m = math.hypot(
        nav_pose[0] - world_contract_map_pose[0],
        nav_pose[1] - world_contract_map_pose[1],
    )
    assert error_m >= 0.6

    # The mapped direction of a world-x segment measures the frame rotation,
    # independently of the translation introduced by anchoring at SETTLED.
    nav_spawn = aligner.align(SPAWN)
    implied_rotation = math.atan2(nav_pose[1] - nav_spawn[1], nav_pose[0] - nav_spawn[0])
    assert implied_rotation == pytest.approx(-0.1082, abs=1e-6)
    assert nav_pose[2] - world_contract_map_pose[2] == pytest.approx(implied_rotation)


@pytest.mark.xfail(strict=True, reason='fix pending: nominal anchor not implemented')
def test_nominal_anchor_matches_world_contract():
    """Nominal anchoring must agree with wall placement regardless of sample order."""
    # Include a rotated start to check the complete SE(2) wall contract as well
    # as the measured run. The first sample is deliberately the settled pose.
    for start in (START, (START[0], START[1], 0.7)):
        aligner = FrameAligner(start, anchor='nominal')
        for world_pose in (SETTLED, SPAWN, (7.0, 0.0, 0.0), (-2.0, 3.0, -0.4), (1.0, -4.0, 2.3)):
            map_pose = aligner.align(world_pose)
            assert map_pose == pytest.approx(compose(start, world_pose))
            assert map_to_sim(map_pose[:2], start) == pytest.approx(world_pose[:2])
            assert compose(inverse(start), map_pose) == pytest.approx(world_pose)
