"""Pure SE(2) helpers (no ROS imports): REP-103 conventions, x forward, y left, yaw counter-clockwise."""
import math
from dataclasses import dataclass
from typing import Optional, Tuple

Pose2D = Tuple[float, float, float]


def wrap_angle(a: float) -> float:
    """Wrap to (-pi, pi]."""
    return math.atan2(math.sin(a), math.cos(a))


def planar_pose(px: float, py: float, qx: float, qy: float, qz: float, qw: float) -> Pose2D:
    """Position and heading of a body whose x axis is its forward axis.

    The heading is the direction of the body x axis projected onto the floor plane; it stays well defined while the
    body is upright and tilts, unlike a naive ``2*atan2(qz, qw)``.
    """
    r00 = 1.0 - 2.0 * (qy * qy + qz * qz)
    r10 = 2.0 * (qx * qy + qz * qw)
    return px, py, math.atan2(r10, r00)


def compose(a: Pose2D, b: Pose2D) -> Pose2D:
    """a (+) b."""
    c, s = math.cos(a[2]), math.sin(a[2])
    return a[0] + c * b[0] - s * b[1], a[1] + s * b[0] + c * b[1], wrap_angle(a[2] + b[2])


def inverse(a: Pose2D) -> Pose2D:
    c, s = math.cos(a[2]), math.sin(a[2])
    return -(c * a[0] + s * a[1]), s * a[0] - c * a[1], wrap_angle(-a[2])


@dataclass
class FrameAligner:
    """Map world poses using a nominal spawn or first-sample anchor.

    With ``anchor='nominal'``, map = start (+) world pose: the spawn pose is
    the world origin, matching wall placement. The default ``'first_sample'``
    instead maps the first received pose to ``start`` and preserves relative motion.
    """

    start: Pose2D
    anchor: str = 'first_sample'
    _origin_inv: Optional[Pose2D] = None

    def __post_init__(self) -> None:
        if self.anchor not in ('first_sample', 'nominal'):
            raise ValueError(f'Unknown frame anchor: {self.anchor!r}')
        if self.anchor == 'nominal':
            self._origin_inv = (0.0, 0.0, 0.0)

    def align(self, world: Pose2D) -> Pose2D:
        if self._origin_inv is None:
            self._origin_inv = inverse(world)
        return compose(self.start, compose(self._origin_inv, world))

    @property
    def initialised(self) -> bool:
        return self._origin_inv is not None
