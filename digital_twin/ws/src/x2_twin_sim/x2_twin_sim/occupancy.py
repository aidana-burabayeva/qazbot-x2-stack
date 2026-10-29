"""Occupancy-map distance field and a 2-D ray-cast laser scan (no ROS imports).

Semantics follow Nav2 map files and the navigation stack's ``loc_trust.OccGrid``: a cell is occupied when its
probability exceeds ``occupied_thresh``; unknown cells are NOT obstacles.
"""
import math
import os
from typing import Sequence

import numpy as np
import yaml
from PIL import Image
from scipy.ndimage import distance_transform_edt


class OccupancyMap:
    def __init__(self, occupied: np.ndarray, resolution: float, origin_xy: Sequence[float]):
        self.res = float(resolution)
        self.ox, self.oy = float(origin_xy[0]), float(origin_xy[1])
        occ = np.asarray(occupied, bool)
        self.h, self.w = occ.shape
        self.dist = distance_transform_edt(~occ) * self.res if occ.any() else np.full(occ.shape, np.inf)

    @classmethod
    def from_yaml(cls, path: str) -> 'OccupancyMap':
        with open(path) as f:
            meta = yaml.safe_load(f)
        img = np.asarray(Image.open(os.path.join(os.path.dirname(path), meta['image'])).convert('L'), float)
        prob = img / 255.0 if meta.get('negate', 0) else (255.0 - img) / 255.0
        return cls(np.flipud(prob) > float(meta.get('occupied_thresh', 0.65)), meta['resolution'], meta['origin'][:2])

    def dist_at(self, pts: np.ndarray) -> np.ndarray:
        """Distance (m) to the nearest occupied cell for map-frame points (inf outside the map)."""
        i = np.floor((pts[:, 1] - self.oy) / self.res).astype(int)
        j = np.floor((pts[:, 0] - self.ox) / self.res).astype(int)
        ok = (i >= 0) & (i < self.h) & (j >= 0) & (j < self.w)
        d = np.full(len(pts), np.inf)
        d[ok] = self.dist[i[ok], j[ok]]
        return d


class RayCaster:
    """Ideal planar lidar: first occupied cell along each ray.

    The scan has ``n_beams`` bins over the full circle (like the robot's ``/qazbot_lio/scan``, where unobserved bins are inf);
    only bins within ``fov_deg`` around the forward axis are cast, the rest stay inf (no return, NOT 'free').
    """

    def __init__(self, occ_map: OccupancyMap, n_beams: int = 360, range_min: float = 0.05, range_max: float = 8.0,
                 step: float = 0.02, hit_distance: float = 0.025, fov_deg: float = 360.0):
        self.map, self.n, self.rmin, self.rmax, self.hit = occ_map, int(n_beams), range_min, range_max, hit_distance
        self.angle_min = -math.pi
        self.angle_increment = 2.0 * math.pi / self.n
        self.steps = np.arange(range_min, range_max, step)
        a = self.angle_min + self.angle_increment * np.arange(self.n)
        self.observed = np.flatnonzero(np.abs(a) <= math.radians(fov_deg) / 2.0 + 1e-9)     # bins inside the field of view
        a = a[self.observed]
        self._xy = np.stack([np.outer(np.cos(a), self.steps), np.outer(np.sin(a), self.steps)], -1).reshape(-1, 2)

    def scan(self, pose, discs: Sequence[Sequence[float]] = ()) -> np.ndarray:
        """Ranges (m, inf for no return) for a sensor at map pose (x, y, yaw).

        ``discs`` are transient circular objects (x, y, radius) in the map frame that are NOT in the map (a person, a box that is
        later removed); they return like walls. Used to create and remove real obstacles at run time.
        """
        x, y, th = pose
        c, s = math.cos(th), math.sin(th)
        pts = np.column_stack((x + c * self._xy[:, 0] - s * self._xy[:, 1], y + s * self._xy[:, 0] + c * self._xy[:, 1]))
        hit = (self.map.dist_at(pts) <= self.hit).reshape(len(self.observed), len(self.steps))
        ranges = np.full(self.n, np.inf)
        ranges[self.observed] = np.where(hit.any(1), self.steps[hit.argmax(1)], np.inf)
        for dx, dy, rad in discs:
            ranges = np.minimum(ranges, self._disc_ranges(pose, dx, dy, rad))
        return ranges

    def _disc_ranges(self, pose, cx: float, cy: float, rad: float) -> np.ndarray:
        """First intersection (m) of each observed ray with a disc; inf elsewhere."""
        x, y, th = pose
        ang = th + self.angle_min + self.angle_increment * self.observed
        ux, uy = np.cos(ang), np.sin(ang)
        fx, fy = x - cx, y - cy
        b = fx * ux + fy * uy
        disc = b * b - (fx * fx + fy * fy - rad * rad)
        t = np.where(disc >= 0.0, -b - np.sqrt(np.maximum(disc, 0.0)), np.inf)
        out = np.full(self.n, np.inf)
        out[self.observed] = np.where((t >= self.rmin) & (t <= self.rmax), t, np.inf)
        return out
