"""Pure application logic for the offline controller comparison (no ROS imports; unit-tested in test_sim_model.py).

RobotModel   unicycle integrating the smoothed cmd_vel; mode 'ideal' (cmd == motion) or 'mc_hyp'.
             'mc_hyp' is a HYPOTHESIS model of the X2 MC response from report 20261102-d05-D1-debugger.md:
               * dead time 0.4 s (x-corr cmd vs speed, LIVE-2),
               * forward speed v = 0.34 + 0.53*cmd for |cmd| >= 0.02 (2-point line through R=1.61 @ 0.30 and
                 ~0.96 @ 0.80; the report says explicitly "NOT a model", shape between/below unknown),
               * first-order lag tau_v 0.25 s (reaches ~0.41 m/s 0.9 s after a step, report: 0.39),
                 tau_w 0.15 s, wz passed 1:1 (H9: pure turns are closed-loop in the adapter, P20/P23),
               * cmd older than 0.15 s -> zero (adapter cmd_timeout).
             Native-odom under-read (D-06) is NOT modelled: odom = ground truth in both modes.
World        occupancy grid (D1 map) + dynamic boxes; 2D ray casting for the synthetic forward LiDAR.
geometry     footprint transform, signed polygon clearance, path cross-track / arc length.
"""
import math
from collections import deque

import numpy as np

FOOTPRINT = np.array([[0.17, 0.229], [0.008, 0.34], [-0.182, 0.34], [-0.305, 0.229],
                      [-0.305, -0.261], [-0.182, -0.372], [0.008, -0.372], [0.17, -0.261]])

# Synthetic sensor = live chest LiDAR as characterised in 20261103-d04b-debugger.md:
# in-band horizontal FOV +-54 deg, marking only r >= 0.45 m (level_bridge), costmap marking <= 3.0, clearing <= 3.5.
LIDAR_FOV_DEG = 54.0
LIDAR_STEP_DEG = 1.0
LIDAR_MIN_MARK = 0.45
LIDAR_MAX_MARK = 3.0
LIDAR_MAX_RANGE = 3.5
import os as _os
# ws/src/lio/level_bridge.py clearing_endpoints default fov (live A); CMP_CLEAR_FOV="-54,54" = D-04 candidate C
CLEAR_FOV_DEG = tuple(float(v) for v in _os.environ.get('CMP_CLEAR_FOV', '-46.0,44.5').split(','))


class RobotModel:
    def __init__(self, mode, x, y, yaw, cmd_timeout=0.15, dead_time=0.4, tau_v=0.25, tau_w=0.15):
        if mode not in ('ideal', 'mc_hyp', 'mc_lin'):
            raise ValueError(mode)
        self.mode = mode
        self.x, self.y, self.yaw = x, y, yaw
        self.v = self.w = 0.0
        self.cmd_timeout = cmd_timeout
        self.dead_time = dead_time
        self.tau_v, self.tau_w = tau_v, tau_w
        self.queue = deque()   # (t, vx_target, wz_target) for the dead time
        self.target = (0.0, 0.0)

    @staticmethod
    def mc_forward(cmd_vx):
        """HYPOTHESIS forward characteristic (D-05 2-point line), sign-symmetric, 0 below the adapter eps."""
        a = abs(cmd_vx)
        if a < 0.02:
            return 0.0
        return math.copysign(min(0.34 + 0.53 * a, 0.80), cmd_vx)

    def step(self, t, dt, cmd, cmd_stamp):
        vx, wz = cmd if (cmd is not None and t - cmd_stamp <= self.cmd_timeout) else (0.0, 0.0)
        if self.mode == 'ideal':
            self.v, self.w = vx, wz
        else:
            fwd = self.mc_forward(vx) if self.mode == 'mc_hyp' else 1.61 * vx   # mc_lin: constant R=1.61 + lag
            self.queue.append((t, fwd, wz))
            while self.queue and t - self.queue[0][0] >= self.dead_time:
                self.target = self.queue.popleft()[1:]
            tv, tw = self.target
            self.v += (tv - self.v) * min(1.0, dt / self.tau_v)
            self.w += (tw - self.w) * min(1.0, dt / self.tau_w)
        # exact unicycle integration over dt
        if abs(self.w) > 1e-6:
            ny = self.yaw + self.w * dt
            self.x += self.v / self.w * (math.sin(ny) - math.sin(self.yaw))
            self.y -= self.v / self.w * (math.cos(ny) - math.cos(self.yaw))
            self.yaw = math.atan2(math.sin(ny), math.cos(ny))
        else:
            self.x += self.v * dt * math.cos(self.yaw)
            self.y += self.v * dt * math.sin(self.yaw)


def read_pgm(path):
    with open(path, 'rb') as f:
        data = f.read()
    parts = data.split(maxsplit=4)
    if parts[0] != b'P5':
        raise ValueError('only binary P5 pgm supported')
    w, h = int(parts[1]), int(parts[2])
    return np.frombuffer(parts[4][-w * h:], dtype=np.uint8).reshape(h, w)


class World:
    """Occupied grid in map frame (row 0 = top of image, as map_server) + list of axis-aligned dynamic boxes."""

    def __init__(self, occ, origin_xy, res):
        self.occ = occ                     # bool [h, w], image orientation
        self.ox, self.oy = origin_xy
        self.res = res
        self.h, self.w = occ.shape
        self.boxes = {}                    # name -> (cx, cy, hx, hy)
        ys, xs = np.nonzero(occ)
        self.occ_pts = np.stack([self.ox + (xs + 0.5) * res, self.oy + (self.h - 1 - ys + 0.5) * res], axis=1)

    @classmethod
    def from_map_yaml(cls, yaml_path, occupied_thresh=0.65):
        import os
        import yaml
        with open(yaml_path) as f:
            m = yaml.safe_load(f)
        img = read_pgm(os.path.join(os.path.dirname(yaml_path), m['image']))
        occ = img <= int(255 * (1.0 - m.get('occupied_thresh', occupied_thresh)))
        return cls(occ, (m['origin'][0], m['origin'][1]), m['resolution'])

    def occupied(self, x, y):
        ix = np.floor((x - self.ox) / self.res).astype(int)
        iy = self.h - 1 - np.floor((y - self.oy) / self.res).astype(int)
        inside = (ix >= 0) & (ix < self.w) & (iy >= 0) & (iy < self.h)
        out = np.zeros(np.shape(x), dtype=bool)
        out[inside] = self.occ[iy[inside], ix[inside]]
        for cx, cy, hx, hy in self.boxes.values():
            out |= (np.abs(x - cx) <= hx) & (np.abs(y - cy) <= hy)
        return out

    def raycast(self, x, y, yaw, fov_deg=LIDAR_FOV_DEG, step_deg=LIDAR_STEP_DEG, max_range=LIDAR_MAX_RANGE,
                dr=0.02, rng=None, noise=0.0):
        """Returns (bearings_body [rad], ranges [m], hit [bool]). Range = first occupied sample (or max_range)."""
        b = np.radians(np.arange(-fov_deg, fov_deg + 1e-9, step_deg))
        r = np.arange(dr, max_range + 1e-9, dr)
        a = yaw + b[:, None]
        px = x + r[None, :] * np.cos(a)
        py = y + r[None, :] * np.sin(a)
        occ = self.occupied(px, py)
        hit = occ.any(axis=1)
        first = np.argmax(occ, axis=1)
        ranges = np.where(hit, r[first], max_range)
        if rng is not None and noise > 0.0:
            ranges = np.where(hit, ranges + rng.normal(0.0, noise, ranges.shape), ranges)
        return b, ranges, hit

    def obstacle_points(self, x, y, radius):
        """Occupied map cell centres within radius + sampled box outlines (2 cm), for clearance metrics."""
        d = np.hypot(self.occ_pts[:, 0] - x, self.occ_pts[:, 1] - y)
        pts = [self.occ_pts[d < radius]]
        for cx, cy, hx, hy in self.boxes.values():
            pts.append(box_outline(cx, cy, hx, hy))
        return np.concatenate(pts) if pts else np.zeros((0, 2))


def box_outline(cx, cy, hx, hy, ds=0.02):
    xs = np.arange(-hx, hx + 1e-9, ds)
    ys = np.arange(-hy, hy + 1e-9, ds)
    return np.concatenate([
        np.stack([cx + xs, np.full_like(xs, cy - hy)], 1), np.stack([cx + xs, np.full_like(xs, cy + hy)], 1),
        np.stack([np.full_like(ys, cx - hx), cy + ys], 1), np.stack([np.full_like(ys, cx + hx), cy + ys], 1)])


def transform_footprint(x, y, yaw, fp=FOOTPRINT):
    c, s = math.cos(yaw), math.sin(yaw)
    return np.stack([x + c * fp[:, 0] - s * fp[:, 1], y + s * fp[:, 0] + c * fp[:, 1]], axis=1)


def polygon_clearance(poly, pts):
    """Signed min distance from points to polygon outline: negative if any point is inside (contact)."""
    if len(pts) == 0:
        return float('inf')
    a = poly
    b = np.roll(poly, -1, axis=0)
    ab = b - a                                                     # [m, 2]
    ap = pts[:, None, :] - a[None, :, :]                           # [n, m, 2]
    t = np.clip((ap * ab).sum(-1) / (ab * ab).sum(-1), 0.0, 1.0)
    proj = a[None] + t[..., None] * ab[None]
    d = np.hypot(*(pts[:, None, :] - proj).transpose(2, 0, 1)).min(axis=1)
    # point-in-polygon (ray crossing)
    x, y = pts[:, 0:1], pts[:, 1:2]
    ay, by = a[None, :, 1], b[None, :, 1]
    cross = ((ay > y) != (by > y)) & (x < (b[None, :, 0] - a[None, :, 0]) * (y - ay) / (by - ay + 1e-12) + a[None, :, 0])
    inside = cross.sum(axis=1) % 2 == 1
    d = np.where(inside, -d, d)
    return float(d.min())


class PathRef:
    def __init__(self, xy):
        self.p = np.asarray(xy, dtype=float)
        seg = np.diff(self.p, axis=0)
        self.seg = seg
        self.seglen = np.hypot(seg[:, 0], seg[:, 1])
        self.s0 = np.concatenate([[0.0], np.cumsum(self.seglen)])
        self.length = float(self.s0[-1])

    def project(self, x, y):
        """(cross-track distance [m], arc length of the projection [m])."""
        ap = np.array([x, y]) - self.p[:-1]
        t = np.clip((ap * self.seg).sum(1) / np.maximum(self.seglen ** 2, 1e-12), 0.0, 1.0)
        proj = self.p[:-1] + t[:, None] * self.seg
        d = np.hypot(proj[:, 0] - x, proj[:, 1] - y)
        i = int(np.argmin(d))
        return float(d[i]), float(self.s0[i] + t[i] * self.seglen[i])

    def point_at(self, s):
        i = int(np.clip(np.searchsorted(self.s0, s) - 1, 0, len(self.seg) - 1))
        t = (s - self.s0[i]) / max(self.seglen[i], 1e-12)
        return self.p[i] + t * self.seg[i], math.atan2(self.seg[i, 1], self.seg[i, 0])


def sign_flips_per_s(t, w, thresh=0.05):
    """Number of sign changes of w (ignoring |w| <= thresh) per second of the series duration."""
    s = [math.copysign(1, v) for v in w if abs(v) > thresh]
    flips = sum(1 for a, b in zip(s, s[1:]) if a != b)
    dur = (t[-1] - t[0]) if len(t) > 1 else 0.0
    return flips / dur if dur > 0 else 0.0, flips
