"""Python emulation of the Nav2 Humble (nav2_costmap_2d 1.1.18) ObstacleLayer
pieces that decide which cells are LETHAL in the X2 local costmap, plus the
RPP (1.1.18) isCollisionImminent check.  Offline tooling only (D-04 P3).

Fidelity notes (see report §P3-fidelity):
* raytraceLine / bresenham2D / MarkCell: transcribed from the installed header
  /opt/ros/humble/include/nav2_costmap_2d/nav2_costmap_2d/costmap_2d.hpp
  (endpoint cell is acted on: `at(offset)` after the loop).
* ObstacleLayer::updateBounds order (clear all clearing observations, then
  mark all marking observations), raytraceFreespace map-edge clipping,
  3D squared-distance marking test, per-source height filter in
  ObservationBuffer::bufferCloud (min <= z <= max in the global frame),
  layer max_obstacle_height, CostmapLayer rolling updateOrigin (int truncation)
  and footprint clearing (setConvexPolygonCost FREE on the padded footprint)
  are from the 1.1.x sources (.cpp not on disk -> recalled, not transcribed).
* Observation persistence 0: one marking + one clearing observation per update.
"""
import math

import numpy as np

FREE, LETHAL = 0, 254


def pad_footprint(fp, padding):
    """nav2_costmap_2d::padFootprint: x += sign(x)*pad, y += sign(y)*pad."""
    fp = np.asarray(fp, float)
    return fp + np.sign(fp) * padding


def bresenham_cells(x0, y0, x1, y1, max_length=None, min_length=0):
    """Vectorised Costmap2D::raytraceLine (header transcription).

    x0, y0: scalar start cell; x1, y1: int arrays of end cells.
    Returns (cells_x, cells_y) of every cell the MarkCell action touches.
    """
    x1 = np.asarray(x1, np.int64)
    y1 = np.asarray(y1, np.int64)
    dxf, dyf = x1 - x0, y1 - y0
    dist = np.hypot(dxf, dyf)
    keep = ~(dist < min_length)
    x1, y1, dxf, dyf, dist = x1[keep], y1[keep], dxf[keep], dyf[keep], dist[keep]
    if len(x1) == 0:
        return np.empty(0, np.int64), np.empty(0, np.int64)
    with np.errstate(divide='ignore', invalid='ignore'):
        mx0 = np.where(dist > 0, (x0 + dxf / dist * min_length), x0).astype(np.int64)
        my0 = np.where(dist > 0, (y0 + dyf / dist * min_length), y0).astype(np.int64)
    dx, dy = x1 - mx0, y1 - my0
    adx, ady = np.abs(dx), np.abs(dy)
    sx = np.where(dx > 0, 1, -1)
    sy = np.where(dy > 0, 1, -1)
    ml = np.inf if max_length is None else max_length
    with np.errstate(divide='ignore', invalid='ignore'):
        scale = np.where(dist == 0, 1.0, np.minimum(1.0, ml / dist))
    xdom = adx >= ady
    da = np.where(xdom, adx, ady)
    db = np.where(xdom, ady, adx)
    end = np.minimum((scale * da).astype(np.int64), da)
    err = (da // 2).astype(np.int64)
    cx, cy = mx0.copy(), my0.copy()
    outx, outy = [], []
    for i in range(int(end.max()) if len(end) else 0):
        act = i < end
        outx.append(cx[act])
        outy.append(cy[act])
        # offset += offset_a
        cx = np.where(act & xdom, cx + sx, cx)
        cy = np.where(act & ~xdom, cy + sy, cy)
        err = np.where(act, err + db, err)
        step_b = act & (err >= da)
        cy = np.where(step_b & xdom, cy + sy, cy)
        cx = np.where(step_b & ~xdom, cx + sx, cx)
        err = np.where(step_b, err - da, err)
    outx.append(cx)
    outy.append(cy)          # final at(offset) for every ray
    return np.concatenate(outx), np.concatenate(outy)


class Observation:
    def __init__(self, points, origin, obstacle_max_range=3.0, obstacle_min_range=0.0,
                 raytrace_max_range=3.5, raytrace_min_range=0.0,
                 min_obstacle_height=0.0, max_obstacle_height=0.0):
        """points: Nx3 in the global frame; origin: sensor origin (x, y, z) global."""
        p = np.asarray(points, float).reshape(-1, 3)
        # ObservationBuffer::bufferCloud height filter (global frame z)
        p = p[(p[:, 2] <= max_obstacle_height) & (p[:, 2] >= min_obstacle_height)]
        self.points = p
        self.origin = np.asarray(origin, float)
        self.obstacle_max_range = obstacle_max_range
        self.obstacle_min_range = obstacle_min_range
        self.raytrace_max_range = raytrace_max_range
        self.raytrace_min_range = raytrace_min_range


class ObstacleLayer:
    """Rolling-window ObstacleLayer (track_unknown_space false, combination max)."""

    def __init__(self, size_m=4.0, res=0.05, max_obstacle_height=2.0,
                 footprint=None, footprint_padding=0.01, footprint_clearing=True):
        self.res = res
        self.n = int(round(size_m / res))
        self.g = np.zeros((self.n, self.n), np.uint8)     # [my, mx]
        self.ox = self.oy = None
        self.max_obstacle_height = max_obstacle_height
        self.fp = None if footprint is None else pad_footprint(footprint, footprint_padding)
        self.footprint_clearing = footprint_clearing

    # --- Costmap2D geometry -------------------------------------------------
    def world_to_map(self, wx, wy):
        mx = np.floor((np.asarray(wx) - self.ox) / self.res).astype(np.int64)
        my = np.floor((np.asarray(wy) - self.oy) / self.res).astype(np.int64)
        ok = (np.asarray(wx) >= self.ox) & (np.asarray(wy) >= self.oy) & (mx < self.n) & (my < self.n)
        return mx, my, ok

    def cell_distance(self, d):
        return int(max(0.0, math.ceil(d / self.res)))

    def update_origin(self, rx, ry):
        nox, noy = rx - self.n * self.res / 2, ry - self.n * self.res / 2
        if self.ox is None:
            self.ox, self.oy = nox, noy
            return
        cx = int((nox - self.ox) / self.res)
        cy = int((noy - self.oy) / self.res)
        if cx == 0 and cy == 0:
            return
        g = np.zeros_like(self.g)
        n = self.n
        sx0, sx1 = max(0, cx), min(n, n + cx)
        sy0, sy1 = max(0, cy), min(n, n + cy)
        if sx0 < sx1 and sy0 < sy1:
            g[sy0 - cy:sy1 - cy, sx0 - cx:sx1 - cx] = self.g[sy0:sy1, sx0:sx1]
        self.g = g
        self.ox += cx * self.res
        self.oy += cy * self.res

    # --- ObstacleLayer ------------------------------------------------------
    def raytrace_freespace(self, obs):
        ox, oy = obs.origin[0], obs.origin[1]
        x0, y0, ok0 = self.world_to_map(ox, oy)
        if not ok0:
            return
        p = obs.points
        if len(p) == 0:
            return
        wx, wy = p[:, 0].copy(), p[:, 1].copy()
        a, b = wx - ox, wy - oy
        lo_x, lo_y = self.ox, self.oy
        hi_x, hi_y = self.ox + self.n * self.res, self.oy + self.n * self.res
        with np.errstate(divide='ignore', invalid='ignore'):
            m = wx < lo_x
            t = (lo_x - ox) / a[m]; wx[m] = lo_x; wy[m] = oy + b[m] * t
            m = wy < lo_y
            t = (lo_y - oy) / b[m]; wx[m] = ox + a[m] * t; wy[m] = lo_y
            m = wx > hi_x
            t = (hi_x - ox) / a[m]; wx[m] = hi_x - .001; wy[m] = oy + b[m] * t
            m = wy > hi_y
            t = (hi_y - oy) / b[m]; wx[m] = ox + a[m] * t; wy[m] = hi_y - .001
        x1, y1, ok = self.world_to_map(wx, wy)
        cx, cy = bresenham_cells(int(x0), int(y0), x1[ok], y1[ok],
                                 self.cell_distance(obs.raytrace_max_range),
                                 self.cell_distance(obs.raytrace_min_range))
        inb = (cx >= 0) & (cy >= 0) & (cx < self.n) & (cy < self.n)
        self.g[cy[inb], cx[inb]] = FREE

    def mark(self, obs):
        p = obs.points
        if len(p) == 0:
            return
        p = p[p[:, 2] <= self.max_obstacle_height]
        d2 = ((p - obs.origin) ** 2).sum(axis=1)
        p = p[(d2 < obs.obstacle_max_range ** 2) & (d2 >= obs.obstacle_min_range ** 2)]
        mx, my, ok = self.world_to_map(p[:, 0], p[:, 1])
        self.g[my[ok], mx[ok]] = LETHAL

    def footprint_cells(self, pose):
        """Costmap2D::setConvexPolygonCost cells (outline + convex fill)."""
        x, y, th = pose
        c, s = math.cos(th), math.sin(th)
        poly = np.array([[x + c * px - s * py, y + s * px + c * py] for px, py in self.fp])
        mx, my, ok = self.world_to_map(poly[:, 0], poly[:, 1])
        if not ok.all():
            return np.empty(0, np.int64), np.empty(0, np.int64)
        xs, ys = [], []
        for i in range(len(mx)):
            j = (i + 1) % len(mx)
            ax, ay = bresenham_cells(int(mx[i]), int(my[i]), np.array([mx[j]]), np.array([my[j]]))
            xs.append(ax); ys.append(ay)
        xs, ys = np.concatenate(xs), np.concatenate(ys)
        fx, fy = [], []
        for cx in np.unique(xs):
            col = ys[xs == cx]
            yy = np.arange(col.min(), col.max() + 1)
            fx.append(np.full(len(yy), cx)); fy.append(yy)
        return np.concatenate(fx), np.concatenate(fy)

    def update(self, pose, marking=(), clearing=()):
        """One Costmap2DROS::updateMap cycle for this layer."""
        self.update_origin(pose[0], pose[1])
        for o in clearing:
            self.raytrace_freespace(o)
        for o in marking:
            self.mark(o)
        if self.footprint_clearing and self.fp is not None:
            fx, fy = self.footprint_cells(pose)
            self.g[fy, fx] = FREE

    # --- helpers ------------------------------------------------------------
    def lethal_world(self):
        my, mx = np.nonzero(self.g == LETHAL)
        return np.column_stack((self.ox + (mx + 0.5) * self.res, self.oy + (my + 0.5) * self.res))


def line_cells(x0, y0, x1, y1):
    return bresenham_cells(int(x0), int(y0), np.array([x1]), np.array([y1]))


def footprint_cost_lethal(layer, fp, pose):
    """FootprintCollisionChecker::footprintCostAtPose >= LETHAL (outline only)."""
    x, y, th = pose
    c, s = math.cos(th), math.sin(th)
    pts = [(x + c * px - s * py, y + s * px + c * py) for px, py in fp]
    mx, my, ok = layer.world_to_map([p[0] for p in pts], [p[1] for p in pts])
    if not ok.all():
        return False          # off-map: treated as not lethal here (LETHAL-only metric)
    for i in range(len(pts)):
        j = (i + 1) % len(pts)
        cx, cy = line_cells(mx[i], my[i], mx[j], my[j])
        if (layer.g[cy, cx] >= LETHAL).any():
            return True
    return False


def rpp_collision_imminent(layer, fp, pose, v, w, carrot=0.5, max_t=1.5):
    """RegulatedPurePursuitController::isCollisionImminent (Humble 1.1.x)."""
    if footprint_cost_lethal(layer, fp, pose):
        return True
    x, y, th = pose
    if abs(v) < 0.01 and abs(w) > 0.01:
        r_max = float(np.max(np.hypot(fp[:, 0], fp[:, 1])))
        dt = 2.0 * math.sin((layer.res / 2) / r_max) / abs(w)
    else:
        dt = layer.res / abs(v)
    i = 1
    cx, cy, ct = x, y, th
    while i * dt < max_t:
        i += 1
        cx += dt * v * math.cos(ct)
        cy += dt * v * math.sin(ct)
        ct += dt * w
        if math.hypot(cx - x, cy - y) > carrot:
            break
        if footprint_cost_lethal(layer, fp, (cx, cy, ct)):
            return True
    return False
