"""Pure application logic for the offline nav2_collision_monitor (Humble 1.1.18) evaluation.

No ROS imports here: everything is plain numpy so it is unit-testable (test/test_cm_eval_lib.py).

Contents
- point_in_polygon_count(): same even-odd ray-cast rule as nav2_collision_monitor Polygon::isPointInside
  (polygon.cpp, 1.1.18) - points exactly on an edge may differ, irrelevant at mm scale.
- emulate_decision(): re-implementation of CollisionMonitor::process() for STOP / SLOWDOWN / APPROACH(circle)
  as of tag 1.1.18 (collision_monitor_node.cpp, polygon.cpp getCollisionTime, kinematics.cpp).
- transform_cloud(), height_filter(): PointCloud::getData() equivalent (z band in base frame, keep x,y).
- classify_output(): map a published cmd_vel_out to STOP / SLOWDOWN / APPROACH / PASS.
- latency_stats(): p50 / p99 / max helper.
"""
import math

import numpy as np

STOP, SLOWDOWN, APPROACH, PASS = 'STOP', 'SLOWDOWN', 'APPROACH', 'PASS'


def point_in_polygon_mask(points_xy, poly):
    """Even-odd rule, vectorised. points_xy: (N,2); poly: (M,2) vertices. Returns bool mask (N,)."""
    pts = np.asarray(points_xy, dtype=float).reshape(-1, 2)
    poly = np.asarray(poly, dtype=float).reshape(-1, 2)
    x, y = pts[:, 0], pts[:, 1]
    inside = np.zeros(len(pts), dtype=bool)
    n = len(poly)
    j = n - 1
    for i in range(n):
        xi, yi = poly[i]
        xj, yj = poly[j]
        cond = (yi > y) != (yj > y)
        with np.errstate(divide='ignore', invalid='ignore'):
            xint = (xj - xi) * (y - yi) / (yj - yi) + xi
        inside ^= cond & (x < xint)
        j = i
    return inside


def point_in_polygon_count(points_xy, poly):
    return int(point_in_polygon_mask(points_xy, poly).sum())


def circle_count(points_xy, radius):
    pts = np.asarray(points_xy, dtype=float).reshape(-1, 2)
    return int((pts[:, 0] ** 2 + pts[:, 1] ** 2 < radius ** 2).sum())


def transform_cloud(points_sensor, rot_base_sensor, t_base_sensor):
    """Sensor-frame xyz (N,3) -> base frame using a fixed (static) transform."""
    p = np.asarray(points_sensor, dtype=float).reshape(-1, 3)
    return p @ np.asarray(rot_base_sensor, dtype=float).T + np.asarray(t_base_sensor, dtype=float)


def height_filter(points_base, min_height, max_height):
    """PointCloud::getData(): keep x,y of points with min_height <= z <= max_height (base frame)."""
    p = np.asarray(points_base, dtype=float).reshape(-1, 3)
    m = (p[:, 2] >= min_height) & (p[:, 2] <= max_height)
    return p[m, :2]


def _project(dt, pose, vel):
    """kinematics.cpp projectState()."""
    x, y, th = pose
    vx, vy, w = vel
    theta = w * dt
    c, s = math.cos(theta), math.sin(theta)
    x += vx * dt
    y += vy * dt
    th += theta
    return (x, y, th), (vx * c - vy * s, vx * s + vy * c, w)


def _to_pose(points_xy, pose):
    """kinematics.cpp transformPoints(): points into the frame of the simulated pose."""
    x, y, th = pose
    c, s = math.cos(th), math.sin(th)
    dx = points_xy[:, 0] - x
    dy = points_xy[:, 1] - y
    return np.column_stack((dx * c + dy * s, -dx * s + dy * c))


def collision_time_circle(points_xy, vel, radius, max_points, time_before_collision, step):
    """Polygon::getCollisionTime() for a circle footprint (Circle::getPointsInside). -1 = no collision."""
    pts = np.asarray(points_xy, dtype=float).reshape(-1, 2)
    if circle_count(pts, radius) >= max_points:  # note: >= here, > below (as in 1.1.18)
        return 0.0
    pose = (0.0, 0.0, 0.0)
    v = tuple(vel)
    t = 0.0
    while t <= time_before_collision:
        pose, v = _project(step, pose, v)
        if circle_count(_to_pose(pts, pose), radius) > max_points:
            return t
        t += step
    return -1.0


def emulate_decision(points_xy, cmd_vel, polygons):
    """CollisionMonitor::process(). polygons: ordered list of dicts:
    {name, action: 'stop'|'slowdown'|'approach', points:(M,2) or radius, max_points, slowdown_ratio,
     time_before_collision, simulation_time_step}.
    Returns (action, (vx, vy, wz), polygon_name)."""
    req = tuple(float(v) for v in cmd_vel)
    action, name = PASS, None
    pts = np.asarray(points_xy, dtype=float).reshape(-1, 2)

    def less(a, b):  # Velocity::operator< (types.hpp 1.1.18)
        return _vel_mag(a) < _vel_mag(b)

    for p in polygons:
        if action == STOP:
            break
        if p['action'] in ('stop', 'slowdown'):
            n = (point_in_polygon_count(pts, p['points']) if 'points' in p
                 else circle_count(pts, p['radius']))
            if n > p['max_points']:
                if p['action'] == 'stop':
                    action, req, name = STOP, (0.0, 0.0, 0.0), p['name']
                else:
                    safe = tuple(v * p['slowdown_ratio'] for v in cmd_vel)
                    if less(safe, req):
                        action, req, name = SLOWDOWN, safe, p['name']
        elif p['action'] == 'approach':
            ct = collision_time_circle(pts, cmd_vel, p['radius'], p['max_points'],
                                       p['time_before_collision'], p['simulation_time_step'])
            if ct >= 0.0:
                ratio = ct / p['time_before_collision']
                safe = tuple(v * ratio for v in cmd_vel)
                if less(safe, req):
                    action, req, name = APPROACH, safe, p['name']
    return action, req, name


def _vel_mag(v):
    # types.hpp (1.1.18) Velocity::operator<: compares x^2 + y^2 + tw^2.
    return v[0] * v[0] + v[1] * v[1] + v[2] * v[2]


def classify_output(out_vel, in_vel, slowdown_ratio, tol=1e-6):
    """Classify a published cmd_vel_out vs the commanded input."""
    ox, oy, ow = out_vel
    ix, iy, iw = in_vel
    if abs(ox) < tol and abs(oy) < tol and abs(ow) < tol:
        return STOP
    if abs(ox - ix) < tol and abs(oy - iy) < tol and abs(ow - iw) < tol:
        return PASS
    if abs(ix) > tol and abs(ox / ix - slowdown_ratio) < 1e-4:
        return SLOWDOWN
    return APPROACH


def latency_stats(values):
    v = np.asarray([x for x in values if x is not None and np.isfinite(x)], dtype=float)
    if not len(v):
        return dict(n=0, p50=None, p99=None, max=None, min=None)
    return dict(n=int(len(v)), p50=float(np.percentile(v, 50)), p99=float(np.percentile(v, 99)),
                max=float(v.max()), min=float(v.min()))


def polygons_from_params(p):
    """Build emulate_decision() polygon list from the CM ros__parameters dict (same YAML as the node)."""
    out = []
    for name in p['polygons']:
        q = p[name]
        d = dict(name=name, action=q['action_type'], max_points=int(q.get('max_points', 3)),
                 slowdown_ratio=float(q.get('slowdown_ratio', 0.5)),
                 time_before_collision=float(q.get('time_before_collision', 2.0)),
                 simulation_time_step=float(q.get('simulation_time_step', 0.1)))
        if q['type'] == 'circle':
            d['radius'] = float(q['radius'])
        else:
            pts = list(q['points'])
            d['points'] = np.array(pts, dtype=float).reshape(-1, 2)
        if q.get('enabled', True):
            out.append(d)
    return out
