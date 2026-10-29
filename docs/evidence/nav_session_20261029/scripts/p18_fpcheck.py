#!/usr/bin/env python3
"""P18 2026-10-29: read-only footprint/rotation check on live costmaps (no commands sent).
Mimics Humble RPP 1.1.18 inCollision(): FootprintCollisionChecker::footprintCost rasterises the
polygon OUTLINE and reports collision iff max cost >= LETHAL(254) (OccupancyGrid value 100).
Also reports centre-cell cost (Smac2D start validity: centre >= INSCRIBED(253, grid 99) is invalid)."""
import math, time, sys, rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy
from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import PolygonStamped
import tf2_ros
NS = '/qazbot_d1_map'
# Candidate polygon in base_footprint (same as nav_control/nav_full X2_FOOTPRINT) + padding 0.01
POLY = [(0.17, 0.229), (0.008, 0.34), (-0.182, 0.34), (-0.305, 0.229),
        (-0.305, -0.261), (-0.182, -0.372), (0.008, -0.372), (0.17, -0.261)]
PAD = 0.01
def pad(p):
    return [(x + math.copysign(PAD, x), y + math.copysign(PAD, y)) for x, y in p]
def circle(r, n=16):
    return [(r * math.cos(2 * math.pi * i / n), r * math.sin(2 * math.pi * i / n)) for i in range(n)]
rclpy.init(); n = Node('orch_p18_fpcheck'); d = {}
q = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL, reliability=ReliabilityPolicy.RELIABLE)
for k, t in (('lc', 'local_costmap/costmap'), ('gc', 'global_costmap/costmap')):
    n.create_subscription(OccupancyGrid, f'{NS}/nav2/{t}', lambda m, k=k: d.__setitem__(k, m), q)
for k, t in (('lf', 'local_costmap/published_footprint'), ('gf', 'global_costmap/published_footprint')):
    n.create_subscription(PolygonStamped, f'{NS}/nav2/{t}', lambda m, k=k: d.__setitem__(k, m), 10)
buf = tf2_ros.Buffer(); tf2_ros.TransformListener(buf, n, qos=10)
n.destroy_subscription  # noqa
# TF is namespaced: rebuild listener on remapped topics
from tf2_msgs.msg import TFMessage
def add(m, static):
    for tr in m.transforms:
        (buf.set_transform_static if static else buf.set_transform)(tr, 'p18')
n.create_subscription(TFMessage, f'{NS}/tf', lambda m: add(m, False), 50)
n.create_subscription(TFMessage, f'{NS}/tf_static', lambda m: add(m, True),
                      QoSProfile(depth=50, durability=DurabilityPolicy.TRANSIENT_LOCAL, reliability=ReliabilityPolicy.RELIABLE))
t0 = time.time()
while time.time() - t0 < 12 and not all(k in d for k in ('lc', 'gc', 'lf', 'gf')):
    rclpy.spin_once(n, timeout_sec=0.2)
for _ in range(15): rclpy.spin_once(n, timeout_sec=0.1)
def pose(frame):
    try:
        tr = buf.lookup_transform(frame, 'base_footprint', rclpy.time.Time()).transform
        qq = tr.rotation
        return tr.translation.x, tr.translation.y, math.atan2(2 * (qq.w * qq.z + qq.x * qq.y), 1 - 2 * (qq.y ** 2 + qq.z ** 2))
    except Exception as e:
        return None
def cost(g, x, y):
    I = g.info; i = int((x - I.origin.position.x) / I.resolution); j = int((y - I.origin.position.y) / I.resolution)
    if 0 <= i < I.width and 0 <= j < I.height: return g.data[j * I.width + i]
    return None
def fpcost(g, px, py, th, fp):
    pts = [(px + x * math.cos(th) - y * math.sin(th), py + x * math.sin(th) + y * math.cos(th)) for x, y in fp]
    m = 0; r = g.info.resolution / 4
    for a, b in zip(pts, pts[1:] + pts[:1]):
        L = math.hypot(b[0] - a[0], b[1] - a[1]); k = max(1, int(L / r))
        for s in range(k + 1):
            c = cost(g, a[0] + (b[0] - a[0]) * s / k, a[1] + (b[1] - a[1]) * s / k)
            if c is not None and c > m: m = c
    return m
def nearest(g, px, py):
    I = g.info; best = (9e9, 0)
    i0 = int((px - I.origin.position.x) / I.resolution); j0 = int((py - I.origin.position.y) / I.resolution); w = int(1.5 / I.resolution)
    for j in range(max(0, j0 - w), min(I.height, j0 + w)):
        for i in range(max(0, i0 - w), min(I.width, i0 + w)):
            if g.data[j * I.width + i] >= 100:
                x = I.origin.position.x + (i + .5) * I.resolution; y = I.origin.position.y + (j + .5) * I.resolution
                r = math.hypot(x - px, y - py)
                if r < best[0]: best = (r, math.atan2(y - py, x - px))
    return best
fps = {'circle0.464': circle(0.464), 'poly+pad0.01': pad(POLY)}
PAD2 = 0.01 + 0.05 / math.sqrt(2)
fps['poly+pad0.045(global)'] = [(x + math.copysign(PAD2, x), y + math.copysign(PAD2, y)) for x, y in POLY]
pf = d.get('lf'); print('local published_footprint (%d pts):' % len(pf.polygon.points) if pf else 'no local footprint',
      [(round(p.x, 3), round(p.y, 3)) for p in pf.polygon.points] if pf else '')
for gk, fk in (('lc', 'lf'), ('gc', 'gf')):
    g = d.get(gk)
    if g is None: print(gk, 'missing'); continue
    P = pose(g.header.frame_id)
    if P is None and d.get(fk) is not None:  # fallback: recover pose from published footprint
        pts = [(p.x, p.y) for p in d[fk].polygon.points]
        if len(pts) == 16:
            P = (sum(x for x, _ in pts) / 16, sum(y for _, y in pts) / 16, math.atan2(pts[0][1] - sum(y for _, y in pts) / 16, pts[0][0] - sum(x for x, _ in pts) / 16))
        elif len(pts) == len(POLY):
            loc = fps['poly+pad0.045(global)'] if fk == 'gf' else pad(POLY); a0, a1 = loc[0], loc[4]; b0, b1 = pts[0], pts[4]
            th = math.atan2(b1[1] - b0[1], b1[0] - b0[0]) - math.atan2(a1[1] - a0[1], a1[0] - a0[0])
            P = (b0[0] - (a0[0] * math.cos(th) - a0[1] * math.sin(th)), b0[1] - (a0[0] * math.sin(th) + a0[1] * math.cos(th)), th)
        print(f'[{gk}] pose from published_footprint (yaw approximate for circle)')
    if P is None: print(gk, 'no TF', g.header.frame_id); continue
    px, py, th = P; r, b = nearest(g, px, py)
    print(f'[{gk}] frame={g.header.frame_id} robot=({px:.2f},{py:.2f},yaw {math.degrees(th):.0f}deg) centre_cost={cost(g, px, py)} '
          f'nearest_lethal={r:.3f} m at {math.degrees(b - th):.0f}deg (robot frame)')
    for name, fp in fps.items():
        sweep = [(a, fpcost(g, px, py, th + math.radians(a), fp)) for a in range(-180, 180, 5)]
        bad = [a for a, c in sweep if c >= 100]
        print(f'   {name}: current-yaw outline max={fpcost(g, px, py, th, fp)}  lethal at {len(bad)}/72 yaw offsets {bad[:12]}{"..." if len(bad) > 12 else ""}')
