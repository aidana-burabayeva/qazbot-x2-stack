#!/usr/bin/env python3
"""Read-only: one local costmap + robot pose in odom; list lethal/inscribed cells within 1.0 m of base_footprint with
bearing (deg, robot frame) and whether inside the clearing FOV (-46..+44.5) and beyond RANGE_MIN 0.45."""
import sys, math, time, json, numpy as np, rclpy
from nav_msgs.msg import OccupancyGrid, Odometry
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy
rclpy.init(); n = rclpy.create_node('costmap_near_1102'); st = {}
n.create_subscription(OccupancyGrid, '/qazbot_d1_map/nav2/local_costmap/costmap', lambda m: st.__setitem__('c', m),
                      QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL, reliability=ReliabilityPolicy.RELIABLE))
n.create_subscription(Odometry, '/qazbot_native/odom', lambda m: st.__setitem__('o', m), 10)
end = time.time() + 8
while time.time() < end and not ('c' in st and 'o' in st): rclpy.spin_once(n, timeout_sec=0.1)
c, o = st['c'], st['o']; p = o.pose.pose; yaw = 2 * math.atan2(p.orientation.z, p.orientation.w)
g = np.array(c.data, dtype=np.int16).reshape(c.info.height, c.info.width); res = c.info.resolution
iy, ix = np.nonzero(g >= 99)
wx = c.info.origin.position.x + (ix + .5) * res; wy = c.info.origin.position.y + (iy + .5) * res
dx, dy = wx - p.position.x, wy - p.position.y
bx, by = math.cos(-yaw) * dx - math.sin(-yaw) * dy, math.sin(-yaw) * dx + math.cos(-yaw) * dy
r = np.hypot(bx, by); b = np.degrees(np.arctan2(by, bx)); k = r < 1.0
print('costmap frame', c.header.frame_id, 'size', c.info.width, c.info.height, 'res', res, 'cells>=99 total', len(r), 'within 1 m', int(k.sum()))
for x, y, rr, bb, v in sorted(zip(bx[k], by[k], r[k], b[k], g[iy[k], ix[k]]), key=lambda t: t[2])[:40]:
    print(' x %+.2f y %+.2f r %.2f bearing %+6.1f cost %d inFOV %s r>0.45 %s' % (x, y, rr, bb, v, -46 <= bb <= 44.5, rr > 0.45))
# --- map check (appended): lethal cells within 1.5 m -> map frame via map->odom, distance to nearest map wall
from tf2_msgs.msg import TFMessage
sys.path.insert(0, '/home/agi/control_backend/ws/src/navigation'); import loc_trust as L
tf = {}
n.create_subscription(TFMessage, '/qazbot_d1_map/tf', lambda m: [tf.__setitem__(t.header.frame_id + '>' + t.child_frame_id, t.transform) for t in m.transforms], 50)
end = time.time() + 4
while time.time() < end and 'map>odom' not in tf: rclpy.spin_once(n, timeout_sec=0.1)
T = tf['map>odom']; ty = 2 * math.atan2(T.rotation.z, T.rotation.w)
grid = L.OccGrid.from_pgm('/home/agi/control_backend/ws/src/maps/d1_qazbot_20261029/map.yaml')
k = (r < 1.5) & (g[iy, ix] == 100)
mx = T.translation.x + math.cos(ty) * wx[k] - math.sin(ty) * wy[k]; my = T.translation.y + math.sin(ty) * wx[k] + math.cos(ty) * wy[k]
d = grid.dist_at(np.column_stack((mx, my)))
print('LETHAL within 1.5 m:', int(k.sum()))
for a, b2, rr, bb, dd in zip(mx, my, r[k], b[k], d): print(' map (%.2f, %.2f) r %.2f bearing %+6.1f dist_to_map_wall %.2f' % (a, b2, rr, bb, dd))
