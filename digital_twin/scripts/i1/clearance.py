"""Distance to the nearest occupied cell (m) at points along the open bearing from a pose: clearance.py x,y,yaw_deg bearing_deg"""
import math
import sys

import numpy as np

from x2_twin_sim.occupancy import OccupancyMap

m = OccupancyMap.from_yaml('/opt/x2twin/stack/maps/first_map_nav2_v2/map.yaml')
x, y, yaw = (float(v) for v in sys.argv[1].split(','))
brg = float(sys.argv[2])
for r in (0.0, 0.5, 1.0, 1.5, 2.0, 2.5, 3.0):
    a = math.radians(yaw + brg)
    p = np.array([[x + r * math.cos(a), y + r * math.sin(a)]])
    print('range %.1f -> (%.2f, %.2f) clearance %.2f m' % (r, p[0, 0], p[0, 1], m.dist_at(p)[0]))
