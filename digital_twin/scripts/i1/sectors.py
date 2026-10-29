"""No-return sectors (bearing relative to heading, deg) of the twin scan at given poses: sectors.py x,y,yaw_deg ..."""
import math
import sys

import numpy as np

from x2_twin_sim.occupancy import OccupancyMap, RayCaster

m = OccupancyMap.from_yaml('/opt/x2twin/stack/maps/first_map_nav2_v2/map.yaml')
rc = RayCaster(m, n_beams=1440, range_min=0.45, range_max=8.0, fov_deg=float(__import__("os").environ.get("FOV", "103")))
rel = np.degrees(rc.angle_min + rc.angle_increment * np.arange(rc.n))
for arg in sys.argv[1:]:
    x, y, yaw = (float(v) for v in arg.split(','))
    s = rc.scan((x, y, math.radians(yaw)))
    ob = np.zeros(rc.n, bool)
    ob[rc.observed] = True
    nr = ob & np.isinf(s)
    runs, i = [], 0
    while i < rc.n:
        if nr[i]:
            j = i
            while j + 1 < rc.n and nr[j + 1]:
                j += 1
            runs.append((round(float(rel[i]), 1), round(float(rel[j]), 1)))
            i = j + 1
        else:
            i += 1
    print(arg, 'no-return sectors:', runs)
