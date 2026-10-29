"""Find free poses/yaws in the twin's map whose forward FOV has a contiguous no-return sector (open > range_max) - the geometry in which
inf bins exist, i.e. where stale marks can persist (the Goal01 situation)."""
import math

import numpy as np

from x2_twin_sim.occupancy import OccupancyMap, RayCaster

m = OccupancyMap.from_yaml('/opt/x2twin/stack/maps/first_map_nav2_v2/map.yaml')
rc = RayCaster(m, n_beams=1440, range_min=0.45, range_max=8.0, fov_deg=360.0)
ang = np.degrees(rc.angle_min + rc.angle_increment * np.arange(rc.n))
from scipy.ndimage import label

free = m.dist >= 0.6
lab, _ = label(free)
si, sj = int((-4.825 - m.oy) / m.res), int((-1.275 - m.ox) / m.res)
inside = lab == lab[si, sj]                                  # free space connected to START (excludes the map exterior)
best = []
for x in np.arange(m.ox + 1, m.ox + m.w * m.res - 1, 1.0):
    for y in np.arange(m.oy + 1, m.oy + m.h * m.res - 1, 1.0):
        if not inside[int((y - m.oy) / m.res), int((x - m.ox) / m.res)]:
            continue
        for yaw in range(0, 360, 15):
            rel = (ang - yaw + 180) % 360 - 180
            k = (rel >= -46) & (rel <= 44.5)
            nr = np.isinf(rc.scan((x, y, math.radians(yaw)))[k])
            n = int(nr.sum())
            if 40 <= n <= 250:
                best.append((n, round(float(x), 2), round(float(y), 2), yaw))
best.sort(reverse=True)
for b in best[:12]:
    print('no-return bins in FOV %d at x=%.2f y=%.2f yaw=%d deg' % b)
print('candidates', len(best))
