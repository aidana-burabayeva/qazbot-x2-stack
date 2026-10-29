"""Where does the twin's map give long (>=8 m) free rays? Prints, for the START pose and candidates, the clear-bearing sectors."""
import math
import sys

import numpy as np

from x2_twin_sim.occupancy import OccupancyMap, RayCaster

m = OccupancyMap.from_yaml('/opt/x2twin/stack/maps/first_map_nav2_v2/map.yaml')
rc = RayCaster(m, n_beams=1440, range_min=0.45, range_max=8.0, fov_deg=360.0)
poses = [(-1.275, -4.825, 0.0)] + [tuple(map(float, p.split(','))) for p in sys.argv[1:]]
for pose in poses:
    s = rc.scan(pose)
    ang = np.degrees(rc.angle_min + rc.angle_increment * np.arange(rc.n))
    rel = (ang + 180) % 360 - 180
    for lo, hi in ((-46, 44.5),):
        k = (rel >= lo) & (rel <= hi)
        print(pose, 'in-FOV bins', k.sum(), 'no-return bins', int(np.isinf(s[k]).sum()),
              'sectors of no-return (deg):', [(round(float(a), 1), round(float(b), 1)) for a, b in
                                              zip(rel[k][np.flatnonzero(np.diff(np.r_[0, np.isinf(s[k]).astype(int)]) == 1)],
                                                  rel[k][np.flatnonzero(np.diff(np.r_[np.isinf(s[k]).astype(int), 0]) == -1)])])
