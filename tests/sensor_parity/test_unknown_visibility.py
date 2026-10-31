"""Conservative map-raycaster: unknown cells do not authorize free paths behind them."""
import sys
from pathlib import Path

import numpy as np
from PIL import Image
import yaml

sys.path.insert(0,str(Path(__file__).resolve().parents[2]/'digital_twin/ws/src/x2_twin_sim'))
from x2_twin_sim.occupancy import OccupancyMap, RayCaster


def test_unknown_strip_blocks_evidence_for_wall_and_disc_behind(tmp_path):
    image=np.full((80,80),254,np.uint8)
    image[:,70:]=0
    image[:,48:52]=205
    Image.fromarray(image).save(tmp_path/'map.pgm')
    path=tmp_path/'map.yaml'
    path.write_text(yaml.safe_dump(dict(image='map.pgm',resolution=.1,origin=[-4.,-4.,0.],free_thresh=.196,occupied_thresh=.65)))
    caster=RayCaster(OccupancyMap.from_yaml(str(path)),n_beams=1440,fov_deg=103,range_max=8)
    assert not np.isfinite(caster.scan((0.,0.,0.))[720])
    assert not np.isfinite(caster.scan((0.,0.,0.),[(2.,0.,.2)])[720])
    assert np.isfinite(caster.scan((0.,0.,0.),[(.6,0.,.1)])[720])
