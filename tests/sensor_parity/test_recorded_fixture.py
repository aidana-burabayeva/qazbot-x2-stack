import hashlib
import json
import sys
from pathlib import Path

import numpy as np

ROOT=Path(__file__).resolve().parents[2]
sys.path.insert(0,str(ROOT/'digital_twin/ws/src/x2_twin_sim'))
from x2_twin_sim.sensor_observations import bounded_observations


def test_recorded_profile_provenance_and_bounded_clearing():
    data=json.loads((ROOT/'tools/nightly/fixtures/x2_recorded_lidar.json').read_text())
    manifest=json.loads((ROOT/'docs/evidence/i1_cloud_registered_udp_20261028_092700Z/archive_manifest.json').read_text())
    assert data['source_raw_sha256']==manifest['raw_sha256']
    assert hashlib.sha256((ROOT/'ws/src/lio/level_bridge.py').read_bytes()).hexdigest()==data['bridge_sha256']
    assert data['usable_frames']==71
    assert min(f['finite_bins'] for f in data['frames'])==202
    assert max(f['finite_bins'] for f in data['frames'])==215
    f=data['fixture'];r=np.full(f['bins'],np.inf)
    r[f['finite_bin_indices']]=f['finite_ranges']
    a=f['angle_min']+np.arange(f['bins'])*f['angle_increment']
    mark,clear=bounded_observations(r,a,np.ones(f['bins'],bool),f['range_min'],f['range_max'])
    assert len(mark)==len(f['finite_bin_indices'])
    # Real 3-D data can prove more rays (floor/height-slice boundaries). Planar twin must not invent them.
    assert len(clear)<=len(mark)<len(f['clearing_xyz'])
    assert np.isfinite(mark).all() and np.isfinite(clear).all()
    assert bounded_observations(np.full_like(r,np.inf),a,np.ones(len(r),bool),.45,8)[1].size==0


def test_no_return_nan_blind_and_out_of_range_never_clear():
    r=np.array([np.inf,np.nan,1.,8.,.2,2.])
    a=np.zeros(len(r));seen=np.array([1,1,0,1,1,1],bool)
    mark,clear=bounded_observations(r,a,seen,.45,8)
    assert len(mark)==len(clear)==1
    assert mark[0,0]==2 and clear[0,0]<2
