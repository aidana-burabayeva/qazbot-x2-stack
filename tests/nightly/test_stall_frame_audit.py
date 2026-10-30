"""An initial-pose reanchor must not silently be confused with the generated world's fixed map transform."""
import importlib.util
import math
from pathlib import Path
import sys
import numpy as np
import pytest
ROOT=Path(__file__).resolve().parents[2]
spec=importlib.util.spec_from_file_location('audit',ROOT/'tools/nightly/audit_stall_frames.py')
audit=importlib.util.module_from_spec(spec);spec.loader.exec_module(audit)
sys.path.insert(0,str(ROOT/'digital_twin/ws/src/x2_twin_sim'))
from x2_twin_sim.geometry import FrameAligner,compose


def test_rigid_fit_recovers_known_rotation_and_translation():
    points=np.array([[0.,0.],[1.,0.],[1.,7.],[-1.,3.]])
    angle=-.1;c,s=math.cos(angle),math.sin(angle);r=np.array([[c,-s],[s,c]])
    target=points@r.T+np.array([-8.535,-15.434])
    fitted,t,rmse=audit.fit_transform(points,target)
    assert fitted==pytest.approx(r,abs=1e-12)
    assert t==pytest.approx([-8.535,-15.434],abs=1e-12)
    assert rmse<1e-12


def test_nonzero_first_hal_yaw_changes_map_world_contract():
    start=(-8.535,-15.434,.002)
    aligner=FrameAligner(start)
    aligner.align((0.,0.,.1))
    current=(0.,7.,.1)
    published=aligner.align(current)
    physical=compose(start,current)
    assert math.hypot(published[0]-physical[0],published[1]-physical[1])==pytest.approx(2*7*math.sin(.05))
    assert abs(math.remainder(published[2]-physical[2],2*math.pi))==pytest.approx(.1)


def test_zero_origin_keeps_both_transforms_identical():
    start=(-8.535,-15.434,.002);aligner=FrameAligner(start);aligner.align((0.,0.,0.))
    assert aligner.align((.2,7.,1.))==pytest.approx(compose(start,(.2,7.,1.)))
