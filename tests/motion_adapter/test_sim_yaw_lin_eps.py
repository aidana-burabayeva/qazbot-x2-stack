"""One-variable SIM fixture for s1_0046; never changes vx or production defaults."""
import sys
from pathlib import Path
import pytest
sys.path.insert(0,str(Path(__file__).resolve().parents[2]/'ws/src/navigation'))
from x2_motion_adapter import AdapterCore, Config, McState, sim_yaw_lin_eps


def test_default_without_fixture(monkeypatch):
    monkeypatch.delenv('QAZBOT_SIM_YAW_LIN_EPS',raising=False)
    assert sim_yaw_lin_eps()==.02


def test_fixture_requires_all_sim_markers(monkeypatch):
    monkeypatch.setenv('QAZBOT_SIM_YAW_LIN_EPS','.10')
    markers={'QAZBOT_SIM':'1','TWIN_SANDBOX':'1','ROS_DOMAIN_ID':'87'}
    for k,v in markers.items():monkeypatch.setenv(k,v)
    assert sim_yaw_lin_eps()==.10
    for missing in markers:
        monkeypatch.delenv(missing)
        with pytest.raises(ValueError):sim_yaw_lin_eps()
        monkeypatch.setenv(missing,markers[missing])


@pytest.mark.parametrize('value',['nan','inf','-.01','.11'])
def test_invalid_threshold_rejected(monkeypatch,value):
    for k,v in {'QAZBOT_SIM':'1','TWIN_SANDBOX':'1','ROS_DOMAIN_ID':'87','QAZBOT_SIM_YAW_LIN_EPS':value}.items():monkeypatch.setenv(k,v)
    with pytest.raises(ValueError):sim_yaw_lin_eps()


@pytest.mark.parametrize('vx',[.019,.03,.09,.10,-.03])
def test_ab_changes_only_yaw_activation_and_preserves_vx(vx):
    output=[]
    for threshold in [.02,.10]:
        core=AdapterCore(Config(yaw_lin_eps=threshold))
        mc=McState(stamp=100.,source='rc',body=1,fsm=4)
        d=core.step(100.,(vx,0.,-.4),100.,mc,yaw_rate=0.)
        assert d.publish[0]==vx
        assert ('yaw_loop' in d.notes)==(abs(vx)<threshold)
        output.append(d.publish)
    if abs(vx)>=.10:assert output[0]==output[1]
