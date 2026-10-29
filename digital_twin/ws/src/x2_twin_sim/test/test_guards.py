import pytest

from x2_twin_sim.guards import require_sim_environment


def test_accepts_the_isolated_environment():
    require_sim_environment({'TWIN_SANDBOX': '1', 'ROS_DOMAIN_ID': '87'})


@pytest.mark.parametrize('env', [{}, {'TWIN_SANDBOX': '1'}, {'TWIN_SANDBOX': '1', 'ROS_DOMAIN_ID': '0'},
                                 {'TWIN_SANDBOX': '0', 'ROS_DOMAIN_ID': '87'}])
def test_refuses_anything_else(env):
    with pytest.raises(RuntimeError):
        require_sim_environment(env)
