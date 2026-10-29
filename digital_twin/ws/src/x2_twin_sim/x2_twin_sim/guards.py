"""Isolation guard: twin nodes must never run next to the physical robot."""
import os

_REQUIRED = {'TWIN_SANDBOX': '1', 'ROS_DOMAIN_ID': '87'}


def require_sim_environment(env=None):
    """Raise unless the process runs in the isolated twin environment.

    The twin uses ROS domain 87 and the explicit ``TWIN_SANDBOX=1`` marker (set by the container entrypoint).
    This is a second line of defence: the primary one is the container network being ``internal``.
    """
    env = os.environ if env is None else env
    wrong = {k: env.get(k) for k, v in _REQUIRED.items() if env.get(k) != v}
    if wrong:
        raise RuntimeError(f'refusing to run outside the isolated twin environment: {wrong} (expected {_REQUIRED})')
