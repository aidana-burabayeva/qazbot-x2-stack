"""TWIN-02: compiled C++ recovery core + real TrustEvaluator + real AdapterCore.

The event clock is deterministic (including actual 40 s gaps without wall-clock sleeps).
Reset evidence uses simulator-known independent pose; no such provider is claimed on the real X2.
"""
import ctypes as C
import subprocess
import sys
from pathlib import Path

import pytest

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / 'tests/loc_trust'))
from test_loc_trust import L, make_inputs, room_grid, simulate_scan
from x2_motion_adapter import AdapterCore, Config, McState, LATCHED


@pytest.fixture(scope='module')
def lib(tmp_path_factory):
    out = tmp_path_factory.mktemp('reanchor') / 'core.so'
    subprocess.run(['g++', '-std=c++17', '-Wall', '-Wextra', '-Werror', '-O2', '-shared', '-fPIC',
                    str(ROOT / 'tests/lio/reanchor_ffi.cpp'), '-o', str(out)], check=True)
    lib = C.CDLL(str(out))
    lib.ra_new.restype = C.c_void_p
    lib.ra_delete.argtypes = [C.c_void_p]
    lib.ra_update.argtypes = [C.c_void_p] + [C.c_double]*5 + [C.POINTER(C.c_double)]
    lib.ra_begin.argtypes = [C.c_void_p, C.c_ulong, C.c_int, C.c_int]
    lib.ra_confirm.argtypes = [C.c_void_p, C.c_ulong] + [C.c_double]*8 + [C.c_int]*3
    lib.ra_release.argtypes = [C.c_void_p, C.c_ulong, C.c_int, C.c_double]
    lib.ra_motion.argtypes = [C.c_void_p, C.c_double, C.c_int]
    return lib


@pytest.mark.parametrize('jump', [.467, .5, 1.563, 1.6])
@pytest.mark.parametrize('gap', [.1, 40.])
@pytest.mark.parametrize('confirmed', [False, True])
def test_reset_gap_trust_and_adapter(lib, jump, gap, confirmed):
    core = lib.ra_new()
    buf = (C.c_double*3)()
    grid = room_grid()
    zero = (0., 0., 0.)
    scan = simulate_scan(grid, zero)
    trust = L.TrustEvaluator(grid, L.TrustConfig(admission_policy='strict'))
    inp = lambda t, **kw: make_inputs(t, zero, zero, scan, **kw)
    trust.confirm_anchor(inp(100.))
    assert trust.evaluate(inp(100.01))['motion_ok']
    adapter = AdapterCore(Config())
    mc = lambda t: McState(stamp=t, source='rc', body=1, fsm=4)
    assert lib.ra_update(core, 100., *zero, 100., buf)
    assert lib.ra_motion(core, 100., True)
    assert adapter.step(100., (.3, 0., 0.), 100., mc(100.)).publish[0] > 0
    try:
        # Explicit MC mode event closes the interlock before the stream disappears.
        if confirmed:
            assert lib.ra_begin(core, 1, True, True)
            trust.relocalize()  # fresh qualification is required, not merely old READY
        t = 100. + gap
        for now in (100.01, t):
            verdict = trust.evaluate(inp(now, odom_stamp=100., odom_msg_stamp=100.))
            allowed = lib.ra_motion(core, now, verdict['motion_ok'])
            if confirmed or now - 100. > .2:
                assert not allowed
                d = adapter.step(now, (.3, 0., 0.), now, mc(now), localized=False)
                assert d.publish in (None, (0., 0., 0.))
        if not confirmed:
            assert not lib.ra_update(core, t, jump, 0., .7, t, buf)
            assert not lib.ra_motion(core, t, True)
            for dt in (.01, .02, .1):
                assert not lib.ra_update(core, t+dt, jump, 0., .7, t+dt, buf)
            assert not lib.ra_confirm(core, 1, t, jump, 0., .7, *zero, t, True, True, True)
            return
        # MC is back in STAND, but mode alone does not confirm that the source reset.
        assert not lib.ra_confirm(core, 1, t, jump, 0., .7, *zero, t, False, True, True)
        assert not lib.ra_motion(core, t, True)
        # Sim-truth independently confirms the unchanged physical pose at the new sample stamp.
        assert lib.ra_confirm(core, 1, t, jump, 0., .7, *zero, t, True, True, True)
        for dt in (.01, .02):
            assert lib.ra_update(core, t+dt, jump, 0., .7, t+dt, buf)
            assert tuple(buf) == pytest.approx(zero, abs=1e-12)
            assert not trust.evaluate(inp(t+dt))['motion_ok']
            assert not lib.ra_motion(core, t+dt, True)
        assert not lib.ra_release(core, 1, False, t+.02)
        trust.confirm_anchor(inp(t+.03))
        verdict = trust.evaluate(inp(t+.04))
        assert verdict['motion_ok'], verdict
        assert lib.ra_release(core, 1, verdict['motion_ok'], t+.04)
        assert lib.ra_motion(core, t+.04, verdict['motion_ok'])
        # Recovered localisation never cancels the motion adapter's earlier latch.
        assert adapter.state == LATCHED
        assert adapter.step(t+.04, (.3, 0., 0.), t+.04, mc(t+.04)).publish in (None, (0.,0.,0.))
        assert adapter.resume(now=t+.05)
        assert adapter.step(t+.06, (.3,0.,0.), 100., mc(t+.06)).publish in (None, (0.,0.,0.))
        assert adapter.step(t+.07, (.3,0.,0.), t+.07, mc(t+.07)).publish[0] > 0
    finally:
        lib.ra_delete(core)
