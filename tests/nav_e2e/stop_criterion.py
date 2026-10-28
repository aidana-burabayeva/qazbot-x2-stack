"""Stop criterion for dry-run tests (replaces "last N outputs are zero").

Nav2 1.1.18 velocity_smoother may stop publishing after a stop, so trailing zeros are NOT required.
Evidence used instead:
  1. the terminal (or cancel) status was observed, at harness time t_terminal;
  2. the last command received by the model (value + time) is recorded;
  3. no nonzero command arrives after t_terminal + one control period (1/controller_frequency). For an
     UNGATED smoother output, `decel` (m/s^2) permits only a monotone ramp-down bounded by that limit
     until |v_last|/decel after the grace; nothing nonzero after that;
  4. the model velocity is zero and its pose stationary over the observation window afterwards;
  5. after recovery/resume (t_terminal = recovery time) the same rules show no re-motion.
"""
import math

CMD_EPS = 1e-9


def nonzero(c):
    return any(abs(v) > CMD_EPS for v in c[1:4])


def stop_check(cmds, odoms, t_terminal, period, window_start, decel=None, v_eps=0.01, pose_eps=0.005):
    """cmds: [(t, vx, vy, wz)] as received by the model; odoms: [(t, x, y, yaw, v, w)] of the model.
    window_start: harness time from which the model must be stopped and stationary (after it was observed
    at |v| < v_eps). Returns (ok, evidence)."""
    ev = dict(terminal_observed=t_terminal is not None)
    if t_terminal is None:
        return False, ev
    last = cmds[-1] if cmds else None
    last_nz = next((c for c in reversed(cmds) if nonzero(c)), None)
    ev['last_cmd'] = None if last is None else dict(t_rel=round(last[0] - t_terminal, 3), value=last[1:4])
    ev['last_nonzero_cmd'] = None if last_nz is None else dict(t_rel=round(last_nz[0] - t_terminal, 3), value=last_nz[1:4])
    grace = t_terminal + period
    late = [c for c in cmds if c[0] > grace and nonzero(c)]
    if decel and late:
        before = [c for c in cmds if c[0] <= grace]
        v0 = max((abs(c[1]) for c in before[-1:]), default=0.0)
        w0 = max((abs(c[3]) for c in before[-1:]), default=0.0)
        ramp_end = grace + max(v0, w0) / decel
        mags = [(abs(c[1]), abs(c[3])) for c in late]
        monotone = all(b[0] <= a[0] + 1e-9 and b[1] <= a[1] + 1e-9 for a, b in zip([(v0, w0)] + mags, mags))
        ev['ramp'] = dict(v0=v0, w0=w0, ramp_end_rel=round(ramp_end - t_terminal, 3), monotone=monotone)
        late = [c for c in late if c[0] > ramp_end or not monotone]
    ev['nonzero_after_terminal'] = [dict(t_rel=round(c[0] - t_terminal, 3), value=c[1:4]) for c in late[:5]]
    win = [o for o in odoms if o[0] >= window_start]
    ev['window_samples'] = len(win)
    if win:
        ev['model_max_speed'] = max(max(abs(o[4]), abs(o[5])) for o in win)
        ev['pose_drift'] = max(math.hypot(o[1] - win[0][1], o[2] - win[0][2]) for o in win)
        ev['yaw_drift'] = max(abs(math.atan2(math.sin(o[3] - win[0][3]), math.cos(o[3] - win[0][3]))) for o in win)
    win_cmds = [c for c in cmds if c[0] >= window_start and nonzero(c)]
    ev['nonzero_in_window'] = len(win_cmds)
    ok = (not late and len(win) >= 2 and ev['model_max_speed'] < v_eps and ev['pose_drift'] < pose_eps
          and ev['yaw_drift'] < pose_eps and not win_cmds)
    return ok, ev
