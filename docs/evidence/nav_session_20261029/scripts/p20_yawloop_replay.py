#!/usr/bin/env python3
"""P20 offline sanity check of the adapter yaw-rate loop (no robot, no unit-test suite).
(A) open-loop replay: chest-yaw traces of the p19 calib2 run -> YawRateEstimator -> AdapterCore (what u would be sent).
(B) closed-loop sim: AdapterCore -> MC model (measured static curve, 0.4 s dead time, 0.2 s lag, gait ripple, optional
    'after walk' gain 0.5 and stall episodes) -> chest yaw @30 Hz -> estimator -> AdapterCore. Reports tracking/oscillation."""
import json, math, os, random, sys
sys.path.insert(0, '/home/agi/control_backend/ws/src/navigation')
from x2_motion_adapter import AdapterCore, Config, McState, YawRateEstimator, ACTIVE

EV = os.path.dirname(os.path.abspath(__file__))
DT = 0.02


def core():
    c = AdapterCore(Config(live=False, require_localization=False))
    return c


def mc(now):
    return McState(stamp=now, source='rc', body=1, fsm=4)


def replay():
    rows = []
    for l in open(os.path.join(EV, 'p19_turn_calib2_20261029_185054_raw.jsonl'), 'rb'):
        try:
            rows.append(json.loads(l.decode()))
        except Exception:
            pass
    ch = [r for r in rows if r['k'] == 'chest']
    cm = [r for r in rows if r['k'] == 'cmd']
    st = [r for r in rows if r['k'] == 'step']
    out = []
    for s in st:
        c = core(); est = YawRateEstimator()
        t0 = s['t']; t1 = t0 + 11.0
        ci = [x for x in ch if t0 <= x['t'] <= t1]; cc = [x for x in cm if t0 <= x['t'] <= t1]
        t, i, j, cmd, us = t0, 0, 0, None, []
        while t < t1:
            while i < len(ci) and ci[i]['t'] <= t:
                est.update(ci[i]['t'], ci[i]['yaw'], ci[i]['t']); i += 1
            while j < len(cc) and cc[j]['t'] <= t:
                cmd = (cc[j]['v'], 0.0, cc[j]['w']); cs = cc[j]['t']; j += 1
            d = c.step(t, cmd, cs if cmd else float('-inf'), mc(t), yaw_rate=est.get(t, 0.3))
            if d.publish is not None and 'yaw_loop' in d.notes:
                us.append((round(t - t0, 2), cmd[2], est.get(t, 0.3), d.publish[2]))
            t += DT
        if us:
            u = [x[3] for x in us]
            out.append({'step': s['kind'], 'w': s['w'], 'n': len(us), 'u_first': round(u[0], 2),
                        'u_max_abs': round(max(abs(x) for x in u), 2), 'u_end': round(u[-1], 2),
                        'meas_mean': round(sum(x[2] for x in us) / len(us), 2)})
    return out


def sim(w_cmd_fn, T, gain=1.0, stalls=(), seed=0, yaw_loop=True):
    """w_cmd_fn(t, yaw) -> Nav2 wz. Returns trace dict."""
    rnd = random.Random(seed)
    c = AdapterCore(Config(live=False, require_localization=False, yaw_loop=yaw_loop)); est = YawRateEstimator()
    ccw = ((0.0, 0.0), (0.35, 0.0), (0.4, 0.10), (0.5, 0.10), (0.6, 0.45), (0.8, 1.25), (1.0, 1.87))
    cw = ((0.0, 0.0), (0.3, 0.0), (0.4, 0.27), (0.6, 0.54), (0.8, 1.1), (1.0, 1.6))
    def curve(u):
        tab = ccw if u > 0 else cw; a = abs(u)
        for (x0, y0), (x1, y1) in zip(tab, tab[1:]):
            if a <= x1: return math.copysign(y0 + (y1 - y0) * (a - x0) / (x1 - x0), u)
        return math.copysign(tab[-1][1], u)
    hist, rate, yaw, t, next_meas = [], 0.0, 0.0, 0.0, 0.0
    tr = []
    while t < T:
        w = w_cmd_fn(t, yaw)
        d = c.step(t, (0.0, 0.0, w), t, mc(t), yaw_rate=est.get(t, 0.3))
        u = d.publish[2] if d.publish is not None else 0.0
        hist.append(u)
        u_del = hist[-20] if len(hist) >= 20 else 0.0              # 0.4 s dead time
        g = gain * (0.0 if any(a <= t < b for a, b in stalls) else 1.0)
        rate += (g * curve(u_del) - rate) * DT / 0.2                # 0.2 s lag
        ripple = 0.35 * math.sin(2 * math.pi * 1.1 * t) if abs(rate) > 0.05 else 0.0
        yaw += (rate + ripple) * DT
        if t >= next_meas:
            est.update(t, yaw + rnd.gauss(0, 0.003), t); next_meas += 1 / 30.0
        tr.append((t, w, u, rate, yaw))
        t += DT
    return tr


def summarize(tr, a, b):
    seg = [x for x in tr if a <= x[0] < b]
    rs = [x[3] for x in seg]; us = [x[2] for x in seg]
    m = sum(rs) / len(rs)
    sign_flips = sum(1 for i in range(1, len(us)) if us[i] * us[i - 1] < 0)
    return {'rate_mean': round(m, 2), 'rate_std': round((sum((r - m) ** 2 for r in rs) / len(rs)) ** 0.5, 2),
            'u_min': round(min(us), 2), 'u_max': round(max(us), 2), 'u_sign_flips': sign_flips}


def turn90(sign, gain=1.0, stalls=()):
    """Live-test emulator: P on yaw error (|w| = clamp(1.0*|e|, 0.1, 0.5)) until |e| < 5 deg, then zero; re-command
    only after 1.5 s settle if |e| > 5 deg, at most 3 corrections (RPP itself stops inside yaw_goal_tolerance 20 deg)."""
    st = {'goal': sign * math.pi / 2, 'done_t': None, 'mode': 'turn', 'since': 0.0, 'corr': 0}
    def fn(t, yaw):
        e = st['goal'] - yaw
        if st['mode'] == 'turn':
            if abs(e) < math.radians(5) or (st['done_t'] is None and e * sign < 0):
                st['mode'], st['since'] = 'settle', t
                if st['done_t'] is None: st['done_t'] = t
                return 0.0
            return math.copysign(min(0.5, max(0.1, abs(e))), e)
        if t - st['since'] > 1.5 and abs(e) > math.radians(5) and st['corr'] < 3:
            st['mode'] = 'turn'; st['corr'] += 1
        return 0.0
    tr = sim(fn, 20.0, gain, stalls)
    final = tr[-1][4]
    peak = max(x[4] * sign for x in tr)
    return {'time_s': st['done_t'] and round(st['done_t'], 1), 'final_err_deg': round(math.degrees(final - st['goal']), 1),
            'overshoot_deg': round(math.degrees(peak - math.pi / 2), 1), 'corrections': st['corr'],
            'peak_rate': round(max(abs(x[3]) for x in tr), 2)}


if __name__ == '__main__':
    res = {'A_replay_calib2': replay()}
    B = {}
    for name, w, gain, stalls in (('stand CCW 0.5', 0.5, 1.0, ()), ('stand CW 0.5', -0.5, 1.0, ()),
                                  ('after-walk CCW 0.6 (gain 0.5)', 0.6, 0.5, ()),
                                  ('after-walk CW 0.6 (gain 0.5)', -0.6, 0.5, ()),
                                  ('stand CCW 0.6 stall 3-5 s', 0.6, 1.0, ((3.0, 5.0),))):
        on = sim(lambda t, y: w if t < 8 else 0.0, 11.0, gain, stalls)
        off = sim(lambda t, y: w if t < 8 else 0.0, 11.0, gain, stalls, yaw_loop=False)
        B[name] = {'loop_2_8s': summarize(on, 2, 8), 'passthrough_2_8s': summarize(off, 2, 8),
                   'u_after_zero_cmd': max(abs(x[2]) for x in on if x[0] > 8.2)}
    res['B_closed_loop'] = B
    res['C_turn90'] = {'CCW stand': turn90(1), 'CW stand': turn90(-1), 'CCW after-walk': turn90(1, 0.5),
                       'CW after-walk': turn90(-1, 0.5), 'CCW stand stall 1-3 s': turn90(1, 1.0, ((1.0, 3.0),))}
    print(json.dumps(res, indent=1))
    json.dump(res, open(os.path.join(EV, 'p20_yawloop_replay.json'), 'w'), indent=1)
