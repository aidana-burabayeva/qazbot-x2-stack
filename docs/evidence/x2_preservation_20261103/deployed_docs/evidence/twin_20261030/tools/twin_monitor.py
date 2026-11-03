#!/usr/bin/env python3
"""No-DDS load/impact monitor for the twin recorders. Every 10 s appends to monitor.csv:
load avg, CPU busy %, recorder CPU %, key-node CPU %, bag bytes, Nav2 scan (/qazbot_lio/scan) age stats and Trust T1 fractions
taken from the already-running orch status_logger JSONL (Trust status at 2 Hz).
Guard: 60 s scan-age p95 > 1.0 s (no-recorder OFF window 19:02-19:03 reached p95 1.89; 0.5 s is exceeded without any recorder) for 3 consecutive samples -> SIGINT hf recorder; if hf already stopped and
p95 > 1.5 s for 6 consecutive samples -> SIGINT core recorder. Events -> recorder_events.log. Guard ignores the first 75 s after a recorder (re)start (one-off start-up discovery burst)."""
import os, sys, time, json, glob, signal, subprocess
D = '/home/agi/control_backend/docs/evidence/twin_20261030'
LOG = '/home/agi/control_backend/docs/evidence/nav_20261030_1623/status_log_162726.jsonl'
WATCH = {'fastlio': 'lib/fast_lio/fastlio_mapping --ros-args', 'level_bridge': 'level_bridge.py', 'raw_scan_bridge': 'raw_scan_bridge.py',
         'controller': 'nav2_controller/controller_server', 'adapter': 'x2_motion_adapter.py', 'trust': 'localization_trust_node.py'}
HZ = os.sysconf('SC_CLK_TCK')
def pidof(sub):
    for p in os.listdir('/proc'):
        if p.isdigit():
            try:
                c = open(f'/proc/{p}/cmdline', 'rb').read().replace(b'\0', b' ').decode(errors='ignore')
                if sub in c and 'twin_monitor' not in c: return int(p)
            except Exception: pass
def ticks(pid):
    try: s = open(f'/proc/{pid}/stat').read().rsplit(')', 1)[1].split(); return int(s[11]) + int(s[12])
    except Exception: return None
def cpu_total():
    v = list(map(int, open('/proc/stat').readline().split()[1:])); return sum(v), v[3] + v[4]
def rec_pid(name):
    try:
        p = int(open(f'{D}/rec_{name}.pid').read().strip())
        return p if b'ros2 bag record' in open(f'/proc/{p}/cmdline', 'rb').read().replace(b'\0', b' ') else None
    except Exception: return None
def window(win=60.0):
    end = time.time(); sz = os.path.getsize(LOG); f = open(LOG, 'rb'); f.seek(max(0, sz - 900000))
    ages = []; n = t1 = ss = nok = 0; last_t = None
    for line in f.read().decode(errors='ignore').splitlines()[1:]:
        try: r = json.loads(line)
        except Exception: continue
        t = r.get('t', 0); last_t = t
        lt = r.get('localization_trust')
        if t < end - win or not lt: continue
        n += 1; a = (lt.get('input_ages') or {}).get('scan')
        if a is not None: ages.append(a)
        rs = [str(x) for x in (lt.get('reasons') or [])]
        if lt.get('t1_transient') or any(x.startswith('T1') for x in rs): t1 += 1
        if any(x.startswith('T1') and 'scan' in x for x in rs): ss += 1
        if not lt.get('motion_ok'): nok += 1
    ages.sort(); q = lambda p: round(ages[min(len(ages) - 1, int(p * len(ages)))], 3) if ages else ''
    return dict(log_age=round(end - last_t, 1) if last_t else '', n=n, scan_last=round(ages[-1], 3) if False else (round(ages and ages[-1] or 0, 3) if ages else ''),
                scan_p50=q(.5), scan_p95=q(.95), scan_max=round(ages[-1], 3) if ages else '',
                t1_frac=round(t1 / n, 3) if n else '', t1_scan_frac=round(ss / n, 3) if n else '', not_ok_frac=round(nok / n, 3) if n else '')
def du(path):
    s = 0
    for fp in glob.glob(path + '/**/*', recursive=True):
        try: s += os.path.getsize(fp)
        except Exception: pass
    return s
def rec_age(pid):
    try:
        st = float(open(f'/proc/{pid}/stat').read().rsplit(')', 1)[1].split()[19]) / HZ
        return float(open('/proc/uptime').read().split()[0]) - st
    except Exception: return 1e9
def event(msg):
    open(f'{D}/recorder_events.log', 'a').write(time.strftime('%F %T ') + msg + '\n')
cols = ['epoch', 'time', 'load1', 'load5', 'load15', 'cpu_busy_pct', 'rec_core_pid', 'rec_core_cpu', 'rec_hf_pid', 'rec_hf_cpu'] + \
       [k + '_cpu' for k in WATCH] + ['bags_bytes', 'status_log_age', 'n', 'scan_p50', 'scan_p95', 'scan_max', 't1_frac', 't1_scan_frac', 'not_ok_frac', 'guard']
out = f'{D}/monitor.csv'; new = not os.path.exists(out); fo = open(out, 'a', buffering=1)
if new: fo.write(','.join(cols) + '\n')
prev = {}; prev_tot = cpu_total(); bad1 = bad2 = 0; period = float(sys.argv[1]) if len(sys.argv) > 1 else 10.0
pids = {k: pidof(v) for k, v in WATCH.items()}
for k in pids: prev[k] = ticks(pids[k]) if pids[k] else None
for k in ('core', 'hf'): p = rec_pid(k); prev['rec_' + k] = (p, ticks(p) if p else None)
while True:
    time.sleep(period)
    tot = cpu_total(); dt_all = tot[0] - prev_tot[0]; busy = 100.0 * (1 - (tot[1] - prev_tot[1]) / dt_all) if dt_all else 0; prev_tot = tot
    row = dict(epoch=round(time.time(), 1), time=time.strftime('%H:%M:%S'), cpu_busy_pct=round(busy, 1))
    row['load1'], row['load5'], row['load15'] = os.getloadavg()
    for k in ('core', 'hf'):
        p = rec_pid(k); op, ot = prev.get('rec_' + k, (None, None)); t = ticks(p) if p else None
        row[f'rec_{k}_pid'] = p or ''; row[f'rec_{k}_cpu'] = round(100.0 * (t - ot) / HZ / period, 1) if (p and p == op and t is not None and ot is not None) else ''
        prev['rec_' + k] = (p, t)
    for k, v in WATCH.items():
        if not pids[k] or ticks(pids[k]) is None: pids[k] = pidof(v); prev[k] = ticks(pids[k]) if pids[k] else None; row[k + '_cpu'] = ''; continue
        t = ticks(pids[k]); row[k + '_cpu'] = round(100.0 * (t - prev[k]) / HZ / period, 1) if prev[k] is not None else ''; prev[k] = t
    row['bags_bytes'] = du(f'{D}/bags')
    try: w = window()
    except Exception as e: w = {}
    row.update(status_log_age=w.get('log_age', ''), **{k: w.get(k, '') for k in ('n', 'scan_p50', 'scan_p95', 'scan_max', 't1_frac', 't1_scan_frac', 'not_ok_frac')})
    g = ''
    fresh = w.get('log_age') != '' and w.get('log_age', 99) < 30 and w.get('n', 0) >= 20
    p95 = w.get('scan_p95') if fresh and w.get('scan_p95') != '' else None
    bad1 = bad1 + 1 if (p95 is not None and p95 > 1.0) else 0
    bad2 = bad2 + 1 if (p95 is not None and p95 > 1.5) else 0
    hf, core = rec_pid('hf'), rec_pid('core')
    # grace: recorder start-up (DDS discovery, latched topics) causes a one-off ~10 s scan-age burst; ignore 75 s after any (re)start
    young = [p for p in (hf, core) if p and rec_age(p) < 75]
    if young: bad1 = bad2 = 0
    if hf and bad1 >= 3:
        os.kill(hf, signal.SIGINT); g = 'drop_hf'; event(f'GUARD drop hf recorder pid={hf}: scan p95(60s)={p95} >1.0 for {bad1} samples'); bad1 = 0
    elif not hf and core and bad2 >= 6:
        os.kill(core, signal.SIGINT); g = 'stop_core'; event(f'GUARD stop core recorder pid={core}: scan p95(60s)={p95} >1.5 for {bad2} samples'); bad2 = 0
    row['guard'] = g
    fo.write(','.join(str(row.get(c, '')) for c in cols) + '\n')
