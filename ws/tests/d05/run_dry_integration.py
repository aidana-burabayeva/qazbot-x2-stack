import csv, json, os, signal, statistics, subprocess, sys, tempfile, time

TOOL, STUB = sys.argv[1], sys.argv[2]
assert os.environ['ROS_DOMAIN_ID'] == '79'
tmp = tempfile.mkdtemp(prefix='d05_int_')
fails = []


def check(name, cond, info=''):
    print(('PASS ' if cond else 'FAIL ') + name + (f'  [{info}]' if info else ''))
    if not cond:
        fails.append(name)


def stub(scn):
    out = os.path.join(tmp, f'rx_{scn}.jsonl')
    p = subprocess.Popen([sys.executable, STUB, '--scenario', scn, '--out', out], stderr=subprocess.PIPE)
    time.sleep(2.0)   # discovery of the stub
    return p, out


def rx(out):
    return [json.loads(l) for l in open(out) if l.strip()]


def tool(args, csvp, sig_after=None):
    p = subprocess.Popen([sys.executable, TOOL, '--log-csv', csvp, '--preflight-timeout', '6'] + args,
                         stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True)
    if sig_after:
        time.sleep(sig_after)
        p.send_signal(signal.SIGINT)
    o = p.communicate(timeout=60)[0]
    return p.returncode, o


def rows(csvp):
    return list(csv.DictReader(open(csvp))) if os.path.exists(csvp) else []


# 1. nominal run
s, out = stub('ok'); c1 = os.path.join(tmp, 'ok.csv')
code, o = tool(['--vx', '0.3', '--duration', '4', '--run-id', 'T1'], c1)
s.terminate(); s.wait(); r = rx(out)
nz = [x for x in r if abs(x['vx']) > 1e-9]; zs = [x for x in r if x['vx'] == 0.0 and x['t'] >= nz[-1]['t']] if nz else []
dts = [b['t'] - a['t'] for a, b in zip(nz, nz[1:])]
check('nominal exit 0', code == 0, o[-200:])
check('nominal nonzero msgs ~ 4 s * 20 Hz', 78 <= len(nz) <= 81, len(nz))
check('nominal vx constant, vy=wz=0', all(abs(x['vx'] - 0.3) < 1e-9 and x['vy'] == 0 and x['wz'] == 0 for x in nz))
check('nominal rate: mean dt 0.05 +-2%, max dt < 0.10 (adapter cmd timeout 0.15 s)', 0.049 <= statistics.mean(dts) <= 0.051 and max(dts) < 0.10, f'mean {statistics.mean(dts):.4f} median {statistics.median(dts):.4f} max {max(dts):.3f}')
check('nominal span ~4 s incl settle', 3.85 <= nz[-1]['t'] - nz[0]['t'] + 0.05 <= 4.1, f'{nz[-1]["t"]-nz[0]["t"]+0.05:.3f}')
check('nominal explicit zero burst >=10 at end', len(zs) >= 10, len(zs))
ev = [x['event'] for x in rows(c1)]
check('nominal csv start,settle_end,cmd_end,stop', ev == ['start', 'settle_end', 'cmd_end', 'stop'], ev)

# 2. refusals (no nonzero command may be sent)
for scn, why in (('not_armed', 'not armed'), ('rc_active', 'RC active'), ('dry_adapter', 'not live')):
    s, out = stub(scn); cp = os.path.join(tmp, scn + '.csv')
    code, o = tool(['--vx', '0.3', '--duration', '4'], cp)
    s.terminate(); s.wait(); r = rx(out)
    check(f'{scn}: exit 3 (refused)', code == 3, f'code {code}')
    check(f'{scn}: reason mentions "{why}"', why in o)
    check(f'{scn}: zero nonzero commands sent', not any(abs(x['vx']) > 1e-9 for x in r))
    check(f'{scn}: csv refused row', [x['event'] for x in rows(cp)] == ['refused'])

# 3. limits (no ROS traffic at all)
code, o = tool(['--vx', '0.7', '--duration', '4'], os.path.join(tmp, 'l.csv'))
check('vx 0.7 -> exit 2', code == 2)
code, o = tool(['--vx', '0.3', '--duration', '13'], os.path.join(tmp, 'l.csv'))
check('duration 13 -> exit 2', code == 2)

# 4. adapter latches mid-run -> abort + zero
s, out = stub('latch_mid'); cp = os.path.join(tmp, 'latch.csv')
code, o = tool(['--vx', '0.3', '--duration', '10'], cp)
s.terminate(); s.wait(); r = rx(out); nz = [x for x in r if abs(x['vx']) > 1e-9]
check('latch: exit 4', code == 4, f'code {code}')
check('latch: stopped early (<6 s of cmds)', nz[-1]['t'] - nz[0]['t'] < 6, f'{nz[-1]["t"]-nz[0]["t"]:.2f}')
check('latch: zeros after last nonzero', sum(1 for x in r if x['vx'] == 0 and x['t'] > nz[-1]['t']) >= 10)
check('latch: csv abort row with reason', [x['event'] for x in rows(cp)][-1] == 'abort' and 'latched' in rows(cp)[-1]['reason'])

# 5. SIGINT mid-run -> zero
s, out = stub('ok'); cp = os.path.join(tmp, 'sigint.csv')
code, o = tool(['--vx', '0.3', '--duration', '10'], cp, sig_after=5.0)   # 5 s covers ~3 s of preflight listen + run
s.terminate(); s.wait(); r = rx(out); nz = [x for x in r if abs(x['vx']) > 1e-9]
check('sigint: exit 4', code == 4, f'code {code}')
check('sigint: ran then stopped before 10 s', 0.5 < nz[-1]['t'] - nz[0]['t'] < 9, f'{nz[-1]["t"]-nz[0]["t"]:.2f}')
check('sigint: zeros after last nonzero', sum(1 for x in r if x['vx'] == 0 and x['t'] > nz[-1]['t']) >= 10)
check('sigint: csv abort row', [x['event'] for x in rows(cp)][-1] == 'abort')

print('RESULT', 'FAIL ' + str(fails) if fails else 'ALL PASS', ' tmp', tmp)
sys.exit(1 if fails else 0)
