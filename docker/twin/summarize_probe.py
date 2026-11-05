import csv, math, sys
rows = [r for r in csv.DictReader(open(sys.argv[1])) if r['source'] == 'hal_odom']
wrap = lambda a: math.atan2(math.sin(a), math.cos(a))
def yaw(r): return float(r['yaw'])
t = [float(r['t']) for r in rows]
# first/last stationary windows
a, b = rows[int(len(rows) * 0.05)], rows[-1]
# movement onset: first time position/yaw changes > thresholds from t0
x0, y0, w0 = float(a['x']), float(a['y']), yaw(a)
on = next((float(r['t']) for r in rows if math.hypot(float(r['x']) - x0, float(r['y']) - y0) > 0.03 or abs(wrap(yaw(r) - w0)) > 0.05), None)
tot = 0.0; prev = w0
for r in rows:
    d = wrap(yaw(r) - prev); tot += d; prev = yaw(r)
print('%-22s onset=%s  displacement=%.2f m  net yaw=%+.1f deg  (unwrapped %+.1f)' % (sys.argv[1].split('/')[-1], on, math.hypot(float(b['x']) - x0, float(b['y']) - y0), math.degrees(wrap(yaw(b) - w0)), math.degrees(tot)))
