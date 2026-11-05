# usage: python3 tol2_stateful.py  (needs ana.pkl produced by ana.py from the E2E-02 bag, and tol.py in the same dir)
exec(open('tol.py').read().split("variants=")[0])
XT, YT = 0.30, 0.35
i_xy = next(i for i, p in enumerate(P) if p[0] >= 0 and err(p[1], p[2]) <= XT)
p = P[i_xy]
print('first inside xy 0.30 m: t=%.1f s, d=%.3f m, yaw err %.2f rad, speed %.2f m/s' % (p[0], err(p[1], p[2]), wrap(p[3] - GYAW), math.hypot(*vel(i_xy))))
acc = next(i for i in range(i_xy, len(P)) if abs(wrap(P[i][3] - GYAW)) <= YT)
pa = P[acc]
print('stateful SimpleGoalChecker would accept at t=%.1f s: d=%.3f m, yaw err %.2f rad' % (pa[0], err(pa[1], pa[2]), wrap(pa[3] - GYAW)))
rx, ry = rest(acc)
print('prediction (not a measurement): rest = pose + 0.38 s * v -> error %.3f m' % err(rx, ry))
for dt in (0.5, 1.0, 2.0, 5.0, 10.0):
    q = min((x for x in P if x[0] >= pa[0] + dt), key=lambda x: x[0])
    print('recorded +%4.1f s: moved %.2f m, distance to goal %.3f m, yaw err %.2f' % (dt, math.hypot(q[1] - pa[1], q[2] - pa[2]), err(q[1], q[2]), wrap(q[3] - GYAW)))
