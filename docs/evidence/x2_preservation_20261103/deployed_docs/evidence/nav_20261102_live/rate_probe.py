#!/usr/bin/env python3
"""Count messages per topic for N s (read-only). argv: N topic:pkg/msg/Type ..."""
import sys, time, importlib, rclpy
from rclpy.qos import qos_profile_sensor_data
rclpy.init(); n = rclpy.create_node('rate_probe_1102'); c = {}
for a in sys.argv[2:]:
    t, ty = a.split(':'); p, _, m = ty.split('/'); M = getattr(importlib.import_module(p + '.msg'), m); c[t] = 0
    n.create_subscription(M, t, lambda _m, t=t: c.__setitem__(t, c[t] + 1), qos_profile_sensor_data)
end = time.time() + float(sys.argv[1])
while time.time() < end: rclpy.spin_once(n, timeout_sec=0.05)
for t, k in c.items(): print('%-45s %7.1f Hz' % (t, k / float(sys.argv[1])))
