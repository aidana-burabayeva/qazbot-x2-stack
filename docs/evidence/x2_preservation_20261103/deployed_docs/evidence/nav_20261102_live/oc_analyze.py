#!/usr/bin/env python3
"""Offline analysis of oc_bag: trajectories (AMCL map pose, native odom, FAST-LIO odom), Trust timeline, scan score along path."""
import sys, json, math, numpy as np
sys.path.insert(0, '/home/agi/control_backend/ws/src/navigation')
import loc_trust as L
import rosbag2_py
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message
bag = sys.argv[1]
r = rosbag2_py.SequentialReader(); r.open(rosbag2_py.StorageOptions(uri=bag, storage_id='mcap'), rosbag2_py.ConverterOptions('', ''))
types = {t.name: t.type for t in r.get_all_topics_and_types()}
want = {'/qazbot_d1_map/amcl_pose', '/qazbot_native/odom', '/qazbot_lio/odometry', '/qazbot_d1_map/localization_trust/status', '/qazbot_d1_map/tf', '/qazbot_lio/scan', '/qazbot_native/status', '/aima/mc/common/state'}
yaw = lambda q: 2 * math.atan2(q.z, q.w)
amcl, nat, lio, trust, mo, scans, natst, mcs = [], [], [], [], [], [], [], []
while r.has_next():
    t, data, ts = r.read_next()
    if t not in want: continue
    m = deserialize_message(data, get_message(types[t])); ts *= 1e-9
    if t.endswith('amcl_pose'): p = m.pose.pose; amcl.append((ts, p.position.x, p.position.y, yaw(p.orientation)))
    elif t == '/qazbot_native/odom': p = m.pose.pose; nat.append((ts, p.position.x, p.position.y, yaw(p.orientation)))
    elif t == '/qazbot_lio/odometry': p = m.pose.pose; lio.append((ts, p.position.x, p.position.y, yaw(p.orientation)))
    elif t.endswith('trust/status'): d = json.loads(m.data); trust.append((ts, d.get('ready'), d.get('motion_ok'), d.get('reasons'), d.get('metrics', {})))
    elif t.endswith('/tf'):
        for x in m.transforms:
            if x.header.frame_id == 'map': tr = x.transform.translation; mo.append((ts, tr.x, tr.y, yaw(x.transform.rotation)))
    elif t == '/qazbot_lio/scan' and len(scans) < 100000: scans.append((ts, m))
    elif t == '/qazbot_native/status': natst.append((ts, json.loads(m.data)))
    elif t == '/aima/mc/common/state': mcs.append((ts, m.action_info.action_desc, m.input_source.name))
def plen(a):
    a = np.array(a); return float(np.sum(np.hypot(np.diff(a[:, 1]), np.diff(a[:, 2])))) if len(a) > 1 else 0
nat_a = np.array(nat); mov = np.hypot(np.diff(nat_a[:, 1]), np.diff(nat_a[:, 2])) / np.maximum(np.diff(nat_a[:, 0]), 1e-3)
moving_t = nat_a[1:, 0][mov > 0.05]
t0, t1 = (moving_t[0], moving_t[-1]) if len(moving_t) else (nat_a[0, 0], nat_a[-1, 0])
out = dict(bag_span=[nat_a[0, 0], nat_a[-1, 0]], walk=[t0, t1, t1 - t0], path_native=plen(nat), path_lio=plen(lio), path_amcl=plen(amcl), n_amcl=len(amcl))
def at(a, t):
    a = np.array(a); i = np.searchsorted(a[:, 0], t); i = min(max(i, 0), len(a) - 1); return a[i, 1:]
for name, a in (('native', nat), ('lio', lio), ('amcl', amcl)):
    if len(a) > 1:
        s, e = at(a, t0 - 2), at(a, t1 + 2); out[name + '_net'] = [float(math.hypot(e[0] - s[0], e[1] - s[1])), math.degrees(L.wrap(e[2] - s[2]))]
mo_a = np.array(mo); dmo = np.hypot(np.diff(mo_a[:, 1]), np.diff(mo_a[:, 2])); dmy = np.abs([L.wrap(x) for x in np.diff(mo_a[:, 3])])
out['map_odom'] = dict(n=len(mo_a), max_step_m=float(dmo.max()), max_step_deg=math.degrees(float(dmy.max())), total_corr_m=float(math.hypot(mo_a[-1, 1] - mo_a[0, 1], mo_a[-1, 2] - mo_a[0, 2])), steps_gt_0_1=int((dmo > 0.1).sum()))
reasons = {}
for ts, rd, mok, rs, m in trust:
    for x in rs or []: reasons[x] = reasons.get(x, 0) + 1
out['trust'] = dict(n=len(trust), ready_frac=float(np.mean([x[1] for x in trust])), motion_ok_frac=float(np.mean([bool(x[2]) for x in trust])), reasons=reasons)
sc = np.array([(ts, m.get('scan_score', np.nan), m.get('second_mode_weight', np.nan), m.get('sigma_xy', np.nan)) for ts, rd, mok, rs, m in trust])
for lab, (a, b) in (('office_before', (sc[0, 0], t0)), ('walk', (t0, t1)), ('after', (t1, sc[-1, 0]))):
    k = (sc[:, 0] >= a) & (sc[:, 0] <= b)
    if k.any(): out['score_' + lab] = dict(n=int(k.sum()), score_med=float(np.nanmedian(sc[k, 1])), score_min=float(np.nanmin(sc[k, 1])), second_med=float(np.nanmedian(sc[k, 2])), sigma_med=float(np.nanmedian(sc[k, 3])))
out['native_latched'] = sorted({str(d.get('latched')) for _, d in natst}); out['native_fault'] = sorted({str(d.get('fault')) for _, d in natst})
out['mc_states'] = sorted({(a, b) for _, a, b in mcs})
json.dump(dict(out, amcl=amcl, native=nat[::10], lio=lio[::5], score=sc.tolist()), open(bag.rstrip('/') + '_analysis.json', 'w'), default=float)
print(json.dumps(out, default=lambda x: round(float(x), 3), indent=0))
