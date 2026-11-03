#!/usr/bin/env python3
"""Offline per-goal analysis of a nav_bag: goal, start/final AMCL pose, plan length + clearance to map walls, actual path
(native odom), command chain stats (cmd_vel_nav -> cmd_vel -> cmd_vel_authorized -> adapter dry_run_cmd / MC velocity),
measured speeds, map->odom corrections, Trust events, pelvis (body_pose) and action status. argv: BAG [robot_radius]"""
import sys, json, math, numpy as np
sys.path.insert(0, '/home/agi/control_backend/ws/src/navigation')
import loc_trust as L
import rosbag2_py
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message
bag = sys.argv[1]; RR = float(sys.argv[2]) if len(sys.argv) > 2 else 0.454
N = '/qazbot_d1_map'
r = rosbag2_py.SequentialReader(); r.open(rosbag2_py.StorageOptions(uri=bag, storage_id='mcap'), rosbag2_py.ConverterOptions('', ''))
types = {t.name: t.type for t in r.get_all_topics_and_types()}
yaw = lambda q: 2 * math.atan2(q.z, q.w)
D = {k: [] for k in ('goal', 'amcl', 'odom', 'mo', 'plan', 'nav', 'cmd', 'auth', 'dry', 'mcv', 'trust', 'gc', 'ad', 'act', 'body', 'mc')}
skip = {'/qazbot_lio/scan', '/aima/mc/leg_odometry', N + '/nav2/lookahead_collision_arc', N + '/tf_static', '/qazbot_native/status'}
while r.has_next():
    t, data, ts = r.read_next(); ts *= 1e-9
    if t in skip or t not in types: continue
    try: m = deserialize_message(data, get_message(types[t]))
    except Exception: continue
    tw = lambda m: (ts, m.linear.x, m.linear.y, m.angular.z)
    if t == N + '/goal_pose': p = m.pose; D['goal'].append((ts, p.position.x, p.position.y, yaw(p.orientation)))
    elif t == N + '/amcl_pose': p = m.pose.pose; D['amcl'].append((ts, p.position.x, p.position.y, yaw(p.orientation)))
    elif t == '/qazbot_native/odom': p = m.pose.pose; v = m.twist.twist; D['odom'].append((ts, p.position.x, p.position.y, yaw(p.orientation), v.linear.x, v.angular.z))
    elif t == N + '/tf':
        for x in m.transforms:
            if x.header.frame_id == 'map': tr = x.transform.translation; D['mo'].append((ts, tr.x, tr.y, yaw(x.transform.rotation)))
    elif t == N + '/nav2/plan' and len(m.poses): D['plan'].append((ts, [(q.pose.position.x, q.pose.position.y) for q in m.poses]))
    elif t == N + '/nav2/cmd_vel_nav': D['nav'].append(tw(m))
    elif t == N + '/cmd_vel': D['cmd'].append(tw(m))
    elif t == N + '/cmd_vel_authorized': D['auth'].append(tw(m))
    elif t == N + '/motion_adapter/dry_run_cmd': D['dry'].append(tw(m))
    elif t == '/aima/mc/locomotion/velocity':
        try: D['mcv'].append((ts, m.source.name if hasattr(m, 'source') else '', m.forward_velocity, m.lateral_velocity, m.angular_velocity))
        except Exception: pass
    elif t == N + '/localization_trust/status': d = json.loads(m.data); D['trust'].append((ts, d.get('ready'), d.get('motion_ok'), d.get('reasons'), d.get('metrics', {}).get('scan_score'), d.get('metrics', {}).get('second_mode_weight')))
    elif t == N + '/goal_coordinator/status': D['gc'].append((ts, m.data))
    elif t == N + '/motion_adapter/status': D['ad'].append((ts, m.data))
    elif t.endswith('navigate_to_pose/_action/status'): D['act'].append((ts, t, [s.status for s in m.status_list]))
    elif t == '/aima/mc/body_pose': D['body'].append((ts, str(m)[:0]))
    elif t == '/aima/mc/common/state': D['mc'].append((ts, m.action_info.action_desc, m.input_source.name))
out = {}
grid = L.OccGrid.from_pgm('/home/agi/control_backend/ws/src/maps/d1_qazbot_20261029/map.yaml')
def at(a, t, i0=1):
    a2 = [x for x in a if x[0] <= t]; return a2[-1] if a2 else (a[0] if a else None)
def stats(a, k):
    v = np.array([x[k] for x in a]); return dict(n=len(v), max=float(np.max(v)), min=float(np.min(v)), mean_abs=float(np.mean(np.abs(v)))) if len(v) else None
if D['goal']:
    g = D['goal'][0]; t0 = g[0]; out['goal'] = g[1:]
    s = at(D['amcl'], t0); out['start_amcl'] = s[1:] if s else None
    if D['plan']:
        P = np.array(D['plan'][0][1]); out['plan_first'] = dict(n=len(P), length=float(np.sum(np.hypot(*np.diff(P, axis=0).T))),
            min_wall_dist=float(np.min(grid.dist_at(P))), clearance_minus_radius=float(np.min(grid.dist_at(P)) - RR))
        out['n_plans'] = len(D['plan'])
    for k in ('nav', 'cmd', 'auth', 'dry'):
        out['cmd_' + k] = dict(vx=stats(D[k], 1), wz=stats(D[k], 3)) if D[k] else None
    od = [x for x in D['odom'] if x[0] >= t0]
    if od:
        o = np.array(od); out['native_path'] = float(np.sum(np.hypot(np.diff(o[:, 1]), np.diff(o[:, 2])))); out['native_speed'] = dict(vx_max=float(np.max(np.abs(o[:, 4]))), wz_max=float(np.max(np.abs(o[:, 5]))))
    mo = [x for x in D['mo'] if x[0] >= t0]
    if len(mo) > 1:
        m = np.array(mo); out['map_odom_steps_gt_0_05'] = int((np.hypot(np.diff(m[:, 1]), np.diff(m[:, 2])) > 0.05).sum()); out['map_odom_total'] = float(math.hypot(m[-1, 1] - m[0, 1], m[-1, 2] - m[0, 2]))
    tr = [x for x in D['trust'] if x[0] >= t0]
    ev = {}
    for x in tr:
        for y in x[3] or []: ev[y] = ev.get(y, 0) + 1
    out['trust_events'] = ev; out['trust_score_med'] = float(np.median([x[4] for x in tr if x[4] is not None])) if tr else None
    e = D['amcl'][-1]; out['final_amcl'] = e[1:]; out['final_err_xy'] = math.hypot(e[1] - g[1], e[2] - g[2]); out['final_err_yaw_deg'] = math.degrees(L.wrap(e[3] - g[3]))
out['mc_states'] = sorted({x[1:] for x in D['mc']}); out['mc_vel_sources'] = sorted({x[1] for x in D['mcv']})
out['action_status_seq'] = [s for _, t, s in D['act'] if t.startswith(N + '/navigate')][-3:]
out['n_body_pose'] = len(D['body'])
json.dump(dict(out, plan=D['plan'][0][1] if D['plan'] else None), open(bag.rstrip('/') + '_nav_analysis.json', 'w'), default=float)
print(json.dumps(out, default=lambda x: round(float(x), 3) if isinstance(x, (float, np.floating)) else str(x), indent=0))
