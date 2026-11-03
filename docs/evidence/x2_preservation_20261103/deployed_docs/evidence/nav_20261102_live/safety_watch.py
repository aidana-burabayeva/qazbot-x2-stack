#!/usr/bin/env python3
"""Operator-agreed abort criteria for live goals (2026-11-02): while GC busy, call goal_coordinator/stop ONCE if
scan_score < 0.20 for > 2 s, any T1 reason for > 2 s, AMCL pose step > 0.5 m or > 20 deg between updates, or goal > 120 s.
Prints one line per state change / action (event stream)."""
import time, json, math, rclpy
from std_msgs.msg import String
from std_srvs.srv import Trigger
from geometry_msgs.msg import PoseWithCovarianceStamped
rclpy.init(); n = rclpy.create_node('safety_watch_1102'); N = '/qazbot_d1_map'
st = dict(busy=False, since=None, low=None, t1=None, last=None, fired=False)
stop = n.create_client(Trigger, N + '/goal_coordinator/stop')
def say(*a): print(time.strftime('%H:%M:%S'), *a, flush=True)
def fire(why):
    if st['fired']: return
    st['fired'] = True; say('ABORT_STOP', why); stop.call_async(Trigger.Request())
def gc(m):
    b = json.loads(m.data).get('busy', False)
    if b and not st['busy']: st.update(since=time.time(), fired=False, low=None, t1=None); say('goal busy')
    if not b and st['busy']: say('goal idle', json.loads(m.data).get('fault'), json.loads(m.data).get('last_result', ''))
    st['busy'] = b
def tr(m):
    d = json.loads(m.data); s = d.get('metrics', {}).get('scan_score', 1.0); t1 = [r for r in d.get('reasons', []) if r.startswith('T1')]
    now = time.time()
    st['low'] = (st['low'] or now) if s < 0.20 else None; st['t1'] = (st['t1'] or now) if t1 else None
    if st['busy']:
        if st['low'] and now - st['low'] > 2: fire('scan_score<0.20 >2s (%.2f)' % s)
        if st['t1'] and now - st['t1'] > 2: fire('T1 >2s %s' % t1)
        if st['since'] and now - st['since'] > 120: fire('goal >120 s')
def ap(m):
    p = m.pose.pose; y = 2 * math.atan2(p.orientation.z, p.orientation.w); c = (p.position.x, p.position.y, y)
    if st['last'] and st['busy']:
        dxy = math.hypot(c[0] - st['last'][0], c[1] - st['last'][1]); dy = abs(math.degrees(math.atan2(math.sin(c[2] - st['last'][2]), math.cos(c[2] - st['last'][2]))))
        if dxy > 0.5 or dy > 20: fire('AMCL step %.2f m %.1f deg' % (dxy, dy))
    st['last'] = c
n.create_subscription(String, N + '/goal_coordinator/status', gc, 10)
n.create_subscription(String, N + '/localization_trust/status', tr, 10)
n.create_subscription(PoseWithCovarianceStamped, N + '/amcl_pose', ap, 10)
say('safety_watch armed')
rclpy.spin(n)
