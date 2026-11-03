#!/usr/bin/env python3
"""Read-only: sample Trust status + amcl_pose for N s at 1 Hz; print compact lines."""
import sys, time, json, math, rclpy
from std_msgs.msg import String
from geometry_msgs.msg import PoseWithCovarianceStamped
rclpy.init(); n = rclpy.create_node('trust_watch_1102'); st = {}; ap = {}
n.create_subscription(String, '/qazbot_d1_map/localization_trust/status', lambda m: st.update(json.loads(m.data)), 10)
def onp(m):
    p = m.pose.pose; ap.update(x=p.position.x, y=p.position.y, yaw=2 * math.atan2(p.orientation.z, p.orientation.w))
n.create_subscription(PoseWithCovarianceStamped, '/qazbot_d1_map/amcl_pose', onp, 10)
end = time.time() + float(sys.argv[1]); nxt = time.time() + 1
while time.time() < end:
    rclpy.spin_once(n, timeout_sec=0.05)
    if time.time() >= nxt and st:
        m = st.get('metrics', {}); nxt += 1
        print(time.strftime('%H:%M:%S'), 'pose' , {k: round(v, 3) for k, v in ap.items()}, 'ready', st.get('ready'), 'mok', st.get('motion_ok'),
              'score %.2f sig %.2f 2nd %.2f' % (m.get('scan_score', -1), m.get('sigma_xy', -1), m.get('second_mode_weight', -1)), st.get('reasons'), flush=True)
