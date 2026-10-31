"""Records raw MuJoCo pelvis pose (/aima/hal/odom/state) and the NAV pose (TF map->odom->base_footprint on /qazbot_d1_map/tf) at ~10 Hz.
truth_map = start (+) raw_world_pose  (the world.py contract: walls were generated with this transform). Output CSV on stdout at exit.
usage: pose_probe.py <seconds> <start_x> <start_y> <start_yaw>"""
import csv, math, sys, time
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data, QoSProfile, ReliabilityPolicy, DurabilityPolicy
from nav_msgs.msg import Odometry
from tf2_msgs.msg import TFMessage
secs, sx, sy, syaw = float(sys.argv[1]), *map(float, sys.argv[2:5])
yaw = lambda q: math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))
rclpy.init(); n = Node('pose_probe'); raw = [None]; edges = {}; rows = []; t0 = time.time(); last = [0.0]
def on_raw(m): p = m.pose.pose; raw[0] = (p.position.x, p.position.y, yaw(p.orientation))
def on_tf(m):
    for t in m.transforms:
        edges[(t.header.frame_id, t.child_frame_id)] = (t.transform.translation.x, t.transform.translation.y, yaw(t.transform.rotation))
def comp(a, b): c, s = math.cos(a[2]), math.sin(a[2]); return (a[0] + c * b[0] - s * b[1], a[1] + s * b[0] + c * b[1], a[2] + b[2])
n.create_subscription(Odometry, '/aima/hal/odom/state', on_raw, qos_profile_sensor_data)
n.create_subscription(TFMessage, '/qazbot_d1_map/tf', on_tf, 100)
def sample():
    now = time.time() - t0
    if now - last[0] < 0.1 or raw[0] is None: return
    mo, ob = edges.get(('map', 'odom')), edges.get(('odom', 'base_footprint'))
    if ob is None: return
    nav = comp(mo, ob) if mo else ob
    truth = comp((sx, sy, syaw), raw[0])
    dx, dy = nav[0] - truth[0], nav[1] - truth[1]; dyaw = math.atan2(math.sin(nav[2] - truth[2]), math.cos(nav[2] - truth[2]))
    last[0] = now; rows.append((round(now, 2), *[round(v, 4) for v in (*raw[0], *nav, *truth, math.hypot(dx, dy), dyaw)]))
end = t0 + secs
while time.time() < end and rclpy.ok(): rclpy.spin_once(n, timeout_sec=0.05); sample()
w = csv.writer(sys.stdout); w.writerow('t,raw_x,raw_y,raw_yaw,nav_x,nav_y,nav_yaw,truth_x,truth_y,truth_yaw,pos_err,yaw_err'.split(',')); w.writerows(rows)
