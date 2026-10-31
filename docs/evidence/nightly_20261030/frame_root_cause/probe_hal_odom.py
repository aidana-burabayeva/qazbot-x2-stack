import math, sys, time
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from nav_msgs.msg import Odometry
rclpy.init(); n = Node('frame_probe'); t0 = [None]; last = [-1]; rows = []
def yaw(q): return math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))
def cb(m):
    now = time.time()
    if t0[0] is None: t0[0] = now
    dt = now - t0[0]
    p = m.pose.pose
    if len(rows) < 5 or int(dt) != last[0]:
        last[0] = int(dt); rows.append((round(dt, 2), round(p.position.x, 4), round(p.position.y, 4), round(p.position.z, 3), round(yaw(p.orientation), 4)))
n.create_subscription(Odometry, '/aima/hal/odom/state', cb, qos_profile_sensor_data)
end = time.time() + float(sys.argv[1])
while time.time() < end and rclpy.ok(): rclpy.spin_once(n, timeout_sec=0.2)
for r in rows: print('t=%6.2f  x=%8.4f y=%8.4f z=%6.3f yaw=%8.4f' % r)
print('first_sample_after_probe_start_s', rows[0][0] if rows else None, 'n_rows', len(rows))
