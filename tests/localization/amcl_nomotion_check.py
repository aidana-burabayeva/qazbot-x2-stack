"""Seed AMCL, then repeatedly request a no-motion update and log the estimate. Publishes no velocity."""
import sys, math, time, json, numpy as np, rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy
from geometry_msgs.msg import PoseWithCovarianceStamped
from std_srvs.srv import Empty
from nav2_msgs.msg import ParticleCloud
x0, y0, yaw0, sxy, syaw, n = float(sys.argv[1]), float(sys.argv[2]), math.radians(float(sys.argv[3])), float(sys.argv[4]), float(sys.argv[5]), int(sys.argv[6])
rclpy.init(); node = Node('amcl_rest_check'); got = []; parts = []
node.create_subscription(PoseWithCovarianceStamped, '/qazbot_d1_map/amcl_pose', got.append, QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE, durability=DurabilityPolicy.TRANSIENT_LOCAL))
node.create_subscription(ParticleCloud, '/qazbot_d1_map/particle_cloud', parts.append, QoSProfile(depth=2, reliability=ReliabilityPolicy.BEST_EFFORT))
pub = node.create_publisher(PoseWithCovarianceStamped, '/qazbot_d1_map/initialpose', 1)
cli = node.create_client(Empty, '/qazbot_d1_map/request_nomotion_update'); assert cli.wait_for_service(timeout_sec=30)
def spin(t):
    end = time.time() + t
    while time.time() < end: rclpy.spin_once(node, timeout_sec=0.05)
spin(1.0)
m = PoseWithCovarianceStamped(); m.header.frame_id = 'map'
m.pose.pose.position.x, m.pose.pose.position.y = x0, y0; m.pose.pose.orientation.z, m.pose.pose.orientation.w = math.sin(yaw0 / 2), math.cos(yaw0 / 2)
c = [0.0] * 36; c[0] = c[7] = sxy ** 2; c[35] = math.radians(syaw) ** 2; m.pose.covariance = c
while pub.get_subscription_count() == 0: spin(0.2)
pub.publish(m); spin(1.0); got.clear()
rows = []
for i in range(n):
    f = cli.call_async(Empty.Request()); rclpy.spin_until_future_complete(node, f, timeout_sec=5); spin(0.6)
    if got:
        p = got[-1].pose; q = p.pose.orientation; yaw = math.degrees(2 * math.atan2(q.z, q.w))
        rows.append((p.pose.position.x, p.pose.position.y, yaw, p.covariance[0], p.covariance[7], p.covariance[35]))
r = np.array(rows)
if len(r):
    print('updates observed', len(r), 'of', n)
    print('first x,y,yaw %.3f %.3f %.1f | last x,y,yaw %.3f %.3f %.1f' % (*r[0, :3], *r[-1, :3]))
    print('last var x,y,yaw(rad2): %.4f %.4f %.4f' % tuple(r[-1, 3:]))
    print('seed->last distance %.3f m, yaw diff %.1f deg' % (math.hypot(r[-1,0]-x0, r[-1,1]-y0), (r[-1,2]-math.degrees(yaw0)+180)%360-180))
    print('range over run: x %.3f y %.3f yaw %.1f' % (np.ptp(r[:,0]), np.ptp(r[:,1]), np.ptp(r[:,2])))
else: print('no amcl_pose received')
if parts:
    P = np.array([[q.pose.position.x, q.pose.position.y] for q in parts[-1].particles]); print('particles', len(P), 'spread std x,y %.3f %.3f' % (P[:,0].std(), P[:,1].std()))
