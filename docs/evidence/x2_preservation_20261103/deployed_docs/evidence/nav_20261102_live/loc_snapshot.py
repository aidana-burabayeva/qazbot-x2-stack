#!/usr/bin/env python3
"""Read-only localization snapshot -> JSON: Trust status, native status, amcl_pose, particle cloud stats, map->odom,
odom->base_footprint, AMCL params, Trust ambiguity at current pose. argv: OUT_JSON"""
import sys, time, json, math, numpy as np, rclpy
sys.path.insert(0, '/home/agi/control_backend/ws/src/navigation')
import loc_trust as L
from std_msgs.msg import String
from sensor_msgs.msg import LaserScan
from tf2_msgs.msg import TFMessage
from nav2_msgs.msg import ParticleCloud
from geometry_msgs.msg import PoseWithCovarianceStamped
from rcl_interfaces.srv import GetParameters
from rclpy.qos import qos_profile_sensor_data, QoSProfile, DurabilityPolicy, ReliabilityPolicy
NS = '/qazbot_d1_map'
rclpy.init(); n = rclpy.create_node('loc_snapshot_1102'); d = {'wall': time.time()}; scans = []; tfs = {}
yaw = lambda q: 2 * math.atan2(q.z, q.w)
n.create_subscription(String, NS + '/localization_trust/status', lambda m: d.__setitem__('trust', json.loads(m.data)), 10)
n.create_subscription(String, '/qazbot_native/status', lambda m: d.__setitem__('native', json.loads(m.data)), 10)
def onp(m):
    p = m.pose.pose; c = np.array(m.pose.covariance).reshape(6, 6)
    d['amcl_pose'] = dict(stamp=m.header.stamp.sec + m.header.stamp.nanosec * 1e-9, x=p.position.x, y=p.position.y, yaw=yaw(p.orientation), sxx=c[0, 0], syy=c[1, 1], syaw=c[5, 5])
n.create_subscription(PoseWithCovarianceStamped, NS + '/amcl_pose', onp, QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL, reliability=ReliabilityPolicy.RELIABLE))
def onpc(m):
    xy = np.array([[q.pose.position.x, q.pose.position.y, yaw(q.pose.orientation), q.weight] for q in m.particles])
    if len(xy):
        w = xy[:, 3] / max(xy[:, 3].sum(), 1e-12); mx, my = (w * xy[:, 0]).sum(), (w * xy[:, 1]).sum()
        far = np.hypot(xy[:, 0] - mx, xy[:, 1] - my) > 0.5
        d['particles'] = dict(n=len(xy), mean=[mx, my], std=[float(xy[:, 0].std()), float(xy[:, 1].std()), float(xy[:, 2].std())], weight_far_0_5m=float(w[far].sum()))
n.create_subscription(ParticleCloud, NS + '/particle_cloud', onpc, qos_profile_sensor_data)
def ontf(m):
    for t in m.transforms:
        tr = t.transform.translation; tfs[t.header.frame_id + '->' + t.child_frame_id] = dict(x=tr.x, y=tr.y, yaw=yaw(t.transform.rotation), stamp=t.header.stamp.sec + t.header.stamp.nanosec * 1e-9)
n.create_subscription(TFMessage, NS + '/tf', ontf, QoSProfile(depth=50))
n.create_subscription(LaserScan, '/qazbot_lio/scan', lambda m: scans.append(m), qos_profile_sensor_data)
end = time.time() + 8
while time.time() < end: rclpy.spin_once(n, timeout_sec=0.05)
d['tf'] = tfs
cli = n.create_client(GetParameters, NS + '/amcl/get_parameters')
names = ['max_beams', 'laser_model_type', 'update_min_d', 'update_min_a', 'alpha1', 'alpha2', 'alpha3', 'alpha4', 'alpha5', 'do_beamskip', 'min_particles', 'max_particles', 'recovery_alpha_slow', 'recovery_alpha_fast', 'resample_interval', 'z_hit', 'z_rand', 'sigma_hit', 'laser_max_range']
if cli.wait_for_service(timeout_sec=3):
    f = cli.call_async(GetParameters.Request(names=names)); rclpy.spin_until_future_complete(n, f, timeout_sec=3)
    if f.result():
        d['amcl_params'] = {k: (v.double_value if v.type == 3 else v.integer_value if v.type == 2 else v.bool_value if v.type == 1 else v.string_value) for k, v in zip(names, f.result().values)}
mo, ob = tfs.get('map->odom'), tfs.get('odom->base_footprint')
if mo and ob and scans:
    pose = L.compose((mo['x'], mo['y'], mo['yaw']), (ob['x'], ob['y'], ob['yaw'])); d['map_base'] = pose
    cfg = L.TrustConfig(); grid = L.OccGrid.from_pgm('/home/agi/control_backend/ws/src/maps/d1_qazbot_20261029/map.yaml')
    pts = L.scan_endpoints(scans[-1].ranges, scans[-1].angle_min, scans[-1].angle_increment, cfg.rmin, cfg.rmax)
    d['scan_points'] = len(pts); d['ambiguity'] = L.ambiguity(pts, grid, pose, cfg)
json.dump(d, open(sys.argv[1], 'w'), indent=1, default=float)
t = d.get('trust', {}); m = t.get('metrics', {})
print(json.dumps(dict(ready=t.get('ready'), motion_ok=t.get('motion_ok'), reasons=t.get('reasons'), score=m.get('scan_score'), sigma=m.get('sigma_xy'), second=m.get('second_mode_weight'),
    map_base=d.get('map_base'), amcl=d.get('amcl_pose'), particles=d.get('particles'), native=(d.get('native') or {}).get('latched'), native_fault=(d.get('native') or {}).get('fault'),
    amb=d.get('ambiguity'), params=d.get('amcl_params')), default=float))
