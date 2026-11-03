import sys, pickle
import rosbag2_py
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message
from rosidl_runtime_py import message_to_ordereddict
bag, out = sys.argv[1], sys.argv[2]
want = ['/aima/mc/leg_odometry','/aima/mc/common/state','/aima/mc/locomotion/velocity','/qazbot_d1_map/cmd_vel_authorized','/qazbot_native/odom']
rd = rosbag2_py.SequentialReader()
rd.open(rosbag2_py.StorageOptions(uri=bag, storage_id='mcap'), rosbag2_py.ConverterOptions('', ''))
cls = {t.name: get_message(t.type) for t in rd.get_all_topics_and_types() if t.name in want}
rd.set_filter(rosbag2_py.StorageFilter(topics=list(cls)))
R = {t: [] for t in cls}
def st(h): return h.stamp.sec + h.stamp.nanosec*1e-9
def flat(d, pre=''):
    o = {}
    for k, v in d.items():
        if k == 'header': continue
        if isinstance(v, dict): o.update(flat(v, pre+k+'.'))
        else: o[pre+k] = v
    return o
last = None
while rd.has_next():
    n, data, ts = rd.read_next()
    m = deserialize_message(data, cls[n])
    if n == '/aima/mc/leg_odometry':
        p, q, tw = m.pose.pose.position, m.pose.pose.orientation, m.twist.twist
        c = m.pose.covariance
        R[n].append((st(m.header), ts*1e-9, p.x,p.y,p.z,q.x,q.y,q.z,q.w, tw.linear.x,tw.linear.y,tw.linear.z,tw.angular.x,tw.angular.y,tw.angular.z, m.header.frame_id, m.child_frame_id, tuple(c)))
    elif n == '/qazbot_native/odom':
        p, q, tw = m.pose.pose.position, m.pose.pose.orientation, m.twist.twist
        R[n].append((st(m.header), ts*1e-9, p.x,p.y,p.z,q.z,q.w, tw.linear.x,tw.linear.y,tw.angular.z))
    elif n == '/aima/mc/common/state':
        f = flat(message_to_ordereddict(m))
        R[n].append((ts*1e-9, st(m.header), f))
    elif n == '/aima/mc/locomotion/velocity':
        R[n].append((ts*1e-9, st(m.header), m.source, m.forward_velocity, m.lateral_velocity, m.angular_velocity))
    else:
        R[n].append((ts*1e-9, m.twist.linear.x if hasattr(m,'twist') else m.linear.x, m.twist.angular.z if hasattr(m,'twist') else m.angular.z))
pickle.dump(R, open(out, 'wb'))
print(bag, {k: len(v) for k, v in R.items()}, flush=True)
