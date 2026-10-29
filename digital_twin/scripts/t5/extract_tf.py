"""Dump map->odom and odom->base_footprint (recorded on the physical X2) from a rosbag2 sqlite bag to JSON.
usage: extract_tf.py BAG.db3 OUT.json"""
import json, math, sqlite3, sys
from rclpy.serialization import deserialize_message
from tf2_msgs.msg import TFMessage


def yaw(q):
    return math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))


c = sqlite3.connect('file:%s?mode=ro' % sys.argv[1], uri=True)
out = {'map_odom': [], 'odom_base': []}
for name, ts, data in c.execute(
        "select t.name, m.timestamp, m.data from messages m join topics t on t.id=m.topic_id "
        "where t.name in ('/tf','/qazbot_d1_map/tf') order by m.timestamp"):
    for t in deserialize_message(data, TFMessage).transforms:
        pair = (t.header.frame_id, t.child_frame_id)
        tr = t.transform
        row = [ts * 1e-9, t.header.stamp.sec + t.header.stamp.nanosec * 1e-9, tr.translation.x, tr.translation.y, yaw(tr.rotation)]
        if pair == ('map', 'odom'):
            out['map_odom'].append(row)
        elif pair == ('odom', 'base_footprint'):
            out['odom_base'].append(row)
json.dump(out, open(sys.argv[2], 'w'))
print({k: len(v) for k, v in out.items()})
