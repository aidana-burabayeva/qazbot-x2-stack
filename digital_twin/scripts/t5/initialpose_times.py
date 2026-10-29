"""Header-fixed copy of a bag -> initialpose receive/stamp times. usage: initialpose_times.py BAG OUT.json"""
import json, sqlite3, sys
from rclpy.serialization import deserialize_message
from geometry_msgs.msg import PoseWithCovarianceStamped
c = sqlite3.connect('file:%s?mode=ro' % sys.argv[1], uri=True)
out = []
for ts, data in c.execute("select m.timestamp,m.data from messages m join topics t on t.id=m.topic_id where t.name='/qazbot_d1_map/initialpose' order by m.timestamp"):
    m = deserialize_message(data, PoseWithCovarianceStamped)
    out.append([ts * 1e-9, m.header.stamp.sec + m.header.stamp.nanosec * 1e-9, m.pose.pose.position.x, m.pose.pose.position.y])
json.dump(out, open(sys.argv[2], 'w')); print(sys.argv[1].split('/')[2], len(out), [round(x[0], 2) for x in out])
