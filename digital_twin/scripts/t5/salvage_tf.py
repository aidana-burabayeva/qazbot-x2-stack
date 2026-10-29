"""Salvage map->odom / odom->base TF from a damaged rosbag2 sqlite file: read rows by rowid, skip unreadable ones.
usage: salvage_tf.py BAG.db3 OUT.json"""
import json, math, sqlite3, sys
from rclpy.serialization import deserialize_message
from tf2_msgs.msg import TFMessage

c = sqlite3.connect('file:%s?mode=ro' % sys.argv[1], uri=True)
topics = {}
try:
    topics = {i: n for i, n in c.execute('select id,name from topics')}
except Exception as e:
    print('topics unreadable', e)
want = {i for i, n in topics.items() if n in ('/tf', '/qazbot_d1_map/tf')}
print('topics', topics)


def yaw(q):
    return math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))


out = {'map_odom': [], 'odom_base': []}
bad = ok = 0
try:
    maxid = c.execute('select max(id) from messages').fetchone()[0]
except Exception as e:
    maxid = 0
    print('max id unreadable', e)
step = 2000
for lo in range(0, (maxid or 0) + 1, step):
    try:
        rows = c.execute('select topic_id,timestamp,data from messages where id>=? and id<? order by id', (lo, lo + step)).fetchall()
    except Exception:
        # fall back to single rows
        rows = []
        for i in range(lo, lo + step):
            try:
                r = c.execute('select topic_id,timestamp,data from messages where id=?', (i,)).fetchone()
                if r:
                    rows.append(r)
            except Exception:
                bad += 1
    for tid, ts, data in rows:
        if tid not in want:
            continue
        try:
            for t in deserialize_message(data, TFMessage).transforms:
                pair = (t.header.frame_id, t.child_frame_id); tr = t.transform
                row = [ts * 1e-9, t.header.stamp.sec + t.header.stamp.nanosec * 1e-9, tr.translation.x, tr.translation.y, yaw(tr.rotation)]
                if pair == ('map', 'odom'):
                    out['map_odom'].append(row)
                elif pair == ('odom', 'base_footprint'):
                    out['odom_base'].append(row)
            ok += 1
        except Exception:
            bad += 1
json.dump(out, open(sys.argv[2], 'w'))
print({k: len(v) for k, v in out.items()}, 'tf msgs ok', ok, 'unreadable', bad)
