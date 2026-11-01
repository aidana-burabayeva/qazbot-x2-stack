"""ROS-independent recording reader for AMCL replay inputs.

Recorded AMCL output is reference data only. In particular it must never replace
an operator's initialpose, including a reseed or an intentionally invalid input.
"""
import math


def read_replay_inputs(db, deserialize, message_types, max_seconds=0.):
    topics = {
        '/qazbot_d1_map/tf': 'tf',
        '/qazbot_lio/scan': 'scan',
        '/qazbot_d1_map/initialpose': 'initialpose',
    }
    msgs, odom_base, rec_map_odom = [], [], []
    rows = db.execute(
        'SELECT m.timestamp,t.name,m.data FROM messages m JOIN topics t ON t.id=m.topic_id '
        'WHERE t.name IN (?,?,?) ORDER BY m.timestamp,m.id', tuple(topics))
    for ts_ns, topic, data in rows:
        kind = topics[topic]
        msg = deserialize(bytes(data), message_types[kind])
        ts = ts_ns / 1e9
        if kind == 'tf':
            keep = []
            for tf in msg.transforms:
                edge = tf.header.frame_id, tf.child_frame_id
                if edge not in (('map', 'odom'), ('odom', 'base_footprint')):
                    continue
                tr, q = tf.transform.translation, tf.transform.rotation
                pose = (tr.x, tr.y, math.atan2(2*(q.w*q.z+q.x*q.y), 1-2*(q.y*q.y+q.z*q.z)))
                if edge == ('map', 'odom'):
                    if not rec_map_odom or rec_map_odom[-1][1] != pose:
                        rec_map_odom.append((ts, pose))
                else:
                    odom_base.append((ts, pose))
                    keep.append(tf)
            if not keep:
                continue
            msg.transforms = keep
        msgs.append((ts, kind, msg))
    if msgs and max_seconds:
        start = msgs[0][0]
        msgs = [row for row in msgs if row[0] - start <= max_seconds]
    if not any(kind == 'initialpose' for _, kind, _ in msgs):
        raise ValueError('replay requires recorded initialpose in the selected window; no inferred seed')
    if not any(kind == 'tf' for _, kind, _ in msgs) or not any(kind == 'scan' for _, kind, _ in msgs):
        raise ValueError('replay requires recorded scan and odom->base_footprint inputs')
    return msgs, odom_base, rec_map_odom


def shift_input_stamps(kind, msg, delta_ns, time_type):
    """Shift source stamps once by the playback offset; preserve zero seed's 'now'."""
    headers = [tf.header for tf in msg.transforms] if kind == 'tf' else [msg.header]
    for header in headers:
        stamp_ns = header.stamp.sec * 10**9 + header.stamp.nanosec
        if kind == 'initialpose' and stamp_ns == 0:
            continue
        shifted = stamp_ns + delta_ns
        header.stamp = time_type(sec=shifted // 10**9, nanosec=shifted % 10**9)
