#!/usr/bin/env python3
"""Finite passive capture of every discovered TF tree and robot kinematic sources.
No motion publishers, service calls, frame relays or changes to the running stack.
"""
import argparse
import json
import signal
import sqlite3
import subprocess
import time
from pathlib import Path

import rclpy
from rclpy.serialization import deserialize_message
from tf2_msgs.msg import TFMessage

p = argparse.ArgumentParser(description=__doc__)
p.add_argument('--output', type=Path, required=True)
p.add_argument('--duration', type=float, default=8.)
a = p.parse_args()
if a.output.exists() or not 0 < a.duration <= 120:
    p.error('output must be new; duration must be in (0,120] seconds')
a.output.mkdir(parents=True)
rclpy.init()
n = rclpy.create_node('qazbot_full_frame_inventory', enable_rosout=False)
t0 = time.monotonic()
while time.monotonic() - t0 < 3:
    rclpy.spin_once(n, timeout_sec=.1)
all_topics = dict(n.get_topic_names_and_types())
selected = {}
explicit = {'/aima/mc/locomotion/velocity', '/aima/mc/common/state',
            '/qazbot_d1_map/cmd_vel_authorized', '/qazbot_d1_map/cmd_vel',
            '/qazbot_d1_map/nav2/cmd_vel_nav', '/qazbot_d1_map/localization_trust/status',
            '/qazbot_d1_map/motion_adapter/status', '/qazbot_d1_map/goal_coordinator/status',
            '/qazbot_lio/scan', '/qazbot_d1_map/amcl_pose', '/qazbot_d1_map/particle_cloud'}
for topic, types in sorted(all_topics.items()):
    if ('tf2_msgs/msg/TFMessage' in types or 'nav_msgs/msg/Odometry' in types or
        (topic.startswith('/aima/hal/') and any(t in types for t in ['sensor_msgs/msg/Imu', 'aimdk_msgs/msg/JointStateArray'])) or
        topic in explicit):
        selected[topic] = types
required = ['/tf', '/tf_static', '/aima/hal/joint/leg/state', '/aima/hal/joint/waist/state',
            '/aima/mc/leg_odometry', '/qazbot_lio/odometry', '/qazbot_lio/odom_level']
manifest = {'qualification': 'Stationary capture; no motion commands. TF trees remain separated by topic.',
            'topics': selected, 'missing_required': [t for t in required if t not in selected], 'publishers': {}}
for topic in selected:
    manifest['publishers'][topic] = [{'node': i.node_namespace.rstrip('/') + '/' + i.node_name,
                                    'reliability': i.qos_profile.reliability.name,
                                    'durability': i.qos_profile.durability.name}
                                   for i in n.get_publishers_info_by_topic(topic)]
n.destroy_node()
rclpy.shutdown()
(a.output / 'manifest.json').write_text(json.dumps(manifest, indent=2))
if manifest['missing_required']:
    raise RuntimeError('Required sources absent: ' + repr(manifest['missing_required']))
# Explicit subscriber QoS prevents a transient vendor publisher excluding the volatile SDK source.
qos = []
for topic in selected:
    static = topic.endswith('/tf_static')
    qos += [topic + ':', '  history: keep_last', '  depth: 1000',
            '  reliability: ' + ('reliable' if static else 'best_effort'),
            '  durability: ' + ('transient_local' if static else 'volatile')]
(a.output / 'qos.yaml').write_text('\n'.join(qos) + '\n')
with (a.output / 'bag.log').open('w') as log:
    proc = subprocess.Popen(['ros2', 'bag', 'record', '--qos-profile-overrides-path', str(a.output/'qos.yaml'),
                             '-o', str(a.output/'bag')] + list(selected), stdout=log, stderr=subprocess.STDOUT)
    try:
        deadline = time.monotonic() + 20
        while True:
            text = (a.output/'bag.log').read_text()
            if proc.poll() is not None:
                raise RuntimeError('Recorder exited before capture')
            if 'incompatible QoS' in text or 'will not be recorded' in text:
                raise RuntimeError('QoS mismatch; capture cannot be called complete')
            if all(('Subscribed to topic ' + repr(t)) in text for t in required):
                break
            if time.monotonic() > deadline:
                raise RuntimeError('Required subscriptions did not become ready')
            time.sleep(.2)
        print('Required subscriptions ready; recording all discovered TF/kinematics.', flush=True)
        proc.wait(timeout=a.duration)
        raise RuntimeError('Recorder exited unexpectedly')
    except subprocess.TimeoutExpired:
        pass
    finally:
        if proc.poll() is None:
            proc.send_signal(signal.SIGINT)
            proc.wait(timeout=15)
if proc.returncode != 0:
    raise RuntimeError('Recorder exit ' + str(proc.returncode))
# Inspect the finalized bag. Keep parent/child edges separated by the original TF topic.
con = sqlite3.connect(next((a.output/'bag').glob('*.db3')).resolve().as_uri()+'?mode=ro', uri=True)
topics = {i: (name, typ) for i, name, typ in con.execute('SELECT id,name,type FROM topics')}
counts = {topics[i][0]: count for i, count in con.execute('SELECT topic_id,count(*) FROM messages GROUP BY topic_id')}
edges = {}
for tid, (topic, typ) in topics.items():
    if typ != 'tf2_msgs/msg/TFMessage':
        continue
    tree = edges.setdefault(topic, {})
    for timestamp, blob in con.execute('SELECT timestamp,data FROM messages WHERE topic_id=? ORDER BY timestamp', (tid,)):
        for tf in deserialize_message(blob, TFMessage).transforms:
            key = tf.header.frame_id + ' -> ' + tf.child_frame_id
            source = tf.header.stamp.sec + tf.header.stamp.nanosec*1e-9
            entry = tree.setdefault(key, {'count': 0, 'source_first': source, 'source_last': source})
            entry['count'] += 1
            entry['source_last'] = source
con.close()
manifest.update(message_counts=counts, tf_edges=edges,
                required_without_messages=[t for t in required if not counts.get(t)])
(a.output/'manifest.json').write_text(json.dumps(manifest, indent=2))
print(json.dumps({'output': str(a.output), 'topics': len(selected),
                  'tf_edges_by_topic': {k: len(v) for k,v in edges.items()},
                  'required_without_messages': manifest['required_without_messages']}, indent=2))
if manifest['required_without_messages']:
    raise RuntimeError('Required topics had no messages')
