#!/usr/bin/env python3
"""Read-only ROS graph + TF + MC-state snapshot for live dry-run bring-up (hardware DDS).

Creates ONE subscriber-only node. Publishes nothing, calls no service, takes no lock.
Usage: live_graph_snapshot.py OUT.json [discovery_s] [sample_s]
discovery_s / sample_s are observation windows (evidence collection), not runtime timeouts.
"""
import json
import sys
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from tf2_msgs.msg import TFMessage

WATCH = ['/aima/mc/locomotion/velocity', '/cmd_vel', '/qazbot_d1_map/cmd_vel', '/qazbot_d1_map/cmd_vel_nav',
         '/qazbot_d1_map/cmd_vel_authorized', '/qazbot_d1_map/motion_adapter/dry_run_cmd',
         '/qazbot_d1_map/goal_pose', '/qazbot_d1_map/initialpose', '/qazbot_d1_map/map',
         '/tf', '/tf_static', '/qazbot_d1_map/tf', '/qazbot_d1_map/tf_static', '/qazbot_lio/tf',
         '/qazbot_lio/scan', '/qazbot_lio/odom_level', '/aima/mc/common/state']
TF_TOPICS = ['/tf', '/tf_static', '/qazbot_d1_map/tf', '/qazbot_d1_map/tf_static']


def qos(q):
    return {'rel': q.reliability.name, 'dur': q.durability.name, 'depth': q.depth}


def main():
    out = sys.argv[1]
    discovery = float(sys.argv[2]) if len(sys.argv) > 2 else 8.0
    sample = float(sys.argv[3]) if len(sys.argv) > 3 else 4.0
    rclpy.init()
    n = Node('qazbot_graph_snapshot')
    end = time.monotonic() + discovery
    while time.monotonic() < end:
        rclpy.spin_once(n, timeout_sec=0.1)
    tf = {t: {} for t in TF_TOPICS}
    t0 = {}

    def cb(topic):
        def f(m):
            for x in m.transforms:
                key = f'{x.header.frame_id}->{x.child_frame_id}'
                d = tf[topic].setdefault(key, {'n': 0})
                d['n'] += 1
                d['last_stamp'] = x.header.stamp.sec + x.header.stamp.nanosec * 1e-9
        return f
    subs = []
    for t in TF_TOPICS:
        static = t.endswith('tf_static')
        q = QoSProfile(depth=100, reliability=ReliabilityPolicy.RELIABLE,
                       durability=DurabilityPolicy.TRANSIENT_LOCAL if static else DurabilityPolicy.VOLATILE)
        subs.append(n.create_subscription(TFMessage, t, cb(t), q))
    mc = {}
    try:
        from aimdk_msgs.msg import McCommonState
        from rosidl_runtime_py import message_to_ordereddict

        def mc_cb(m):
            if not mc:
                mc.update(json.loads(json.dumps(message_to_ordereddict(m), default=str)))
        n.create_subscription(McCommonState, '/aima/mc/common/state', mc_cb,
                              QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT,
                                         durability=DurabilityPolicy.VOLATILE))
    except ImportError as e:
        mc['error'] = f'aimdk_msgs unavailable: {e}'
    end = time.monotonic() + sample
    while time.monotonic() < end:
        rclpy.spin_once(n, timeout_sec=0.05)
    for t in tf.values():
        for d in t.values():
            d['hz'] = round(d['n'] / sample, 2)
    me = ('qazbot_graph_snapshot', '/')
    nodes = sorted(f'{ns.rstrip("/")}/{nm}' for nm, ns in n.get_node_names_and_namespaces() if (nm, ns) != me)
    topics = {name: types for name, types in n.get_topic_names_and_types()}
    endpoints = {}
    for t in WATCH:
        endpoints[t] = {
            'publishers': [{'node': f'{i.node_namespace.rstrip("/")}/{i.node_name}', 'type': i.topic_type, **qos(i.qos_profile)}
                           for i in n.get_publishers_info_by_topic(t)],
            'subscribers': [{'node': f'{i.node_namespace.rstrip("/")}/{i.node_name}', 'type': i.topic_type, **qos(i.qos_profile)}
                            for i in n.get_subscriptions_info_by_topic(t) if i.node_name != me[0]]}
    services = {name: types for name, types in n.get_service_names_and_types()}
    actions = sorted(s[:-len('/_action/send_goal')] for s in services if s.endswith('/_action/send_goal'))
    ours = {}
    for nm, ns in n.get_node_names_and_namespaces():
        if ns.startswith('/qazbot') or nm.startswith('qazbot') or nm in ('goal_coordinator', 'motion_adapter', 'localization_trust'):
            try:
                cl = n.get_client_names_and_types_by_node(nm, ns)
                pubs = n.get_publisher_names_and_types_by_node(nm, ns)
            except Exception as e:  # node vanished between calls
                cl, pubs = [('error', [str(e)])], []
            ours[f'{ns.rstrip("/")}/{nm}'] = {'clients': [c for c, _ in cl], 'publishers': [p for p, _ in pubs]}
    snap = {'wall_time': time.strftime('%Y-%m-%dT%H:%M:%S%z'), 'discovery_s': discovery, 'sample_s': sample,
            'nodes': nodes, 'topics': topics, 'endpoints': endpoints, 'tf': tf, 'actions': actions,
            'mc_input_source_services': sorted(s for s, ty in services.items() if any('McInputSource' in x for x in ty)),
            'our_nodes': ours, 'mc_common_state_once': mc}
    with open(out, 'w') as f:
        json.dump(snap, f, indent=1, sort_keys=True)
    print(f'{out}: {len(nodes)} nodes, {len(topics)} topics, mc_pubs='
          f'{[p["node"] for p in endpoints["/aima/mc/locomotion/velocity"]["publishers"]]}, '
          f'cmd_vel_pubs={[p["node"] for p in endpoints["/cmd_vel"]["publishers"]]}')
    n.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
