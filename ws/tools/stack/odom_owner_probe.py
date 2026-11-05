#!/usr/bin/env python3
"""Read-only check: exactly ONE owner of TF odom->base_footprint on /qazbot_d1_map/tf (subscribe/graph only).

Graph: nodes publishing /qazbot_d1_map/tf among the two possible owners (level_bridge, native_odom_bridge; with
odom_source:=native_imu level_bridge creates no TF publisher at all).
Data: every odom->base_footprint TF sample over D seconds is matched by stamp with the odometry message of each
candidate (/qazbot_lio/odom_level, /qazbot_native/odom: the owner publishes TF and odometry from the same sample, same
stamp and pose). A single owner means one candidate explains (almost) all TF samples and the other none.
Usage: odom_owner_probe.py [SECONDS] [EXPECTED_SOURCE lio|native_imu]   Prints JSON; exit 0 ok, 1 violation.
"""
import json
import sys
import time

OWNERS = {'/qazbot_level_bridge': 'lio', '/qazbot_native_odom_bridge': 'native_imu'}
ODOM_TOPICS = {'lio': '/qazbot_lio/odom_level', 'native_imu': '/qazbot_native/odom'}
TF_TOPIC = '/qazbot_d1_map/tf'
POSE_TOL = 1e-6


def graph_owners(tf_publisher_nodes):
    """Sources whose bridge node publishes the TF topic (node names with namespace)."""
    return sorted({OWNERS[n] for n in tf_publisher_nodes if n in OWNERS})


def match_fraction(tf_samples, odom_samples):
    """Fraction of TF samples {stamp_ns: (x, y)} that an odometry stream {stamp_ns: (x, y)} reproduces exactly.
    Native publishes no Odometry until its 0.3 s twist baseline is full, so only TF samples newer than the first
    odometry sample count."""
    if not tf_samples or not odom_samples:
        return 0.0
    first = min(odom_samples)
    keys = [k for k in tf_samples if k >= first]
    if not keys:
        return 0.0
    hit = sum(1 for k in keys if k in odom_samples and
              all(abs(a - b) <= POSE_TOL for a, b in zip(tf_samples[k], odom_samples[k])))
    return hit / len(keys)


def verdict(owners, fractions, n_tf, expected=None):
    """(ok, problems) from the graph owners and the per-source match fractions."""
    problems = []
    if len(owners) != 1:
        problems.append(f'graph: {len(owners)} odom->base_footprint owner nodes {owners}')
    if n_tf == 0:
        problems.append('no odom->base_footprint TF received')
    else:
        explained = [s for s, f in fractions.items() if f >= 0.95]
        foreign = [s for s, f in fractions.items() if 0.0 < f and s not in explained]
        if len(explained) != 1:
            problems.append(f'data: TF explained by {explained or "no"} source(s), fractions {fractions}')
        if foreign:
            problems.append(f'data: TF partly matches {foreign} (two publishers?) fractions {fractions}')
        if owners and explained and explained != owners:
            problems.append(f'data owner {explained} != graph owner {owners}')
    if expected and owners and owners != [expected]:
        problems.append(f'owner {owners} != expected {expected}')
    return not problems, problems


def main():
    import rclpy
    from nav_msgs.msg import Odometry
    from rclpy.qos import QoSProfile, ReliabilityPolicy
    from std_msgs.msg import String
    from tf2_msgs.msg import TFMessage

    dur = float(sys.argv[1]) if len(sys.argv) > 1 else 5.0
    expected = sys.argv[2] if len(sys.argv) > 2 else None
    rclpy.init()
    n = rclpy.create_node('qazbot_odom_owner_probe_ro')
    t0 = time.monotonic()
    while time.monotonic() - t0 < 4.0:              # discovery
        rclpy.spin_once(n, timeout_sec=0.1)
    pubs = [f"{i.node_namespace.rstrip('/')}/{i.node_name}" for i in n.get_publishers_info_by_topic(TF_TOPIC)]
    owners = graph_owners(pubs)
    tf, odom = {}, {s: {} for s in ODOM_TOPICS}

    def key(h):
        return h.stamp.sec * 10**9 + h.stamp.nanosec

    def on_tf(m):
        for t in m.transforms:
            if (t.header.frame_id, t.child_frame_id) == ('odom', 'base_footprint'):
                tf[key(t.header)] = (t.transform.translation.x, t.transform.translation.y)

    def on_odom(src):
        return lambda m: odom[src].__setitem__(key(m.header), (m.pose.pose.position.x, m.pose.pose.position.y))

    n.create_subscription(TFMessage, TF_TOPIC, on_tf, QoSProfile(depth=100))
    for src, topic in ODOM_TOPICS.items():
        n.create_subscription(Odometry, topic, on_odom(src),
                              QoSProfile(depth=100, reliability=ReliabilityPolicy.BEST_EFFORT))
    native_status = []
    n.create_subscription(String, '/qazbot_native/status', lambda m: native_status.append(m.data), QoSProfile(depth=5))
    t0 = time.monotonic()
    while time.monotonic() - t0 < dur:
        rclpy.spin_once(n, timeout_sec=0.05)
    # the newest TF samples may still be waiting for their odometry message: ignore the last 0.2 s
    if tf:
        newest = max(tf)
        tf = {k: v for k, v in tf.items() if newest - k > 2 * 10**8}
    fractions = {s: round(match_fraction(tf, odom[s]), 3) for s in ODOM_TOPICS}
    ok, problems = verdict(owners, fractions, len(tf), expected)
    span = (max(tf) - min(tf)) * 1e-9 if len(tf) > 1 else 0.0
    print(json.dumps(dict(ok=ok, problems=problems, tf_publishers=pubs, owner_graph=owners, match_fraction=fractions,
                          tf_samples=len(tf), tf_rate_hz=round((len(tf) - 1) / span, 1) if span else None,
                          odom_samples={s: len(v) for s, v in odom.items()},
                          native_status_last=json.loads(native_status[-1]) if native_status else None), indent=1))
    n.destroy_node()
    rclpy.try_shutdown()
    return 0 if ok else 1


if __name__ == '__main__':
    sys.exit(main())
