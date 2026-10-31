#!/usr/bin/env python3
"""SIM-only live graph and observation contract probe; not a Nav2 ObstacleLayer regression."""
import json
import os
import time

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan, PointCloud2
from sensor_msgs_py import point_cloud2 as pc2


def main():
    if any(os.environ.get(k) != v for k,v in {'TWIN_SANDBOX':'1','QAZBOT_SIM':'1','ROS_DOMAIN_ID':'87'}.items()):
        raise SystemExit('refused: isolated SIM environment required')
    rclpy.init(); node = Node('costmap_sensor_contract_probe')
    records = {}; errors = []; counts = {'scan':0,'obstacles':0,'clearing':0}
    def callback(kind, msg):
        counts[kind] += 1
        key = msg.header.stamp.sec * 10**9 + msg.header.stamp.nanosec
        if key <= 0 or msg.header.frame_id != 'base_footprint':
            errors.append('invalid_header:' + kind)
        record = records.setdefault(key,{})
        if kind == 'scan':
            record[kind] = msg
        else:
            try:
                p = np.asarray(pc2.read_points_numpy(msg,field_names=('x','y','z'))).reshape(-1,3)
                if not np.isfinite(p).all():
                    errors.append('nonfinite_pointcloud:' + kind)
                record[kind] = p
            except (ValueError, TypeError, AssertionError) as exc:
                errors.append('invalid_cloud:' + str(exc))
    subscriptions = []
    topics = {'scan':'/qazbot_lio/scan','obstacles':'/qazbot_lio/scan_obstacles','clearing':'/qazbot_lio/scan_clearing'}
    def receiver(kind):
        def receive(msg):
            callback(kind,msg)
        return receive
    for kind,topic in topics.items():
        subscriptions.append(node.create_subscription(LaserScan if kind=='scan' else PointCloud2,topic,receiver(kind),qos_profile_sensor_data))
    # Repeated endpoint snapshots retain discovery of competing publishers, including ones that disappear.
    graph = {k:{} for k in topics}
    def inspect():
        for kind,topic in topics.items():
            for info in node.get_publishers_info_by_topic(topic):
                gid = bytes(info.endpoint_gid).hex()
                if not gid or int(gid,16) == 0:
                    errors.append('unknown_publisher_gid:' + kind)
                graph[kind][gid] = {'node':info.node_namespace.rstrip('/')+'/'+info.node_name,'type':info.topic_type}
    timer = node.create_timer(.2,inspect)
    end = time.monotonic()+6
    try:
        while time.monotonic()<end:
            rclpy.spin_once(node,timeout_sec=.1)
        inspect()
        joined = [d for d in records.values() if all(k in d for k in counts)]
        if len(joined)<3:
            errors.append('insufficient_synchronized_samples')
        nonempty = {'obstacles':False,'clearing':False}
        for d in joined:
            scan = d['scan']; marks = d['obstacles']; clear = d['clearing']
            if len(scan.ranges)!=1440:
                errors.append('unexpected_scan_bins'); continue
            finite = np.asarray(scan.ranges); bins = np.isfinite(finite)
            def indices(p):
                return np.rint((np.arctan2(p[:,1],p[:,0])-scan.angle_min)/scan.angle_increment).astype(int) % 1440
            for kind,p in (('obstacles',marks),('clearing',clear)):
                nonempty[kind] |= len(p)>0
                r = np.hypot(p[:,0],p[:,1]); idx = indices(p)
                tolerance = 4*np.finfo(np.float32).eps*np.maximum(r,1)
                if (not np.all(bins[idx]) or np.any(r > finite[idx]+tolerance) or np.any(r<=scan.range_min)):
                    errors.append('unproven_or_out_of_range:' + kind)
            if len(clear):
                angles = np.degrees(np.arctan2(clear[:,1],clear[:,0]))
                if np.any((angles < -46.0001) | (angles > 44.5001)):
                    errors.append('clearing_outside_production_policy')
        if not all(nonempty.values()):
            errors.append('insufficient_finite_returns')
        for kind,pubs in graph.items():
            expected_type = 'sensor_msgs/msg/' + ('LaserScan' if kind=='scan' else 'PointCloud2')
            if len(pubs)!=1 or any(p['type']!=expected_type for p in pubs.values()):
                errors.append('publisher_contract:' + kind)
        result = dict(passed=not errors, errors=sorted(set(errors)), counts=counts, synchronized=len(joined), publishers=graph,
                      scope='live graph/messages only; not costmap behavior, raw E1R or MC physics qualification')
        print('SENSORJSON'+json.dumps(result))
        return 0 if result['passed'] else 1
    finally:
        del timer, subscriptions
        node.destroy_node();rclpy.shutdown()


if __name__ == '__main__':
    raise SystemExit(main())
