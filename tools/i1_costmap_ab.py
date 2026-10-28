#!/usr/bin/env python3
"""Actual bridge -> DDS/TF -> Nav2 ObstacleLayer A/B. No goals or velocity.

Requires isolated ROS_DOMAIN_ID=88 and a process-local UDP-only profile.
Uses test-only topics/frames; never subscribes to or publishes hardware topics.
"""
import argparse
import copy
import importlib.util
import json
import math
import os
from pathlib import Path
import signal
import subprocess
import time
from types import SimpleNamespace as NS

import numpy as np
import rclpy
from geometry_msgs.msg import TransformStamped
from lifecycle_msgs.msg import Transition
from lifecycle_msgs.srv import ChangeState
from nav_msgs.msg import OccupancyGrid
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy
from sensor_msgs.msg import LaserScan, PointCloud2
from sensor_msgs_py import point_cloud2 as pc2
from std_msgs.msg import Header
from tf2_msgs.msg import TFMessage


def module(name, path):
    spec = importlib.util.spec_from_file_location(name, path)
    m = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(m)
    m.RANGE_MAX = 8.
    return m


class Bench(Node):
    def __init__(self, args):
        super().__init__('i1_isolated_costmap_bench')
        self.args, self.scan, self.clear, self.grid = args, None, None, None
        self.obstacles = None
        self.maps = 0
        self.tf = self.create_publisher(TFMessage, '/qazbot_i1_test/tf_static', QoSProfile(
            depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL, reliability=ReliabilityPolicy.RELIABLE))
        tr = TransformStamped()
        tr.header.frame_id, tr.child_frame_id = 'qazbot_i1_test_odom', 'qazbot_i1_test_base'
        tr.transform.rotation.w = 1.
        self.tf.publish(TFMessage(transforms=[tr]))

    def until(self, predicate, label):
        # Explicit test observation window; no runtime Nav2 tolerance/retries.
        begin = time.monotonic()
        while not predicate():
            if time.monotonic()-begin > self.args.observation_window:
                raise RuntimeError('No measured acknowledgement: '+label)
            rclpy.spin_once(self, timeout_sec=.05)

    def change(self, namespace, transition):
        client = self.create_client(ChangeState, namespace+'/costmap/change_state')
        self.until(client.service_is_ready, 'lifecycle service '+namespace)
        request = ChangeState.Request()
        request.transition.id = transition
        future = client.call_async(request)
        self.until(future.done, 'lifecycle response')
        if not future.result().success:
            raise RuntimeError('Lifecycle transition rejected '+str(transition))
        self.destroy_client(client)

    def cell(self, x, y):
        m = self.grid
        ix, iy = int((x-m.info.origin.position.x)/m.info.resolution), int((y-m.info.origin.position.y)/m.info.resolution)
        return int(m.data[iy*m.info.width+ix])

    def lethal(self, x, y):
        res = self.grid.info.resolution
        return any(self.cell(x+dx*res, y+dy*res) == 100 for dx in (-1, 0, 1) for dy in (-1, 0, 1))

    def run(self, mode, path):
        ns = '/qazbot_i1_test/'+mode
        scan_topic = ns+'/scan'
        params = {'global_frame': 'qazbot_i1_test_odom', 'robot_base_frame': 'qazbot_i1_test_base',
                  'update_frequency': 5., 'publish_frequency': 5., 'always_send_full_costmap': True,
                  'rolling_window': True, 'width': self.args.width, 'height': self.args.width, 'resolution': .05,
                  'robot_radius': .454, 'footprint_padding': .01, 'track_unknown_space': True,
                  'plugins': ['obstacle_layer', 'inflation_layer'],
                  'inflation_layer.plugin': 'nav2_costmap_2d::InflationLayer',
                  'inflation_layer.inflation_radius': .464, 'inflation_layer.cost_scaling_factor': 3.,
                  'obstacle_layer.plugin': 'nav2_costmap_2d::ObstacleLayer',
                  'obstacle_layer.observation_sources': 'scan bounded_rays' if mode == 'candidate' else 'scan',
                  'obstacle_layer.scan.topic': scan_topic, 'obstacle_layer.scan.data_type': 'LaserScan',
                  'obstacle_layer.scan.marking': True, 'obstacle_layer.scan.clearing': mode == 'baseline',
                  'obstacle_layer.scan.inf_is_valid': False, 'obstacle_layer.scan.obstacle_max_range': 3.,
                  'obstacle_layer.scan.raytrace_max_range': 3.5}
        if mode == 'candidate':
            params.update({'obstacle_layer.scan.topic': scan_topic+'_obstacles',
                           'obstacle_layer.scan.data_type': 'PointCloud2',
                           'obstacle_layer.bounded_rays.topic': scan_topic+'_clearing',
                           'obstacle_layer.bounded_rays.data_type': 'PointCloud2',
                           'obstacle_layer.bounded_rays.marking': False,
                           'obstacle_layer.bounded_rays.clearing': True,
                           'obstacle_layer.bounded_rays.raytrace_max_range': 3.5})
        params_file = self.args.output/(mode+'_params.yaml')
        params_file.write_text(json.dumps({'/**': {'ros__parameters': params}}, indent=2))
        log = open(self.args.output/(mode+'_costmap.log'), 'w')
        process = subprocess.Popen([self.args.costmap_binary, '--ros-args', '-r', '__node:=costmap',
            '-r', '__ns:='+ns, '--params-file', str(params_file), '-r', '/tf:=/qazbot_i1_test/tf',
            '-r', '/tf_static:=/qazbot_i1_test/tf_static'], stdout=log, stderr=subprocess.STDOUT)
        publishers, timer, sub = [], None, None
        configured, activated = False, False
        result = {'mode': mode, 'checks': [], 'params': params}
        try:
            scan_pub = self.create_publisher(LaserScan, scan_topic, 10)
            cloud_pub = self.create_publisher(PointCloud2, scan_topic+'_clearing', 10)
            obstacle_pub = self.create_publisher(PointCloud2, scan_topic+'_obstacles', 10)
            publishers = [scan_pub, cloud_pub, obstacle_pub]
            def publish():
                stamp = self.get_clock().now().to_msg()
                for pub, msg in ((scan_pub, self.scan), (cloud_pub, self.clear), (obstacle_pub, self.obstacles)):
                    if msg is not None:
                        msg.header.stamp = stamp
                        pub.publish(msg)
            timer = self.create_timer(.1, publish)
            # Humble standalone executable explicitly constructs namespace
            # /costmap in NodeOptions, overriding the command-line __ns.
            self.change('/costmap', Transition.TRANSITION_CONFIGURE)
            configured = True
            def grid_message(msg):
                self.grid = msg
                self.maps += 1
            def topic():
                return next((n for n, t in self.get_topic_names_and_types()
                             if n.startswith('/costmap/') and 'nav_msgs/msg/OccupancyGrid' in t), None)
            self.until(lambda: topic() is not None, 'published costmap topic')
            sub = self.create_subscription(OccupancyGrid, topic(), grid_message, QoSProfile(
                depth=1, reliability=ReliabilityPolicy.RELIABLE, durability=DurabilityPolicy.TRANSIENT_LOCAL))
            self.change('/costmap', Transition.TRANSITION_ACTIVATE)
            activated = True
            self.until(lambda: self.grid is not None, 'first costmap')
            m = module('i1_'+mode, path)
            def scene(points):
                self.scan, self.clear, self.obstacles = None, None, None
                if points is None:
                    return
                fake = NS(n_cloud=0, every=1, poses={0: (0, 0, 0, np.zeros(3))}, R_lv=np.eye(3),
                          pub_scan=NS(publish=lambda x: setattr(self, 'scan', x)),
                          pub_clearing=NS(publish=lambda x: setattr(self, 'clear', x)),
                          pub_obstacles=NS(publish=lambda x: setattr(self, 'obstacles', x)))
                m.LevelBridge.on_cloud(fake, pc2.create_cloud_xyz32(Header(), points))
                for msg in (self.scan, self.clear, self.obstacles):
                    if msg is not None:
                        msg.header.frame_id = 'qazbot_i1_test_base'
            def check(label, points, predicate):
                scene(points)
                start_maps = self.maps
                self.until(lambda: self.maps >= start_maps+3 and predicate(), label)
                result['checks'].append({'name': label, 'pass': True, 'maps': self.maps-start_maps})
                print(json.dumps({'mode': mode, 'check': label, 'pass': True}), flush=True)
            far = min(2.825, self.args.width/2-.175)
            check('far_mark', [[far, 0, 0]], lambda: self.lethal(far, 0))
            check('near_mark_preserves_occluded_far', [[.725, 0, 0]], lambda: self.lethal(.725, 0) and self.lethal(far, 0))
            same = lambda: self.lethal(.725, 0) and self.lethal(far, 0)
            check('sensor_missing_preserves', None, same)
            check('empty_cloud_preserves', [], same)
            check('partial_cloud_preserves_unseen_bearing', [[2*math.cos(.5), 2*math.sin(.5), 0]], same)
            check('near_floor_does_not_clear_far', [[1, 0, -1]], same)
            cleared = lambda: not self.lethal(.725, 0) and not self.lethal(far, 0)
            check('confirmed_far_floor_clears' if mode == 'candidate' else 'baseline_reproduces_stale',
                  [[1, 0, -1], [8, 0, -1]], cleared if mode == 'candidate' else same)
            check('real_obstacle_kept_with_far_beam', [[.825, 0, 0], [8, 0, -1]], lambda: self.lethal(.825, 0))
            def polar(r, deg, z):
                a = math.radians(deg)
                return [r*math.cos(a), r*math.sin(a), z]
            outside = polar(1.425, 50, 0)
            check('outside_fov_mark', [outside], lambda: self.lethal(*outside[:2]))
            check('outside_fov_removal_stays_unknown', [polar(8, 50, -1)], lambda: self.lethal(*outside[:2]))
            check('unobserved_area_stays_unknown', [polar(8, 0, -1)], lambda: self.cell(-1.5, -1.5) == -1)
            result['result'] = 'PASS'
        except Exception as exc:
            result.update(result='FAIL', error=repr(exc))
        finally:
            result['functional_result'] = result.get('result', 'FAIL')
            self.scan, self.clear, self.obstacles = None, None, None
            try:
                if activated:
                    self.change('/costmap', Transition.TRANSITION_DEACTIVATE)
                if configured:
                    self.change('/costmap', Transition.TRANSITION_CLEANUP)
            except Exception as exc:
                result.update(result='FAIL', shutdown_error=repr(exc))
            (self.args.output/(mode+'_result.json')).write_text(json.dumps(result, indent=2)+'\n')
            if timer:
                self.destroy_timer(timer)
            if sub:
                self.destroy_subscription(sub)
            for pub in publishers:
                self.destroy_publisher(pub)
            process.send_signal(signal.SIGINT)
            result['server_exit_code'] = process.wait()
            if result['server_exit_code'] != 0:
                result['result'] = 'FAIL'
            (self.args.output/(mode+'_result.json')).write_text(json.dumps(result, indent=2)+'\n')
            # A second server with Humble's fixed /costmap/costmap name must
            # not inherit the first server's stale service-discovery entries.
            self.until(lambda: not any(name == 'costmap' and ns == '/costmap'
                                       for name, ns in self.get_node_names_and_namespaces()),
                       'previous costmap removed from discovery')
            log.close()
            self.grid, self.maps = None, 0
        return result


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('--baseline', type=Path, required=True)
    p.add_argument('--candidate', type=Path, required=True)
    p.add_argument('--output', type=Path, required=True)
    p.add_argument('--costmap-binary', default='/opt/ros/humble/lib/nav2_costmap_2d/nav2_costmap_2d')
    p.add_argument('--observation-window', type=float, default=20.)
    p.add_argument('--width', type=int, choices=(4, 8), default=4)
    args = p.parse_args()
    if os.environ.get('ROS_DOMAIN_ID') != '88':
        p.error('isolated ROS_DOMAIN_ID=88 is mandatory')
    if 'udp' not in os.environ.get('FASTRTPS_DEFAULT_PROFILES_FILE', '').lower():
        p.error('explicit process-local UDP profile is mandatory')
    args.output.mkdir(parents=True, exist_ok=False)
    rclpy.init(args=[])
    node = Bench(args)
    results = [node.run('baseline', args.baseline), node.run('candidate', args.candidate)]
    output = {'results': results, 'scope': 'actual bridge conversion, DDS, TF, lifecycle, ObservationBuffer, ObstacleLayer and costmap; no RPP action, no goals, no cmd_vel, no MC'}
    (args.output/'results.json').write_text(json.dumps(output, indent=2)+'\n')
    print(json.dumps(output), flush=True)
    node.destroy_node()
    rclpy.try_shutdown()
    return 0 if all(x['result'] == 'PASS' for x in results) else 1


if __name__ == '__main__':
    raise SystemExit(main())
