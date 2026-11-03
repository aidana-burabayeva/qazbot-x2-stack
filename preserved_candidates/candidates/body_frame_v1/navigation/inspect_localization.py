#!/usr/bin/env python3
"""Inspect private localization; optional native global initialization only.

Never publishes a motion command, changes production localization, or seeds
the filter with a guessed pose.
"""
import collections
import hashlib
import json
import sys
import time

import rclpy
from lifecycle_msgs.srv import GetState
from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import PoseWithCovarianceStamped
from rcl_interfaces.msg import Parameter, ParameterType, ParameterValue
from rcl_interfaces.srv import SetParameters
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy, qos_profile_sensor_data
from rosidl_runtime_py.convert import message_to_ordereddict
from std_srvs.srv import Empty
from tf2_msgs.msg import TFMessage

NS = '/qazbot_d1_map'


def main():
    rclpy.init()
    node = Node('qazbot_localization_inspector')
    result = {'poses': [], 'tf_edges': {}, 'map': None, 'states': {}, 'actions': {}}

    def pose(m):
        result['poses'].append(message_to_ordereddict(m))

    def grid(m):
        result['map'] = {'frame': m.header.frame_id,
                         'info': message_to_ordereddict(m.info),
                         'counts': dict(collections.Counter(m.data)),
                         'grid_sha256': hashlib.sha256(bytes(v % 256 for v in m.data)).hexdigest()}

    def tf(m):
        for t in m.transforms:
            edge = t.header.frame_id + '->' + t.child_frame_id
            result['tf_edges'][edge] = message_to_ordereddict(t)

    # AMCL offers transient_local: retain the last estimate for late inspection.
    # Its original stamp remains visible; a retained sample is not a new update.
    node.create_subscription(PoseWithCovarianceStamped, NS + '/amcl_pose', pose,
                             QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
    node.create_subscription(OccupancyGrid, NS + '/map', grid,
                             QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
    node.create_subscription(TFMessage, NS + '/tf', tf, qos_profile_sensor_data)

    def call(kind, name, req):
        cli = node.create_client(kind, name)
        if not cli.wait_for_service(timeout_sec=8):
            raise RuntimeError('Service missing: ' + name)
        future = cli.call_async(req)
        rclpy.spin_until_future_complete(node, future, timeout_sec=8)
        if not future.done() or future.exception():
            raise RuntimeError('Service did not complete: ' + name)
        return future.result()

    try:
        for name in ('map_server', 'amcl'):
            state = call(GetState, NS + '/' + name + '/get_state', GetState.Request())
            result['states'][name] = state.current_state.label
        if '--global' in sys.argv:
            req = SetParameters.Request(parameters=[Parameter(
                name='max_beams', value=ParameterValue(
                    type=ParameterType.PARAMETER_INTEGER, integer_value=1440))])
            param_result = call(SetParameters, NS + '/amcl/set_parameters', req)
            result['actions']['max_beams'] = [message_to_ordereddict(r) for r in param_result.results]
            if not all(r.successful for r in param_result.results):
                raise RuntimeError('Cannot configure sensor sampling')
            call(Empty, NS + '/reinitialize_global_localization', Empty.Request())
            result['actions']['global_initialization'] = 'completed'
        start = time.monotonic()
        while time.monotonic() - start < 15:
            rclpy.spin_once(node, timeout_sec=0.1)
        for topic in (NS + '/tf', NS + '/map', NS + '/amcl_pose'):
            result[topic + '_publishers'] = [p.node_name for p in node.get_publishers_info_by_topic(topic)]
        with open(sys.argv[1], 'w') as out:
            json.dump(result, out, indent=2)
        print(json.dumps({'states': result['states'], 'map': result['map'],
                          'poses': len(result['poses']), 'edges': list(result['tf_edges']),
                          'actions': result['actions']}))
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
