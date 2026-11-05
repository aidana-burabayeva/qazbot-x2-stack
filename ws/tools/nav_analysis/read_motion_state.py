#!/usr/bin/env python3
"""Read SDK motion state; never register a source, change modes or publish velocity."""
import json
import rclpy
from aimdk_msgs.srv import GetMcLocomotionSpeedStatus, GetCurrentInputSource
from rosidl_runtime_py.convert import message_to_ordereddict

rclpy.init()
node = rclpy.create_node('qazbot_motion_readonly_audit')
try:
    for cls, name in [(GetMcLocomotionSpeedStatus, 'GetMcLocomotionSpeedStatus'),
                      (GetCurrentInputSource, 'GetCurrentInputSource')]:
        client = node.create_client(cls, '/aimdk_5Fmsgs/srv/' + name)
        if not client.wait_for_service(timeout_sec=5):
            raise RuntimeError(name + ' missing')
        req = cls.Request()
        req.request.header.stamp = node.get_clock().now().to_msg()
        future = client.call_async(req)
        rclpy.spin_until_future_complete(node, future, timeout_sec=5)
        if not future.done():
            raise RuntimeError(name + ' diagnostic request incomplete')
        print(json.dumps({'service': name, 'response': message_to_ordereddict(future.result())}))
finally:
    node.destroy_node()
    rclpy.shutdown()
