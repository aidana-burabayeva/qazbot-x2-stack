#!/usr/bin/env python3
"""P20: ONE SetMcInputSource call (argv[1] = DELETE) for argv[2] source name, priority 30, timeout 1000. Prints the code."""
import sys, time
import rclpy
from aimdk_msgs.msg import McInputAction
from aimdk_msgs.srv import SetMcInputSource

act = {'DELETE': McInputAction.INPUTACTION_DELETE}[sys.argv[1]]
rclpy.init(); n = rclpy.create_node('p20_mc_source')
cli = n.create_client(SetMcInputSource, '/aimdk_5Fmsgs/srv/SetMcInputSource')
if not cli.wait_for_service(timeout_sec=10.0):
    sys.exit('service not available; nothing sent')
r = SetMcInputSource.Request(); r.action.value = act
r.input_source.name, r.input_source.priority, r.input_source.timeout = sys.argv[2], 30, 1000
r.request.header.stamp = n.get_clock().now().to_msg()
f = cli.call_async(r); rclpy.spin_until_future_complete(n, f, timeout_sec=5.0)
print(sys.argv[1], sys.argv[2], 'code', f.result().response.header.code if f.done() and f.result() else 'timeout')
n.destroy_node(); rclpy.try_shutdown()
