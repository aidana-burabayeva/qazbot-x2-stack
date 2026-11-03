#!/usr/bin/env python3
"""Stage 1 mapping client: start_mapping -> (operator walks) -> stop_mapping:<name>
-> poll GetStoredMapByName until code==0 (<=120 s) -> output/stage1_map_check.json.

Official interfaces (see stage1_common.py for citations):
- /integrated_command std_msgs/String, RELIABLE + TRANSIENT_LOCAL
  https://x2-aimdk.agibot.com/en/latest/Interface/perception/SLAM.html ; /home/agi/aimdk/src/py_examples/py_examples/slam.py
- /aimdk_5Fmsgs/srv/GetStoredMapByName
  https://x2-aimdk.agibot.com/en/latest/Interface/perception/SLAM.html ; /home/agi/aimdk/src/py_examples/py_examples/get_map.py
"""
import select
import sys
import time

import stage1_common as c

try:
    import rclpy
    from rclpy.node import Node
    from rclpy.qos import QoSProfile, QoSDurabilityPolicy, QoSReliabilityPolicy
    from std_msgs.msg import String
    from std_msgs.msg import Header
    from aimdk_msgs.srv import GetStoredMapByName
except ImportError as exc:  # allows unit tests without ROS
    rclpy = None
    Node = object
    _IMPORT_ERR = exc

EXIT_OK = 0
EXIT_NO_SUBSCRIBER = 2
EXIT_BAD_ARGS = 3
EXIT_NOT_SAVED = 4
NO_SUB_ERROR = ('ERROR: /integrated_command has 0 subscribers; start_mapping NOT published '
                '(SLAM command subscriber absent, blocker B2)')
NO_CONFIRM_NOTE = ('NOTE: the official docs define no confirmation topic for start_mapping/stop_mapping; '
                   'success is judged only by GetStoredMapByName code==0.')


class SlamMappingClient(Node):
    def __init__(self):
        super().__init__('slam_mapping_client')
        self.declare_parameter('map_name', '')
        # QoS as in slam.py: TRANSIENT_LOCAL + RELIABLE
        qos = QoSProfile(depth=10, durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
                         reliability=QoSReliabilityPolicy.RELIABLE)
        self.pub = self.create_publisher(String, c.INTEGRATED_COMMAND_TOPIC, qos)
        self.cli = self.create_client(GetStoredMapByName, c.GET_STORED_MAP_BY_NAME_SRV)

    def publish_text(self, text):
        msg = String()
        msg.data = text
        self.get_logger().info(f'Publishing: [{text}]')
        self.pub.publish(msg)

    def subscriber_count(self):
        return self.pub.get_subscription_count()

    def call_get_map(self, map_name, per_call_timeout=5.0):
        if not self.cli.wait_for_service(timeout_sec=1.0):
            return None
        req = GetStoredMapByName.Request()
        req.header = Header()
        req.header.stamp = self.get_clock().now().to_msg()
        req.header.frame_id = ''
        req.map_name = map_name
        fut = self.cli.call_async(req)
        rclpy.spin_until_future_complete(self, fut, timeout_sec=per_call_timeout)
        return fut.result()


def run_mapping(node, map_name, json_path=c.DEFAULT_JSON, sub_wait_s=10.0,
                wait_stop=None, sleep=time.sleep, now=time.monotonic):
    """Core flow, node duck-typed (publish_text, subscriber_count, call_get_map, log)."""
    log = node.log
    if not c.validate_map_name(map_name):
        log('ERROR: parameter map_name is required and must be non-empty')
        return EXIT_BAD_ARGS
    n = c.wait_for_subscribers(node.subscriber_count, sub_wait_s, sleep, now)
    if n < 1:
        log(NO_SUB_ERROR)
        return EXIT_NO_SUBSCRIBER
    log(f'/integrated_command subscribers: {n}')
    node.publish_text(c.START_PAYLOAD)
    log(NO_CONFIRM_NOTE)
    t0 = now()
    log('Mapping requested. Operator: walk the area; press Enter (or Ctrl+C) to stop and save.')
    wait_stop(lambda: now() - t0)
    node.publish_text(c.build_stop_payload(map_name))
    log(f'stop_mapping sent; polling GetStoredMapByName for up to {c.SAVE_TIMEOUT_S:.0f} s')
    ok, resp = c.poll_saved_map(lambda: node.call_get_map(map_name), c.SAVE_TIMEOUT_S,
                                sleep, now, log)
    if not ok:
        code = None if resp is None else resp.code
        log(f'ERROR: map NOT confirmed saved (last code={code}) within {c.SAVE_TIMEOUT_S:.0f} s')
        return EXIT_NOT_SAVED
    rec = c.build_map_check(map_name, resp)
    c.write_map_check(json_path, rec)
    log(f'Map saved (GetStoredMapByName code==0). map_id={rec["map_id"]} map_path={rec["map_path"]}; '
        f'JSON: {json_path}')
    if not rec['grid_data_present']:
        log('NOTE: data[] is empty (srv comment: field reserved); validity judged by code/map_path')
    return EXIT_OK


def stdin_wait_factory(log, period=5.0):
    def wait(elapsed):
        last = 0.0
        try:
            while True:
                r, _, _ = select.select([sys.stdin], [], [], 1.0)
                if r:
                    sys.stdin.readline()
                    return
                e = elapsed()
                if e - last >= period:
                    last = e
                    log(f'Mapping elapsed: {e:.0f} s')
        except KeyboardInterrupt:
            log('SIGINT received, stopping mapping')
    return wait


def main(args=None):
    if rclpy is None:
        print(f'ERROR: ROS imports failed: {_IMPORT_ERR}', file=sys.stderr)
        return EXIT_BAD_ARGS
    rclpy.init(args=args)
    node = SlamMappingClient()
    node.log = node.get_logger().info
    map_name = node.get_parameter('map_name').get_parameter_value().string_value
    try:
        rc = run_mapping(node, map_name, wait_stop=stdin_wait_factory(node.log))
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return rc


if __name__ == '__main__':
    sys.exit(main())
