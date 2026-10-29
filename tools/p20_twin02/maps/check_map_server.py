"""One-shot integration check of trinary map semantics using the actual Nav2 map server."""
import collections
import json
import subprocess
import time

import numpy as np
import rclpy
from lifecycle_msgs.srv import ChangeState
from nav_msgs.msg import OccupancyGrid
from PIL import Image
from rclpy.qos import QoSProfile, DurabilityPolicy


def main():
    rclpy.init()
    node = rclpy.create_node('t2_map_semantics_check')
    proc = subprocess.Popen(['ros2', 'run', 'nav2_map_server', 'map_server', '--ros-args',
                             '-p', 'yaml_filename:=/map/map.yaml'], stdout=subprocess.DEVNULL)
    try:
        client = node.create_client(ChangeState, '/map_server/change_state')
        if not client.wait_for_service(timeout_sec=20):
            raise RuntimeError('map_server lifecycle unavailable')
        for transition in (1, 3):
            req = ChangeState.Request()
            req.transition.id = transition
            fut = client.call_async(req)
            rclpy.spin_until_future_complete(node, fut, timeout_sec=20)
            assert fut.done() and fut.result().success
        received = []
        sub = node.create_subscription(OccupancyGrid, '/map', received.append,
                                       QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
        end = time.monotonic() + 20
        while not received and time.monotonic() < end:
            rclpy.spin_once(node, timeout_sec=.1)
        assert received, 'no map received'
        msg = received[0]
        actual = np.asarray(msg.data).reshape(msg.info.height, msg.info.width)
        pixels = np.flipud(np.asarray(Image.open('/map/map.pgm')))
        expected = np.where(pixels == 205, -1, np.where(pixels == 0, 100, 0))
        assert np.array_equal(actual, expected), 'map_server classification differs'
        print(json.dumps({'result': 'PASS', 'width': msg.info.width, 'height': msg.info.height,
                          'occupancy_counts': dict(collections.Counter(map(int, actual.flat))),
                          'unknown_became_free': int(((pixels == 205) & (actual == 0)).sum())}, indent=2))
    finally:
        proc.terminate()
        try:
            proc.wait(timeout=10)
        except subprocess.TimeoutExpired:
            proc.kill()
            proc.wait()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
