import json
import signal
import time
import traceback

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from rosidl_runtime_py.convert import message_to_ordereddict
from tf2_msgs.msg import TFMessage
from std_msgs.msg import String
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan, PointCloud2
from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import PoseWithCovarianceStamped
from aimdk_msgs.msg import McLocomotionVelocity, McCommonState

rclpy.init()
n = Node('i1_passive_observer_light')
running = True


def stop(*_):
    global running
    running = False


signal.signal(signal.SIGTERM, stop)
signal.signal(signal.SIGINT, stop)
f = open('/home/agi/control_backend/docs/evidence/i1_deploy_2d240dbd/passive_observer.jsonl', 'a', buffering=1)
seen = {}
endpoints = {}
counts = {}
last = {}


def emit(kind, **data):
    f.write(json.dumps(dict(kind=kind, wall=time.time(), **data)) + '\n')


def received(key, msg):
    counts[key] = counts.get(key, 0) + 1
    d = {'receipt_wall': time.time()}
    if hasattr(msg, 'header'):
        d['stamp'] = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        d['age'] = time.time() - d['stamp']
    last[key] = d


def tf_cb(msg):
    for t in msg.transforms:
        pair = t.header.frame_id + '->' + t.child_frame_id
        gid = 'unavailable_in_humble_message_info'
        seen.setdefault(pair, set()).add(gid)
        received(pair, t)
        if pair == 'map->odom':
            emit('map_odom', gid=gid, transform=message_to_ordereddict(t))


def status_cb(msg):
    received('trust', msg)
    emit('trust', value=json.loads(msg.data))


def seed_cb(msg):
    received('initialpose', msg)
    emit('initialpose', value=message_to_ordereddict(msg))


def amcl_cb(msg):
    received('amcl_pose', msg)
    emit('amcl_pose', value=message_to_ordereddict(msg))


def velocity_cb(msg):
    received('mc_velocity', msg)
    emit('mc_velocity', value=message_to_ordereddict(msg))


def mc_cb(msg):
    received('mc_state', msg)
    last['mc_state']['value'] = message_to_ordereddict(msg)


be = QoSProfile(depth=100, reliability=ReliabilityPolicy.BEST_EFFORT)
n.create_subscription(TFMessage, '/qazbot_d1_map/tf', tf_cb, 100)
n.create_subscription(String, '/qazbot_d1_map/localization_trust/status', status_cb, 100)
n.create_subscription(String, '/qazbot_native/status', lambda m: emit('native_status', value=json.loads(m.data)), 10)
n.create_subscription(PoseWithCovarianceStamped, '/qazbot_d1_map/initialpose', seed_cb, 10)
n.create_subscription(PoseWithCovarianceStamped, '/qazbot_d1_map/amcl_pose', amcl_cb, be)
n.create_subscription(Odometry, '/qazbot_native/odom', lambda m: received('native_odom', m), be)
n.create_subscription(LaserScan, '/qazbot_lio/scan', lambda m: received('scan', m), be)
n.create_subscription(McLocomotionVelocity, '/aima/mc/locomotion/velocity', velocity_cb, be)
n.create_subscription(McCommonState, '/aima/mc/common/state', mc_cb, be)
def cloud_cb(key, msg):
    received(key, msg)
    last[key].update(width=msg.width, height=msg.height, frame=msg.header.frame_id)
for key in ('scan_obstacles', 'scan_clearing'):
    n.create_subscription(PointCloud2, '/qazbot_lio/' + key, lambda m, k=key: cloud_cb(k, m), be)
def grid_cb(msg):
    received('costmap', msg)
    last['costmap'].update(width=msg.info.width, height=msg.info.height,
        lethal=sum(x == 100 for x in msg.data), unknown=sum(x == -1 for x in msg.data))
    if counts['costmap'] % 10 == 1:
        emit('costmap', value=message_to_ordereddict(msg))
n.create_subscription(OccupancyGrid, '/costmap/costmap', grid_cb, be)
emit('started')
next_summary = time.monotonic() + 1
try:
    while running:
        rclpy.spin_once(n, timeout_sec=0.1)
        if time.monotonic() >= next_summary:
            for topic in ('/qazbot_d1_map/tf', '/qazbot_d1_map/tf_static', '/qazbot_lio/scan_obstacles', '/qazbot_lio/scan_clearing', '/qazbot_lio/cloud_registered', '/qazbot_native/odom', '/qazbot_lio/scan', '/aima/mc/locomotion/velocity', '/cmd_vel', '/qazbot_d1_map/cmd_vel', '/qazbot_d1_map/cmd_vel_nav', '/qazbot_d1_map/cmd_vel_authorized'):
                endpoints[topic] = [{'node': i.node_namespace.rstrip('/') + '/' + i.node_name, 'gid': bytes(i.endpoint_gid).hex()} for i in n.get_publishers_info_by_topic(topic)]
            emit('summary', counts=counts, last=last, tf_gids={k: sorted(v) for k, v in seen.items()}, publishers=endpoints)
            next_summary = time.monotonic() + 1
except Exception:
    emit('error', traceback=traceback.format_exc())
    traceback.print_exc()
finally:
    emit('stopped')
    f.close()
    n.destroy_node()
    rclpy.shutdown()
