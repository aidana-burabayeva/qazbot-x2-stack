#!/usr/bin/env python3
"""Benchmark: Python candidate level_bridge.py vs the C++ lidar_projector on the SAME fixture inputs.

    source /opt/ros/humble/setup.bash; source <install>/setup.bash
    ROS_DOMAIN_ID=77 ROS_LOCALHOST_ONLY=1 nice -n 19 python3 bench_compare.py [--passes 20]

Part 1 (per-cloud compute, in process): Python = LevelBridge.on_cloud of the candidate (read_points_numpy, numpy math,
  create_cloud_xyz32 x2, LaserScan, publish into a stub) ; C++ = CloudProcessor::process (bench_core: levelling to body
  frame, ranges, obstacles, clearing; no message construction). p50/p95/p99 over passes x clouds.
Part 2 (whole node process, isolated domain, same replay): each node is started with range_max 8.0, scan_every 1,
  publish_tf false, fed the fixture through rclpy with a wait for every scan; reports process CPU time per processed
  cloud (utime+stime, includes 981 IMU + odometry callbacks, DDS, message copies) and peak RSS (VmHWM).
"""
import argparse
import os
import subprocess
import sys
import time

import numpy as np
import rclpy
from ament_index_python.packages import get_package_prefix
from nav_msgs.msg import Odometry
from rclpy.qos import QoSProfile, QoSReliabilityPolicy
from sensor_msgs.msg import Imu, LaserScan, PointCloud2, PointField

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
sys.path.insert(0, os.path.dirname(HERE))
import fixture  # noqa: E402
import test_parity as tp  # noqa: E402

FIXTURE = os.environ.get('FIXTURE', os.path.join(os.path.dirname(HERE), 'data', 'static_204153_w50.bin'))
IMU = '/aima/hal/sensor/lidar_chest_front/imu'


def pct(x):
    x = np.sort(np.asarray(x))
    return [float(x[int(f * (len(x) - 1))]) for f in (0.5, 0.95, 0.99)]


def python_percall(events, passes):
    mod = tp.load_candidate()
    s, _ = tp.python_reference(mod, events, 8.0, 1)   # levels and fills pose cache with the fixture
    import types
    from builtin_interfaces.msg import Time
    clouds = []
    for e in events:
        if e.kind == 'cloud' and (e.sec * 10 ** 9 + e.nanosec) in s.poses:
            xyz, dense = e.data
            m = PointCloud2()
            m.header.stamp = Time(sec=e.sec, nanosec=e.nanosec)
            m.height, m.width = 1, len(xyz)
            m.fields = [PointField(name=n, offset=4 * i, datatype=PointField.FLOAT32, count=1) for i, n in enumerate('xyz')]
            m.is_bigendian, m.point_step, m.row_step, m.is_dense = False, 12, 12 * len(xyz), dense
            m.data = np.ascontiguousarray(xyz, dtype='<f4').tobytes()
            clouds.append(m)
    ms = []
    for _ in range(passes):
        for m in clouds:
            s.n_cloud = 0
            s.pub_scan.msgs.clear(); s.pub_obstacles.msgs.clear(); s.pub_clearing.msgs.clear()
            t0 = time.perf_counter()
            mod.LevelBridge.on_cloud(s, m)
            ms.append((time.perf_counter() - t0) * 1e3)
            assert s.pub_scan.msgs
    return len(clouds), pct(ms)


def proc_cpu_s(pid):
    f = open(f'/proc/{pid}/stat').read().rsplit(')', 1)[1].split()
    return (int(f[11]) + int(f[12])) / os.sysconf('SC_CLK_TCK')


def vm(pid, key):
    for l in open(f'/proc/{pid}/status'):
        if l.startswith(key):
            return int(l.split()[1]) / 1024.0


def make_cloud(e):
    xyz, dense = e.data
    m = PointCloud2()
    m.header.stamp.sec, m.header.stamp.nanosec = e.sec, e.nanosec
    m.height, m.width = 1, len(xyz)
    names = [('x', 0), ('y', 4), ('z', 8), ('intensity', 32), ('normal_x', 16), ('normal_y', 20), ('normal_z', 24), ('curvature', 36)]
    m.fields = [PointField(name=n, offset=o, datatype=PointField.FLOAT32, count=1) for n, o in names]
    m.is_bigendian, m.point_step, m.row_step, m.is_dense = False, 48, 48 * len(xyz), dense
    raw = np.zeros((len(xyz), 12), dtype='<f4')
    raw[:, :3] = xyz
    m.data = raw.tobytes()
    return m


def make_odom(e):
    m = Odometry()
    m.header.stamp.sec, m.header.stamp.nanosec = e.sec, e.nanosec
    (m.pose.pose.position.x, m.pose.pose.position.y, m.pose.pose.position.z) = e.data[0]
    (m.pose.pose.orientation.x, m.pose.pose.orientation.y, m.pose.pose.orientation.z, m.pose.pose.orientation.w) = e.data[1]
    m.pose.covariance = list(e.data[2])
    return m


def replay_node(cmd, node_name, replay, n_imu, expect_keys):
    """Start `cmd`, replay the canonical event list through it, return (CPU ms / cloud, peak RSS MB, RSS MB, clouds)."""
    from rcl_interfaces.srv import GetParameters
    node = rclpy.create_node('bench_feeder')
    be = QoSProfile(depth=400, reliability=QoSReliabilityPolicy.BEST_EFFORT)
    p_imu = node.create_publisher(Imu, IMU, be)
    p_odom = node.create_publisher(Odometry, '/Odometry', 100)
    p_cloud = node.create_publisher(PointCloud2, '/cloud_registered', 20)
    cli = node.create_client(GetParameters, f'/{node_name}/get_parameters')
    got = set()
    node.create_subscription(LaserScan, '/scan', lambda m: got.add(m.header.stamp.sec * 10 ** 9 + m.header.stamp.nanosec),
                             QoSProfile(depth=100, reliability=QoSReliabilityPolicy.BEST_EFFORT))
    proc = subprocess.Popen(cmd, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL, env=dict(os.environ))
    try:
        def spin_until(cond, timeout=60):
            end = time.monotonic() + timeout
            while not cond():
                assert time.monotonic() < end, 'timeout'
                rclpy.spin_once(node, timeout_sec=0.05)

        def barrier():   # parameter-service round trip: subscriptions published before it are processed first
            fut = cli.call_async(GetParameters.Request(names=['range_max']))
            spin_until(fut.done)

        spin_until(lambda: p_odom.get_subscription_count() == 1 and p_cloud.get_subscription_count() == 1 and
                   p_imu.get_subscription_count() == 1 and cli.service_is_ready())
        cpu0 = proc_cpu_s(proc.pid)
        p_odom.publish(make_odom(replay[0]))
        barrier()
        for e in replay[1:1 + n_imu]:
            m = Imu(); m.header.stamp.sec, m.header.stamp.nanosec = e.sec, e.nanosec
            m.linear_acceleration.x, m.linear_acceleration.y, m.linear_acceleration.z = e.data
            p_imu.publish(m); rclpy.spin_once(node, timeout_sec=0)
        spin_until(lambda: node.count_subscribers(IMU) == 0)
        n = 0
        for e in replay[1 + n_imu:]:
            if e.kind == 'odom':
                p_odom.publish(make_odom(e)); barrier()
                continue
            p_cloud.publish(make_cloud(e))
            key = e.sec * 10 ** 9 + e.nanosec
            if key in expect_keys:
                spin_until(lambda: key in got)
                n += 1
        cpu = proc_cpu_s(proc.pid) - cpu0
        return cpu / n * 1e3, vm(proc.pid, 'VmHWM'), vm(proc.pid, 'VmRSS'), n
    finally:
        proc.terminate()
        proc.wait(timeout=20)
        node.destroy_node()


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--passes', type=int, default=20)
    a = ap.parse_args()
    assert os.environ.get('ROS_DOMAIN_ID') == '77', 'run in the isolated domain 77'
    events = fixture.read(FIXTURE)
    n_py, py = python_percall(events, a.passes)
    prefix = get_package_prefix('qazbot_lidar_projector')
    bench = os.path.join(prefix, 'lib', 'qazbot_lidar_projector', 'bench_core')
    line = subprocess.run([bench, FIXTURE, str(a.passes), '8.0'], check=True, capture_output=True, text=True).stdout.strip()
    cpp = dict(kv.split('=') for kv in line.split())
    print(f'Part 1 per-cloud compute, {n_py} clouds x {a.passes} passes (ms)')
    print(f'  python on_cloud : p50 {py[0]:.3f}  p95 {py[1]:.3f}  p99 {py[2]:.3f}')
    print(f'  C++ core process: p50 {float(cpp["p50_ms"]):.3f}  p95 {float(cpp["p95_ms"]):.3f}  p99 {float(cpp["p99_ms"]):.3f}')
    import tempfile
    replay, n_imu = fixture.canonical_replay(events)
    dump = os.path.join(prefix, 'lib', 'qazbot_lidar_projector', 'parity_dump')
    with tempfile.TemporaryDirectory() as d:
        canon, out = os.path.join(d, 'c.bin'), os.path.join(d, 'x.bin')
        fixture.write(canon, replay)
        subprocess.run([dump, canon, out, '8.0', '1'], check=True)
        _, _, cl = tp.read_dump(out)
    exp = {k for k, *_ in cl}
    rclpy.init()
    cands = {
        'python node': ('level_bridge', ['python3', tp.CANDIDATE, '--ros-args', '-p', 'range_max:=8.0', '-p', 'scan_every:=1',
                                         '-p', 'publish_tf:=false']),
        'C++ node': ('lidar_projector', [os.path.join(prefix, 'lib', 'qazbot_lidar_projector', 'lidar_projector'), '--ros-args',
                                         '-p', 'range_max:=8.0', '-p', 'scan_every:=1', '-p', 'publish_tf:=false']),
    }
    print('Part 2 whole node process (same replay, isolated domain)')
    for name, (node_name, cmd) in cands.items():
        cpu_ms, hwm, rss, n = replay_node(cmd, node_name, replay, n_imu, exp)
        print(f'  {name:12s}: CPU {cpu_ms:.2f} ms per processed cloud ({n} clouds)  peak RSS {hwm:.1f} MB  RSS {rss:.1f} MB')
    rclpy.shutdown()


if __name__ == '__main__':
    main()
