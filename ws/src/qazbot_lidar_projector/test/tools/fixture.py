"""Fixture container shared by make_fixture.py, the parity test and the integration test (numpy only).

Little-endian. Header: b'LBFX1\\0\\0\\0', u32 n_events. Event: u8 kind (0 imu, 1 odom, 2 cloud), i32 sec, u32 nanosec, then
  imu:   3 f64 linear acceleration
  odom:  3 f64 position, 4 f64 quaternion (x y z w), 36 f64 pose covariance (row-major)
  cloud: u8 is_dense, u32 n, n*3 f32 (x y z)
"""
import struct
from collections import namedtuple

import numpy as np

MAGIC = b'LBFX1\0\0\0'
KINDS = ('imu', 'odom', 'cloud')
Event = namedtuple('Event', 'kind sec nanosec data')


def cloud_xyz(msg):
    """x, y, z of a PointCloud2 as an (n, 3) float32 array, whatever the field order/padding."""
    off = {f.name: f.offset for f in msg.fields}
    raw = np.frombuffer(bytes(msg.data), dtype=np.uint8).reshape(-1, msg.point_step)
    cols = [raw[:, off[c]:off[c] + 4].copy().view('<f4').reshape(-1) for c in 'xyz']
    return np.stack(cols, axis=1).astype(np.float32)


def write(path, events):
    with open(path, 'wb') as f:
        f.write(MAGIC + struct.pack('<I', len(events)))
        for e in events:
            f.write(struct.pack('<BiI', KINDS.index(e.kind), e.sec, e.nanosec))
            if e.kind == 'imu':
                f.write(struct.pack('<3d', *e.data))
            elif e.kind == 'odom':
                p, q, c = e.data
                f.write(struct.pack('<3d4d36d', *p, *q, *c))
            else:
                xyz, dense = e.data
                f.write(struct.pack('<BI', int(dense), len(xyz)))
                f.write(np.ascontiguousarray(xyz, dtype='<f4').tobytes())


def read(path):
    with open(path, 'rb') as f:
        buf = f.read()
    assert buf[:8] == MAGIC
    n, = struct.unpack_from('<I', buf, 8)
    pos, out = 12, []
    for _ in range(n):
        k, sec, nsec = struct.unpack_from('<BiI', buf, pos)
        pos += 9
        kind = KINDS[k]
        if kind == 'imu':
            data, pos = struct.unpack_from('<3d', buf, pos), pos + 24
        elif kind == 'odom':
            v = struct.unpack_from('<43d', buf, pos)
            pos += 43 * 8
            data = (v[0:3], v[3:7], v[7:43])
        else:
            dense, cnt = struct.unpack_from('<BI', buf, pos)
            pos += 5
            xyz = np.frombuffer(buf, dtype='<f4', count=cnt * 3, offset=pos).reshape(-1, 3).astype(np.float32)
            pos += cnt * 12
            data = (xyz, bool(dense))
        out.append(Event(kind, sec, nsec, data))
    return out


def canonical_replay(events, level_s=1.0):
    """Replay order that does not depend on callback races between the IMU, odometry and cloud topics.

    [first odometry] + [IMU samples up to the one that completes levelling] + [every later odometry/cloud in bag order].
    Odometry between the first one and the end of the levelling window is dropped (it would change the orientation the
    IMU samples are rotated with, depending on delivery timing); later IMU samples are ignored by the node anyway.
    Returns (events, number_of_imu_events_in_the_levelling_window).
    """
    i0 = next(i for i, e in enumerate(events) if e.kind == 'odom')
    out, t0, j = [events[i0]], None, i0 + 1
    while j < len(events):
        e = events[j]
        j += 1
        if e.kind != 'imu':
            continue
        t = e.sec + e.nanosec * 1e-9
        t0 = t if t0 is None else t0
        out.append(e)
        if t - t0 >= level_s:
            break
    n_imu = len(out) - 1
    out += [e for e in events[j:] if e.kind != 'imu']
    return out, n_imu
