#!/usr/bin/env python3
"""Per-topic rate/bandwidth/stamp-lag + TF tree (frames, edge rates, lag) from a closed rosbag2 split. Run with nice.
usage: bag_stats.py <bag_dir> [storage=mcap] > out.txt"""
import sys, collections, statistics as st
import rosbag2_py
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message
uri = sys.argv[1]; sid = sys.argv[2] if len(sys.argv) > 2 else 'mcap'
r = rosbag2_py.SequentialReader(); r.open(rosbag2_py.StorageOptions(uri=uri, storage_id=sid), rosbag2_py.ConverterOptions('cdr', 'cdr'))
types = {t.name: t.type for t in r.get_all_topics_and_types()}
cls = {}
cnt = collections.Counter(); byt = collections.Counter(); t0 = {}; t1 = {}; lag = collections.defaultdict(list)
edges = collections.defaultdict(lambda: {'n': 0, 'lag': [], 'topic': set(), 'first': None, 'last': None, 'xyz': None})
sample = {}
HDR_SKIP = ('std_msgs/msg/String', 'std_msgs/msg/Bool', 'geometry_msgs/msg/Twist', 'aimdk_msgs/msg/McRLDebug')
while r.has_next():
    topic, data, ts = r.read_next(); cnt[topic] += 1; byt[topic] += len(data); t0.setdefault(topic, ts); t1[topic] = ts
    ty = types[topic]
    if ty in HDR_SKIP and topic not in ('/aima/mc/common/state',): 
        if ty == 'std_msgs/msg/String' and cnt[topic] == 1: sample[topic] = deserialize_message(data, get_message(ty)).data[:600]
        continue
    if cnt[topic] % (1 if 'tf' in topic else 5) and topic not in ('/aima/mc/common/state',): continue  # subsample non-TF topics for lag
    if ty not in cls: cls[ty] = get_message(ty)
    try: m = deserialize_message(data, cls[ty])
    except Exception: continue
    if ty == 'tf2_msgs/msg/TFMessage':
        for tr in m.transforms:
            k = (tr.header.frame_id, tr.child_frame_id); e = edges[k]; e['n'] += 1; e['topic'].add(topic)
            s = tr.header.stamp.sec + tr.header.stamp.nanosec * 1e-9; e['first'] = e['first'] or ts; e['last'] = ts
            e['lag'].append(ts * 1e-9 - s) if 'static' not in topic else None
            t = tr.transform.translation; q = tr.transform.rotation; e['xyz'] = (round(t.x, 4), round(t.y, 4), round(t.z, 4), round(q.x, 4), round(q.y, 4), round(q.z, 4), round(q.w, 4))
        continue
    h = getattr(m, 'header', None)
    if h is not None and hasattr(h, 'stamp'):
        s = h.stamp.sec + h.stamp.nanosec * 1e-9
        if s > 0: lag[topic].append(ts * 1e-9 - s)
    if topic == '/aima/mc/common/state' and topic not in sample: sample[topic] = str(m)[:1500]
    if topic == '/aima/mc/common/state': sample.setdefault('mc_states', set()).add(str(getattr(m, 'fsm_state', ''))[:200] + ' | ' + str(getattr(m, 'input_source', ''))[:200])
dur = lambda t: max((t1[t] - t0[t]) / 1e9, 1e-9)
pct = lambda v, p: sorted(v)[min(len(v) - 1, int(p * len(v)))]
print(f"# bag {uri}\n# topic | type | count | rate_hz | bytes/msg | KiB/s | stamp lag (recv - header) p50/p95/max s")
for t in sorted(cnt):
    l = lag.get(t); ls = f"{pct(l,.5):.3f}/{pct(l,.95):.3f}/{max(l):.3f}" if l else '-'
    print(f"{t} | {types[t]} | {cnt[t]} | {cnt[t]/dur(t) if cnt[t]>1 else 0:.2f} | {byt[t]/cnt[t]:.0f} | {byt[t]/dur(t)/1024:.2f} | {ls}")
print("\n# TF edges: parent -> child | topics | count | rate_hz | lag p50/p95/max s (recv - stamp) | last transform xyz,qxyzw")
for (p, c), e in sorted(edges.items()):
    d = max((e['last'] - e['first']) / 1e9, 1e-9); l = e['lag']
    ls = f"{pct(l,.5):.3f}/{pct(l,.95):.3f}/{max(l):.3f}" if l else 'static'
    print(f"{p} -> {c} | {','.join(sorted(e['topic']))} | {e['n']} | {e['n']/d if e['n']>1 else 0:.1f} | {ls} | {e['xyz']}")
print("\n# samples")
for k, v in sample.items(): print(k, '=>', v if not isinstance(v, set) else sorted(v)[:10])
