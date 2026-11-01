"""Offline preservation of archived operator inputs; no ROS imports or graph.

The observer archives contain actual seeds, not complete scan/odom bags. SQLite
transport, scan and odom below are explicitly synthetic test scaffolding.
"""
import ast
import bisect
from copy import deepcopy
import gzip
import json
import math
from pathlib import Path
import sqlite3
import sys
import textwrap
import threading
from types import SimpleNamespace as NS

import pytest

ROOT = Path(__file__).resolve().parents[2]
PACKAGE = ROOT / 'digital_twin/ws/src/x2_twin_scenarios'
sys.path.insert(0, str(PACKAGE))
ENTRYPOINTS = [
    ROOT / 'tools/twin/replay_amcl.py',
    PACKAGE / 'x2_twin_scenarios/analysis/replay_amcl.py',
]
ARCHIVES = [
    ROOT / 'docs/evidence/t5live1_observer_20261028.jsonl.gz',
    ROOT / 'docs/evidence/i1_deploy_2d240dbd/passive_observer.jsonl.gz',
]


def archived_seeds(path):
    with gzip.open(path, 'rt') as stream:
        return [row for line in stream if (row := json.loads(line))['kind'] == 'initialpose']


def obj(value):
    if isinstance(value, dict):
        return NS(**{key: obj(item) for key, item in value.items()})
    if isinstance(value, list):
        return [obj(item) for item in value]
    return value


def compose(a, b):
    c, s = math.cos(a[2]), math.sin(a[2])
    return a[0]+c*b[0]-s*b[1], a[1]+s*b[0]+c*b[1], a[2]+b[2]


def reader_source(source):
    start = source.index('# ---------------------------------------------------------------- read the recording')
    start = source.rfind('\n', 0, start) + 1
    end = source.index('# ---------------------------------------------------------------- run', start)
    return textwrap.dedent(source[start:end])


def run_reader(source, tmp_path, seeds, *, reference_x=90., max_seconds=0., first_stamp_delay_ns=0):
    """Execute actual reader statements only; ROS and process startup are excluded."""
    bag = tmp_path / 'bag'
    bag.mkdir()
    map_dir = tmp_path / 'ws/src/maps/fixture'
    map_dir.mkdir(parents=True)
    (map_dir / 'manifest.json').write_text(json.dumps({'pgm_sha256': 'test-map'}))
    db = sqlite3.connect(bag / 'fixture.db3')
    db.executescript('CREATE TABLE topics(id INTEGER PRIMARY KEY, name TEXT);'
                     'CREATE TABLE messages(id INTEGER PRIMARY KEY, topic_id INTEGER, timestamp INTEGER, data BLOB);')
    topics = ['/qazbot_d1_map/tf', '/qazbot_lio/scan',
              '/qazbot_d1_map/localization_trust/status', '/qazbot_d1_map/initialpose']
    db.executemany('INSERT INTO topics VALUES (?, ?)', enumerate(topics, 1))
    def add(topic_id, ts, message):
        db.execute('INSERT INTO messages(topic_id,timestamp,data) VALUES (?,?,?)',
                   (topic_id, ts, json.dumps(message).encode()))
    t0 = round(seeds[0]['wall'] * 1e9) - 1_000_000_000 if seeds else 10_000_000_000
    stamp_ns = t0 - first_stamp_delay_ns
    stamp = {'sec': stamp_ns // 10**9, 'nanosec': stamp_ns % 10**9}
    def tf(parent, child, x):
        return {'header': {'stamp': stamp, 'frame_id': parent}, 'child_frame_id': child,
                'transform': {'translation': {'x': x, 'y': 0., 'z': 0.},
                              'rotation': {'x': 0., 'y': 0., 'z': 0., 'w': 1.}}}
    transforms = [tf('odom', 'base_footprint', 1.)]
    if reference_x is not None:
        transforms.append(tf('map', 'odom', reference_x))
    add(1, t0, {'transforms': transforms})
    add(2, t0 + 1, {'header': {'stamp': stamp, 'frame_id': 'base_footprint'}, 'ranges': [1.]})
    add(3, t0 + 2, {'data': json.dumps({'map_sha256': 'test-map'})})
    for row in seeds:
        add(4, round(row['wall'] * 1e9), row['value'])
    db.commit()
    db.close()
    env = dict(sqlite3=sqlite3, bag=bag, root=tmp_path, STACK=tmp_path / 'ws/src',
               args=NS(max_seconds=max_seconds), json=json, bisect=bisect, math=math,
               deserialize_message=lambda raw, cls: obj(json.loads(raw)),
               TFMessage=object, LaserScan=object, PoseWithCovarianceStamped=object,
               String=object, compose=compose,
               yaw_of=lambda q: math.atan2(2*(q.w*q.z+q.x*q.y), 1-2*(q.y*q.y+q.z*q.z)))
    # The shared reader remains ROS-independent in the fixed entrypoints.
    try:
        from x2_twin_scenarios.analysis.replay_inputs import read_replay_inputs
        env['read_replay_inputs'] = read_replay_inputs
    except ModuleNotFoundError:
        pass
    exec(compile(reader_source(source), '<actual replay reader>', 'exec'), env)
    env['db'].close()
    return env


def assert_preserved(env, seeds):
    actual = [(ts, msg) for ts, kind, msg in env['msgs'] if kind == 'initialpose']
    assert len(actual) == len(seeds), 'replay dropped recorded operator initialpose/reseeds'
    for (ts, msg), seed in zip(actual, seeds):
        assert ts == round(seed['wall'] * 1e9) / 1e9
        assert msg == obj(seed['value']), 'header, pose and all covariance entries must survive'


@pytest.mark.parametrize('entrypoint', ENTRYPOINTS, ids=['host', 'packaged'])
@pytest.mark.parametrize('archive', ARCHIVES, ids=['four-seeds', 'passive-seed'])
def test_archived_operator_seeds_preserved(entrypoint, archive, tmp_path):
    seeds = archived_seeds(archive)
    assert len(seeds) == (4 if archive == ARCHIVES[0] else 1)
    env = run_reader(entrypoint.read_text(), tmp_path, seeds)
    assert_preserved(env, seeds)


@pytest.mark.parametrize('entrypoint', ENTRYPOINTS, ids=['host', 'packaged'])
@pytest.mark.parametrize('reference_x', [None, -900., 900.])
def test_seed_inputs_independent_of_recorded_amcl_output(entrypoint, reference_x, tmp_path):
    seeds = archived_seeds(ARCHIVES[0])
    env = run_reader(entrypoint.read_text(), tmp_path, seeds, reference_x=reference_x)
    assert_preserved(env, seeds)
    assert all((tf.header.frame_id, tf.child_frame_id) == ('odom', 'base_footprint')
               for _, kind, msg in env['msgs'] if kind == 'tf' for tf in msg.transforms)


@pytest.mark.parametrize('entrypoint', ENTRYPOINTS, ids=['host', 'packaged'])
def test_missing_operator_seed_refused(entrypoint, tmp_path):
    with pytest.raises(ValueError, match='recorded initialpose'):
        run_reader(entrypoint.read_text(), tmp_path, [])


@pytest.mark.parametrize('entrypoint', ENTRYPOINTS, ids=['host', 'packaged'])
def test_window_without_seed_refused(entrypoint, tmp_path):
    with pytest.raises(ValueError, match='recorded initialpose'):
        run_reader(entrypoint.read_text(), tmp_path, archived_seeds(ARCHIVES[0]), max_seconds=.5)


@pytest.mark.parametrize('entrypoint', ENTRYPOINTS, ids=['host', 'packaged'])
def test_window_preserves_only_seeds_inside_it(entrypoint, tmp_path):
    seeds = archived_seeds(ARCHIVES[0])
    env = run_reader(entrypoint.read_text(), tmp_path, seeds, max_seconds=2.)
    assert_preserved(env, seeds[:1])


@pytest.mark.parametrize('entrypoint', ENTRYPOINTS, ids=['host', 'packaged'])
def test_original_fail_repeated_from_frozen_baseline(entrypoint, tmp_path):
    baseline = json.loads((ROOT / 'docs/evidence/amcl_initialpose_v3_attempt4_20261101/baseline_readers.json').read_text())
    reader = baseline['readers'][str(entrypoint.relative_to(ROOT))]['reader']
    # Run the unchanged original reader again, not a reimplementation of its bug.
    source = reader + '\n# ---------------------------------------------------------------- run\n'
    seeds = archived_seeds(ARCHIVES[0])
    env = run_reader(source, tmp_path, seeds)
    with pytest.raises(AssertionError, match='dropped recorded operator'):
        assert_preserved(env, seeds)
    assert env['seed'][0] == 91.  # synthetic recorded map->odom + odom, never an operator input


@pytest.mark.parametrize('entrypoint', ENTRYPOINTS, ids=['host', 'packaged'])
def test_original_source_age_fail_repeated(entrypoint, tmp_path):
    baseline = json.loads((ROOT / 'docs/evidence/amcl_initialpose_v3_attempt4_20261101/baseline_readers.json').read_text())
    env = run_reader(entrypoint.read_text(), tmp_path, archived_seeds(ARCHIVES[0]), first_stamp_delay_ns=300_000_000)
    env['n'] = NS(get_clock=lambda: NS(now=lambda: NS(nanoseconds=50_000_000_000)))
    alignment = baseline['readers'][str(entrypoint.relative_to(ROOT))]['clock_alignment_source']
    exec(compile(alignment, '<original clock alignment>', 'exec'), env)
    correct_offset = 50_500_000_000 - round(env['t0'] * 1e9)
    assert env['delta'] - correct_offset == 300_000_000


def stamp_ns(stamp):
    return stamp.sec * 10**9 + stamp.nanosec


def playback_statements(source):
    tree = ast.parse(source)
    loop = next(node for node in ast.walk(tree) if isinstance(node, ast.For)
                and isinstance(node.target, ast.Tuple)
                and [getattr(item, 'id', '') for item in node.target.elts] == ['ts', 'kind', 'm'])
    # Execute preparation + the actual playback loop, never init/start/cleanup.
    names = {'first_hdr', 'now', 'delta', 'play0', 'publishers'}
    preparation = sorted([node for node in ast.walk(tree) if isinstance(node, ast.Assign)
                          and any(isinstance(t, ast.Name) and t.id in names for t in node.targets)],
                         key=lambda node: node.lineno)
    return ast.Module(body=preparation + [loop], type_ignores=[])


@pytest.mark.parametrize('entrypoint', ENTRYPOINTS, ids=['host', 'packaged'])
def test_actual_dispatch_preserves_seed_age_order_and_spacing(entrypoint, tmp_path):
    from x2_twin_scenarios.analysis.replay_inputs import shift_input_stamps
    source = entrypoint.read_text()
    env = run_reader(source, tmp_path, archived_seeds(ARCHIVES[0]), first_stamp_delay_ns=300_000_000)
    before = deepcopy(env['msgs'])
    clock = NS(now=0.)
    published = []
    def publisher(kind):
        return NS(publish=lambda msg: published.append((clock.now, kind, deepcopy(msg))))
    env.update(n=NS(get_clock=lambda: NS(now=lambda: NS(nanoseconds=50_000_000_000))),
               time=NS(monotonic=lambda: clock.now),
               rclpy=NS(spin_once=lambda *a, **kw: setattr(clock, 'now', clock.now + .002)),
               Time=NS, shift_input_stamps=shift_input_stamps,
               tf_pub=publisher('tf'), scan_pub=publisher('scan'), init_pub=publisher('initialpose'))
    exec(compile(playback_statements(source), '<actual playback loop>', 'exec'), env)
    expected_delta = 50_500_000_000 - round(env['t0'] * 1e9)
    assert env['delta'] == expected_delta, 'align receive clock, preserving recorded source age'
    assert [kind for _, kind, _ in published] == [kind for _, kind, _ in before]
    for (receipt, kind, shifted), (ts, _, original) in zip(published, before):
        assert receipt - env['play0'] == pytest.approx(ts - env['t0'], abs=.0021)
        if kind == 'initialpose':
            assert shifted.pose == original.pose
            assert stamp_ns(shifted.header.stamp) - stamp_ns(original.header.stamp) == expected_delta
    # No seed may be emitted outside that recorded-event loop.
    assert not any(isinstance(node, ast.Call) and isinstance(node.func, ast.Attribute)
                   and isinstance(node.func.value, ast.Name) and node.func.value.id == 'init_pub'
                   and node.func.attr == 'publish' for node in ast.walk(ast.parse(source)))


def test_zero_stamp_and_invalid_seed_preserved():
    from x2_twin_scenarios.analysis.replay_inputs import shift_input_stamps
    msg = obj(archived_seeds(ARCHIVES[0])[0]['value'])
    msg.header.stamp = NS(sec=0, nanosec=0)
    msg.header.frame_id = 'invalid-frame-for-regression'
    msg.pose.pose.orientation.w = 0.
    original = deepcopy(msg)
    shift_input_stamps('initialpose', msg, 123_456_789, NS)
    assert msg == original, 'zero means now; invalid inputs must reach AMCL unchanged'


@pytest.mark.parametrize('archive', ARCHIVES, ids=['four-seeds', 'passive-seed'])
def test_saved_initialpose_amcl_callbacks_replayed_offline(archive):
    """Recorded callback inputs only: no AMCL rerun, freshness or READY claim."""
    import numpy as np
    sys.path.insert(0, str(ROOT / 'ws/src/navigation'))
    import loc_trust as L
    source = (ROOT / 'ws/src/navigation/localization_trust_node.py').read_text()
    cls = next(node for node in ast.parse(source).body if isinstance(node, ast.ClassDef) and node.name == 'TrustNode')
    methods = [node for node in cls.body if isinstance(node, ast.FunctionDef) and node.name in ('_initialpose', '_amcl')]
    clock = NS(now=0.)
    scope = {'np': np, 'time': NS(monotonic=lambda: clock.now)}
    exec(compile(ast.Module(body=methods, type_ignores=[]), '<actual Trust callbacks>', 'exec'), scope)
    node = NS(cfg=L.TrustConfig(admission_policy='go2'), ev=L.TrustEvaluator(None),
              lock=threading.Lock(), seed_stamp=None, pending_map_steps=[],
              seed_update_enabled=False, seed_update_future=None, odom=None,
              _odom_at=lambda *a: None, get_logger=lambda: NS(warn=lambda *a: None),
              _stamp=lambda *a: clock.now)
    # Source-stamp/freshness validation is separately covered by Trust regressions;
    # observer receipt differs from the original node's receipt and cannot replace it.
    with gzip.open(archive, 'rt') as stream:
        rows = [row for line in stream if (row := json.loads(line))['kind'] in ('initialpose', 'amcl_pose')]
    seeds = accepted = 0
    for row in rows:
        clock.now = row['wall']
        if row['kind'] == 'initialpose':
            scope['_initialpose'](node, obj(row['value']))
            seeds += 1
            assert not node.ev.initialized
        else:
            pending = node.seed_stamp is not None
            scope['_amcl'](node, obj(row['value']))
            if pending and node.ev.initialized:
                accepted += 1
    assert seeds == accepted == len(archived_seeds(archive))


@pytest.mark.parametrize('log_name,expected', [
    ('t5live1_localization_20261028.log.gz', 4),
    ('i1_deploy_2d240dbd/localization.log.gz', 1),
])
def test_preserved_warning_free_criterion_still_fails(log_name, expected):
    """Re-evaluate the historical FAIL, without relabeling it a new runtime run."""
    import re
    from decimal import Decimal
    with gzip.open(ROOT / 'docs/evidence' / log_name, 'rt') as stream:
        lines = stream.readlines()
    warnings = [line for line in lines if 'Failed to transform initial pose in time' in line]
    assert len(warnings) == expected
    for line in warnings:
        requested, latest = re.search(r'Requested time ([\d.]+) but the latest data is at time ([\d.]+)', line).groups()
        deficit = Decimal(requested) - Decimal(latest)
        assert Decimal('.008') < deficit < Decimal('.035')
    # The strict no-warning criterion FAIL is intentionally retained.
    assert all('extrapolation into the future' in line for line in warnings)
