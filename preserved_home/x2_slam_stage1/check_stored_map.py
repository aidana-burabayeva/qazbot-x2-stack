#!/usr/bin/env python3
"""Standalone read-only check of a stored map via GetStoredMapByName (+ optional map.db).

Official refs: https://x2-aimdk.agibot.com/en/latest/Interface/perception/SLAM.html ;
/home/agi/aimdk/src/py_examples/py_examples/get_map.py ;
map.db query from docs: sqlite3 /agibot/data/var/MapManagerModule/map.db "SELECT * FROM map;"
"""
import argparse
import sqlite3
import sys

import stage1_common as c

try:
    import rclpy
    from std_msgs.msg import Header
    from aimdk_msgs.srv import GetStoredMapByName
except ImportError as exc:
    rclpy = None
    _IMPORT_ERR = exc


def read_db(path):
    """Read-only 'SELECT * FROM map;'. Returns (rows as dicts, error str|None)."""
    try:
        con = sqlite3.connect(f'file:{path}?mode=ro', uri=True)
        con.row_factory = sqlite3.Row
        try:
            rows = [dict(r) for r in con.execute('SELECT * FROM map;')]
        finally:
            con.close()
        return rows, None
    except PermissionError as e:
        return None, f'PERMISSION DENIED reading {path} (blocker B3): {e}'
    except sqlite3.OperationalError as e:
        msg = str(e)
        if 'unable to open' in msg and not _exists(path):
            return None, f'map.db not found at {path}: {msg}'
        if 'unable to open' in msg:
            return None, f'cannot open {path} (likely permission denied, blocker B3): {msg}'
        return None, f'sqlite error on {path}: {msg}'


def _exists(path):
    import os
    return os.path.exists(path)


def validate_response(map_name, resp):
    """Return list of problems (empty == OK)."""
    problems = []
    if resp is None:
        return ['no response from GetStoredMapByName']
    if resp.code != 0:
        problems.append(f'code={resp.code} (expected 0)')
    if not getattr(resp, 'map_id', 0):
        problems.append('map_id is 0/missing')
    if not getattr(resp, 'map_path', ''):
        problems.append('map_path empty')
    info = getattr(resp, 'map_info', None)
    if info is None or not (info.width > 0 and info.height > 0 and info.resolution > 0):
        problems.append('map_info invalid (width/height/resolution)')
    if not len(getattr(resp, 'data', []) or []):
        problems.append('data[] empty (warning: field reserved per srv comment)')
    return problems


def cross_check_db(rows, record):
    """Cross-check map_name/map_id against map table rows; returns list of messages."""
    msgs = []
    name_keys = ('map_name', 'name')
    match = [r for r in rows if any(r.get(k) == record['map_name'] for k in name_keys)]
    if not match:
        return [f'DB: no row with map_name={record["map_name"]!r}; columns={list(rows[0]) if rows else "n/a"}']
    ids = {r.get('map_id') for r in match}
    if record['map_id'] is not None and record['map_id'] not in ids:
        msgs.append(f'DB: map_id mismatch service={record["map_id"]} db={sorted(i for i in ids if i is not None)}')
    else:
        msgs.append('DB: map_name/map_id consistent')
    return msgs


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument('map_name')
    ap.add_argument('--json', default=c.DEFAULT_JSON)
    ap.add_argument('--db', help='path to map.db (opened read-only)')
    ap.add_argument('--timeout', type=float, default=10.0)
    a = ap.parse_args(argv)
    if not c.validate_map_name(a.map_name):
        print('ERROR: map_name must be non-empty', file=sys.stderr)
        return 3
    if rclpy is None:
        print(f'ERROR: ROS imports failed: {_IMPORT_ERR}', file=sys.stderr)
        return 3
    rclpy.init()
    node = rclpy.create_node('check_stored_map')
    rc = 0
    try:
        cli = node.create_client(GetStoredMapByName, c.GET_STORED_MAP_BY_NAME_SRV)
        if not cli.wait_for_service(timeout_sec=a.timeout):
            print(f'ERROR: service {c.GET_STORED_MAP_BY_NAME_SRV} unavailable (blocker B1)', file=sys.stderr)
            return 2
        req = GetStoredMapByName.Request()
        req.header = Header()
        req.header.stamp = node.get_clock().now().to_msg()
        req.map_name = a.map_name
        fut = cli.call_async(req)
        rclpy.spin_until_future_complete(node, fut, timeout_sec=a.timeout)
        resp = fut.result()
        problems = validate_response(a.map_name, resp)
        hard = [p for p in problems if 'warning' not in p]
        if resp is not None and resp.code == 0:
            rec = c.build_map_check(a.map_name, resp)
            c.write_map_check(a.json, rec)
            print(f'JSON updated: {a.json}')
        for p in problems:
            print(('FAIL: ' if p in hard else 'WARN: ') + p)
        if hard:
            rc = 1
        else:
            print('OK: GetStoredMapByName code==0 and fields valid')
        if a.db:
            rows, err = read_db(a.db)
            if err:
                print('ERROR: ' + err, file=sys.stderr)
                rc = rc or 5
            elif resp is not None and resp.code == 0:
                for m in cross_check_db(rows, rec):
                    print(m)
                    if 'mismatch' in m or 'no row' in m:
                        rc = rc or 6
    finally:
        node.destroy_node()
        rclpy.shutdown()
    return rc


if __name__ == '__main__':
    sys.exit(main())
