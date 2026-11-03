import json
import os
import sys
import tempfile
import unittest
from types import SimpleNamespace as NS

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
import stage1_common as c
import slam_mapping_client as smc
import check_stored_map as csm


def fake_resp(code=0, data=(1, 2)):
    return NS(code=code, map_id=7, map_version=3, map_path='/x/y.png',
              map_info=NS(resolution=0.05, width=10, height=20), data=list(data))


class FakeNode:
    def __init__(self, subs, resps):
        self.subs, self.resps, self.pub, self.logs = subs, list(resps), [], []

    def log(self, m): self.logs.append(m)
    def subscriber_count(self): return self.subs
    def publish_text(self, t): self.pub.append(t)
    def call_get_map(self, name): return self.resps.pop(0) if self.resps else None


class Clock:
    def __init__(self): self.t = 0.0
    def now(self): return self.t
    def sleep(self, s): self.t += s


class T(unittest.TestCase):
    def run_flow(self, node, name='m1', path=None):
        ck = Clock()
        path = path or tempfile.mktemp(suffix='.json')
        rc = smc.run_mapping(node, name, json_path=path, sub_wait_s=2, wait_stop=lambda e: None,
                             sleep=ck.sleep, now=ck.now)
        return rc, path

    def test_payload(self):
        self.assertEqual(c.build_stop_payload('office_test_01'), 'stop_mapping:office_test_01')

    def test_no_subscribers_no_publish(self):
        n = FakeNode(0, [])
        rc, path = self.run_flow(n)
        self.assertEqual(rc, smc.EXIT_NO_SUBSCRIBER)
        self.assertEqual(n.pub, [])
        self.assertIn(smc.NO_SUB_ERROR, n.logs)
        self.assertFalse(os.path.exists(path))

    def test_empty_name(self):
        n = FakeNode(1, [])
        rc, _ = self.run_flow(n, name='  ')
        self.assertEqual(rc, smc.EXIT_BAD_ARGS)
        self.assertEqual(n.pub, [])

    def test_success_code0(self):
        n = FakeNode(1, [fake_resp(5), fake_resp(0)])
        rc, path = self.run_flow(n)
        self.assertEqual(rc, 0)
        self.assertEqual(n.pub, ['start_mapping', 'stop_mapping:m1'])
        self.assertTrue(any('Map saved' in m for m in n.logs))
        self.assertEqual(json.load(open(path))['map_id'], 7)

    def test_no_saved_without_code0(self):
        n = FakeNode(1, [fake_resp(5)] * 200)
        rc, path = self.run_flow(n)
        self.assertEqual(rc, smc.EXIT_NOT_SAVED)
        self.assertFalse(any('saved' in m.lower() and 'NOT' not in m for m in n.logs))
        self.assertFalse(os.path.exists(path))

    def test_json_writer(self):
        rec = c.build_map_check('m1', fake_resp(data=[]))
        self.assertFalse(rec['grid_data_present'])
        self.assertFalse(rec['rviz_visible'])
        self.assertEqual(rec['width'], 10)
        p = tempfile.mktemp(suffix='.json')
        c.write_map_check(p, rec)
        d = json.load(open(p)); d['rviz_visible'] = True; json.dump(d, open(p, 'w'))
        c.write_map_check(p, c.build_map_check('m1', fake_resp()))
        d = json.load(open(p))
        self.assertTrue(d['grid_data_present'])
        self.assertTrue(d['rviz_visible'])  # preserved
        for k in ('map_name', 'map_id', 'map_path', 'map_version', 'resolution', 'width', 'height'):
            self.assertIn(k, d)

    def test_missing_field_null(self):
        r = fake_resp(); del r.map_version
        self.assertIsNone(c.build_map_check('m', r)['map_version'])

    def test_validate_and_db(self):
        self.assertEqual([p for p in csm.validate_response('m', fake_resp()) if 'warning' not in p], [])
        self.assertTrue(csm.validate_response('m', fake_resp(code=1)))
        self.assertTrue(csm.validate_response('m', None))
        import sqlite3
        p = tempfile.mktemp(suffix='.db')
        con = sqlite3.connect(p); con.execute('CREATE TABLE map(map_id INT, map_name TEXT)')
        con.execute("INSERT INTO map VALUES (7,'m1')"); con.commit(); con.close()
        rows, err = csm.read_db(p)
        self.assertIsNone(err)
        rec = c.build_map_check('m1', fake_resp())
        self.assertIn('consistent', csm.cross_check_db(rows, rec)[0])
        _, err = csm.read_db('/nonexistent/dir/map.db')
        self.assertIn('not found', err)


if __name__ == '__main__':
    unittest.main()
