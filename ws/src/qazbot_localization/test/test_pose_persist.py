"""Confirmed-pose persistence (Step B): qazbot_localization/scripts/pose_store.py + seed_pose.py --from-file refusal paths.
Pure Python, no ROS graph. Run: cd tests/loc_trust && python3 -m unittest -v test_pose_persist"""
import hashlib
import json
import os
import subprocess
import sys
import tempfile
import unittest
from unittest import mock

from test_loc_trust import COV, HERE, L, make_inputs, room_grid, simulate_scan

NAV = os.path.join(HERE, '..', 'scripts')
sys.path.insert(0, NAV)
import pose_store as P  # noqa: E402

MAP_YAML = os.path.join(HERE, '..', '..', 'qazbot_maps', 'maps', 'first_map_nav2_v2', 'map.yaml')
SHA = P.map_image_sha256(MAP_YAML)


class Store(unittest.TestCase):
    def setUp(self):
        self.dir = tempfile.TemporaryDirectory()
        self.path = os.path.join(self.dir.name, 'pose.json')

    def tearDown(self):
        self.dir.cleanup()

    def test_map_sha_is_the_manifest_image_hash(self):
        with open(os.path.join(os.path.dirname(MAP_YAML), 'manifest.json')) as f:
            self.assertEqual(SHA, json.load(f)['pgm_sha256'])
        with open(os.path.join(os.path.dirname(MAP_YAML), 'map.pgm'), 'rb') as f:
            self.assertEqual(SHA, hashlib.sha256(f.read()).hexdigest())

    def test_round_trip_keeps_pose_and_covariance(self):
        P.save(self.path, (1.0, -2.0, 0.5), COV, SHA, 'ready')
        rec, why = P.load(self.path, SHA)
        self.assertEqual(why, '')
        self.assertEqual((rec['x'], rec['y'], rec['yaw']), (1.0, -2.0, 0.5))
        self.assertEqual(rec['covariance'], [float(v) for v in COV])
        self.assertEqual(rec['source'], 'ready')
        self.assertFalse(os.path.exists(self.path + '.tmp'))

    def test_atomic_write_keeps_old_file_when_replace_fails(self):
        P.save(self.path, (1.0, 2.0, 0.0), COV, SHA, 'ready')
        with open(self.path) as f:
            before = f.read()
        with mock.patch.object(P.os, 'replace', side_effect=OSError('disk')):
            with self.assertRaises(OSError):
                P.save(self.path, (9.0, 9.0, 0.0), COV, SHA, 'ready')
        with open(self.path) as f:
            self.assertEqual(f.read(), before)
        self.assertEqual(P.load(self.path, SHA)[0]['x'], 1.0)

    def test_invalid_pose_or_covariance_is_not_written(self):
        for pose, cov in (((float('nan'), 0, 0), COV), ((0, 0, 0), [0.0] * 36), ((0, 0, 0), COV[:35])):
            with self.assertRaises(ValueError):
                P.save(self.path, pose, cov, SHA, 'ready')
        self.assertFalse(os.path.exists(self.path))

    def test_map_sha_mismatch_rejected(self):
        P.save(self.path, (1.0, 2.0, 0.0), COV, 'f' * 64, 'ready')
        self.assertEqual(P.load(self.path, SHA), (None, 'map_sha256_mismatch'))

    def test_missing_corrupt_or_foreign_file_ignored(self):
        self.assertEqual(P.load(self.path, SHA), (None, 'no_file'))
        P.save(self.path, (1.0, 2.0, 0.0), COV, SHA, 'ready')
        with open(self.path) as f:
            good = f.read()
        for text, reason in ((good[: len(good) // 2], 'unreadable'), ('[]', 'format'), ('', 'unreadable'),
                             (good.replace('"format": 1', '"format": 2'), 'format')):
            with open(self.path, 'w') as f:
                f.write(text)
            rec, why = P.load(self.path, SHA)
            self.assertIsNone(rec)
            self.assertTrue(why.startswith(reason), (text[:30], why))
        rec = json.loads(good)
        rec['covariance'][0] = -1.0
        with open(self.path, 'w') as f:
            json.dump(rec, f)
        self.assertEqual(P.load(self.path, SHA), (None, 'covariance'))


class SaverWithEvaluator(unittest.TestCase):
    """The node calls saver.on_verdict(r, compose(map_odom, odom), amcl_cov, map_ok) after every evaluation."""
    def setUp(self):
        self.dir = tempfile.TemporaryDirectory()
        self.path = os.path.join(self.dir.name, 'pose.json')
        self.g = room_grid()
        self.mo, self.odom = (0.3, -0.2, 0.05), (0.1, 0.0, -0.05)
        self.pose = L.compose(self.mo, self.odom)
        self.scan = simulate_scan(self.g, self.pose)
        self.ev = L.TrustEvaluator(self.g, L.TrustConfig(admission_policy='strict'))
        self.saver = P.ConfirmedPoseSaver(self.path, SHA, log=lambda text: None)
        self.saves = []
        real = P.save
        def counting(*a, **k):
            self.saves.append(a[4])
            return real(*a, **k)
        patcher = mock.patch.object(P, 'save', side_effect=counting)
        patcher.start()
        self.addCleanup(patcher.stop)

    def tearDown(self):
        self.dir.cleanup()

    def tick(self, t, **kw):
        inp = make_inputs(t, self.mo, self.odom, self.scan, **kw)
        r = self.ev.evaluate(inp)
        self.saver.on_verdict(r, L.compose(inp.map_odom, inp.odom_base), inp.amcl_cov, inp.map_ok)
        return r

    def test_saves_only_on_confirmed_ready_rising_edge(self):
        for k in range(10):                               # clean AMCL output, but no operator confirmation
            self.assertFalse(self.tick(1 + 0.2 * k)['ready'])
        self.assertEqual(self.saves, [])
        self.assertFalse(os.path.exists(self.path))
        self.ev.confirm_anchor(make_inputs(3, self.mo, self.odom, self.scan))
        self.assertTrue(self.tick(3.2)['ready'])
        for k in range(10):                               # READY held: no further writes
            self.assertTrue(self.tick(3.4 + 0.2 * k)['ready'])
        self.assertEqual(self.saves, ['ready'])
        rec, why = P.load(self.path, SHA)
        self.assertEqual(why, '')
        self.assertAlmostEqual(rec['x'], self.pose[0])
        self.assertAlmostEqual(rec['y'], self.pose[1])
        self.assertAlmostEqual(rec['yaw'], self.pose[2])
        self.assertEqual(rec['covariance'], [float(v) for v in COV])
        self.ev.relocalize()                              # initialpose: READY lost, nothing saved
        self.assertFalse(self.tick(6)['ready'])
        self.ev.confirm_anchor(make_inputs(7, self.mo, self.odom, self.scan))
        self.assertTrue(self.tick(7.2)['ready'])
        self.assertEqual(self.saves, ['ready', 'ready'])

    def test_failed_gate_after_confirm_does_not_save(self):
        self.ev.confirm_anchor(make_inputs(1, self.mo, self.odom, self.scan))
        r = self.tick(1.2, amcl_cov=COV * 100)            # sigma_xy 1 m: T2 blocks the gate
        self.assertFalse(r['ready'])
        self.assertEqual(self.saves, [])

    def test_map_mismatch_never_saved(self):
        self.assertIsNone(self.saver.on_verdict(dict(ready=True), self.pose, list(COV), 'size'))
        self.assertIsNone(P.ConfirmedPoseSaver(self.path, SHA, log=lambda t: None).on_verdict(
            dict(ready=True), self.pose, list(COV), None))
        self.assertFalse(os.path.exists(self.path))

    def test_disabled_without_path(self):
        saver = P.ConfirmedPoseSaver('', SHA, log=lambda t: None)
        self.assertIsNone(saver.on_verdict(dict(ready=True), self.pose, list(COV), ''))
        self.assertEqual(self.saves, [])

    def test_shutdown_saves_only_clean_ready(self):
        cases = [(dict(ready=True, motion_ok=True, diagnostics=[]), True),
                 (dict(ready=True, motion_ok=True, diagnostics=['T4_no_baseline']), True),
                 (dict(ready=True, motion_ok=True, diagnostics=['T4_scan_score']), False),
                 (dict(ready=True, motion_ok=True, diagnostics=['T6_anchor_disagreement']), False),
                 (dict(ready=True, motion_ok=False, diagnostics=[]), False),
                 (dict(ready=False, motion_ok=False, diagnostics=[]), False)]
        for verdict, saved in cases:
            self.saves.clear()
            out = self.saver.on_shutdown(verdict, self.pose, list(COV), '')
            self.assertEqual(out is not None, saved, verdict)
            self.assertEqual(self.saves, ['shutdown'] if saved else [], verdict)


class SeededPoseIsNotReady(unittest.TestCase):
    def test_seed_initialpose_clears_ready_until_confirm(self):
        """The trust node handles every initialpose (also the re-seed) with relocalize(): no anchor, READY false on
        perfectly clean inputs, until the operator's confirm_anchor and a clean gate."""
        g = room_grid()
        pose = (0.0, 0.0, 0.0)
        scan = simulate_scan(g, pose)
        ev = L.TrustEvaluator(g, L.TrustConfig(admission_policy='strict'))
        ev.relocalize()
        for k in range(30):
            r = ev.evaluate(make_inputs(1 + 0.2 * k, pose, pose, scan))
            self.assertFalse(r['ready'] or r['motion_ok'] or r['trusted'], r)
        self.assertIsNone(ev.anchor)
        self.assertEqual(ev.ready_cause, 'initialpose')
        ev.confirm_anchor(make_inputs(8, pose, pose, scan))
        self.assertTrue(ev.evaluate(make_inputs(8.2, pose, pose, scan))['ready'])


class SeedRefusals(unittest.TestCase):
    """seed_pose.py --from-file exits 0 and publishes nothing (it never creates a ROS node) when the file is unusable."""
    def run_seed(self, path, *extra):
        return subprocess.run([sys.executable, os.path.join(NAV, 'seed_pose.py'), '--from-file', path,
                               '--map-yaml', MAP_YAML, *extra], capture_output=True, text=True, timeout=60)

    def test_refusals(self):
        with tempfile.TemporaryDirectory() as d:
            path = os.path.join(d, 'pose.json')
            out = self.run_seed(path)
            self.assertEqual(out.returncode, 0, out.stderr)
            self.assertEqual(json.loads(out.stdout)['reason'], 'no_file')
            P.save(path, (1.0, 2.0, 0.0), COV, 'f' * 64, 'ready')
            self.assertEqual(json.loads(self.run_seed(path).stdout)['reason'], 'map_sha256_mismatch')
            with open(path, 'w') as f:
                f.write('{"format": 1, "x": ')
            self.assertTrue(json.loads(self.run_seed(path).stdout)['reason'].startswith('unreadable'))
            for out in (self.run_seed(path, '--x', '1'), subprocess.run(
                    [sys.executable, os.path.join(NAV, 'seed_pose.py'), '--from-file', path],
                    capture_output=True, text=True, timeout=60)):
                self.assertEqual(out.returncode, 2, out.stderr)   # argparse usage error, nothing published


class LaunchSeed(unittest.TestCase):
    """localization.launch.py: one-shot seed process, default on (pose_store.DEFAULT_PATH), off with seed_pose_file:='',
    and its exit does not trigger the dependency-exit shutdown guard. Parses only; starts nothing."""
    def test_seed_process(self):
        import importlib.util
        from launch import LaunchContext
        from launch.actions import DeclareLaunchArgument, ExecuteProcess, RegisterEventHandler, TimerAction
        from launch.utilities import perform_substitutions
        spec = importlib.util.spec_from_file_location('loc_launch', os.path.join(HERE, '..', 'launch', 'localization.launch.py'))
        mod = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(mod)
        ents = mod.generate_launch_description().entities
        procs = [e for e in ents if isinstance(e, ExecuteProcess)] + [
            a for e in ents if isinstance(e, TimerAction) for a in e.actions if isinstance(a, ExecuteProcess)]
        seeds = [p for p in procs if any('seed_pose.py' in getattr(s, 'text', '') for c in p.cmd for s in c)]
        self.assertEqual(len(seeds), 1)
        seed = seeds[0]
        default = [e for e in ents if isinstance(e, DeclareLaunchArgument) and e.name == 'seed_pose_file']
        self.assertEqual(perform_substitutions(LaunchContext(), default[0].default_value), P.DEFAULT_PATH)
        for value, on in ((P.DEFAULT_PATH, True), ('', False)):
            ctx = LaunchContext()
            ctx.launch_configurations.update(seed_pose_file=value, map_yaml=MAP_YAML)
            self.assertEqual(seed.condition.evaluate(ctx), on)
            if on:
                cmd = [perform_substitutions(ctx, c) for c in seed.cmd]
                self.assertEqual(cmd[-4:], ['--from-file', value, '--map-yaml', MAP_YAML])
        # OnProcessExit(target_action=a) keeps `a` as its (private, humble) action matcher
        matchers = [e.event_handler._OnActionEventBase__action_matcher for e in ents if isinstance(e, RegisterEventHandler)]
        self.assertEqual([p for p in procs if any(m is p for m in matchers)], [p for p in procs if p is not seed])


if __name__ == '__main__':
    unittest.main()
