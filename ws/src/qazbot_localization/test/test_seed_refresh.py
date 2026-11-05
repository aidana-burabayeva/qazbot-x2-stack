"""Exercise the actual asynchronous refresh callback with deterministic clocks and service futures."""
import ast
from pathlib import Path
from types import SimpleNamespace as N
import unittest

NAV=Path(__file__).resolve().parents[2]/'ws/src/navigation'


class SeedRefresh(unittest.TestCase):
    def setUp(self):
        self.now=10.0;self.calls=[];self.warnings=[]
        source=(NAV/'localization_trust_node.py').read_text()
        cls=next(n for n in ast.parse(source).body if isinstance(n,ast.ClassDef) and n.name=='TrustNode')
        fn=next(n for n in cls.body if isinstance(n,ast.FunctionDef) and n.name=='_update_seed_without_motion')
        env=dict(time=N(monotonic=lambda:self.now),Empty=N(Request=lambda:object()))
        exec(compile(ast.Module(body=[fn],type_ignores=[]),'seed_refresh_callback','exec'),env)
        self.tick=lambda:env['_update_seed_without_motion'](self.node)
        self.node=N(seed_update=dict(start=9.,attempts=0,last_request=float('-inf'),scan=None),
                    seed_update_future=None,seed_stamp=100.,scan_received=9.9,scan_src=100.2,scan_t=10.,odom_t=10.,
                    cfg=N(scan_max_age_s=.5,odom_max_age_s=.5),_moving=lambda:False,
                    get_logger=lambda:N(info=lambda text:None,warn=self.warnings.append))
        self.node.nomotion=N(service_is_ready=lambda:True,call_async=self.call)
    def call(self,req):
        self.calls.append(req)
        return N(done=lambda:True,exception=lambda:None)
    def test_service_success_does_not_initialize_and_requests_are_bounded(self):
        for i in range(7):
            self.now=10.+i*.7
            self.node.scan_t=self.node.odom_t=self.now
            self.node.scan_src=100.2+i*.7
            self.tick()
        self.assertEqual(len(self.calls),3)
        self.assertEqual(self.node.seed_stamp,100.)
    def test_stale_data_motion_and_unavailable_service_do_not_request(self):
        for name,value in [('scan_t',8.),('odom_t',8.),('scan_src',99.),('scan_received',8.),('_moving',lambda:True)]:
            before=getattr(self.node,name);setattr(self.node,name,value);self.tick();setattr(self.node,name,before)
        self.node.nomotion.service_is_ready=lambda:False;self.tick()
        self.assertEqual(self.calls,[])
    def test_deadline_stops_without_granting_initialization(self):
        self.now=15.1;self.tick()
        self.assertIsNone(self.node.seed_update)
        self.assertEqual(self.node.seed_stamp,100.)
        self.assertTrue(self.warnings)
        self.assertEqual(self.calls,[])
    def test_waits_for_service_future_and_cancels_timeout(self):
        cancelled=[]
        self.node.seed_update.update(attempts=1,last_request=9.8,scan=100.)
        self.node.seed_update_future=N(done=lambda:False,cancel=lambda:cancelled.append(True))
        self.tick();self.assertEqual(self.calls,[])
        self.now=10.9;self.node.scan_t=self.node.odom_t=self.now
        self.tick();self.assertEqual(cancelled,[True]);self.assertEqual(len(self.calls),1)
    def test_amcl_acceptance_finishes_helper_without_another_request(self):
        self.node.seed_stamp=None;self.tick()
        self.assertIsNone(self.node.seed_update)
        self.assertEqual(self.calls,[])


if __name__=='__main__':unittest.main()
