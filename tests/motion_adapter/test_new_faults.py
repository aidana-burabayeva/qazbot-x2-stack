"""New fault paths: actual AdapterCore, no ROS and no MC."""
import unittest
from test_adapter_core import Sim, OURS
from x2_motion_adapter import ACTIVE, LATCHED, DRY_RUN

class NewFaults(unittest.TestCase):
    def test_faults_latch_and_do_not_auto_resume(self):
        for reason in ('localization_untrusted','foreign_velocity_publisher','vendor_autonomy_active'):
            with self.subTest(reason=reason):
                s=Sim();s.run(.2,resend=(.3,0,0),mc_source=OURS)
                kw={};localized=True
                if reason=='localization_untrusted':localized=False;kw['loc_reason']=reason
                if reason=='foreign_velocity_publisher':kw['foreign_pub']='other'
                if reason=='vendor_autonomy_active':s.mc.source='pnc'
                s.send(.3);d=s.core.step(s.t,s.cmd,s.cmd_t,s.mc,localized,**kw)
                self.assertEqual(d.state,LATCHED);self.assertEqual(d.reason,reason)
                self.assertEqual(d.publish,(0.,0.,0.))
                s.mc.source=OURS
                for d in s.run(.3,resend=(.3,0,0),mc_source=OURS):
                    self.assertEqual(d.state,LATCHED)
                    self.assertIn(d.publish,(None,(0.,0.,0.)))
    def test_idle_refuses_each_fault(self):
        for reason in ('localization_untrusted','foreign_velocity_publisher','vendor_autonomy_active'):
            s=Sim();s.send(.3);kw={};localized=True
            if reason=='localization_untrusted':localized=False;kw['loc_reason']=reason
            if reason=='foreign_velocity_publisher':kw['foreign_pub']='other'
            if reason=='vendor_autonomy_active':s.mc.source='pnc'
            d=s.core.step(s.t,s.cmd,s.cmd_t,s.mc,localized,**kw)
            self.assertEqual(d.state,LATCHED);self.assertIsNone(d.publish)
    def test_dry_run_refuses_each_fault_like_live(self):
        # superseded: test_dry_run_reports_each_fault_without_mc_output (dry run state was DRY_RUN, never latched)
        for reason in ('localization_untrusted','foreign_velocity_publisher','vendor_autonomy_active'):
            s=Sim(live=False);s.send(.3);kw={};localized=True
            if reason=='localization_untrusted':localized=False;kw['loc_reason']=reason
            if reason=='foreign_velocity_publisher':kw['foreign_pub']='other'
            if reason=='vendor_autonomy_active':s.mc.source='pnc'
            d=s.core.step(s.t,s.cmd,s.cmd_t,s.mc,localized,**kw)
            self.assertEqual(d.state,LATCHED);self.assertEqual(d.reason,reason);self.assertIsNone(d.publish)

if __name__=='__main__':unittest.main()
