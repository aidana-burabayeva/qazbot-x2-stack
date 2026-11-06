"""Goal Coordinator, existing behaviour (T5 item 4, no logic change): an adapter operator-takeover latch while a goal
is busy trips the coordinator (gate closed, zero, Nav2 goal cancelled, latched); commands of the old goal are not
forwarded, and resume is refused until the old action is terminal. No ROS graph (see test_gc_t1_pause)."""
import json
import os
import sys
import unittest

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from test_gc_t1_pause import LIVE_OK, Pub, make  # noqa: E402
from geometry_msgs.msg import Twist  # noqa: E402
from std_msgs.msg import String  # noqa: E402


class Resp:
    pass


class Takeover(unittest.TestCase):
    def test_adapter_takeover_latch_cancels_goal_and_blocks_old_commands(self):
        n, c = make()
        child = n.child
        n.adapter(String(data=json.dumps(dict(LIVE_OK, state='latched', reason='operator_takeover',
                                              latch='operator_takeover', ready=False, motion_armed=False))))
        self.assertTrue(n.latched)
        self.assertFalse(n.gate)
        self.assertEqual(n.fault, 'operator_takeover')
        self.assertEqual(child.cancels, 1)
        self.assertEqual(n.pub.msgs[-1], Twist())
        n.pub = Pub()
        n.get_publishers_info_by_topic = lambda t: [type('E', (), {'endpoint_gid': b'1'})()]
        n.resolve_topic_name = lambda t: t
        m = Twist()
        m.linear.x = 0.3
        n.command(m)                                                 # Nav2 still computing the old goal
        self.assertEqual(n.pub.msgs, [])
        self.assertFalse(n.resume(None, Resp()).success)             # old action not terminal (busy)


if __name__ == '__main__':
    unittest.main()
