"""The truth localiser owns exactly map -> odom (identity) and reports a trust status shaped like the real node's."""
import pytest

pytest.importorskip('geometry_msgs')
from builtin_interfaces.msg import Time   # noqa: E402
from x2_twin_sim.loc.truth_localizer_node import identity_map_odom, trust_status   # noqa: E402


def test_map_odom_is_identity():
    tf = identity_map_odom(Time(sec=1, nanosec=2))
    t, r = tf.transform.translation, tf.transform.rotation
    assert (tf.header.frame_id, tf.child_frame_id) == ('map', 'odom')
    assert (t.x, t.y, t.z) == (0.0, 0.0, 0.0) and (r.x, r.y, r.z, r.w) == (0.0, 0.0, 0.0, 1.0)


def test_status_marks_the_source():
    s = trust_status()
    assert s['trusted'] is True and s['ready'] is True and s['reasons'] == [] and s['source'] == 'truth_localizer'
