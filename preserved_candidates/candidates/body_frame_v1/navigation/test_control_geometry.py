import math
import unittest
import numpy as np
from control_geometry import transform, world_base, planar_pose, invert_rigid, rotation_quaternion


def yaw_pose(x, y, angle):
    return transform([x,y,0], [0,0,math.sin(angle/2),math.cos(angle/2)])


class ControlGeometryTests(unittest.TestCase):
    def test_sensor_lever_arm_rotates_with_body(self):
        # Body at (2,3), facing +y; IMU 0.1m forward lies at (2,3.1).
        wi = transform([2,3.1,.9], [0,0,math.sqrt(.5),math.sqrt(.5)])
        bi = transform([.1,0,.9], [0,0,0,1])
        np.testing.assert_allclose(planar_pose(np.eye(3), world_base(wi,bi)), [2,3,math.pi/2], atol=1e-12)

    def test_torso_yaw_does_not_move_commanded_pelvis(self):
        # IMU/chest rotates relative to a stationary pelvis. Using its heading
        # directly would invent a robot turn; the body transform cancels it.
        body = yaw_pose(4,-2,.3)
        for chest_yaw in [-.7,0,.8]:
            bi = transform([.1,0,.33], [0,0,math.sin(chest_yaw/2),math.cos(chest_yaw/2)])
            np.testing.assert_allclose(world_base(body@bi,bi),body,atol=1e-12)

    def test_world_need_not_have_z_up(self):
        # FAST-LIO's world can start in a rotated sensor frame.
        level = np.array([[0,1,0],[0,0,1],[1,0,0]],float)
        # This orientation maps body +x onto world +z, hence level +y.
        pose = transform([0,2,3], [-.5,-.5,-.5,.5])
        np.testing.assert_allclose(planar_pose(level,pose),[2,3,math.pi/2],atol=1e-12)

    def test_bad_rotations_and_nonfinite_inputs_are_rejected(self):
        for xyz,q in [([0,0,float('nan')],[0,0,0,1]),([0,0,0],[0,0,0,0]),([0,0,0],[0,0,0,2])]:
            with self.assertRaises(ValueError):transform(xyz,q)
        reflection=np.eye(4);reflection[0,0]=-1
        with self.assertRaises(ValueError):invert_rigid(reflection)

    def test_vertical_body_heading_is_not_fabricated(self):
        vertical=transform([0,0,0],[0,math.sqrt(.5),0,math.sqrt(.5)])
        with self.assertRaises(ValueError):planar_pose(np.eye(3),vertical)

    def test_level_rotation_quaternion_including_half_turns(self):
        for q in ([0,0,0,1],[1,0,0,0],[0,1,0,0],[0,0,1,0],[.5,.5,.5,.5],[-.5,-.5,-.5,.5]):
            R = transform([0,0,0],q)[:3,:3]
            np.testing.assert_allclose(transform([0,0,0],rotation_quaternion(R))[:3,:3],R,atol=1e-12)


if __name__=='__main__':unittest.main()
