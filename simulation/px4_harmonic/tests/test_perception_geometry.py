import math
import unittest
import numpy as np
from perception_geometry import (PoseHistory,pose_matrix,slerp,rotation_matrix,
                                 px4_pose_enu,simulation_sample_time,mean_rigid_transform)


class GeometryTests(unittest.TestCase):
    def test_model_base_sensor_chain(self):
        world_model=pose_matrix([0,0,-.013],[1,0,0,0])
        model_base=pose_matrix([0,0,.24],[1,0,0,0])
        base_sensor=pose_matrix([0,0,.3],[1,0,0,0])
        sensor=world_model@model_base@base_sensor
        np.testing.assert_allclose(sensor[:3,3],[0,0,.527])

    def test_translation_and_rotated_pose_interpolation(self):
        h=PoseHistory(max_gap=1.)
        h.add(10,[0,0,0],[1,0,0,0]);h.add(11,[2,0,0],[0,0,0,1])
        mid=h.at(10.5)
        np.testing.assert_allclose(mid[:3,3],[1,0,0])
        np.testing.assert_allclose(mid[:3,:3]@[1,0,0],[0,1,0],atol=1e-8)
        with self.assertRaises(ValueError):h.at(11.1)
        with self.assertRaises(ValueError):h.at(9.9)

    def test_reset_gap_and_quaternion_sign(self):
        h=PoseHistory(max_gap=.1)
        h.add(1,[0,0,0],[1,0,0,0]);h.add(2,[0,0,0],[1,0,0,0])
        with self.assertRaisesRegex(ValueError,'gap'):h.at(1.5)
        with self.assertRaisesRegex(ValueError,'reset'):h.add(1.9,[0,0,0],[1,0,0,0])
        np.testing.assert_allclose(rotation_matrix(slerp([1,0,0,0],[-1,0,0,0],.5)),np.eye(3))

class AlignmentTests(unittest.TestCase):
    def test_px4_to_enu_position_and_heading(self):
        pose=px4_pose_enu([1,2,-3],[1,0,0,0])
        np.testing.assert_allclose(pose[:3,3],[2,1,3])
        np.testing.assert_allclose(pose[:3,:3]@[1,0,0],[0,1,0])

    def test_timestamp_mapping_keeps_microsecond_precision(self):
        self.assertAlmostEqual(simulation_sample_time(1792904201914107,-1792903815530304),386.383803)
        with self.assertRaises(ValueError):simulation_sample_time(1,-100)

    def test_rotated_world_origin_calibration_and_rejection(self):
        q=[math.cos(.1),0,0,math.sin(.1)]
        expected=pose_matrix([1,2,-.24],q)
        samples=[expected.copy() for _ in range(50)]
        result,spread,angle=mean_rigid_transform(samples)
        np.testing.assert_allclose(result,expected,atol=1e-12)
        self.assertAlmostEqual(spread,0)
        self.assertLess(angle,1e-6)
        samples[-1][:3,3]+=[1,0,0]
        with self.assertRaisesRegex(ValueError,'inconsistent'):mean_rigid_transform(samples)


if __name__=='__main__':unittest.main()
