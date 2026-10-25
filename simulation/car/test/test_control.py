import math
import unittest
from slamdunk_car.control import DriveGuard,source_stamp,track_setpoint,wrap

class ControlTests(unittest.TestCase):
    def ready(self):
        g=DriveGuard();g.set_request(.4,.8,10.);g.scan([float('inf')]*360,.08,15.,10.);g.cloud([],10.);g.odom(0.,10.);return g
    def test_startup_has_no_motion(self):
        self.assertEqual(DriveGuard().step(10.,.1),(0.,0.))
    def test_acceleration_and_limits(self):
        g=self.ready()
        for i in range(100):
            g.set_request(100.,100.,10.);v,w=g.step(10.,.025)
            self.assertLessEqual(v,.4);self.assertLessEqual(w,.8)
        self.assertAlmostEqual(v,.4);self.assertAlmostEqual(w,.8)
    def test_command_and_each_sensor_expire_independently(self):
        for attr in ('request_at','scan_at','cloud_at','odom_at'):
            with self.subTest(attr=attr):
                g=self.ready();setattr(g,attr,9.);self.assertEqual(g.step(10.,.025),(0.,0.))
    def test_moving_obstacle_triggers_braking_envelope(self):
        g=self.ready();g.odom(.4,10.);g.scan([.60]*360,.08,15.,10.)
        self.assertEqual(g.step(10.,.025),(0.,0.));self.assertIn('braking',g.reason)
    def test_reset_preserves_operator_stop(self):
        g=self.ready();g.estopped=True;g.reset();self.assertTrue(g.estopped)
    def test_low_obstacle_detected_in_3d(self):
        g=self.ready();g.cloud([(.45,0.,.08)],10.);self.assertEqual(g.step(10.,.025),(0.,0.))
    def test_floor_overhead_and_own_body_are_not_obstacles(self):
        g=self.ready();g.cloud([(.4,0.,0.),(.4,0.,2.),(.2,.1,.2)],10.);self.assertGreater(g.step(10.,.025)[0],0)
    def test_invalid_scan_does_not_refresh(self):
        g=self.ready();self.assertFalse(g.scan([math.nan]*360,.08,15.,11.));self.assertEqual(g.scan_at,10.)
    def test_nonfinite_command_withdraws_previous_request(self):
        g=self.ready();g.set_request(math.nan,0.,10.);self.assertEqual(g.step(10.,.1),(0.,0.))
    def test_latched_stop(self):
        g=self.ready();g.estopped=True;self.assertEqual(g.step(10.,.1),(0.,0.))
    def test_source_timestamp_rejections(self):
        for args in [(0,0,10.),(9,0,10.),(11,0,10.),(10,1_000_000_000,10.)]:
            with self.assertRaises(ValueError):source_stamp(*args)
        self.assertAlmostEqual(source_stamp(10,0,10.1),10.)
    def test_turn_first_and_wrapped_heading(self):
        v,w=track_setpoint((0,0,0),(0,.3,.3),(0,0,0),0,0);self.assertEqual(v,0);self.assertGreater(w,0)
        self.assertAlmostEqual(wrap(math.pi+.01),-math.pi+.01)
    def test_untrackable_or_vertical_reference_rejected(self):
        for target,velocity in [((1.,0,.3),(0,0,0)),((.1,0,.8),(0,0,0)),((.1,0,.3),(0,0,.1))]:
            with self.assertRaises(ValueError):track_setpoint((0,0,0),target,velocity,0,0)
    def test_straight_setpoint_converges_in_simple_planar_model(self):
        pose=[0.,0.,0.]
        for _ in range(300):
            v,w=track_setpoint(pose,(.5,0,.3),(0,0,0),0,0);v=min(v,.4);w=max(-.8,min(.8,w))
            pose[0]+=v*math.cos(pose[2])*.025;pose[1]+=v*math.sin(pose[2])*.025;pose[2]+=w*.025
        self.assertLess(abs(pose[0]-.5),.005);self.assertAlmostEqual(pose[1],0.)
