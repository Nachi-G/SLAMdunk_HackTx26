import math
import unittest
import numpy as np
from sando_bridge_core import CubicTrajectory,PreviewWatchdog,yaw_enu_to_ned,maximum_norm


def line(start=100.,duration=2.,x=0.,speed=.1):
    return CubicTrajectory([start,start+duration],[[[0,0,speed,x],[0,0,0,0],[0,0,0,2]]])


class CoreTests(unittest.TestCase):
    def test_segment_boundary_and_ned_derivatives(self):
        t=CubicTrajectory([100,101,102],[[[0,0,.1,0],[0,0,0,0],[0,0,0,2]],
                                       [[0,0,.1,.1],[0,0,0,0],[0,0,0,2]]])
        p=t.evaluate(101).ned()
        np.testing.assert_allclose(p.position,[0,.1,-2])
        np.testing.assert_allclose(p.velocity,[0,.1,0])
        with self.assertRaises(ValueError):t.evaluate(102)
        self.assertAlmostEqual(yaw_enu_to_ned(0),math.pi/2)
        self.assertAlmostEqual(yaw_enu_to_ned(math.pi/2),0)

    def test_interior_speed_peak_detected(self):
        # Velocity 1.1-(t-1)^2 peaks inside; endpoints are below limit.
        with self.assertRaisesRegex(ValueError,'dynamics'):
            CubicTrajectory([100,102],[[[-1/3,1,.1,0],[0,0,0,0],[0,0,0,2]]])
        self.assertAlmostEqual(maximum_norm([[.1,2,-1],[0],[0]],2),1.1)

    def test_bad_times_nonfinite_and_segment_jump(self):
        for times,coeff in [([1,1],[[[0,0,0,0]]*3]),([1,2],[[[math.nan,0,0,0]]*3])]:
            with self.assertRaises(ValueError):CubicTrajectory(times,coeff)
        with self.assertRaisesRegex(ValueError,'Discontinuous'):
            CubicTrajectory([100,101,102],[[[0,0,0,0],[0,0,0,0],[0,0,0,2]],
                                           [[0,0,0,1],[0,0,0,0],[0,0,0,2]]])

    def test_watchdog_stale_state_clock_and_timer(self):
        for args,reason in [((100.02,10.02,[0,0,2],.31),'stale valid state'),
                            ((100.5,10.02,[0,0,2],0),'Wall clock jumped'),
                            ((100.3,10.3,[0,0,2],0),'timer stalled')]:
            w=PreviewWatchdog(); w.accept(line(),100,10,[0,0,2]);w.tick(100,10,[0,0,2],0)
            with self.assertRaisesRegex(ValueError,reason):w.tick(*args)
            self.assertIsNone(w.trajectory)

    def test_replan_deadline_and_expiration(self):
        w=PreviewWatchdog();w.accept(line(),100,10,[0,0,2]);w.tick(100,10,[0,0,2],0)
        for i in range(1,21):w.tick(100+i*.05,10+i*.05,[0,0,2],0)
        with self.assertRaisesRegex(ValueError,'accepted replan'):
            w.tick(101.05,11.05,[0,0,2],0)
        w=PreviewWatchdog();w.accept(line(duration=.3),100,10,[0,0,2]);w.tick(100,10,[0,0,2],0)
        with self.assertRaisesRegex(ValueError,'expired or less'):
            w.tick(100.16,10.16,[0,0,2],0)

    def test_replan_jump_blocks(self):
        w=PreviewWatchdog();w.accept(line(),100,10,[0,0,2])
        with self.assertRaisesRegex(ValueError,'jump'):
            w.accept(line(x=.3),100.1,10.1,[0,0,2])
        self.assertIsNone(w.trajectory)


if __name__=='__main__':unittest.main()
