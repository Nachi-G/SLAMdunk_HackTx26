import unittest
import numpy as np
from quick_px4_demo.planner import (trees,route,choose_velocity,clearance,CHALLENGE_WAYPOINTS,CHALLENGE_MOVERS)

class ChallengeTests(unittest.TestCase):
    def test_full_course_tracking_lag_multiple_phases(self):
        self.assertEqual(len(CHALLENGE_MOVERS),12)
        static=trees()
        for offset in (0.,12.,25.):
            with self.subTest(phase=offset):
                position=np.array([13.,0.]);velocity=np.zeros(2);command=np.zeros(2)
                waypoint=0;path=None;minimum=100.;blocks=0
                for i in range(1450):
                    t=offset+i*.1;goal=np.asarray(CHALLENGE_WAYPOINTS[waypoint])
                    if path is None:path=route(position,goal,t,static,profile='challenge')
                    command,valid,_=choose_velocity(position,velocity,goal,path,t,static,command,.1,profile='challenge')
                    blocks+=not valid
                    velocity+=(command-velocity)*.4;position+=velocity*.1
                    minimum=min(minimum,float(clearance(position,t+.1,static,margin=0,profile='challenge')))
                    if np.linalg.norm(position-goal)<.4:
                        waypoint+=1;path=None
                        if waypoint==len(CHALLENGE_WAYPOINTS):break
                self.assertEqual(waypoint,len(CHALLENGE_WAYPOINTS))
                self.assertEqual(blocks,0)
                self.assertGreater(minimum,.25)
                print(f'Challenge phase {offset}: {i*.1:.1f}s, clearance {minimum:.3f}m, {waypoint} checkpoints')
