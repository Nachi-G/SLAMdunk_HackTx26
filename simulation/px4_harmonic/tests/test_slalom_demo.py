import subprocess
from pathlib import Path
import unittest
import numpy as np
from quick_px4_demo.planner import (trees,route,choose_velocity,clearance,movers,SLALOM_WAYPOINTS)


class SlalomTests(unittest.TestCase):
    def test_native_obstacles_match_planner(self):
        for profile in ('baseline','slalom','challenge'):
            args=[str(Path('quick_px4_demo/obstacle_rpc').resolve())]
            if profile!='baseline':args.append('--'+profile)
            args.append('--describe')
            text=subprocess.check_output(args,text=True)
            actual=np.asarray([list(map(float,line.split())) for line in text.splitlines()])
            np.testing.assert_allclose(actual,np.asarray(movers(profile)),atol=1e-9)

    def test_returns_to_start_and_weaves_in_order_with_lag(self):
        static=trees()
        for offset in (0.,12.,25.):
            with self.subTest(phase=offset):
                position=np.array([11.85,-.23]);velocity=np.zeros(2);command=np.zeros(2)
                waypoint=0;hits=[];path=None;minimum=100.;blocks=0
                for i in range(1600):
                    t=offset+i*.1;goal=np.asarray(SLALOM_WAYPOINTS[waypoint])
                    if path is None:path=route(position,goal,t,static,profile='slalom')
                    command,valid,_=choose_velocity(position,velocity,goal,path,t,static,command,.1,profile='slalom')
                    blocks+=not valid
                    velocity+=(command-velocity)*.4;position+=velocity*.1
                    minimum=min(minimum,float(clearance(position,t+.1,static,margin=0,profile='slalom')))
                    if np.linalg.norm(position-goal)<.4:
                        hits.append(position.copy());waypoint+=1;path=None
                        if waypoint==len(SLALOM_WAYPOINTS):break
                self.assertEqual(waypoint,len(SLALOM_WAYPOINTS))
                self.assertEqual(blocks,0)
                self.assertGreater(minimum,.25)
                self.assertGreater(hits[1][1],2.3)
                self.assertLess(hits[2][1],.1)
                self.assertGreater(hits[3][1],.8)
                self.assertLess(np.linalg.norm(hits[-1]-[13,0]),.4)

if __name__=='__main__':unittest.main()
